#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <map>
#include <memory>
#include <set>

#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/raw_ostream.h"

#include "r4300bt.hh"
#include "r4300cfg.hh"
#include "interpret.hh"

#pragma GCC diagnostic ignored "-Winvalid-offsetof"

/* A scratch word in guest RAM that a squashed store writes instead of its real address.
 * It must live in guest RAM rather than a host alloca: selecting between `ram + off` and
 * a host object would give LLVM a pointer into two possible objects and turn every such
 * store into one that may alias anything, which is exactly what the escape analysis in
 * DESIGN_R4300.md depends on not happening. */
static const uint32_t BITBUCKET_PA = 0x00700b00u;

typedef uint32_t (*r4300_fn_t)(state_t *, uint8_t *);

/* ---- callbacks into the interpreter ---- */
/* How much of the audio path is still interpreted, i.e. how much is left to translate. */
uint64_t g_r4300bt_callee_insns = 0, g_r4300bt_calls = 0;
std::map<uint32_t, uint64_t> g_r4300bt_fallback_targets;

extern "C" {
  /* Run a callee to completion: the interpreter resumes at `target` and stops when it
   * comes back to `link`.  Used for any call whose target we did not translate. */
  void r4300bt_call(state_t *s, uint32_t target, uint32_t link) {
    g_r4300bt_calls++;
    if(getenv("R4300BT_FALLBACKS") != nullptr) {
      extern std::map<uint32_t, uint64_t> g_r4300bt_fallback_targets;
      g_r4300bt_fallback_targets[target]++;
    }
    s->pc = static_cast<int64_t>(static_cast<int32_t>(target));
    uint64_t budget = 1ULL << 32;
    while(static_cast<uint32_t>(s->pc) != link) {
      execMips(s);
      g_r4300bt_callee_insns++;
      if(--budget == 0) {
	fprintf(stderr, "r4300bt: call to %08x never returned\n", target);
	exit(-1);
      }
    }
  }
}

namespace {

/* Codegen context for one guest function.  GPRs are allocas that mem2reg promotes; the
 * FP file is left in state_t because the interpreter's exact layout already lives there
 * (flat 64-bit slots, a single writing only bits[31:0]) and FP is a thin slice of this
 * code -- reproducing that layout is worth more than keeping it in registers. */
struct fnCtx {
  llvm::LLVMContext &ctx;
  llvm::Module *mod;
  llvm::IRBuilder<> *b;
  llvm::Function *fn = nullptr;
  llvm::Value *vState = nullptr, *vRam = nullptr;

  llvm::Type *i1, *i8, *i16, *i32, *i64, *f32, *f64, *ptr;

  llvm::AllocaInst *gpr[32] = {nullptr};
  llvm::AllocaInst *bailSlot = nullptr;
  llvm::BasicBlock *exitBB = nullptr;
  std::map<uint32_t, llvm::BasicBlock*> blocks;
  const std::map<uint32_t, llvm::Function*> *decls = nullptr;  /* every translated entry */

  /* Non-null while emitting the delay slot of a branch-likely: every architectural
   * effect is selected on it, so the slot runs unconditionally but only commits when the
   * branch is taken.  See DESIGN_R4300.md, "Branch-likely". */
  llvm::Value *pred = nullptr;

  fnCtx(llvm::LLVMContext &c, llvm::Module *m, llvm::IRBuilder<> *bb) : ctx(c), mod(m), b(bb) {
    i1 = llvm::Type::getInt1Ty(c);    i8  = llvm::Type::getInt8Ty(c);
    i16 = llvm::Type::getInt16Ty(c);  i32 = llvm::Type::getInt32Ty(c);
    i64 = llvm::Type::getInt64Ty(c);  f32 = llvm::Type::getFloatTy(c);
    f64 = llvm::Type::getDoubleTy(c); ptr = llvm::PointerType::get(c, 0);
  }

  llvm::Value *c32(uint32_t x) { return llvm::ConstantInt::get(i32, x); }
  llvm::Value *c64(uint64_t x) { return llvm::ConstantInt::get(i64, x); }

  llvm::Value *getGPR(uint32_t r) {
    return (r == 0) ? c64(0) : b->CreateLoad(i64, gpr[r]);
  }
  void setGPR(uint32_t r, llvm::Value *v) {
    if(r == 0) {
      return;                       /* $0 is hardwired; the interpreter re-zeroes it */
    }
    if(pred != nullptr) {
      v = b->CreateSelect(pred, v, b->CreateLoad(i64, gpr[r]));
    }
    b->CreateStore(v, gpr[r]);
  }
  llvm::Value *sext32(llvm::Value *v32) { return b->CreateSExt(v32, i64); }
  llvm::Value *lo32(llvm::Value *v64) { return b->CreateTrunc(v64, i32); }

  llvm::Value *statePtr(size_t off) {
    return b->CreateGEP(i8, vState, c64(off));
  }

  /* Guest physical address, matching r4300_t::ptr_any: the game's TLB alias at
   * 0x70000000 has PFN 0, so its physical address is the low 28 bits; the plain kseg0
   * mask would leave bit 28 set and land 256 MB away. */
  llvm::Value *physAddr(llvm::Value *ea64) {
    llvm::Value *va = lo32(ea64);
    llvm::Value *isAlias = b->CreateICmpEQ(b->CreateLShr(va, c32(28)), c32(7));
    return b->CreateSelect(isAlias, b->CreateAnd(va, c32(0x0fffffff)),
			   b->CreateAnd(va, c32(0x1fffffff)));
  }
  llvm::Value *ramPtr(llvm::Value *pa32) {
    return b->CreateGEP(i8, vRam, b->CreateZExt(pa32, i64));
  }

  llvm::Value *effAddr(uint32_t insn) {
    const uint32_t base = (insn >> 21) & 31;
    const int64_t simm = static_cast<int16_t>(insn & 0xffff);
    return b->CreateAdd(getGPR(base), c64(static_cast<uint64_t>(simm)));
  }

  /* Big-endian guest memory.  A load is safe to perform unconditionally even under a
   * predicate: the address is masked into the backing store, so it cannot fault. */
  llvm::Value *loadMem(llvm::Value *ea, unsigned nbytes) {
    llvm::Type *t = (nbytes == 8) ? i64 : (nbytes == 4) ? i32 : (nbytes == 2) ? i16 : i8;
    llvm::Value *p = ramPtr(physAddr(ea));
    llvm::Value *v = b->CreateAlignedLoad(t, p, llvm::MaybeAlign(1));
    if(nbytes > 1) {
      v = b->CreateUnaryIntrinsic(llvm::Intrinsic::bswap, v);
    }
    return v;
  }
  void storeMem(llvm::Value *ea, llvm::Value *val, unsigned nbytes) {
    llvm::Value *pa = physAddr(ea);
    if(pred != nullptr) {
      pa = b->CreateSelect(pred, pa, c32(BITBUCKET_PA));
    }
    if(nbytes > 1) {
      val = b->CreateUnaryIntrinsic(llvm::Intrinsic::bswap, val);
    }
    b->CreateAlignedStore(val, ramPtr(pa), llvm::MaybeAlign(1));
  }

  /* FP file, in state_t, in the interpreter's layout: 32 flat 64-bit slots.  A single
   * occupies bits[31:0] and a write to it must leave bits[63:32] alone. */
  /* Arithmetic, cvt and trunc index cpr1 directly (`*(T*)(s->cpr1+fs)`), which is only
   * ever given even registers under FR=0. */
  llvm::Value *loPtr() { return statePtr(offsetof(state_t, lo)); }
  llvm::Value *hiPtr() { return statePtr(offsetof(state_t, hi)); }
  /* the FP condition codes: bit `cc` of fcr1[CP1_CR25], and CP1_CR25 is index 2 */
  llvm::Value *fccPtr() { return statePtr(offsetof(state_t, fcr1) + 4 * 2); }

  llvm::Value *fprPtr(uint32_t r) {
    return statePtr(offsetof(state_t, cpr1) + 8 * r);
  }
  /* The GPR<->FPR moves and the FP loads/stores go through fpr_hw()/fpr_dreg(), which
   * honour SR.FR.  With FR=0 -- what this code runs under -- a single lives in the
   * (r&1) 32-bit half of slot (r&~1), so `mtc1 $t,$f5` writes the HIGH half of $f4;
   * that pairing is how the compiler builds double constants.  A double is the whole
   * slot (r&~1). */
  llvm::Value *fprHalfPtr(uint32_t r) {
    return statePtr(offsetof(state_t, cpr1) + 8 * (r & ~1u) + 4 * (r & 1u));
  }
  llvm::Value *fprDblPtr(uint32_t r) {
    return statePtr(offsetof(state_t, cpr1) + 8 * (r & ~1u));
  }
  /* a store into state_t, predicated like every other architectural effect */
  void storeGuarded(llvm::Value *p, llvm::Value *v) {
    if(pred != nullptr) {
      v = b->CreateSelect(pred, v, b->CreateAlignedLoad(v->getType(), p, llvm::MaybeAlign(4)));
    }
    b->CreateAlignedStore(v, p, llvm::MaybeAlign(4));
  }

  llvm::BasicBlock *newBlock(const char *nm) {
    return llvm::BasicBlock::Create(ctx, nm, fn);
  }
  llvm::BasicBlock *blockFor(uint32_t addr) {
    auto it = blocks.find(addr);
    if(it != blocks.end()) {
      return it->second;
    }
    char nm[32];
    snprintf(nm, sizeof(nm), "b%08x", addr);
    llvm::BasicBlock *bb = llvm::BasicBlock::Create(ctx, nm, fn);
    blocks[addr] = bb;
    return bb;
  }
  /* Leave translated code: state_t is already current for the GPRs we spill here, and
   * the interpreter picks up at `pc`. */
  void bailTo(uint32_t pc) {
    b->CreateStore(c32(pc), bailSlot);
    b->CreateBr(exitBB);
  }
};

}; // namespace

namespace {

/* ---- FP register access, in the interpreter's layout ---- */
static llvm::Value *getF32(fnCtx &f, uint32_t r) {
  return f.b->CreateAlignedLoad(f.f32, f.fprPtr(r), llvm::MaybeAlign(4));
}
static llvm::Value *getF64(fnCtx &f, uint32_t r) {
  return f.b->CreateAlignedLoad(f.f64, f.fprPtr(r), llvm::MaybeAlign(8));
}
/* `*(float*)(cpr1+fd) = x`: a 32-bit store, so bits[63:32] of the slot survive */
static void setF32(fnCtx &f, uint32_t r, llvm::Value *v) {
  f.storeGuarded(f.fprPtr(r), v);
}
static void setF64(fnCtx &f, uint32_t r, llvm::Value *v) {
  f.storeGuarded(f.fprPtr(r), v);
}

/* float -> int32 exactly as the interpreter's `(int32_t)nearbyint(v)` compiles on x86:
 * cvttsd2si, which yields 0x80000000 for NaN and for anything out of range.  A bare
 * fptosi is poison there, and poison is not something to hand an optimizer. */
static llvm::Value *toI32(fnCtx &f, llvm::Value *v, llvm::Type *ft) {
  llvm::Value *lo = llvm::ConstantFP::get(ft, -2147483648.0);
  llvm::Value *hi = llvm::ConstantFP::get(ft, 2147483648.0);
  llvm::Value *ok = f.b->CreateAnd(f.b->CreateFCmpOGE(v, lo), f.b->CreateFCmpOLT(v, hi));
  return f.b->CreateSelect(ok, f.b->CreateFPToSI(v, f.i32), f.c32(0x80000000u));
}

/* One COP1 instruction.  false means "not translated": the caller bails. */
static bool emitCop1(fnCtx &f, uint32_t insn) {
  const uint32_t rs = (insn >> 21) & 31, rt = (insn >> 16) & 31;
  const uint32_t fs = (insn >> 11) & 31, fd = (insn >> 6) & 31, ft = (insn >> 16) & 31;
  switch(rs)
    {
    case 0x00:                                        /* mfc1 */
      f.setGPR(rt, f.sext32(f.b->CreateAlignedLoad(f.i32, f.fprHalfPtr(fs), llvm::MaybeAlign(4))));
      return true;
    case 0x04:                                        /* mtc1 */
      f.storeGuarded(f.fprHalfPtr(fs), f.lo32(f.getGPR(rt)));
      return true;
    case 0x10: case 0x11: {                           /* FMT_S / FMT_D arithmetic */
      const bool dbl = (rs == 0x11);
      llvm::Type *ft_ty = dbl ? f.f64 : f.f32;
      auto get = [&](uint32_t r) { return dbl ? getF64(f, r) : getF32(f, r); };
      auto put = [&](uint32_t r, llvm::Value *v) { if(dbl) { setF64(f, r, v); } else { setF32(f, r, v); } };
      switch(insn & 0x3f)
	{
	case 0x00: put(fd, f.b->CreateFAdd(get(fs), get(ft))); return true;
	case 0x01: put(fd, f.b->CreateFSub(get(fs), get(ft))); return true;
	case 0x02: put(fd, f.b->CreateFMul(get(fs), get(ft))); return true;
	case 0x03: put(fd, f.b->CreateFDiv(get(fs), get(ft))); return true;
	case 0x05: put(fd, f.b->CreateUnaryIntrinsic(llvm::Intrinsic::fabs, get(fs))); return true;
	case 0x06: put(fd, get(fs)); return true;                          /* mov */
	case 0x07: put(fd, f.b->CreateFNeg(get(fs))); return true;
	case 0x0d:                                                         /* trunc.w */
	  f.storeGuarded(f.fprPtr(fd), toI32(f, get(fs), ft_ty));
	  return true;
	case 0x30: case 0x31: case 0x32: case 0x33:                        /* c.cond */
	case 0x34: case 0x35: case 0x36: case 0x37:
	case 0x38: case 0x39: case 0x3a: case 0x3b:
	case 0x3c: case 0x3d: case 0x3e: case 0x3f: {
	  llvm::Value *cmp = nullptr;
	  switch(insn & 15)
	    {
	    case 1: case 2: cmp = f.b->CreateFCmpOEQ(get(fs), get(ft)); break;   /* UN, EQ */
	    case 12: cmp = f.b->CreateFCmpOLT(get(fs), get(ft)); break;          /* LT */
	    case 14: cmp = f.b->CreateFCmpOLE(get(fs), get(ft)); break;          /* LE */
	    default: return false;
	    }
	  const uint32_t cc = (insn >> 8) & 7;
	  llvm::Value *old = f.b->CreateAlignedLoad(f.i32, f.fccPtr(), llvm::MaybeAlign(4));
	  llvm::Value *set = f.b->CreateOr(old, f.c32(1u << cc));
	  llvm::Value *clr = f.b->CreateAnd(old, f.c32(~(1u << cc)));
	  f.storeGuarded(f.fccPtr(), f.b->CreateSelect(cmp, set, clr));
	  return true;
	}
	case 0x20:                                                         /* cvt.s */
	  if(not dbl) { return false; }
	  setF32(f, fd, f.b->CreateFPTrunc(get(fs), f.f32));
	  return true;
	case 0x21:                                                         /* cvt.d */
	  if(dbl) { return false; }
	  setF64(f, fd, f.b->CreateFPExt(get(fs), f.f64));
	  return true;
	default: return false;
	}
    }
    case 0x14: {                                      /* FMT_W: cvt.s.w / cvt.d.w */
      llvm::Value *iv = f.b->CreateAlignedLoad(f.i32, f.fprPtr(fs), llvm::MaybeAlign(4));
      switch(insn & 0x3f)
	{
	case 0x20: setF32(f, fd, f.b->CreateSIToFP(iv, f.f32)); return true;
	case 0x21: setF64(f, fd, f.b->CreateSIToFP(iv, f.f64)); return true;
	default: return false;
	}
    }
    default: return false;
    }
}

/* One non-control instruction.  false means "not translated": the caller bails. */
static bool emitOne(fnCtx &f, uint32_t insn) {
  llvm::IRBuilder<> &b = *f.b;
  const uint32_t op = insn >> 26;
  const uint32_t rs = (insn >> 21) & 31, rt = (insn >> 16) & 31;
  const uint32_t rd = (insn >> 11) & 31, sa = (insn >> 6) & 31;
  const uint32_t imm = insn & 0xffff;
  const int64_t simm = static_cast<int16_t>(imm);

  switch(op)
    {
    case 0x00:
      switch(insn & 0x3f)
	{
	case 0x00: f.setGPR(rd, f.sext32(b.CreateShl(f.lo32(f.getGPR(rt)), f.c32(sa)))); return true;
	case 0x02: f.setGPR(rd, f.sext32(b.CreateLShr(f.lo32(f.getGPR(rt)), f.c32(sa)))); return true;
	case 0x03: f.setGPR(rd, f.sext32(b.CreateAShr(f.lo32(f.getGPR(rt)), f.c32(sa)))); return true;
	case 0x04: case 0x06: case 0x07: {
	  llvm::Value *amt = b.CreateAnd(f.lo32(f.getGPR(rs)), f.c32(31));
	  llvm::Value *v = f.lo32(f.getGPR(rt));
	  const uint32_t fn = insn & 0x3f;
	  v = (fn == 0x04) ? b.CreateShl(v, amt) : (fn == 0x06) ? b.CreateLShr(v, amt) : b.CreateAShr(v, amt);
	  f.setGPR(rd, f.sext32(v));
	  return true;
	}
	/* add/sub (0x20/0x22) raise an overflow exception in the interpreter, so they are
	 * not the same instruction as addu/subu and are left to it. */
	case 0x21:
	  f.setGPR(rd, f.sext32(b.CreateAdd(f.lo32(f.getGPR(rs)), f.lo32(f.getGPR(rt))))); return true;
	case 0x23:
	  f.setGPR(rd, f.sext32(b.CreateSub(f.lo32(f.getGPR(rs)), f.lo32(f.getGPR(rt))))); return true;
	case 0x24: f.setGPR(rd, b.CreateAnd(f.getGPR(rs), f.getGPR(rt))); return true;
	case 0x25: f.setGPR(rd, b.CreateOr(f.getGPR(rs), f.getGPR(rt))); return true;
	case 0x26: f.setGPR(rd, b.CreateXor(f.getGPR(rs), f.getGPR(rt))); return true;
	case 0x27: f.setGPR(rd, b.CreateNot(b.CreateOr(f.getGPR(rs), f.getGPR(rt)))); return true;
	case 0x10: f.setGPR(rd, b.CreateAlignedLoad(f.i64, f.hiPtr(), llvm::MaybeAlign(8))); return true;
	case 0x11: f.storeGuarded(f.hiPtr(), f.getGPR(rs)); return true;
	case 0x12: f.setGPR(rd, b.CreateAlignedLoad(f.i64, f.loPtr(), llvm::MaybeAlign(8))); return true;
	case 0x13: f.storeGuarded(f.loPtr(), f.getGPR(rs)); return true;
	case 0x18: {                                                       /* mult */
	  llvm::Value *y = b.CreateMul(b.CreateSExt(f.lo32(f.getGPR(rs)), f.i64),
				       b.CreateSExt(f.lo32(f.getGPR(rt)), f.i64));
	  f.storeGuarded(f.loPtr(), f.sext32(b.CreateTrunc(y, f.i32)));
	  f.storeGuarded(f.hiPtr(), f.sext32(b.CreateTrunc(b.CreateAShr(y, f.c64(32)), f.i32)));
	  return true;
	}
	case 0x19: {                                                       /* multu */
	  llvm::Value *y = b.CreateMul(b.CreateZExt(f.lo32(f.getGPR(rs)), f.i64),
				       b.CreateZExt(f.lo32(f.getGPR(rt)), f.i64));
	  f.storeGuarded(f.loPtr(), f.sext32(b.CreateTrunc(y, f.i32)));
	  f.storeGuarded(f.hiPtr(), f.sext32(b.CreateTrunc(b.CreateLShr(y, f.c64(32)), f.i32)));
	  return true;
	}
	case 0x1a: case 0x1b: {                                            /* div, divu */
	  /* A zero divisor leaves lo/hi untouched, so the result is selected -- but the
	   * division must not be *executed* with a zero divisor either, because that is
	   * a host trap, not merely a bad value.  Hence the substituted divisor, the
	   * same trick as the store bit bucket.  Signed division is done in 64 bits,
	   * as interpret.cc does, so INT_MIN / -1 cannot trap either. */
	  const bool sgn = ((insn & 0x3f) == 0x1a);
	  llvm::Value *a32 = f.lo32(f.getGPR(rs)), *b32 = f.lo32(f.getGPR(rt));
	  /* interpret.cc guards div on the low 32 bits of rt but divu on all 64 */
	  llvm::Value *nz = sgn ? b.CreateICmpNE(b32, f.c32(0))
				: b.CreateICmpNE(f.getGPR(rt), f.c64(0));
	  llvm::Value *ok = b.CreateICmpNE(b32, f.c32(0));
	  llvm::Value *q = nullptr, *r = nullptr;
	  if(sgn) {
	    llvm::Value *a = b.CreateSExt(a32, f.i64);
	    llvm::Value *d = b.CreateSExt(b.CreateSelect(ok, b32, f.c32(1)), f.i64);
	    q = f.sext32(b.CreateTrunc(b.CreateSDiv(a, d), f.i32));
	    r = f.sext32(b.CreateTrunc(b.CreateSRem(a, d), f.i32));
	  }
	  else {
	    llvm::Value *d = b.CreateSelect(ok, b32, f.c32(1));
	    q = f.sext32(b.CreateUDiv(a32, d));
	    r = f.sext32(b.CreateURem(a32, d));
	  }
	  llvm::Value *oldLo = b.CreateAlignedLoad(f.i64, f.loPtr(), llvm::MaybeAlign(8));
	  llvm::Value *oldHi = b.CreateAlignedLoad(f.i64, f.hiPtr(), llvm::MaybeAlign(8));
	  f.storeGuarded(f.loPtr(), b.CreateSelect(nz, q, oldLo));
	  f.storeGuarded(f.hiPtr(), b.CreateSelect(nz, r, oldHi));
	  return true;
	}
	case 0x2a: f.setGPR(rd, b.CreateZExt(b.CreateICmpSLT(f.getGPR(rs), f.getGPR(rt)), f.i64)); return true;
	case 0x2b: f.setGPR(rd, b.CreateZExt(b.CreateICmpULT(f.getGPR(rs), f.getGPR(rt)), f.i64)); return true;
	default: return false;
	}
    /* addi (0x08), like add/sub, raises an overflow exception and is left to the
     * interpreter; only the non-trapping addiu is translated. */
    case 0x09:                                                              /* addiu */
      f.setGPR(rt, f.sext32(b.CreateAdd(f.lo32(f.getGPR(rs)), f.c32(static_cast<uint32_t>(simm))))); return true;
    case 0x0a: f.setGPR(rt, b.CreateZExt(b.CreateICmpSLT(f.getGPR(rs), f.c64(simm)), f.i64)); return true;
    case 0x0b: f.setGPR(rt, b.CreateZExt(b.CreateICmpULT(f.getGPR(rs), f.c64(simm)), f.i64)); return true;
    case 0x0c: f.setGPR(rt, b.CreateAnd(f.getGPR(rs), f.c64(imm))); return true;
    case 0x0d: f.setGPR(rt, b.CreateOr(f.getGPR(rs), f.c64(imm))); return true;
    case 0x0e: f.setGPR(rt, b.CreateXor(f.getGPR(rs), f.c64(imm))); return true;
    case 0x0f: f.setGPR(rt, f.c64(static_cast<int64_t>(static_cast<int32_t>(imm << 16)))); return true;

    case 0x20: f.setGPR(rt, b.CreateSExt(f.loadMem(f.effAddr(insn), 1), f.i64)); return true;   /* lb  */
    case 0x21: f.setGPR(rt, b.CreateSExt(f.loadMem(f.effAddr(insn), 2), f.i64)); return true;   /* lh  */
    case 0x23: f.setGPR(rt, b.CreateSExt(f.loadMem(f.effAddr(insn), 4), f.i64)); return true;   /* lw  */
    case 0x24: f.setGPR(rt, b.CreateZExt(f.loadMem(f.effAddr(insn), 1), f.i64)); return true;   /* lbu */
    case 0x25: f.setGPR(rt, b.CreateZExt(f.loadMem(f.effAddr(insn), 2), f.i64)); return true;   /* lhu */
    case 0x27: f.setGPR(rt, b.CreateZExt(f.loadMem(f.effAddr(insn), 4), f.i64)); return true;   /* lwu */
    case 0x37: f.setGPR(rt, f.loadMem(f.effAddr(insn), 8)); return true;                        /* ld  */
    case 0x28: f.storeMem(f.effAddr(insn), b.CreateTrunc(f.getGPR(rt), f.i8), 1); return true;
    case 0x29: f.storeMem(f.effAddr(insn), b.CreateTrunc(f.getGPR(rt), f.i16), 2); return true;
    case 0x2b: f.storeMem(f.effAddr(insn), f.lo32(f.getGPR(rt)), 4); return true;
    case 0x3f: f.storeMem(f.effAddr(insn), f.getGPR(rt), 8); return true;

    case 0x31:                                                              /* lwc1 */
      f.storeGuarded(f.fprHalfPtr(rt), f.loadMem(f.effAddr(insn), 4)); return true;
    case 0x35:                                                              /* ldc1 */
      f.storeGuarded(f.fprDblPtr(rt), f.loadMem(f.effAddr(insn), 8)); return true;
    case 0x39:                                                              /* swc1 */
      f.storeMem(f.effAddr(insn), b.CreateAlignedLoad(f.i32, f.fprHalfPtr(rt), llvm::MaybeAlign(4)), 4); return true;
    case 0x3d:                                                              /* sdc1 */
      f.storeMem(f.effAddr(insn), b.CreateAlignedLoad(f.i64, f.fprDblPtr(rt), llvm::MaybeAlign(8)), 8); return true;

    case 0x11: return emitCop1(f, insn);
    default: return false;
    }
}

}; // namespace


namespace {

/* Spill/reload the GPR file around a call.  Written naively on purpose: when the callee
 * is another translated function in this same module and the inliner takes it, its entry
 * reload sits directly on top of this spill and GVN deletes both.  What survives is
 * exactly the calls that were not inlined, where the transfer really is needed. */
static void spillGPRs(fnCtx &f) {
  for(int i = 1; i < 32; i++) {
    f.b->CreateAlignedStore(f.b->CreateLoad(f.i64, f.gpr[i]),
			    f.statePtr(offsetof(state_t, gpr) + 8 * i), llvm::MaybeAlign(8));
  }
}
static void reloadGPRs(fnCtx &f) {
  for(int i = 1; i < 32; i++) {
    f.b->CreateStore(f.b->CreateAlignedLoad(f.i64, f.statePtr(offsetof(state_t, gpr) + 8 * i),
					    llvm::MaybeAlign(8)), f.gpr[i]);
  }
}

static llvm::FunctionCallee interpCallee(fnCtx &f) {
  return f.mod->getOrInsertFunction(
    "r4300bt_call", llvm::FunctionType::get(llvm::Type::getVoidTy(f.ctx), {f.ptr, f.i32, f.i32}, false));
}

/* A call to a translated function.  If it bails, the interpreter finishes the callee from
 * wherever it stopped until control comes back to our link address -- so a bail anywhere
 * down the call graph costs correctness nothing and only that subtree's speed. */
static void emitDirectCall(fnCtx &f, llvm::Function *callee, uint32_t link) {
  llvm::Value *r = f.b->CreateCall(callee, {f.vState, f.vRam});
  llvm::Value *bailed = f.b->CreateICmpNE(r, f.c32(0));
  llvm::BasicBlock *fix = f.newBlock("bailfix");
  llvm::BasicBlock *cont = f.newBlock("callcont");
  f.b->CreateCondBr(bailed, fix, cont);
  f.b->SetInsertPoint(fix);
  f.b->CreateCall(interpCallee(f), {f.vState, r, f.c32(link)});
  f.b->CreateBr(cont);
  f.b->SetInsertPoint(cont);
}

static void emitCall(fnCtx &f, llvm::Value *target, uint32_t link, llvm::Function *direct) {
  spillGPRs(f);
  if(direct != nullptr) {
    emitDirectCall(f, direct, link);
  }
  else {
    f.b->CreateCall(interpCallee(f), {f.vState, target, f.c32(link)});
  }
  reloadGPRs(f);
}

/* An indirect call.  Every target the discovery ever saw here is compared against and
 * called directly; anything else falls back to the interpreter.  27 of the 52 jalr sites
 * in this code are monomorphic, so this is usually one compare. */
static void emitIndirectCall(fnCtx &f, llvm::Value *target, uint32_t link,
			     const std::set<uint32_t> &tgts) {
  spillGPRs(f);
  llvm::BasicBlock *cont = f.newBlock("callcont");
  for(uint32_t t : tgts) {
    auto it = f.decls->find(t);
    if(it == f.decls->end()) {
      continue;
    }
    llvm::BasicBlock *hit = f.newBlock("icall");
    llvm::BasicBlock *next = f.newBlock("icmp");
    f.b->CreateCondBr(f.b->CreateICmpEQ(target, f.c32(t)), hit, next);
    f.b->SetInsertPoint(hit);
    emitDirectCall(f, it->second, link);
    f.b->CreateBr(cont);
    f.b->SetInsertPoint(next);
  }
  f.b->CreateCall(interpCallee(f), {f.vState, target, f.c32(link)});
  f.b->CreateBr(cont);
  f.b->SetInsertPoint(cont);
  reloadGPRs(f);
}

/* Condition of a conditional branch, from the registers as they are BEFORE the delay
 * slot runs -- the slot may well overwrite them. */
static llvm::Value *branchCond(fnCtx &f, uint32_t insn) {
  llvm::IRBuilder<> &b = *f.b;
  const uint32_t op = insn >> 26, rs = (insn >> 21) & 31, rt = (insn >> 16) & 31;
  llvm::Value *vs = f.getGPR(rs);
  switch(op)
    {
    case 0x04: case 0x14: return b.CreateICmpEQ(vs, f.getGPR(rt));
    case 0x05: case 0x15: return b.CreateICmpNE(vs, f.getGPR(rt));
    case 0x06: case 0x16: return b.CreateICmpSLE(vs, f.c64(0));
    case 0x07: case 0x17: return b.CreateICmpSGT(vs, f.c64(0));
    case 0x01:
      return ((rt & 1) == 0) ? b.CreateICmpSLT(vs, f.c64(0)) : b.CreateICmpSGE(vs, f.c64(0));
    case 0x11: {                                  /* bc1f / bc1t and the likely forms */
      const uint32_t cc = (insn >> 18) & 7;
      llvm::Value *bit = b.CreateAnd(b.CreateAlignedLoad(f.i32, f.fccPtr(), llvm::MaybeAlign(4)),
				     f.c32(1u << cc));
      llvm::Value *isSet = b.CreateICmpNE(bit, f.c32(0));
      return ((rt & 1) != 0) ? isSet : b.CreateNot(isSet);
    }
    default: return nullptr;
    }
}

/* ---- one guest function ---- */
static void emitBody(fnCtx &f, const r4300_cfg &cfg, const r4300_cfg::func &gf, uint32_t entry) {
  llvm::IRBuilder<> &builder = *f.b;
  llvm::BasicBlock *entryBB = llvm::BasicBlock::Create(f.ctx, "entry", f.fn);
  f.exitBB = llvm::BasicBlock::Create(f.ctx, "exit", f.fn);
  builder.SetInsertPoint(entryBB);
  for(int i = 1; i < 32; i++) {
    f.gpr[i] = builder.CreateAlloca(f.i64);
  }
  f.bailSlot = builder.CreateAlloca(f.i32);
  builder.CreateStore(f.c32(0), f.bailSlot);
  reloadGPRs(f);
  builder.CreateBr(f.blockFor(entry));

  for(const auto &bkv : gf.blocks) {
    const r4300_cfg::block &blk = bkv.second;
    builder.SetInsertPoint(f.blockFor(blk.start));
    uint32_t pc = blk.start;
    bool closed = false;

    while(pc < blk.end) {
      uint32_t insn = 0;
      if(not cfg.read(pc, insn)) {
	f.bailTo(pc);
	closed = true;
	break;
      }
      const uint32_t op = insn >> 26;
      const bool isCtl = (op == 0x02 or op == 0x03 or (op >= 0x04 and op <= 0x07) or
			  op == 0x01 or (op >= 0x14 and op <= 0x17) or
			  (op == 0x11 and ((insn >> 21) & 31) == 0x08) or
			  (op == 0x00 and ((insn & 0x3f) == 0x08 or (insn & 0x3f) == 0x09)));
      if(not isCtl) {
	if(not emitOne(f, insn)) {
	  f.bailTo(pc);
	  closed = true;
	  break;
	}
	pc += 4;
	continue;
      }

      uint32_t slot = 0;
      if(not cfg.read(pc + 4, slot)) {
	f.bailTo(pc);
	closed = true;
	break;
      }
      const uint32_t link = pc + 8;
      /* REGIMM rt: 0=bltz 1=bgez 2=bltzl 3=bgezl, 8-15 traps, 16+ link-and-branch.
       * Only the four plain ones are translated. */
      if(op == 0x01 and (((insn >> 16) & 31) > 3)) {
	f.bailTo(pc);
	closed = true;
	break;
      }
      const bool likely = (op >= 0x14 and op <= 0x17) or
			  (op == 0x01 and (((insn >> 16) & 0x02) != 0)) or
			  (op == 0x11 and (((insn >> 16) & 0x02) != 0));
      llvm::Value *cond = branchCond(f, insn);
      llvm::Value *jrTarget = nullptr;
      if(op == 0x00) {
	jrTarget = f.getGPR((insn >> 21) & 31);      /* read before the slot runs */
      }
      if(op == 0x03 or (op == 0x00 and (insn & 0x3f) == 0x09)) {
	/* jal, and jalr -- which links $31 whatever its rd field says */
	f.setGPR(31, f.c64(static_cast<int64_t>(static_cast<int32_t>(link))));
      }

      f.pred = likely ? cond : nullptr;
      const bool slotOk = (slot == 0) ? true : emitOne(f, slot);
      f.pred = nullptr;
      if(not slotOk) {
	f.bailTo(pc);
	closed = true;
	break;
      }

      const uint32_t jtgt = (pc & 0xf0000000u) | ((insn & 0x3ffffff) << 2);
      if(op == 0x02) {                               /* j */
	if(gf.blocks.count(jtgt)) {
	  builder.CreateBr(f.blockFor(jtgt));
	}
	else {                                       /* tail call out of this function */
	  auto it = f.decls->find(jtgt);
	  emitCall(f, f.c32(jtgt), link, (it == f.decls->end()) ? nullptr : it->second);
	  builder.CreateStore(f.c32(0), f.bailSlot);
	  builder.CreateBr(f.exitBB);
	}
      }
      else if(op == 0x03) {                          /* jal */
	auto it = f.decls->find(jtgt);
	emitCall(f, f.c32(jtgt), link, (it == f.decls->end()) ? nullptr : it->second);
	builder.CreateBr(f.blockFor(link));
      }
      else if(op == 0x00 and (insn & 0x3f) == 0x09) {   /* jalr */
	auto h = cfg.hints.find(pc);
	if(h != cfg.hints.end()) {
	  emitIndirectCall(f, f.lo32(jrTarget), link, h->second);
	}
	else {
	  emitCall(f, f.lo32(jrTarget), link, nullptr);
	}
	builder.CreateBr(f.blockFor(link));
      }
      else if(op == 0x00) {                          /* jr */
	if(((insn >> 21) & 31) == 31) {
	  builder.CreateStore(f.c32(0), f.bailSlot);  /* returned normally */
	  builder.CreateBr(f.exitBB);
	}
	else if(not blk.succs.empty()) {
	  /* a jump table the CFG read out of ROM: every case is a block of this function */
	  /* a jump table repeats targets wherever the guest's switch had several cases
	   * sharing a label, and an LLVM switch may not list a value twice */
	  std::set<uint32_t> uniq(blk.succs.begin(), blk.succs.end());
	  llvm::BasicBlock *dflt = f.newBlock("badtarget");
	  llvm::Value *t32 = f.lo32(jrTarget);
	  llvm::SwitchInst *sw = builder.CreateSwitch(t32, dflt, uniq.size());
	  for(uint32_t t : uniq) {
	    sw->addCase(llvm::cast<llvm::ConstantInt>(f.c32(t)), f.blockFor(t));
	  }
	  builder.SetInsertPoint(dflt);
	  spillGPRs(f);
	  builder.CreateStore(t32, f.bailSlot);
	  builder.CreateBr(f.exitBB);
	}
	else {
	  spillGPRs(f);
	  builder.CreateStore(f.lo32(jrTarget), f.bailSlot);
	  builder.CreateBr(f.exitBB);
	}
      }
      else {                                         /* conditional branch */
	const uint32_t tgt = pc + 4 + (static_cast<int32_t>(static_cast<int16_t>(insn & 0xffff)) << 2);
	builder.CreateCondBr(cond, f.blockFor(tgt), f.blockFor(link));
      }
      closed = true;
      break;
    }
    if(not closed) {
      builder.CreateBr(f.blockFor(blk.end));         /* ran into the next block */
    }
  }

  /* A block a branch named but the CFG did not give us: leave through the interpreter
   * rather than fall off the end of a function. */
  for(auto &kv : f.blocks) {
    if(kv.second->empty()) {
      builder.SetInsertPoint(kv.second);
      spillGPRs(f);
      builder.CreateStore(f.c32(kv.first), f.bailSlot);
      builder.CreateBr(f.exitBB);
    }
  }

  builder.SetInsertPoint(f.exitBB);
  spillGPRs(f);
  builder.CreateRet(builder.CreateLoad(f.i32, f.bailSlot));
}

}; // namespace

/* ---------------------------------------------------------------------------------- */

struct r4300bt::impl {
  std::unique_ptr<llvm::orc::LLJIT> jit;
  std::unique_ptr<llvm::TargetMachine> tm;
  std::map<uint32_t, r4300_fn_t> fns;

  impl();
  void compileAll(r4300bt &owner, const r4300_cfg &cfg, const std::vector<uint32_t> &entries);
};

static void btDie(const char *what, llvm::Error err) {
  fprintf(stderr, "r4300bt: %s: %s\n", what, llvm::toString(std::move(err)).c_str());
  exit(-1);
}

r4300bt::impl::impl() {
  llvm::InitializeNativeTarget();
  llvm::InitializeNativeTargetAsmPrinter();
  auto jtmb = llvm::orc::JITTargetMachineBuilder::detectHost();
  if(not(jtmb)) {
    btDie("detectHost", jtmb.takeError());
  }
  jtmb->setCodeGenOptLevel(llvm::CodeGenOptLevel::Aggressive);
  auto t = jtmb->createTargetMachine();
  if(not(t)) {
    btDie("createTargetMachine", t.takeError());
  }
  tm = std::move(*t);
  auto j = llvm::orc::LLJITBuilder().setJITTargetMachineBuilder(*jtmb).create();
  if(not(j)) {
    btDie("LLJIT", j.takeError());
  }
  jit = std::move(*j);

  llvm::orc::SymbolMap syms;
  syms[jit->mangleAndIntern("r4300bt_call")] =
    llvm::orc::ExecutorSymbolDef(llvm::orc::ExecutorAddr(reinterpret_cast<uintptr_t>(&r4300bt_call)),
				 llvm::JITSymbolFlags::Exported);
  if(auto err = jit->getMainJITDylib().define(llvm::orc::absoluteSymbols(syms))) {
    btDie("define helpers", std::move(err));
  }
}

/* Every function goes into ONE module.  That is the whole point: with the call graph
 * visible, the inliner takes the small callees and GVN then deletes the spill/reload
 * pairs that were only there to hand state across a call boundary. */
void r4300bt::impl::compileAll(r4300bt &owner, const r4300_cfg &cfg,
			       const std::vector<uint32_t> &entries) {
  const auto tBuild = std::chrono::steady_clock::now();
  auto ctx = std::make_unique<llvm::LLVMContext>();
  auto mod = std::make_unique<llvm::Module>("r4300bt", *ctx);
  mod->setDataLayout(tm->createDataLayout());
  mod->setTargetTriple(tm->getTargetTriple().str());
  llvm::IRBuilder<> builder(*ctx);

  llvm::Type *pty = llvm::PointerType::get(*ctx, 0);
  llvm::Type *i32 = llvm::Type::getInt32Ty(*ctx);
  llvm::FunctionType *fty = llvm::FunctionType::get(i32, {pty, pty}, false);

  std::map<uint32_t, llvm::Function*> decls;
  std::vector<uint32_t> todo;
  for(uint32_t e : entries) {
    if(cfg.funcs.count(e) == 0 or decls.count(e) != 0) {
      continue;
    }
    char nm[32];
    snprintf(nm, sizeof(nm), "r4300_%08x", e);
    llvm::Function *fn = llvm::Function::Create(fty, llvm::Function::ExternalLinkage, nm, mod.get());
    /* state_t and the guest RAM image are different objects, and nothing in guest memory
     * reaches state_t: that is what lets the GPRs stay in registers across guest stores. */
    fn->addParamAttr(1, llvm::Attribute::NoAlias);
    decls[e] = fn;
    todo.push_back(e);
  }

  size_t blocks = 0, insns = 0;
  for(uint32_t e : todo) {
    fnCtx f(*ctx, mod.get(), &builder);
    f.fn = decls[e];
    f.vState = f.fn->getArg(0);
    f.vRam = f.fn->getArg(1);
    f.decls = &decls;
    const r4300_cfg::func &gf = cfg.funcs.at(e);
    emitBody(f, cfg, gf, e);
    blocks += gf.blocks.size();
    insns += gf.n_insns;
  }

  const double build_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - tBuild).count();
  std::string verr;
  llvm::raw_string_ostream vos(verr);
  if(llvm::verifyModule(*mod, &vos)) {
    fprintf(stderr, "r4300bt: generated IR does not verify:\n%s\n", vos.str().c_str());
    exit(-1);
  }
  auto dump = [&](const char *tag) {
    if(owner.dump_ir) {
      std::error_code ec;
      llvm::raw_fd_ostream os(std::string("r4300bt") + tag + ".ll", ec);
      mod->print(os, nullptr);
    }
  };
  dump("_in");

  llvm::LoopAnalysisManager lam;
  llvm::FunctionAnalysisManager fam;
  llvm::CGSCCAnalysisManager cgam;
  llvm::ModuleAnalysisManager mam;
  llvm::PassBuilder pb(tm.get());
  pb.registerModuleAnalyses(mam);
  pb.registerCGSCCAnalyses(cgam);
  pb.registerFunctionAnalyses(fam);
  pb.registerLoopAnalyses(lam);
  pb.crossRegisterProxies(lam, fam, cgam, mam);
  const auto tOpt = std::chrono::steady_clock::now();
  pb.buildPerModuleDefaultPipeline(llvm::OptimizationLevel::O2).run(*mod, mam);
  const double opt_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - tOpt).count();
  dump("_opt");

  if(auto err = jit->addIRModule(llvm::orc::ThreadSafeModule(std::move(mod), std::move(ctx)))) {
    btDie("addIRModule", std::move(err));
  }
  for(uint32_t e : todo) {
    char nm[32];
    snprintf(nm, sizeof(nm), "r4300_%08x", e);
    auto sym = jit->lookup(nm);
    if(not(sym)) {
      btDie("lookup", sym.takeError());
    }
    fns[e] = sym->toPtr<r4300_fn_t>();
  }
  fprintf(stderr, "r4300bt: %zu functions, %zu blocks, %zu instructions translated"
	  " (IR build %.2f s, LLVM O2 %.2f s)\n", todo.size(), blocks, insns, build_s, opt_s);
}

r4300bt::r4300bt() : p(new impl()) {}
r4300bt::~r4300bt() { delete p; }

void r4300bt::translate(const r4300_cfg &cfg, const std::vector<uint32_t> &entries) {
  auto t0 = std::chrono::steady_clock::now();
  p->compileAll(*this, cfg, entries);
  compile_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

bool r4300bt::have(uint32_t entry) const {
  return p->fns.count(entry) != 0;
}

uint32_t r4300bt::run(state_t *s, uint8_t *ram, uint32_t entry) {
  uint32_t r = p->fns[entry](s, ram);
  if(r != 0) {
    n_bails++;
  }
  return r;
}

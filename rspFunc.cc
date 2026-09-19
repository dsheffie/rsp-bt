#include <cstddef>

#include "llvm/IR/Intrinsics.h"

#include "rspFunc.hh"
#include "rsp.hh"

#pragma GCC diagnostic ignored "-Winvalid-offsetof"

rspFunc::rspFunc(llvm::LLVMContext &ctx, llvm::Module *mod, llvm::IRBuilder<> *b) : ctx(ctx), mod(mod), b(b) {
  contexts.push_back(std::vector<uint32_t>());
  contextIds[contexts[0]] = 0;
  i1 = llvm::Type::getInt1Ty(ctx);
  i8 = llvm::Type::getInt8Ty(ctx);
  i16 = llvm::Type::getInt16Ty(ctx);
  i32 = llvm::Type::getInt32Ty(ctx);
  i64 = llvm::Type::getInt64Ty(ctx);
  ptr = llvm::PointerType::getUnqual(ctx);
  v16 = llvm::FixedVectorType::get(i16, 8);
  v32 = llvm::FixedVectorType::get(i32, 8);
  v64 = llvm::FixedVectorType::get(i64, 8);
  v1 = llvm::FixedVectorType::get(i1, 8);
  vb8 = llvm::FixedVectorType::get(i8, 8);
}

/* ---- guest state ---- */

llvm::Value *rspFunc::getGPR(uint32_t r) {
  if(r == 0) {
    return c32(0);
  }
  return b->CreateLoad(i32, gpr[r]);
}

void rspFunc::setGPR(uint32_t r, llvm::Value *v) {
  if(r != 0) {
    b->CreateStore(v, gpr[r]);
  }
}

llvm::Value *rspFunc::getVec(uint32_t r) { return b->CreateLoad(v16, vec[r]); }
void rspFunc::setVec(uint32_t r, llvm::Value *v) { b->CreateStore(v, vec[r]); }
llvm::Value *rspFunc::getAcc() { return b->CreateLoad(v64, acc); }
void rspFunc::setAcc(llvm::Value *v) { b->CreateStore(v, acc); }
llvm::Value *rspFunc::getFlag(flag_t f) { return b->CreateLoad(v1, flag[f]); }
void rspFunc::setFlag(flag_t f, llvm::Value *v) { b->CreateStore(v, flag[f]); }
void rspFunc::clearFlag(flag_t f) { b->CreateStore(llvm::Constant::getNullValue(v1), flag[f]); }

/* ---- IR idioms ---- */

llvm::Value *rspFunc::splat64(int64_t x) {
  return llvm::ConstantVector::getSplat(llvm::ElementCount::getFixed(8), llvm::ConstantInt::get(i64, static_cast<uint64_t>(x), true));
}

llvm::Value *rspFunc::splat16(int16_t x) {
  return llvm::ConstantVector::getSplat(llvm::ElementCount::getFixed(8), llvm::ConstantInt::get(i16, static_cast<uint64_t>(x), true));
}

llvm::Value *rspFunc::vtSelect(uint32_t vt, uint32_t e) {
  llvm::Value *t = getVec(vt);
  if(e < 2) {
    return t;
  }
  std::vector<int> mask(8);
  for(uint32_t i = 0; i < 8; i++) {
    mask[i] = static_cast<int>((e < 4) ? ((i & ~1u) | (e & 1u)) : ((e < 8) ? ((i & ~3u) | (e & 3u)) : (e & 7u)));
  }
  return b->CreateShuffleVector(t, mask);
}

llvm::Value *rspFunc::sext48(llvm::Value *a) {
  return b->CreateAShr(b->CreateShl(a, splat64(16)), splat64(16));
}

llvm::Value *rspFunc::satMid(llvm::Value *a) {
  llvm::Value *x = b->CreateAShr(a, splat64(16));
  x = b->CreateBinaryIntrinsic(llvm::Intrinsic::smax, x, splat64(-32768));
  x = b->CreateBinaryIntrinsic(llvm::Intrinsic::smin, x, splat64(32767));
  return b->CreateTrunc(x, v16);
}

llvm::Value *rspFunc::clampLow(llvm::Value *a) {
  llvm::Value *lt = b->CreateICmpSLT(a, splat64(-0x80000000LL));
  llvm::Value *gt = b->CreateICmpSGT(a, splat64(0x7fffffffLL));
  llvm::Value *lo = b->CreateTrunc(a, v16);
  lo = b->CreateSelect(gt, splat16(-1), lo);
  return b->CreateSelect(lt, splat16(0), lo);
}

void rspFunc::setAccLow(llvm::Value *res) {
  llvm::Value *hi = b->CreateAnd(getAcc(), splat64(~0xffffLL));
  setAcc(b->CreateOr(hi, b->CreateZExt(res, v64)));
}

/* ---- memory ---- */

llvm::Value *rspFunc::statePtr(size_t offset) {
  return b->CreateGEP(i8, vState, llvm::ConstantInt::get(i64, offset));
}

llvm::Value *rspFunc::dmemPtr(llvm::Value *ea12) {
  llvm::Value *off = b->CreateAdd(b->CreateZExt(ea12, i64), llvm::ConstantInt::get(i64, offsetof(rsp_t, mem)));
  return b->CreateGEP(i8, vState, off);
}

llvm::Value *rspFunc::fitsInDmem(llvm::Value *ea, unsigned nbytes) {
  return b->CreateICmpULE(b->CreateAnd(ea, c32(0xfff)), c32(0x1000 - nbytes));
}

llvm::BasicBlock *rspFunc::newBlock(const char *name) {
  return llvm::BasicBlock::Create(ctx, std::string(name) + "_" + std::to_string(uuid++), fn);
}

/* DMEM accesses wrap byte-wise at 4 KB and may be unaligned.  The common case is one
 * unaligned load + bswap; an access straddling the end goes through the interpreter. */
llvm::Value *rspFunc::loadMem(llvm::Value *ea, unsigned nbytes) {
  llvm::Value *ea12 = b->CreateAnd(ea, c32(0xfff));
  if(nbytes == 1) {
    return b->CreateZExt(b->CreateLoad(i8, dmemPtr(ea12)), i32);
  }
  llvm::Type *ty = (nbytes == 2) ? i16 : i32;
  llvm::BasicBlock *fastBB = newBlock("ld_fast"), *slowBB = newBlock("ld_slow"), *joinBB = newBlock("ld_join");
  b->CreateCondBr(fitsInDmem(ea, nbytes), fastBB, slowBB);
  b->SetInsertPoint(fastBB);
  llvm::Value *raw = b->CreateAlignedLoad(ty, dmemPtr(ea12), llvm::MaybeAlign(1));
  llvm::Value *fast = b->CreateZExt(b->CreateUnaryIntrinsic(llvm::Intrinsic::bswap, raw), i32);
  b->CreateBr(joinBB);
  b->SetInsertPoint(slowBB);
  llvm::Value *slow = b->CreateCall(helper("rspbt_rd", i32, {ptr, i32, i32}), {vState, ea, c32(nbytes)});
  b->CreateBr(joinBB);
  b->SetInsertPoint(joinBB);
  llvm::PHINode *phi = b->CreatePHI(i32, 2);
  phi->addIncoming(fast, fastBB);
  phi->addIncoming(slow, slowBB);
  return phi;
}

void rspFunc::storeMem(llvm::Value *ea, llvm::Value *val, unsigned nbytes) {
  llvm::Value *ea12 = b->CreateAnd(ea, c32(0xfff));
  if(nbytes == 1) {
    b->CreateStore(b->CreateTrunc(val, i8), dmemPtr(ea12));
    return;
  }
  llvm::Type *ty = (nbytes == 2) ? i16 : i32;
  llvm::BasicBlock *fastBB = newBlock("st_fast"), *slowBB = newBlock("st_slow"), *joinBB = newBlock("st_join");
  b->CreateCondBr(fitsInDmem(ea, nbytes), fastBB, slowBB);
  b->SetInsertPoint(fastBB);
  llvm::Value *x = b->CreateUnaryIntrinsic(llvm::Intrinsic::bswap, b->CreateTrunc(val, ty));
  b->CreateAlignedStore(x, dmemPtr(ea12), llvm::MaybeAlign(1));
  b->CreateBr(joinBB);
  b->SetInsertPoint(slowBB);
  b->CreateCall(helper("rspbt_wr", llvm::Type::getVoidTy(ctx), {ptr, i32, i32, i32}), {vState, ea, val, c32(nbytes)});
  b->CreateBr(joinBB);
  b->SetInsertPoint(joinBB);
}

static std::vector<int> swapPairs(unsigned nbytes) {
  std::vector<int> m(nbytes);
  for(unsigned i = 0; i < nbytes; i++) {
    m[i] = static_cast<int>(i ^ 1u);
  }
  return m;
}

/* A register is 16 big-endian bytes in DMEM; on a little-endian host that is a bitcast
 * and a swap of each byte pair.  All the load/store forms are expressed on bytes and
 * left to LLVM to turn back into lane operations where they line up. */
llvm::Value *rspFunc::toBytes(llvm::Value *v) {
  return b->CreateShuffleVector(b->CreateBitCast(v, llvm::FixedVectorType::get(i8, 16)), swapPairs(16));
}

llvm::Value *rspFunc::fromBytes(llvm::Value *bytes) {
  return b->CreateBitCast(b->CreateShuffleVector(bytes, swapPairs(16)), v16);
}

static llvm::Value *byteIndexVector(rspFunc &f) {
  std::vector<llvm::Constant*> idx;
  for(int i = 0; i < 16; i++) {
    idx.push_back(llvm::ConstantInt::get(f.i8, i));
  }
  return llvm::ConstantVector::get(idx);
}

static llvm::Value *splat8(rspFunc &f, llvm::Value *x8) {
  return f.b->CreateVectorSplat(16, x8);
}

void rspFunc::vecLoad(uint32_t insn, uint32_t base, uint32_t vt, llvm::Value *ea, uint32_t e, unsigned nbytes, vecMemKind kind) {
  llvm::BasicBlock *fastBB = newBlock("vl_fast"), *slowBB = newBlock("vl_slow"), *joinBB = newBlock("vl_join");
  llvm::FixedVectorType *b16 = llvm::FixedVectorType::get(i8, 16);
  llvm::Value *ea12 = b->CreateAnd(ea, c32(0xfff));
  llvm::Value *k = b->CreateAnd(ea, c32(15));                     /* offset inside the row */
  llvm::Value *fastCond = nullptr;
  if(kind == FIXED and (e + nbytes) <= 16) {
    fastCond = fitsInDmem(ea, nbytes);
  }
  else if(kind == ROW_LEFT and e == 0) {
    fastCond = b->CreateICmpULE(ea12, c32(0xff0));                /* a 16-byte window at ea */
  }
  else if(kind == ROW_RIGHT and e == 0) {
    fastCond = b->CreateICmpUGE(ea12, c32(16));                   /* a 16-byte window below ea */
  }
  if(fastCond == nullptr) {
    b->CreateBr(slowBB);
  }
  else {
    b->CreateCondBr(fastCond, fastBB, slowBB);
  }

  b->SetInsertPoint(fastBB);
  if(fastCond != nullptr) {
    llvm::Value *old = toBytes(getVec(vt)), *res = nullptr;
    if(kind == FIXED) {
      llvm::Value *in = b->CreateAlignedLoad(llvm::FixedVectorType::get(i8, nbytes), dmemPtr(ea12), llvm::MaybeAlign(1));
      std::vector<int> widen(16, -1), blend(16);
      for(unsigned i = 0; i < nbytes; i++) {
	widen[i] = static_cast<int>(i);
      }
      for(unsigned i = 0; i < 16; i++) {
	blend[i] = static_cast<int>((i >= e and i < e + nbytes) ? (16 + i - e) : i);
      }
      res = (nbytes == 16) ? in : b->CreateShuffleVector(old, b->CreateShuffleVector(in, widen), blend);
    }
    else {
      /* n = 16 - k bytes are valid on the left (lqv); the other k on the right (lrv).
       * Either way register byte i comes from window byte i: only the mask differs. */
      llvm::Value *n8 = b->CreateTrunc(b->CreateSub(c32(16), k), i8);
      llvm::Value *win = (kind == ROW_LEFT) ? ea12 : b->CreateSub(ea12, c32(16));
      llvm::Value *in = b->CreateAlignedLoad(b16, dmemPtr(win), llvm::MaybeAlign(1));
      llvm::Value *take = (kind == ROW_LEFT) ? b->CreateICmpULT(byteIndexVector(*this), splat8(*this, n8))
					     : b->CreateICmpUGE(byteIndexVector(*this), splat8(*this, n8));
      res = b->CreateSelect(take, in, old);
    }
    setVec(vt, fromBytes(res));
  }
  b->CreateBr(joinBB);

  b->SetInsertPoint(slowBB);
  if(base != 0) {
    b->CreateStore(getGPR(base), statePtr(offsetof(rsp_t, r) + 4*base));
  }
  b->CreateAlignedStore(getVec(vt), statePtr(offsetof(rsp_t, v) + 16*vt), llvm::MaybeAlign(2));   /* rsp_t::v is only 2-aligned */
  b->CreateCall(helper("rspbt_lwc2", llvm::Type::getVoidTy(ctx), {ptr, i32}), {vState, c32(insn)});
  setVec(vt, b->CreateAlignedLoad(v16, statePtr(offsetof(rsp_t, v) + 16*vt), llvm::MaybeAlign(2)));
  b->CreateBr(joinBB);
  b->SetInsertPoint(joinBB);
}

void rspFunc::vecStore(uint32_t insn, uint32_t base, uint32_t vt, llvm::Value *ea, uint32_t e, unsigned nbytes, vecMemKind kind) {
  llvm::BasicBlock *fastBB = newBlock("vs_fast"), *slowBB = newBlock("vs_slow"), *joinBB = newBlock("vs_join");
  llvm::FixedVectorType *b16 = llvm::FixedVectorType::get(i8, 16);
  llvm::Value *ea12 = b->CreateAnd(ea, c32(0xfff));
  llvm::Value *fastCond = nullptr;
  if(kind == FIXED and (e + nbytes) <= 16) {
    fastCond = fitsInDmem(ea, nbytes);
  }
  else if(kind == ROW_LEFT and e == 0) {
    fastCond = b->CreateICmpULE(ea12, c32(0xff0));
  }
  if(fastCond == nullptr) {
    b->CreateBr(slowBB);
  }
  else {
    b->CreateCondBr(fastCond, fastBB, slowBB);
  }

  b->SetInsertPoint(fastBB);
  if(fastCond != nullptr) {
    llvm::Value *bytes = toBytes(getVec(vt));
    if(kind == FIXED) {
      std::vector<int> pick(nbytes);
      for(unsigned i = 0; i < nbytes; i++) {
	pick[i] = static_cast<int>(e + i);
      }
      llvm::Value *out = (nbytes == 16) ? bytes : b->CreateShuffleVector(bytes, pick);
      b->CreateAlignedStore(out, dmemPtr(ea12), llvm::MaybeAlign(1));
    }
    else {
      /* sqv: only the 16-k bytes up to the row end are written: read-modify-write */
      llvm::Value *n8 = b->CreateTrunc(b->CreateSub(c32(16), b->CreateAnd(ea, c32(15))), i8);
      llvm::Value *cur = b->CreateAlignedLoad(b16, dmemPtr(ea12), llvm::MaybeAlign(1));
      llvm::Value *take = b->CreateICmpULT(byteIndexVector(*this), splat8(*this, n8));
      b->CreateAlignedStore(b->CreateSelect(take, bytes, cur), dmemPtr(ea12), llvm::MaybeAlign(1));
    }
  }
  b->CreateBr(joinBB);

  b->SetInsertPoint(slowBB);
  if(base != 0) {
    b->CreateStore(getGPR(base), statePtr(offsetof(rsp_t, r) + 4*base));
  }
  b->CreateAlignedStore(getVec(vt), statePtr(offsetof(rsp_t, v) + 16*vt), llvm::MaybeAlign(2));
  b->CreateCall(helper("rspbt_swc2", llvm::Type::getVoidTy(ctx), {ptr, i32}), {vState, c32(insn)});
  b->CreateBr(joinBB);
  b->SetInsertPoint(joinBB);
}

/* ---- control ---- */

uint32_t rspFunc::pushCtx(uint32_t ctxId, uint32_t link) {
  std::vector<uint32_t> cs = contexts[ctxId];
  if(cs.size() >= MAX_CALL_DEPTH) {
    return 0;                       /* too deep to specialize: the return will be a dispatch */
  }
  cs.push_back(link);
  auto it = contextIds.find(cs);
  if(it != contextIds.end()) {
    return it->second;
  }
  uint32_t id = static_cast<uint32_t>(contexts.size());
  contexts.push_back(cs);
  contextIds[cs] = id;
  return id;
}

uint32_t rspFunc::popCtx(uint32_t ctxId) const {
  std::vector<uint32_t> cs = contexts[ctxId];
  cs.pop_back();
  return contextIds.at(cs);
}

llvm::BasicBlock *rspFunc::blockFor(uint32_t addr, uint32_t ctxId) {
  auto it = blocks.find(blockKey(addr, ctxId));
  if(it != blocks.end()) {
    return it->second;
  }
  char name[48];
  snprintf(name, sizeof(name), "pc_%03x_c%u", addr, ctxId);
  llvm::BasicBlock *bb = llvm::BasicBlock::Create(ctx, name, fn);
  blocks[blockKey(addr, ctxId)] = bb;
  return bb;
}

void rspFunc::exitToInterp(uint32_t pc) {
  b->CreateStore(c32(pc), retSlot);
  b->CreateBr(exitBB);
}

void rspFunc::exitHalted() {
  b->CreateStore(llvm::ConstantInt::get(i8, 1), statePtr(offsetof(rsp_t, halted)));
  b->CreateStore(c32(0xffffffffu), retSlot);
  b->CreateBr(exitBB);
}

/* A devirtualized jr.
 *  - Inside a call context the only anticipated target is the return address on top of
 *    the call string, continuing in the caller's context.  The jal stored that address
 *    as a constant and this clone has no other entry, so LLVM folds the switch into a
 *    direct branch -- including the "addi r5,r31,0 ... jr r5" nested-call idiom.
 *  - Outside any call context it is a real dispatch: a switch over the known roots
 *    (entry, table hints, targets observed at run time).
 * The default always leaves for the interpreter, so a jr that does something else is
 * slow, never wrong. */
void rspFunc::indirectJump(llvm::Value *target) {
  llvm::BasicBlock *missBB = newBlock("jr_miss");
  if(curCtx != 0) {
    llvm::SwitchInst *sw = b->CreateSwitch(target, missBB, 1);
    uint32_t link = contexts[curCtx].back();
    sw->addCase(llvm::cast<llvm::ConstantInt>(c32(link)), blockFor(link, popCtx(curCtx)));
  }
  else {
    llvm::SwitchInst *sw = b->CreateSwitch(target, missBB, static_cast<unsigned>(roots.size()));
    for(uint32_t r : roots) {
      sw->addCase(llvm::cast<llvm::ConstantInt>(c32(r)), blockFor(r, 0));
    }
  }
  b->SetInsertPoint(missBB);
  b->CreateStore(target, retSlot);
  b->CreateBr(exitBB);
}

llvm::FunctionCallee rspFunc::helper(const char *name, llvm::Type *ret, std::vector<llvm::Type*> args) {
  return mod->getOrInsertFunction(name, llvm::FunctionType::get(ret, args, false));
}

#include "llvm/IR/Intrinsics.h"

#include "rspInstruction.hh"
#include "rspFunc.hh"

rspInsn::rspInsn(uint32_t inst, uint32_t addr) : inst(inst), addr(addr) {
  rs = (inst >> 21) & 31;
  rt = (inst >> 16) & 31;
  rd = (inst >> 11) & 31;
  sa = (inst >> 6) & 31;
  uimm = inst & 0xffff;
  simm = static_cast<int16_t>(inst & 0xffff);
}

llvm::Value *rspInsn::effAddr(rspFunc &f) const {
  return f.b->CreateAdd(f.getGPR(rs), f.c32(static_cast<uint32_t>(simm)));
}

/* ------------------------------------------------------------ integer */

class rTypeInsn : public rspInsn {
public:
  rTypeInsn(uint32_t inst, uint32_t addr) : rspInsn(inst, addr) {}
  virtual llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) = 0;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.setGPR(rd, doOp(f, f.getGPR(rs), f.getGPR(rt)));
  }
};

/* the RSP has no overflow trap: add/sub behave as addu/subu */
class insn_add : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateAdd(a, b); }
};
class insn_sub : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateSub(a, b); }
};
class insn_and : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateAnd(a, b); }
};
class insn_or : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateOr(a, b); }
};
class insn_xor : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateXor(a, b); }
};
class insn_nor : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateNot(f.b->CreateOr(a, b)); }
};
class insn_slt : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateZExt(f.b->CreateICmpSLT(a, b), f.i32); }
};
class insn_sltu : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateZExt(f.b->CreateICmpULT(a, b), f.i32); }
};
/* variable shifts: GPR[rt] shifted by GPR[rs] & 31 */
class insn_sllv : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateShl(b, f.b->CreateAnd(a, f.c32(31))); }
};
class insn_srlv : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateLShr(b, f.b->CreateAnd(a, f.c32(31))); }
};
class insn_srav : public rTypeInsn {
public:
  using rTypeInsn::rTypeInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateAShr(b, f.b->CreateAnd(a, f.c32(31))); }
};

class insn_sll : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rd, f.b->CreateShl(f.getGPR(rt), f.c32(sa))); }
};
class insn_srl : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rd, f.b->CreateLShr(f.getGPR(rt), f.c32(sa))); }
};
class insn_sra : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rd, f.b->CreateAShr(f.getGPR(rt), f.c32(sa))); }
};

class insn_addi : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, effAddr(f)); }
};
class insn_slti : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.setGPR(rt, f.b->CreateZExt(f.b->CreateICmpSLT(f.getGPR(rs), f.c32(static_cast<uint32_t>(simm))), f.i32));
  }
};
class insn_sltiu : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.setGPR(rt, f.b->CreateZExt(f.b->CreateICmpULT(f.getGPR(rs), f.c32(static_cast<uint32_t>(simm))), f.i32));
  }
};
class insn_andi : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.b->CreateAnd(f.getGPR(rs), f.c32(uimm))); }
};
class insn_ori : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.b->CreateOr(f.getGPR(rs), f.c32(uimm))); }
};
class insn_xori : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.b->CreateXor(f.getGPR(rs), f.c32(uimm))); }
};
class insn_lui : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.c32(uimm << 16)); }
};

/* loads and stores: DMEM only, wrapping, alignment-free */
class insn_lb : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.setGPR(rt, f.b->CreateSExt(f.b->CreateTrunc(f.loadMem(effAddr(f), 1), f.i8), f.i32));
  }
};
class insn_lbu : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.loadMem(effAddr(f), 1)); }
};
class insn_lh : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.setGPR(rt, f.b->CreateSExt(f.b->CreateTrunc(f.loadMem(effAddr(f), 2), f.i16), f.i32));
  }
};
class insn_lhu : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.loadMem(effAddr(f), 2)); }
};
class insn_lw : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.setGPR(rt, f.loadMem(effAddr(f), 4)); }
};
class insn_sb : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.storeMem(effAddr(f), f.getGPR(rt), 1); }
};
class insn_sh : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.storeMem(effAddr(f), f.getGPR(rt), 2); }
};
class insn_sw : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.storeMem(effAddr(f), f.getGPR(rt), 4); }
};

/* COP0 is the SP/DPC register file: DMA and status, handled by rsp.cc */
class insn_mfc0 : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.setGPR(rt, f.b->CreateCall(f.helper("rspbt_mfc0", f.i32, {f.ptr, f.i32}), {f.vState, f.c32(rd)}));
  }
};
class insn_mtc0 : public rspInsn {
public:
  using rspInsn::rspInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    f.b->CreateCall(f.helper("rspbt_mtc0", llvm::Type::getVoidTy(f.ctx), {f.ptr, f.i32, f.i32}), {f.vState, f.c32(rd), f.getGPR(rt)});
  }
};

class insn_break : public rspInsn {
public:
  using rspInsn::rspInsn;
  bool isBreak() const override { return true; }
  void generateIR(rspFunc &f, rspInsn *delay) override { f.exitHalted(); }
};

/* ------------------------------------------------------------ control */

class iBranchTypeInsn : public rspInsn {
protected:
  uint32_t tAddr, ntAddr;
public:
  iBranchTypeInsn(uint32_t inst, uint32_t addr) : rspInsn(inst, addr) {
    tAddr = (addr + 4 + (static_cast<uint32_t>(simm) << 2)) & 0xffc;
    ntAddr = (addr + 8) & 0xffc;
  }
  bool isControl() const override { return true; }
  void staticTargets(std::vector<uint32_t> &t) const override { t.push_back(tAddr); }
  virtual llvm::Value *condition(rspFunc &f) = 0;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    llvm::Value *vCMP = condition(f);           /* decided before the delay slot runs */
    delay->generateIR(f, nullptr);
    f.b->CreateCondBr(vCMP, f.blockFor(tAddr), f.blockFor(ntAddr));
  }
};
class insn_beq : public iBranchTypeInsn {
public:
  using iBranchTypeInsn::iBranchTypeInsn;
  llvm::Value *condition(rspFunc &f) override { return f.b->CreateICmpEQ(f.getGPR(rs), f.getGPR(rt)); }
};
class insn_bne : public iBranchTypeInsn {
public:
  using iBranchTypeInsn::iBranchTypeInsn;
  llvm::Value *condition(rspFunc &f) override { return f.b->CreateICmpNE(f.getGPR(rs), f.getGPR(rt)); }
};
class insn_blez : public iBranchTypeInsn {
public:
  using iBranchTypeInsn::iBranchTypeInsn;
  llvm::Value *condition(rspFunc &f) override { return f.b->CreateICmpSLE(f.getGPR(rs), f.c32(0)); }
};
class insn_bgtz : public iBranchTypeInsn {
public:
  using iBranchTypeInsn::iBranchTypeInsn;
  llvm::Value *condition(rspFunc &f) override { return f.b->CreateICmpSGT(f.getGPR(rs), f.c32(0)); }
};
class insn_bltz : public iBranchTypeInsn {
public:
  using iBranchTypeInsn::iBranchTypeInsn;
  llvm::Value *condition(rspFunc &f) override { return f.b->CreateICmpSLT(f.getGPR(rs), f.c32(0)); }
};
class insn_bgez : public iBranchTypeInsn {
public:
  using iBranchTypeInsn::iBranchTypeInsn;
  llvm::Value *condition(rspFunc &f) override { return f.b->CreateICmpSGE(f.getGPR(rs), f.c32(0)); }
};

class insn_j : public rspInsn {
protected:
  uint32_t tAddr;
public:
  insn_j(uint32_t inst, uint32_t addr) : rspInsn(inst, addr), tAddr((inst << 2) & 0xffc) {}
  bool isControl() const override { return true; }
  bool fallsThrough() const override { return false; }
  void staticTargets(std::vector<uint32_t> &t) const override { t.push_back(tAddr); }
  void generateIR(rspFunc &f, rspInsn *delay) override {
    delay->generateIR(f, nullptr);
    f.b->CreateBr(f.blockFor(tAddr));
  }
};
class insn_jal : public insn_j {
public:
  using insn_j::insn_j;
  bool isCall() const override { return true; }
  void generateIR(rspFunc &f, rspInsn *delay) override {
    uint32_t link = (addr + 8) & 0xffc;
    f.setGPR(31, f.c32(link));
    delay->generateIR(f, nullptr);
    /* the callee is specialized to this call site, so inside it r31 is this constant
     * and its return folds to a direct branch back here */
    f.b->CreateBr(f.blockFor(tAddr, f.pushCtx(f.curCtx, link)));
  }
};
class insn_jr : public rspInsn {
public:
  using rspInsn::rspInsn;
  bool isControl() const override { return true; }
  bool isIndirect() const override { return true; }
  bool fallsThrough() const override { return false; }
  void generateIR(rspFunc &f, rspInsn *delay) override {
    llvm::Value *vTgt = f.b->CreateAnd(f.getGPR(rs), f.c32(0xffc));
    delay->generateIR(f, nullptr);
    f.indirectJump(vTgt);
  }
};
class insn_jalr : public insn_jr {
public:
  using insn_jr::insn_jr;
  bool makesLinkRoot() const override { return true; }
  void generateIR(rspFunc &f, rspInsn *delay) override {
    llvm::Value *vTgt = f.b->CreateAnd(f.getGPR(rs), f.c32(0xffc));
    f.setGPR(rd, f.c32((addr + 8) & 0xffc));
    delay->generateIR(f, nullptr);
    f.indirectJump(vTgt);
  }
};

/* ------------------------------------------------------------ vector unit
 * A register is <8 x i16>, the accumulators <8 x i64> (48 bits, kept sign-extended),
 * each flag set <8 x i1>.  Semantics follow rsp.cc op for op; what LLVM makes of it
 * (vpmuldq, vpmovsqw, k-masks, or NEON) is the back end's business. */

class vecOpInsn : public rspInsn {
protected:
  uint32_t e, vt, vs, vd;
  llvm::Value *wideS(rspFunc &f, llvm::Value *v) { return f.b->CreateSExt(v, f.v64); }
  llvm::Value *wideU(rspFunc &f, llvm::Value *v) { return f.b->CreateZExt(v, f.v64); }
public:
  vecOpInsn(uint32_t inst, uint32_t addr) : rspInsn(inst, addr) {
    e = (inst >> 21) & 15;
    vt = rt;
    vs = rd;
    vd = sa;
  }
  virtual llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) = 0;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    llvm::Value *s = f.getVec(vs), *t = f.vtSelect(vt, e);
    f.setVec(vd, doOp(f, s, t));
  }
};

class insn_vmulf : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *p = f.b->CreateShl(f.b->CreateMul(wideS(f, s), wideS(f, t)), f.splat64(1));
    f.setAcc(f.sext48(f.b->CreateAdd(p, f.splat64(0x8000))));
    return f.satMid(f.getAcc());
  }
};
class insn_vmudl : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    f.setAcc(f.b->CreateLShr(f.b->CreateMul(wideU(f, s), wideU(f, t)), f.splat64(16)));
    return f.clampLow(f.getAcc());
  }
};
class insn_vmudm : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    f.setAcc(f.sext48(f.b->CreateMul(wideS(f, s), wideU(f, t))));
    return f.satMid(f.getAcc());
  }
};
class insn_vmudn : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    f.setAcc(f.sext48(f.b->CreateMul(wideU(f, s), wideS(f, t))));
    return f.clampLow(f.getAcc());
  }
};
class insn_vmudh : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    f.setAcc(f.sext48(f.b->CreateShl(f.b->CreateMul(wideS(f, s), wideS(f, t)), f.splat64(16))));
    return f.satMid(f.getAcc());
  }
};
class insn_vmacf : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *p = f.b->CreateShl(f.b->CreateMul(wideS(f, s), wideS(f, t)), f.splat64(1));
    f.setAcc(f.sext48(f.b->CreateAdd(f.getAcc(), p)));
    return f.satMid(f.getAcc());
  }
};
class insn_vmadm : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    f.setAcc(f.sext48(f.b->CreateAdd(f.getAcc(), f.b->CreateMul(wideS(f, s), wideU(f, t)))));
    return f.satMid(f.getAcc());
  }
};
class insn_vmadn : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    f.setAcc(f.sext48(f.b->CreateAdd(f.getAcc(), f.b->CreateMul(wideU(f, s), wideS(f, t)))));
    return f.clampLow(f.getAcc());
  }
};
class insn_vmadh : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *p = f.b->CreateShl(f.b->CreateMul(wideS(f, s), wideS(f, t)), f.splat64(16));
    f.setAcc(f.sext48(f.b->CreateAdd(f.getAcc(), p)));
    return f.satMid(f.getAcc());
  }
};

/* vadd/vsub: carry in from VCO, saturated result, low accumulator slice written */
class insn_vadd : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  virtual llvm::Value *combine(rspFunc &f, llvm::Value *a, llvm::Value *b) { return f.b->CreateAdd(a, b); }
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *carry = f.b->CreateZExt(f.getFlag(rspFunc::VCO_L), f.v64);
    llvm::Value *x = combine(f, combine(f, wideS(f, s), wideS(f, t)), carry);
    llvm::Value *hi = f.b->CreateAnd(f.getAcc(), f.splat64(~0xffffLL));
    f.setAcc(f.b->CreateOr(hi, f.b->CreateAnd(x, f.splat64(0xffff))));
    f.clearFlag(rspFunc::VCO_L);
    f.clearFlag(rspFunc::VCO_H);
    llvm::Value *c = f.b->CreateBinaryIntrinsic(llvm::Intrinsic::smax, x, f.splat64(-32768));
    c = f.b->CreateBinaryIntrinsic(llvm::Intrinsic::smin, c, f.splat64(32767));
    return f.b->CreateTrunc(c, f.v16);
  }
};
class insn_vsub : public insn_vadd {
public:
  using insn_vadd::insn_vadd;
  llvm::Value *combine(rspFunc &f, llvm::Value *a, llvm::Value *b) override { return f.b->CreateSub(a, b); }
};
class insn_vaddc : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *x = f.b->CreateAdd(wideU(f, s), wideU(f, t));
    llvm::Value *res = f.b->CreateTrunc(x, f.v16);
    f.setAccLow(res);
    f.setFlag(rspFunc::VCO_L, f.b->CreateICmpNE(f.b->CreateAnd(x, f.splat64(0x10000)), f.splat64(0)));
    f.clearFlag(rspFunc::VCO_H);
    return res;
  }
};
class insn_vsar : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override { return nullptr; }
  void generateIR(rspFunc &f, rspInsn *delay) override {
    /* e picks the accumulator slice: 8 high, 9 middle, 10 low */
    llvm::Value *res = f.splat16(0);
    if(e >= 8 and e <= 10) {
      res = f.b->CreateTrunc(f.b->CreateAShr(f.getAcc(), f.splat64(16 * (10 - e))), f.v16);
    }
    f.setVec(vd, res);
  }
};
class insn_vge : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *gt = f.b->CreateICmpSGT(s, t), *eq = f.b->CreateICmpEQ(s, t);
    llvm::Value *both = f.b->CreateAnd(f.getFlag(rspFunc::VCO_H), f.getFlag(rspFunc::VCO_L));
    llvm::Value *ge = f.b->CreateOr(gt, f.b->CreateAnd(eq, f.b->CreateNot(both)));
    llvm::Value *res = f.b->CreateSelect(ge, s, t);
    f.setAccLow(res);
    f.setFlag(rspFunc::VCC_L, ge);
    f.clearFlag(rspFunc::VCC_H);
    f.clearFlag(rspFunc::VCO_L);
    f.clearFlag(rspFunc::VCO_H);
    return res;
  }
};
class insn_vcl : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *vcoL = f.getFlag(rspFunc::VCO_L), *vcoH = f.getFlag(rspFunc::VCO_H);
    llvm::Value *nH = f.b->CreateNot(vcoH);
    /* carry lanes: le = vce ? (su+tu <= 0x10000) : (su+tu == 0), latched unless VCO_H */
    llvm::Value *sum = f.b->CreateAdd(f.b->CreateZExt(s, f.v32), f.b->CreateZExt(t, f.v32));
    llvm::Value *k64k = llvm::ConstantVector::getSplat(llvm::ElementCount::getFixed(8), llvm::ConstantInt::get(f.i32, 0x10000));
    llvm::Value *le = f.b->CreateSelect(f.getFlag(rspFunc::VCE), f.b->CreateICmpULE(sum, k64k),
					f.b->CreateICmpEQ(sum, llvm::Constant::getNullValue(f.v32)));
    llvm::Value *vccL = f.b->CreateSelect(f.b->CreateAnd(vcoL, nH), le, f.getFlag(rspFunc::VCC_L));
    /* plain lanes: (su - tu) >= 0, latched unless VCO_H */
    llvm::Value *geu = f.b->CreateICmpUGE(s, t);
    llvm::Value *vccH = f.b->CreateSelect(f.b->CreateAnd(f.b->CreateNot(vcoL), nH), geu, f.getFlag(rspFunc::VCC_H));
    llvm::Value *rCarry = f.b->CreateSelect(vccL, f.b->CreateNeg(t), s);
    llvm::Value *rPlain = f.b->CreateSelect(vccH, t, s);
    llvm::Value *res = f.b->CreateSelect(vcoL, rCarry, rPlain);
    f.setAccLow(res);
    f.setFlag(rspFunc::VCC_L, vccL);
    f.setFlag(rspFunc::VCC_H, vccH);
    f.clearFlag(rspFunc::VCO_L);
    f.clearFlag(rspFunc::VCO_H);
    f.clearFlag(rspFunc::VCE);
    return res;
  }
};
class insn_vand : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *res = f.b->CreateAnd(s, t);
    f.setAccLow(res);
    return res;
  }
};
class insn_vxor : public vecOpInsn {
public:
  using vecOpInsn::vecOpInsn;
  llvm::Value *doOp(rspFunc &f, llvm::Value *s, llvm::Value *t) override {
    llvm::Value *res = f.b->CreateXor(s, t);
    f.setAccLow(res);
    return res;
  }
};

/* mfc2/mtc2 address a register by byte offset; byte k is the (k&1 ? low : high) half of lane k>>1 */
class cop2MoveInsn : public rspInsn {
protected:
  uint32_t e;
  llvm::Value *getByte(rspFunc &f, llvm::Value *v, uint32_t k) {
    llvm::Value *lane = f.b->CreateZExt(f.b->CreateExtractElement(v, static_cast<uint64_t>((k & 15) >> 1)), f.i32);
    return f.b->CreateAnd((k & 1) ? lane : f.b->CreateLShr(lane, f.c32(8)), f.c32(0xff));
  }
  llvm::Value *setByte(rspFunc &f, llvm::Value *v, uint32_t k, llvm::Value *b8) {
    uint64_t idx = (k & 15) >> 1;
    llvm::Value *lane = f.b->CreateZExt(f.b->CreateExtractElement(v, idx), f.i32);
    llvm::Value *byte = f.b->CreateAnd(b8, f.c32(0xff));
    llvm::Value *n = (k & 1) ? f.b->CreateOr(f.b->CreateAnd(lane, f.c32(0xff00)), byte)
			     : f.b->CreateOr(f.b->CreateAnd(lane, f.c32(0x00ff)), f.b->CreateShl(byte, f.c32(8)));
    return f.b->CreateInsertElement(v, f.b->CreateTrunc(n, f.i16), idx);
  }
public:
  cop2MoveInsn(uint32_t inst, uint32_t addr) : rspInsn(inst, addr), e((inst >> 7) & 15) {}
};
class insn_mfc2 : public cop2MoveInsn {
public:
  using cop2MoveInsn::cop2MoveInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    llvm::Value *v = f.getVec(rd);
    llvm::Value *x = f.b->CreateOr(f.b->CreateShl(getByte(f, v, e), f.c32(8)), getByte(f, v, e + 1));
    f.setGPR(rt, f.b->CreateSExt(f.b->CreateTrunc(x, f.i16), f.i32));
  }
};
class insn_mtc2 : public cop2MoveInsn {
public:
  using cop2MoveInsn::cop2MoveInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override {
    llvm::Value *x = f.getGPR(rt);
    llvm::Value *v = setByte(f, f.getVec(rd), e, f.b->CreateLShr(x, f.c32(8)));
    if(e != 15) {
      v = setByte(f, v, e + 1, x);
    }
    f.setVec(rd, v);
  }
};

/* vector loads/stores: 7-bit signed offset scaled by the access size; the element
 * field is a BYTE offset into the register.  rspFunc inlines whatever fits inside the
 * register and inside DMEM, and defers the rest to rsp.cc's lwc2/swc2. */
class vecMemInsn : public rspInsn {
protected:
  uint32_t e, vt, base;
  int32_t offs;
  llvm::Value *ea(rspFunc &f, unsigned shift) {
    return f.b->CreateAdd(f.getGPR(base), f.c32(static_cast<uint32_t>(offs << shift)));
  }
public:
  vecMemInsn(uint32_t inst, uint32_t addr) : rspInsn(inst, addr) {
    e = (inst >> 7) & 15;
    vt = rt;
    base = rs;
    offs = static_cast<int32_t>(inst << 25) >> 25;
  }
};
class insn_lsv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecLoad(inst, base, vt, ea(f, 1), e, 2, rspFunc::FIXED); }
};
class insn_llv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecLoad(inst, base, vt, ea(f, 2), e, 4, rspFunc::FIXED); }
};
class insn_ldv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecLoad(inst, base, vt, ea(f, 3), e, 8, rspFunc::FIXED); }
};
class insn_lqv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecLoad(inst, base, vt, ea(f, 4), e, 16, rspFunc::ROW_LEFT); }
};
class insn_lrv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecLoad(inst, base, vt, ea(f, 4), e, 16, rspFunc::ROW_RIGHT); }
};
class insn_ssv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecStore(inst, base, vt, ea(f, 1), e, 2, rspFunc::FIXED); }
};
class insn_slv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecStore(inst, base, vt, ea(f, 2), e, 4, rspFunc::FIXED); }
};
class insn_sdv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecStore(inst, base, vt, ea(f, 3), e, 8, rspFunc::FIXED); }
};
class insn_sqv : public vecMemInsn {
public:
  using vecMemInsn::vecMemInsn;
  void generateIR(rspFunc &f, rspInsn *delay) override { f.vecStore(inst, base, vt, ea(f, 4), e, 16, rspFunc::ROW_LEFT); }
};

/* ------------------------------------------------------------ decode */

rspInsn *rspInsn::decode(uint32_t inst, uint32_t addr) {
  uint32_t op = inst >> 26, fn = inst & 63, rs = (inst >> 21) & 31, rt = (inst >> 16) & 31, rd = (inst >> 11) & 31;
  switch(op)
    {
    case 0x00:
      switch(fn)
	{
	case 0x00: return new insn_sll(inst, addr);
	case 0x02: return new insn_srl(inst, addr);
	case 0x03: return new insn_sra(inst, addr);
	case 0x04: return new insn_sllv(inst, addr);
	case 0x06: return new insn_srlv(inst, addr);
	case 0x07: return new insn_srav(inst, addr);
	case 0x08: return new insn_jr(inst, addr);
	case 0x09: return new insn_jalr(inst, addr);
	case 0x0d: return new insn_break(inst, addr);
	case 0x20: case 0x21: return new insn_add(inst, addr);
	case 0x22: case 0x23: return new insn_sub(inst, addr);
	case 0x24: return new insn_and(inst, addr);
	case 0x25: return new insn_or(inst, addr);
	case 0x26: return new insn_xor(inst, addr);
	case 0x27: return new insn_nor(inst, addr);
	case 0x2a: return new insn_slt(inst, addr);
	case 0x2b: return new insn_sltu(inst, addr);
	default: return nullptr;
	}
    case 0x01:
      if(rt == 0) { return new insn_bltz(inst, addr); }
      if(rt == 1) { return new insn_bgez(inst, addr); }
      return nullptr;
    case 0x02: return new insn_j(inst, addr);
    case 0x03: return new insn_jal(inst, addr);
    case 0x04: return new insn_beq(inst, addr);
    case 0x05: return new insn_bne(inst, addr);
    case 0x06: return new insn_blez(inst, addr);
    case 0x07: return new insn_bgtz(inst, addr);
    case 0x08: case 0x09: return new insn_addi(inst, addr);
    case 0x0a: return new insn_slti(inst, addr);
    case 0x0b: return new insn_sltiu(inst, addr);
    case 0x0c: return new insn_andi(inst, addr);
    case 0x0d: return new insn_ori(inst, addr);
    case 0x0e: return new insn_xori(inst, addr);
    case 0x0f: return new insn_lui(inst, addr);
    case 0x10:
      if(rs == 0) { return new insn_mfc0(inst, addr); }
      if(rs == 4) { return new insn_mtc0(inst, addr); }
      return nullptr;
    case 0x12:
      if(((inst >> 25) & 1) == 0) {
	if(rs == 0) { return new insn_mfc2(inst, addr); }
	if(rs == 4) { return new insn_mtc2(inst, addr); }
	return nullptr;
      }
      switch(fn)
	{
	case 0x00: return new insn_vmulf(inst, addr);
	case 0x04: return new insn_vmudl(inst, addr);
	case 0x05: return new insn_vmudm(inst, addr);
	case 0x06: return new insn_vmudn(inst, addr);
	case 0x07: return new insn_vmudh(inst, addr);
	case 0x08: return new insn_vmacf(inst, addr);
	case 0x0d: return new insn_vmadm(inst, addr);
	case 0x0e: return new insn_vmadn(inst, addr);
	case 0x0f: return new insn_vmadh(inst, addr);
	case 0x10: return new insn_vadd(inst, addr);
	case 0x11: return new insn_vsub(inst, addr);
	case 0x14: return new insn_vaddc(inst, addr);
	case 0x1d: return new insn_vsar(inst, addr);
	case 0x23: return new insn_vge(inst, addr);
	case 0x24: return new insn_vcl(inst, addr);
	case 0x28: return new insn_vand(inst, addr);
	case 0x2c: return new insn_vxor(inst, addr);
	default: return nullptr;
	}
    case 0x20: return new insn_lb(inst, addr);
    case 0x21: return new insn_lh(inst, addr);
    case 0x23: return new insn_lw(inst, addr);
    case 0x24: return new insn_lbu(inst, addr);
    case 0x25: return new insn_lhu(inst, addr);
    case 0x28: return new insn_sb(inst, addr);
    case 0x29: return new insn_sh(inst, addr);
    case 0x2b: return new insn_sw(inst, addr);
    case 0x32:
      switch(rd)
	{
	case 1: return new insn_lsv(inst, addr);
	case 2: return new insn_llv(inst, addr);
	case 3: return new insn_ldv(inst, addr);
	case 4: return new insn_lqv(inst, addr);
	case 5: return new insn_lrv(inst, addr);
	default: return nullptr;
	}
    case 0x3a:
      switch(rd)
	{
	case 1: return new insn_ssv(inst, addr);
	case 2: return new insn_slv(inst, addr);
	case 3: return new insn_sdv(inst, addr);
	case 4: return new insn_sqv(inst, addr);
	default: return nullptr;
	}
    default:
      return nullptr;
    }
}

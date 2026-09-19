#ifndef __RSPINSTRUCTION_HH__
#define __RSPINSTRUCTION_HH__

#include <cstdint>
#include <vector>

struct rspFunc;
namespace llvm { class Value; }

/* One class per RSP instruction, each emitting its own LLVM IR (the mips32-bt Insn
 * shape).  A control-transfer instruction is handed its delay slot and emits it. */
class rspInsn {
protected:
  uint32_t inst, addr;
  uint32_t rs, rt, rd, sa, uimm;
  int32_t simm;
  llvm::Value *effAddr(rspFunc &f) const;           /* GPR[rs] + simm */
public:
  rspInsn(uint32_t inst, uint32_t addr);
  virtual ~rspInsn() {}
  virtual void generateIR(rspFunc &f, rspInsn *delay) = 0;
  virtual bool isControl() const { return false; }  /* has a delay slot, ends the block */
  virtual bool isBreak() const { return false; }
  virtual void staticTargets(std::vector<uint32_t> &t) const {}
  virtual bool fallsThrough() const { return true; }        /* control only: addr+8 reachable */
  virtual bool isCall() const { return false; }             /* jal: target runs in a pushed context */
  virtual bool isIndirect() const { return false; }         /* jr/jalr */
  virtual bool makesLinkRoot() const { return false; }      /* jalr: unknown callee returns to addr+8 by dispatch */
  uint32_t getAddr() const { return addr; }
  /* nullptr when the word is not an instruction the translator (or rsp.cc) implements */
  static rspInsn *decode(uint32_t inst, uint32_t addr);
};

#endif

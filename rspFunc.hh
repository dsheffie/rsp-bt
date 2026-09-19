#ifndef __RSPFUNC_HH__
#define __RSPFUNC_HH__

#include <cstdint>
#include <map>
#include <vector>

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"

/* Codegen context for one microcode image: the role regionCFG plays in mips32-bt.
 * Guest state is held in allocas (one per GPR, vector register, the accumulator and
 * each flag set) that mem2reg promotes, rather than in hand-placed phis: with a
 * static whole-image CFG there is nothing the hand-built SSA would buy. */
struct rspFunc {
  enum flag_t { VCO_L = 0, VCO_H, VCC_L, VCC_H, VCE, NUM_FLAGS };

  llvm::LLVMContext &ctx;
  llvm::Module *mod;
  llvm::IRBuilder<> *b;
  llvm::Function *fn = nullptr;
  llvm::Value *vState = nullptr;                  /* rsp_t* */

  llvm::Type *i1, *i8, *i16, *i32, *i64, *ptr;
  llvm::FixedVectorType *v16, *v32, *v64, *v1, *vb8;   /* <8 x i16/i32/i64/i1/i8> */

  llvm::AllocaInst *gpr[32] = {nullptr};
  llvm::AllocaInst *vec[32] = {nullptr};
  llvm::AllocaInst *acc = nullptr;
  llvm::AllocaInst *flag[NUM_FLAGS] = {nullptr};
  llvm::AllocaInst *retSlot = nullptr;

  llvm::BasicBlock *exitBB = nullptr;
  /* Code is specialized by call string: a context is the stack of return addresses of
   * the jals that led here (context 0 = none).  A block is (address, context). */
  typedef std::pair<uint32_t, uint32_t> blockKey;
  std::map<blockKey, llvm::BasicBlock*> blocks;
  std::vector<std::vector<uint32_t>> contexts;    /* id -> call string */
  std::map<std::vector<uint32_t>, uint32_t> contextIds;
  uint32_t curCtx = 0;                            /* context of the block being emitted */
  static const size_t MAX_CALL_DEPTH = 4;
  std::vector<uint32_t> roots;                    /* dispatch targets: entry, hints, observed */
  uint64_t uuid = 0;

  rspFunc(llvm::LLVMContext &ctx, llvm::Module *mod, llvm::IRBuilder<> *b);

  /* guest state */
  llvm::Value *getGPR(uint32_t r);
  void setGPR(uint32_t r, llvm::Value *v);
  llvm::Value *getVec(uint32_t r);
  void setVec(uint32_t r, llvm::Value *v);
  llvm::Value *getAcc();
  void setAcc(llvm::Value *v);
  llvm::Value *getFlag(flag_t f);
  void setFlag(flag_t f, llvm::Value *v);
  void clearFlag(flag_t f);

  /* constants and small IR idioms */
  llvm::Value *c32(uint32_t x) { return llvm::ConstantInt::get(i32, x); }
  llvm::Value *splat64(int64_t x);
  llvm::Value *splat16(int16_t x);
  llvm::Value *vtSelect(uint32_t vt, uint32_t e);     /* element field applied to vt */
  llvm::Value *sext48(llvm::Value *a);
  llvm::Value *satMid(llvm::Value *a);                /* clamp_s16(acc >> 16) */
  llvm::Value *clampLow(llvm::Value *a);
  void setAccLow(llvm::Value *res);

  /* memory */
  llvm::Value *statePtr(size_t offset);
  llvm::Value *dmemPtr(llvm::Value *ea12);
  llvm::Value *loadMem(llvm::Value *ea, unsigned nbytes);              /* zero-extended i32 */
  void storeMem(llvm::Value *ea, llvm::Value *val, unsigned nbytes);
  /* Vector loads/stores.  FIXED moves nbytes to/from byte offset e of the register.
   * ROW_LEFT is lqv/sqv: the bytes from ea up to the end of its 16-byte row, at the left
   * of the register.  ROW_RIGHT is lrv: the bytes of the row below ea, right-justified.
   * Whatever the inline path cannot express goes through rsp.cc's lwc2/swc2. */
  enum vecMemKind { FIXED, ROW_LEFT, ROW_RIGHT };
  void vecLoad(uint32_t insn, uint32_t base, uint32_t vt, llvm::Value *ea, uint32_t e, unsigned nbytes, vecMemKind kind);
  void vecStore(uint32_t insn, uint32_t base, uint32_t vt, llvm::Value *ea, uint32_t e, unsigned nbytes, vecMemKind kind);
  llvm::Value *fitsInDmem(llvm::Value *ea, unsigned nbytes);
  llvm::Value *toBytes(llvm::Value *v);           /* <8 x i16> -> 16 big-endian bytes */
  llvm::Value *fromBytes(llvm::Value *bytes);

  /* control */
  uint32_t pushCtx(uint32_t ctxId, uint32_t link);     /* context entered by a jal (or same, if too deep) */
  uint32_t popCtx(uint32_t ctxId) const;
  llvm::BasicBlock *blockFor(uint32_t addr) { return blockFor(addr, curCtx); }
  llvm::BasicBlock *blockFor(uint32_t addr, uint32_t ctxId);
  llvm::BasicBlock *newBlock(const char *name);
  void exitToInterp(uint32_t pc);                 /* leave: interpreter resumes at pc */
  void exitHalted();
  void indirectJump(llvm::Value *target);         /* devirtualized jr */
  llvm::FunctionCallee helper(const char *name, llvm::Type *ret, std::vector<llvm::Type*> args);
};

#endif

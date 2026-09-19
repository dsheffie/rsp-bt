#ifndef __RSPBT_HH__
#define __RSPBT_HH__

#include <cstdint>
#include <vector>

struct rsp_t;

/* LLVM-based binary translator for N64 RSP microcode.
 *
 * Control flow is walked statically from a set of roots; each instruction has a class
 * with a generateIR() method (the mips32-bt shape); one microcode image becomes one
 * LLVM function operating directly on rsp_t, so the interpreter in rsp.cc and
 * translated code are interchangeable mid-task.  Translations are memoized by a hash
 * of instruction memory.  No LLVM types leak through this header. */
class rspbt {
public:
  rspbt();
  ~rspbt();

  /* run the image in c.mem's IMEM from pc until BREAK.  'hint_roots' are extra
   * addresses worth treating as indirect-jump targets (e.g. a command dispatch table
   * read from the microcode's data); they affect speed only.  Any jr whose target was
   * not anticipated finishes the task in the interpreter and becomes a root next time. */
  void run(rsp_t &c, uint32_t pc, const std::vector<uint32_t> &hint_roots);

  uint64_t n_compiles = 0, n_fallbacks = 0;
  double compile_seconds = 0.0;
  bool dump_ir = false;          /* write rspbt_<hash>.ll before and after optimization */

private:
  struct impl;
  impl *p;
};

#endif

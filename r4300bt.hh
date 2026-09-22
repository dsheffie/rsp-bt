#ifndef __R4300BT_HH__
#define __R4300BT_HH__

#include <cstdint>
#include <vector>
#include <map>

class state_t;
struct r4300_cfg;

/* LLVM binary translator for the R4300 side of the audio path (see DESIGN_R4300.md).
 *
 * One host function per guest function.  Guest GPRs become allocas that mem2reg
 * promotes, so they live in host registers between the entry and the return; the FP file
 * stays in state_t, where the interpreter's exact layout already is.  Anything not
 * translated -- an unknown opcode, a call whose target we did not translate -- leaves
 * through the interpreter rather than being approximated.
 *
 * No LLVM types leak through this header. */
/* instructions still run by the interpreter on behalf of translated code */
extern uint64_t g_r4300bt_callee_insns, g_r4300bt_calls;
extern std::map<uint32_t, uint64_t> g_r4300bt_fallback_targets;

class r4300bt {
public:
  r4300bt();
  ~r4300bt();

  /* Translate `entries` (and nothing else) out of an already-built CFG.  Calls to
   * functions that were also translated become direct calls; the rest go through the
   * interpreter. */
  void translate(const r4300_cfg &cfg, const std::vector<uint32_t> &entries);

  bool have(uint32_t entry) const;

  /* Run the translated function at `entry`, which must have been translated.  Returns 0
   * if it returned through `jr $ra`; otherwise the guest pc the interpreter should
   * resume at, with state_t already up to date. */
  uint32_t run(state_t *s, uint8_t *ram, uint32_t entry);

  bool dump_ir = false;          /* write r4300bt_<addr>.ll before and after optimization */
  uint64_t n_bails = 0;          /* times translated code handed back to the interpreter */
  double compile_seconds = 0.0;

private:
  struct impl;
  impl *p;
};

#endif

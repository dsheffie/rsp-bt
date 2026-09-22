#ifndef __R4300CFG_HH__
#define __R4300CFG_HH__

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

/* Function discovery and control-flow graphs for MIPS III code.
 *
 * This is the front end of the R4300 translator (see DESIGN_R4300.md): it walks
 * statically from a set of roots, splits each function into basic blocks, and
 * resolves the indirect control flow it can.  Nothing here emits code; it exists so
 * the discovery can be inspected on its own, because a wrong CFG produces a wrong
 * translation that is much harder to read than a wrong picture.
 *
 * Memory is supplied by the caller -- only the harness knows how guest addresses map
 * to host memory (kseg0 plus the game's TLB alias), and the caller also knows the
 * handler pointers, which do not exist until libaudio has initialised. */
struct r4300_cfg {
  /* returns the instruction at a guest address, or nothing if unmapped */
  typedef std::function<bool(uint32_t, uint32_t &)> read_fn;

  struct block {
    uint32_t start = 0, end = 0;            /* [start, end): end is past the delay slot */
    std::vector<uint32_t> succs;
    bool ends_call = false;                 /* jal/jalr: falls through to the next block */
    bool ends_return = false;               /* jr $ra */
    bool ends_trap = false;                 /* break/syscall: no successors */
    /* branch-likely: the delay slot runs only on the taken edge.  The block holds it
     * because that is where it sits in memory, but a translator must emit it on the
     * taken edge alone -- 229 sites here, and only 6 of them are a nop. */
    bool ends_likely = false;
    bool ends_indirect = false;             /* jr through a table, or an unresolved jr */
    bool unresolved = false;                /* an indirect edge we could not resolve */
  };

  struct func {
    uint32_t entry = 0;
    bool leaf = true;                       /* makes no calls at all */
    std::map<uint32_t, block> blocks;
    std::set<uint32_t> calls;               /* direct callees */
    std::set<uint32_t> indirect_sites;      /* jalr pcs inside this function */
    size_t unresolved_calls = 0;            /* jalr sites with no known target */
    size_t n_insns = 0;
  };

  read_fn read;
  /* optional: renders one instruction, so the dumped graphs carry disassembly.  Supplied
   * by the caller because the disassembler lives in the simulator, not here. */
  std::function<std::string(uint32_t, uint32_t)> disasm;
  /* pc -> targets, for indirect control flow the walk cannot work out itself:
   * jalr handler pointers (read from RAM after init) and any jr we failed to
   * recognise as a bounds-checked table. */
  std::map<uint32_t, std::set<uint32_t>> hints;

  std::map<uint32_t, func> funcs;
  std::vector<std::pair<uint32_t, std::string>> problems;   /* pc, what went wrong */
  size_t n_jr_static = 0;    /* jr resolved by reading its table out of ROM */
  size_t n_jr_hint = 0;      /* jr resolved only because we watched it run */
  size_t n_traps = 0;        /* break/syscall sites, the compiler's divide guards */

  void discover(const std::vector<uint32_t> &roots);
  void write_dot(const std::string &dir) const;             /* one .dot per function */
  void report(FILE *f) const;

private:
  bool jump_table(uint32_t jr_pc, std::vector<uint32_t> &out);
  void build(uint32_t entry);
};

#endif

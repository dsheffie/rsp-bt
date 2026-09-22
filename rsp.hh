#ifndef __RSP_HH__
#define __RSP_HH__

#include <cstdint>

/* The N64 RSP: an R4000-derived 32-bit integer core (no TLB, no exceptions, no
 * mult/div, no 64-bit) plus a COP2 vector unit -- 32 registers of 8 x 16-bit lanes
 * and a 48-bit accumulator per lane.  It executes from 4 KB of IMEM against 4 KB of
 * DMEM; COP0 is not a system coprocessor but the SP DMA/status registers.
 * "Microcode" is ordinary MIPS machine code for this core.
 *
 * Only what rspboot + the audio microcode (aspMain) execute is implemented;
 * anything else dies loudly rather than guessing. */
/* Executed vector ALU ops and vector load/stores, for work-per-instruction reporting.
 * Compiled out unless RSP_WORK_COUNTERS is defined -- they sit in the innermost loop. */
extern uint64_t g_rsp_vec, g_rsp_vmem;
#ifdef RSP_WORK_COUNTERS
#define RSP_WORK(c) ((c)++)
#else
#define RSP_WORK(c) do { } while(0)
#endif

struct rsp_t {
  uint32_t r[32] = {0};
  uint32_t pc = 0;
  uint8_t mem[0x2000] = {0};          /* DMEM 0x000-0xfff, IMEM 0x1000-0x1fff */

  int16_t v[32][8] = {{0}};
  int64_t acc[8] = {0};               /* 48-bit, kept sign-extended */
  bool vco_l[8] = {0}, vco_h[8] = {0};   /* carry / not-equal */
  bool vcc_l[8] = {0}, vcc_h[8] = {0};   /* compare / clip */
  bool vce[8] = {0};

  uint32_t sp_mem_addr = 0;
  uint32_t sp_dram_addr = 0;
  /* SP_STATUS.  The eight SIG bits (14..7) are how the CPU and a resident microcode
   * talk to each other -- a command queue signals work with them -- so both sides read
   * and write this same register: the core through COP0 c4, the CPU through the SP
   * register block. */
  uint32_t sp_status = 0;
  /* Bumped whenever a DMA replaces instruction memory.  A translation is only valid for
   * the image it was made from, so an overlay landing in IMEM mid-task invalidates the
   * code currently executing -- the interpreter re-fetches and never notices, a
   * translation would happily keep running the old microcode. */
  uint64_t imem_gen = 0;
  bool halted = true;

  uint8_t *rdram = nullptr;           /* host pointer to guest physical 0 */
  uint64_t n_insns = 0;

  void run(uint32_t start_pc);
  /* public because translated code (rspbt, gemusic's avx512 backend) calls back into
   * them: COP0 moves, and the vector load/store forms on their slow paths */
  uint32_t mfc0(uint32_t rd);
  void mtc0(uint32_t rd, uint32_t x);
  void lwc2(uint32_t insn);
  void swc2(uint32_t insn);
  uint32_t rd32(uint32_t a) const;
  uint16_t rd16(uint32_t a) const;
  void wr32(uint32_t a, uint32_t x);
  void wr16(uint32_t a, uint16_t x);

  /* also driven from the CPU side: a ROM that kicks the RSP itself sets up the same
   * transfer through the SP registers (gemusic's n64_rcp) */
  void dma(uint32_t len_reg, bool to_rsp);
  /* SP_STATUS set/clear pairs; the CPU side writes the same register */
  void apply_status_write(uint32_t x);

private:
  void cop2(uint32_t insn);
  uint16_t get_vflags(uint32_t which) const;   /* VCO/VCC/VCE packed, for cfc2 */
  void set_vflags(uint32_t which, uint16_t v); /* the exact inverse, for ctc2 */
  uint8_t vbyte(uint32_t vt, uint32_t i) const;
  void set_vbyte(uint32_t vt, uint32_t i, uint8_t x);
};

#endif

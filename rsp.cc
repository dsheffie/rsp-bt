#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "rsp.hh"

#define RSP_DIE(...) do { fprintf(stderr, "rsp: " __VA_ARGS__); fprintf(stderr, " @pc=%03x\n", pc); exit(-1); } while(0)

/* ---- memory: every data access wraps inside DMEM, unaligned is legal ---- */

uint32_t rsp_t::rd32(uint32_t a) const {
  return (static_cast<uint32_t>(mem[a & 0xfff]) << 24) | (mem[(a+1) & 0xfff] << 16) |
    (mem[(a+2) & 0xfff] << 8) | mem[(a+3) & 0xfff];
}

uint16_t rsp_t::rd16(uint32_t a) const {
  return static_cast<uint16_t>((mem[a & 0xfff] << 8) | mem[(a+1) & 0xfff]);
}

void rsp_t::wr32(uint32_t a, uint32_t x) {
  mem[a & 0xfff] = static_cast<uint8_t>(x >> 24);
  mem[(a+1) & 0xfff] = static_cast<uint8_t>(x >> 16);
  mem[(a+2) & 0xfff] = static_cast<uint8_t>(x >> 8);
  mem[(a+3) & 0xfff] = static_cast<uint8_t>(x);
}

void rsp_t::wr16(uint32_t a, uint16_t x) {
  mem[a & 0xfff] = static_cast<uint8_t>(x >> 8);
  mem[(a+1) & 0xfff] = static_cast<uint8_t>(x);
}

/* ---- COP0 = SP registers.  A write to RD_LEN/WR_LEN runs the DMA at once, so
 * DMA_BUSY/DMA_FULL/STATUS always read idle. ---- */

/* The length register is three fields, not one: bytes per row in [11:0], rows in
 * [19:12], and the RDRAM stride between rows in [31:20] -- all biased by one except the
 * stride.  A flat transfer is just the one-row case, which is all rspboot ever asks for;
 * a microcode fetching strided data asks for the rest, and dropping COUNT silently
 * delivers only the first row and leaves the caller reading whatever was there before. */
void rsp_t::dma(uint32_t len_reg, bool to_rsp) {
  const uint32_t len   = ((len_reg & 0xfff) | 7) + 1;
  const uint32_t count = ((len_reg >> 12) & 0xff) + 1;
  const uint32_t skip  = (len_reg >> 20) & 0xfff;
  const uint32_t bank  = sp_mem_addr & 0x1000;
  uint32_t off  = sp_mem_addr & 0xff8;
  uint32_t dram = sp_dram_addr & 0xfffff8;

  for(uint32_t row = 0; row < count; row++) {
    uint32_t n = len;
    if(off + n > 0x1000) {
      n = 0x1000 - off;                /* a row never leaves its 4 KB bank */
    }
    if(to_rsp) {
      memcpy(mem + bank + off, rdram + dram, n);
      if(bank == 0x1000) {
	imem_gen++;                    /* the code just changed underneath anything running */
      }
    }
    else {
      memcpy(rdram + dram, mem + bank + off, n);
    }
    off = (off + n) & 0xfff;
    dram = (dram + n + skip) & 0xffffff;
  }
  sp_mem_addr = bank | off;
  sp_dram_addr = dram;
}

/* SP_STATUS writes are set/clear pairs rather than a value: bit 0 clears halt, bit 1
 * sets it, and from bit 9 up the eight signals each get a clear/set pair, clear in the
 * lower bit.  Shared by both sides, since both write this register. */
void rsp_t::apply_status_write(uint32_t x) {
  for(uint32_t i = 0; i < 8; i++) {
    const uint32_t bit = 1u << (7 + i);          /* SIG i sits at bit 7+i when read */
    if((x & (1u << (9 + 2 * i))) != 0)     { sp_status &= ~bit; }
    if((x & (1u << (10 + 2 * i))) != 0)    { sp_status |=  bit; }
  }
}

uint32_t rsp_t::mfc0(uint32_t rd) {
  switch(rd)
    {
    case 0: return sp_mem_addr;
    case 1: return sp_dram_addr;
    /* Transfers finish inside the mtc0 that starts them, so the length field always
     * reads back as drained (it counts down by 8 per word).  A microcode that polls
     * these rather than DMA_BUSY sees the transfer already done. */
    case 2: case 3: return 0xff8;
    case 4: return sp_status;          /* SP_STATUS, including the SIG bits */
    case 5: return 0;                  /* DMA_FULL */
    case 6: return 0;                  /* DMA_BUSY */
    case 7: return 0;                  /* SEMAPHORE: always acquired */
    /* DP command registers, 8..15.  Nothing is rasterised here, so the command buffer
     * reads back empty and idle: start == end == current means drained, which is what a
     * microcode managing the RDP buffer waits for. */
    case 8: case 9: case 10:           /* DPC_START / END / CURRENT */
    case 11:                           /* DPC_STATUS */
    case 12: case 13: case 14: case 15:
      return 0;
    default: break;
    }
  RSP_DIE("mfc0 of unimplemented SP/DPC register %u", rd);
}

void rsp_t::mtc0(uint32_t rd, uint32_t x) {
  switch(rd)
    {
    case 0: sp_mem_addr = x & 0x1fff; break;
    case 1: sp_dram_addr = x & 0xffffff; break;
    case 2: dma(x, true); break;
    case 3: dma(x, false); break;
    case 4: apply_status_write(x); break;
    case 7: break;                     /* SEMAPHORE release */
    case 8: case 9: case 10: case 11:  /* DP command buffer: accepted, nothing drawn */
    case 12: case 13: case 14: case 15: break;
    default: RSP_DIE("mtc0 of unimplemented SP/DPC register %u", rd);
    }
}

/* How much work an RSP instruction represents.  A vector op is 8 lanes of 16-bit, so
 * counting it as "one instruction" against a scalar MIPS one says very little -- build
 * with -DRSP_WORK_COUNTERS to get the element count instead.  Off by default: this is
 * the interpreter's innermost loop. */
uint64_t g_rsp_vec = 0, g_rsp_vmem = 0;

/* ---- COP2 ---- */

static inline int64_t sext48(int64_t x) {
  return (x << 16) >> 16;
}

static inline int16_t clamp_s16(int64_t x) {
  return static_cast<int16_t>(x < -32768 ? -32768 : (x > 32767 ? 32767 : x));
}

/* result of the "low" multiply-accumulates: ACC[15:0], saturated when the 48-bit
 * accumulator no longer fits in a signed 32-bit value */
static inline int16_t clamp_acc_low(int64_t a) {
  if(a < -0x80000000LL) {
    return 0;
  }
  if(a > 0x7fffffffLL) {
    return static_cast<int16_t>(0xffff);
  }
  return static_cast<int16_t>(a & 0xffff);
}

/* element specifier: which lane of vt feeds lane i */
static inline uint32_t vt_lane(uint32_t e, uint32_t i) {
  if(e < 2) {
    return i;
  }
  if(e < 4) {
    return (i & ~1u) | (e & 1u);
  }
  if(e < 8) {
    return (i & ~3u) | (e & 3u);
  }
  return e & 7u;
}

uint8_t rsp_t::vbyte(uint32_t vt, uint32_t i) const {
  uint16_t x = static_cast<uint16_t>(v[vt][(i & 15) >> 1]);
  return static_cast<uint8_t>((i & 1) ? x : (x >> 8));
}

void rsp_t::set_vbyte(uint32_t vt, uint32_t i, uint8_t b) {
  uint16_t x = static_cast<uint16_t>(v[vt][(i & 15) >> 1]);
  x = (i & 1) ? static_cast<uint16_t>((x & 0xff00) | b) : static_cast<uint16_t>((x & 0x00ff) | (b << 8));
  v[vt][(i & 15) >> 1] = static_cast<int16_t>(x);
}

uint16_t rsp_t::get_vflags(uint32_t which) const {
  const bool *lo = (which == 0) ? vco_l : ((which == 1) ? vcc_l : vce);
  const bool *hi = (which == 0) ? vco_h : vcc_h;
  uint16_t v = 0;
  for(uint32_t i = 0; i < 8; i++) {
    v |= static_cast<uint16_t>(lo[i] ? (1u << i) : 0u);
    if(which != 2) {                      /* VCE is eight bits, with no high half */
      v |= static_cast<uint16_t>(hi[i] ? (1u << (i + 8)) : 0u);
    }
  }
  return v;
}

void rsp_t::set_vflags(uint32_t which, uint16_t v) {
  bool *lo = (which == 0) ? vco_l : ((which == 1) ? vcc_l : vce);
  bool *hi = (which == 0) ? vco_h : vcc_h;
  for(uint32_t i = 0; i < 8; i++) {
    lo[i] = ((v >> i) & 1) != 0;
    if(which != 2) {
      hi[i] = ((v >> (i + 8)) & 1) != 0;
    }
  }
}

void rsp_t::cop2(uint32_t insn) {
  if(((insn >> 25) & 1) == 0) {
    uint32_t rt = (insn >> 16) & 31, vd = (insn >> 11) & 31, e = (insn >> 7) & 15;
    switch((insn >> 21) & 31)
      {
      case 0: /* mfc2 */
	r[rt] = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>((vbyte(vd, e) << 8) | vbyte(vd, e+1))));
	break;
      case 4: /* mtc2 */
	set_vbyte(vd, e, static_cast<uint8_t>(r[rt] >> 8));
	if(e != 15) {
	  set_vbyte(vd, e+1, static_cast<uint8_t>(r[rt]));
	}
	break;
      /* The flag registers as a scalar.  vd selects VCO(0), VCC(1), VCE(2), and cfc2
       * sign-extends the 16-bit value to 32.
       *
       * Layout per the SGI RSP Programmer's Guide (figures 2-5..2-7): element i is bit i,
       * and the paired flag is bit i+8.  VCO's low half is CARRY and its high half NOT
       * EQUAL; VCC's low half is the select compare and its high half the clip compare;
       * VCE is eight bits, one per slice, with no high half. */
      case 2: /* cfc2 */
	r[rt] = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(get_vflags(vd))));
	break;
      case 6: /* ctc2 */
	set_vflags(vd, static_cast<uint16_t>(r[rt]));
	break;
      default:
	RSP_DIE("unimplemented cop2 move %08x", insn);
      }
    return;
  }

  uint32_t e = (insn >> 21) & 15, vt = (insn >> 16) & 31, vs = (insn >> 11) & 31, vd = (insn >> 6) & 31;
  uint32_t fn = insn & 63;
  int16_t res[8];
  for(uint32_t i = 0; i < 8; i++) {
    int16_t s = v[vs][i], t = v[vt][vt_lane(e, i)];
    int64_t ss = s, tt = t, su = static_cast<uint16_t>(s), tu = static_cast<uint16_t>(t);
    switch(fn)
      {
      case 0x00: /* vmulf */
	acc[i] = sext48(ss*tt*2 + 0x8000);
	res[i] = clamp_s16(acc[i] >> 16);
	break;
      case 0x04: /* vmudl */
	acc[i] = (su*tu) >> 16;
	res[i] = clamp_acc_low(acc[i]);
	break;
      case 0x05: /* vmudm */
	acc[i] = sext48(ss*tu);
	res[i] = clamp_s16(acc[i] >> 16);
	break;
      case 0x06: /* vmudn */
	acc[i] = sext48(su*tt);
	res[i] = clamp_acc_low(acc[i]);
	break;
      case 0x07: /* vmudh */
	acc[i] = sext48((ss*tt) << 16);
	res[i] = clamp_s16(acc[i] >> 16);
	break;
      case 0x08: /* vmacf */
	acc[i] = sext48(acc[i] + ss*tt*2);
	res[i] = clamp_s16(acc[i] >> 16);
	break;
      case 0x0d: /* vmadm */
	acc[i] = sext48(acc[i] + ss*tu);
	res[i] = clamp_s16(acc[i] >> 16);
	break;
      case 0x0e: /* vmadn */
	acc[i] = sext48(acc[i] + su*tt);
	res[i] = clamp_acc_low(acc[i]);
	break;
      case 0x0f: /* vmadh */
	acc[i] = sext48(acc[i] + ((ss*tt) << 16));
	res[i] = clamp_s16(acc[i] >> 16);
	break;
      case 0x10: /* vadd */
	{
	  int64_t x = ss + tt + (vco_l[i] ? 1 : 0);
	  acc[i] = (acc[i] & ~0xffffLL) | (x & 0xffff);
	  res[i] = clamp_s16(x);
	  vco_l[i] = vco_h[i] = false;
	}
	break;
      case 0x11: /* vsub */
	{
	  int64_t x = ss - tt - (vco_l[i] ? 1 : 0);
	  acc[i] = (acc[i] & ~0xffffLL) | (x & 0xffff);
	  res[i] = clamp_s16(x);
	  vco_l[i] = vco_h[i] = false;
	}
	break;
      case 0x14: /* vaddc */
	{
	  int64_t x = su + tu;
	  res[i] = static_cast<int16_t>(x & 0xffff);
	  acc[i] = (acc[i] & ~0xffffLL) | (x & 0xffff);
	  vco_l[i] = (x >> 16) & 1;
	  vco_h[i] = false;
	}
	break;
      case 0x1d: /* vsar: e selects the accumulator slice */
	res[i] = static_cast<int16_t>((e == 8) ? (acc[i] >> 32) : ((e == 9) ? (acc[i] >> 16) : ((e == 10) ? acc[i] : 0)));
	break;
      case 0x23: /* vge */
	{
	  bool ge = (s > t) or ((s == t) and not(vco_h[i] and vco_l[i]));
	  res[i] = ge ? s : t;
	  acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	  vcc_l[i] = ge;
	  vcc_h[i] = false;
	  vco_l[i] = vco_h[i] = false;
	}
	break;
      case 0x24: /* vcl */
	{
	  if(vco_l[i]) {
	    if(not(vco_h[i])) {
	      int64_t sum = su + tu;
	      vcc_l[i] = vce[i] ? (sum <= 0x10000) : (sum == 0);
	    }
	    res[i] = vcc_l[i] ? static_cast<int16_t>(-tu) : s;
	  }
	  else {
	    if(not(vco_h[i])) {
	      vcc_h[i] = (su - tu) >= 0;
	    }
	    res[i] = vcc_h[i] ? t : s;
	  }
	  acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	  vco_l[i] = vco_h[i] = false;
	  vce[i] = false;
	}
	break;
      case 0x28: /* vand */
	res[i] = s & t;
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      case 0x29: /* vnand */
	res[i] = static_cast<int16_t>(~(s & t));
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      case 0x2a: /* vor */
	res[i] = s | t;
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      case 0x2b: /* vnor */
	res[i] = static_cast<int16_t>(~(s | t));
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      case 0x2d: /* vnxor */
	res[i] = static_cast<int16_t>(~(s ^ t));
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      /* Merge: pick per lane on the low half of VCC, leaving every flag alone.  The
       * guide is explicit that VCC, VCO and VCE are all unchanged here. */
      case 0x27: /* vmrg */
	res[i] = vcc_l[i] ? s : t;
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      /* Conditional negate by the sign of vs, which also yields sign().  No clamping is
       * specified, so vt == -32768 negates to itself. */
      case 0x13: /* vabs */
	res[i] = (s < 0) ? static_cast<int16_t>(-t) : ((s == 0) ? 0 : t);
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      /* Subtract producing borrow and not-equal in the two halves of VCO, for the
       * double-precision sequences.  Deliberately unclamped. */
      case 0x15: /* vsubc */
	{
	  const int32_t d = static_cast<int32_t>(s) - static_cast<int32_t>(t);
	  res[i] = static_cast<int16_t>(d);
	  acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	  vco_l[i] = (d < 0);
	  vco_h[i] = (d != 0);
	}
	break;
      case 0x2c: /* vxor */
	res[i] = s ^ t;
	acc[i] = (acc[i] & ~0xffffLL) | static_cast<uint16_t>(res[i]);
	break;
      default:
	RSP_DIE("unimplemented vector op %02x (%08x)", fn, insn);
      }
  }
  memcpy(v[vd], res, sizeof(res));
}

/* vector loads/stores: offset is 7-bit signed, scaled by the access size */
void rsp_t::lwc2(uint32_t insn) {
  uint32_t base = (insn >> 21) & 31, vt = (insn >> 16) & 31, op = (insn >> 11) & 31, e = (insn >> 7) & 15;
  int32_t offs = static_cast<int32_t>(insn << 25) >> 25;
  switch(op)
    {
    case 0: /* lbv: one byte into byte element e */
      {
	const uint32_t ea = r[base] + offs;
	set_vbyte(vt, e, mem[ea & 0xfff]);
      }
      break;
    case 1: /* lsv */
    case 2: /* llv */
    case 3: /* ldv */
      {
	uint32_t ea = r[base] + (offs << op), n = 1u << op;
	for(uint32_t i = 0; i < n and (e + i) < 16; i++) {
	  set_vbyte(vt, e + i, mem[(ea + i) & 0xfff]);
	}
      }
      break;
    case 4: /* lqv: from ea to the end of its 16-byte row */
      {
	uint32_t ea = r[base] + (offs << 4), n = 16 - (ea & 15);
	for(uint32_t i = 0; i < n and (e + i) < 16; i++) {
	  set_vbyte(vt, e + i, mem[(ea + i) & 0xfff]);
	}
      }
      break;
    /* The packed forms take eight consecutive bytes into the eight lanes.  lpv puts the
     * byte at bits 15..8 and zeroes the rest, so it reads as a signed fraction; luv puts
     * it at 14..7 with bit 15 clear, so it reads as unsigned.  The zeroing is the point:
     * both are there to feed the multipliers directly. */
    /* Transpose load.  The guide describes this in terms of a "Slice" it never binds to
     * a memory offset, so the general mapping is not derivable from it.  The e=0 case is
     * pinned down independently: libdragon loads an identity table through it to seed a
     * predictor filter, and for the result to be an identity, slice s must write register
     * s at lane s.  Both candidate readings of the guide agree there, since the rotation
     * is (s + e/2) & 7.  A non-zero element rotates -- once or twice, the guide is
     * ambiguous -- and is refused rather than guessed at. */
    case 11: /* ltv */
      {
	if(e != 0) {
	  RSP_DIE("ltv element %u: the slice-to-memory mapping is undocumented for a "
		  "non-zero element and has not been measured", e);
	}
	const uint32_t ea = r[base] + (offs << 4);
	const uint32_t group = vt & 0x18;
	for(uint32_t sl = 0; sl < 8; sl++) {
	  const uint32_t m = (ea + 2 * sl) & 0xfff;
	  v[group | sl][sl] = static_cast<int16_t>((mem[m] << 8) | mem[(m + 1) & 0xfff]);
	}
      }
      break;
    case 6: /* lpv */
    case 7: /* luv */
      {
	const uint32_t ea = r[base] + (offs << 3);
	const uint32_t shift = (op == 6) ? 8 : 7;
	for(uint32_t i = 0; i < 8; i++) {
	  v[vt][i] = static_cast<int16_t>(static_cast<uint16_t>(mem[(ea + i) & 0xfff]) << shift);
	}
      }
      break;
    case 5: /* lrv: the bytes of the row below ea, right-justified in the register */
      {
	uint32_t ea = r[base] + (offs << 4), start = 16 - ((ea & 15) - e);
	ea &= ~15u;
	for(uint32_t i = start; i < 16; i++) {
	  set_vbyte(vt, i, mem[ea & 0xfff]);
	  ea++;
	}
      }
      break;
    default:
      RSP_DIE("unimplemented vector load %u", op);
    }
}

void rsp_t::swc2(uint32_t insn) {
  uint32_t base = (insn >> 21) & 31, vt = (insn >> 16) & 31, op = (insn >> 11) & 31, e = (insn >> 7) & 15;
  int32_t offs = static_cast<int32_t>(insn << 25) >> 25;
  switch(op)
    {
    case 0: /* sbv */
      {
	const uint32_t ea = r[base] + offs;
	mem[ea & 0xfff] = vbyte(vt, e);
      }
      break;
    /* The mirrors of lpv/luv: the byte comes back out of bits 15..8 or 14..7. */
    case 6: /* spv */
    case 7: /* suv */
      {
	const uint32_t ea = r[base] + (offs << 3);
	const uint32_t shift = (op == 6) ? 8 : 7;
	for(uint32_t i = 0; i < 8; i++) {
	  mem[(ea + i) & 0xfff] = static_cast<uint8_t>(static_cast<uint16_t>(v[vt][i]) >> shift);
	}
      }
      break;
    case 1: /* ssv */
    case 2: /* slv */
    case 3: /* sdv */
      {
	uint32_t ea = r[base] + (offs << op), n = 1u << op;
	for(uint32_t i = 0; i < n; i++) {
	  mem[(ea + i) & 0xfff] = vbyte(vt, e + i);
	}
      }
      break;
    case 5: /* srv: the mirror of lrv -- to the aligned boundary up to the byte address */
      {
	uint32_t ea = r[base] + (offs << 4);
	const uint32_t start = 16 - ((ea & 15) - e);
	ea &= ~15u;
	for(uint32_t i = start; i < 16; i++) {
	  mem[ea & 0xfff] = vbyte(vt, i);
	  ea++;
	}
      }
      break;
    case 4: /* sqv */
      {
	uint32_t ea = r[base] + (offs << 4), n = 16 - (ea & 15);
	for(uint32_t i = 0; i < n; i++) {
	  mem[(ea + i) & 0xfff] = vbyte(vt, e + i);
	}
      }
      break;
    default:
      RSP_DIE("unimplemented vector store %u", op);
    }
}

/* ---- integer core ---- */

void rsp_t::run(uint32_t start_pc) {
  pc = start_pc & 0xffc;
  halted = false;
  uint32_t next_pc = (pc + 4) & 0xffc;
  while(not(halted)) {
    uint32_t insn = (static_cast<uint32_t>(mem[0x1000 + pc]) << 24) | (mem[0x1001 + pc] << 16) |
      (mem[0x1002 + pc] << 8) | mem[0x1003 + pc];
    uint32_t after = (next_pc + 4) & 0xffc;       /* default: fall through past next_pc */
    uint32_t rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, rd = (insn >> 11) & 31, sa = (insn >> 6) & 31;
    int32_t simm = static_cast<int16_t>(insn);
    uint32_t uimm = insn & 0xffff;
    uint32_t btgt = (next_pc + (static_cast<uint32_t>(simm) << 2)) & 0xffc;
    n_insns++;
    {   /* RSPTRACE=n: the first n instructions of each kick, to see what a microcode
	 * actually does rather than infer it from where it stopped */
      static const uint64_t g_tr = getenv("RSPTRACE") ? strtoull(getenv("RSPTRACE"),0,0) : 0;
      static uint64_t seen = 0;
      if(g_tr && seen < g_tr) {
	fprintf(stderr, "[rsp] %03x  %08x\n", pc, insn);
	seen++;
      }
    }
    const uint32_t opcode = insn >> 26;
    switch(opcode)
      {
      case 0x00:
	switch(insn & 63)
	  {
	  case 0x00: r[rd] = r[rt] << sa; break;
	  case 0x02: r[rd] = r[rt] >> sa; break;
	  case 0x03: r[rd] = static_cast<uint32_t>(static_cast<int32_t>(r[rt]) >> sa); break;
	  case 0x04: r[rd] = r[rt] << (r[rs] & 31); break;
	  case 0x06: r[rd] = r[rt] >> (r[rs] & 31); break;
	  case 0x07: r[rd] = static_cast<uint32_t>(static_cast<int32_t>(r[rt]) >> (r[rs] & 31)); break;
	  case 0x08: after = r[rs] & 0xffc; break;
	  case 0x09: after = r[rs] & 0xffc; r[rd] = (next_pc + 4) & 0xffc; break;
	  case 0x0d: halted = true; break;                       /* break: task done */
	  case 0x20: case 0x21: r[rd] = r[rs] + r[rt]; break;    /* add never traps here */
	  case 0x22: case 0x23: r[rd] = r[rs] - r[rt]; break;
	  case 0x24: r[rd] = r[rs] & r[rt]; break;
	  case 0x25: r[rd] = r[rs] | r[rt]; break;
	  case 0x26: r[rd] = r[rs] ^ r[rt]; break;
	  case 0x27: r[rd] = ~(r[rs] | r[rt]); break;
	  case 0x2a: r[rd] = static_cast<int32_t>(r[rs]) < static_cast<int32_t>(r[rt]); break;
	  case 0x2b: r[rd] = r[rs] < r[rt]; break;
	  default: RSP_DIE("unimplemented special %02x", insn & 63);
	  }
	break;
      case 0x01:
	{
	  bool ltz = static_cast<int32_t>(r[rs]) < 0;
	  if(rt & 0x10) {
	    r[31] = (next_pc + 4) & 0xffc;
	  }
	  if((rt & 1) ? not(ltz) : ltz) {
	    after = btgt;
	  }
	}
	break;
      case 0x02: after = (insn << 2) & 0xffc; break;
      case 0x03: after = (insn << 2) & 0xffc; r[31] = (next_pc + 4) & 0xffc; break;
      case 0x04: if(r[rs] == r[rt]) { after = btgt; } break;
      case 0x05: if(r[rs] != r[rt]) { after = btgt; } break;
      case 0x06: if(static_cast<int32_t>(r[rs]) <= 0) { after = btgt; } break;
      case 0x07: if(static_cast<int32_t>(r[rs]) > 0) { after = btgt; } break;
      case 0x08: case 0x09: r[rt] = r[rs] + static_cast<uint32_t>(simm); break;
      case 0x0a: r[rt] = static_cast<int32_t>(r[rs]) < simm; break;
      case 0x0b: r[rt] = r[rs] < static_cast<uint32_t>(simm); break;
      case 0x0c: r[rt] = r[rs] & uimm; break;
      case 0x0d: r[rt] = r[rs] | uimm; break;
      case 0x0e: r[rt] = r[rs] ^ uimm; break;
      case 0x0f: r[rt] = uimm << 16; break;
      case 0x10:
	if(rs == 0) {
	  r[rt] = mfc0(rd);
	}
	else if(rs == 4) {
	  mtc0(rd, r[rt]);
	}
	else {
	  RSP_DIE("unimplemented cop0 %08x", insn);
	}
	break;
      case 0x12: cop2(insn); RSP_WORK(g_rsp_vec); break;
      case 0x20: r[rt] = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(mem[(r[rs] + simm) & 0xfff]))); break;
      case 0x21: r[rt] = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(rd16(r[rs] + simm)))); break;
      case 0x23: r[rt] = rd32(r[rs] + simm); break;
      case 0x24: r[rt] = mem[(r[rs] + simm) & 0xfff]; break;
      case 0x25: r[rt] = rd16(r[rs] + simm); break;
      case 0x28: mem[(r[rs] + simm) & 0xfff] = static_cast<uint8_t>(r[rt]); break;
      case 0x29: wr16(r[rs] + simm, static_cast<uint16_t>(r[rt])); break;
      case 0x2b: wr32(r[rs] + simm, r[rt]); break;
      case 0x32: lwc2(insn); RSP_WORK(g_rsp_vmem); break;
      case 0x3a: swc2(insn); RSP_WORK(g_rsp_vmem); break;
      default: RSP_DIE("unimplemented opcode %02x (%08x)", opcode, insn);
      }
    r[0] = 0;
    /* delay slot: a taken branch at pc redirects the instruction after next_pc */
    pc = next_pc;
    next_pc = after;
    /* 1ULL: unsigned long is 32 bits on wasm32, where 1UL<<40 masks to 1<<8 and the
     * shift is undefined -- at -O0 the guard fired after 256 instructions, and at -O3
     * the optimiser used the undefined behaviour to mangle the dispatch switch. */
    if(n_insns > (1ULL << 40)) {
      RSP_DIE("runaway");
    }
  }
}

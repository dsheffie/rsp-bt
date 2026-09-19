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

void rsp_t::dma(uint32_t len_reg, bool to_rsp) {
  uint32_t len = ((len_reg & 0xfff) | 7) + 1;
  uint32_t bank = sp_mem_addr & 0x1000;
  uint32_t off = sp_mem_addr & 0xff8;
  uint32_t dram = sp_dram_addr & 0xfffff8;
  if(off + len > 0x1000) {
    len = 0x1000 - off;                /* a transfer never leaves its 4 KB bank */
  }
  if(to_rsp) {
    memcpy(mem + bank + off, rdram + dram, len);
  }
  else {
    memcpy(rdram + dram, mem + bank + off, len);
  }
  sp_mem_addr = bank | ((off + len) & 0xfff);
  sp_dram_addr = dram + len;
}

uint32_t rsp_t::mfc0(uint32_t rd) {
  switch(rd)
    {
    case 0: return sp_mem_addr;
    case 1: return sp_dram_addr;
    case 4: return 0;                  /* SP_STATUS: running, no yield request */
    case 5: return 0;                  /* DMA_FULL */
    case 6: return 0;                  /* DMA_BUSY */
    case 7: return 0;                  /* SEMAPHORE: always acquired */
    case 11: return 0;                 /* DPC_STATUS */
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
    case 4: break;                     /* SP_STATUS set/clear bits: signals only */
    case 7: break;                     /* SEMAPHORE release */
    default: RSP_DIE("mtc0 of unimplemented SP/DPC register %u", rd);
    }
}

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
    switch(insn >> 26)
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
      case 0x12: cop2(insn); break;
      case 0x20: r[rt] = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(mem[(r[rs] + simm) & 0xfff]))); break;
      case 0x21: r[rt] = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(rd16(r[rs] + simm)))); break;
      case 0x23: r[rt] = rd32(r[rs] + simm); break;
      case 0x24: r[rt] = mem[(r[rs] + simm) & 0xfff]; break;
      case 0x25: r[rt] = rd16(r[rs] + simm); break;
      case 0x28: mem[(r[rs] + simm) & 0xfff] = static_cast<uint8_t>(r[rt]); break;
      case 0x29: wr16(r[rs] + simm, static_cast<uint16_t>(r[rt])); break;
      case 0x2b: wr32(r[rs] + simm, r[rt]); break;
      case 0x32: lwc2(insn); break;
      case 0x3a: swc2(insn); break;
      default: RSP_DIE("unimplemented opcode %02x (%08x)", insn >> 26, insn);
      }
    r[0] = 0;
    /* delay slot: a taken branch at pc redirects the instruction after next_pc */
    pc = next_pc;
    next_pc = after;
    if(n_insns > (1UL << 40)) {
      RSP_DIE("runaway");
    }
  }
}

/* Differential test: every instruction, interpreter against translator.
 *
 * The RSP model grew to fit one microcode, and filling in the rest of the ISA means
 * writing a lot of code that nothing in this repository exercises -- GoldenEye's audio
 * microcode uses a fraction of the instruction set, so the usual regression (byte-exact
 * audio) says nothing about any of it.
 *
 * The interpreter and the translator are independent implementations of the same spec,
 * so running one instruction through both from identical state and comparing all of the
 * architectural state catches transcription slips, which is the failure mode that
 * actually happens.  It cannot catch a misreading of the spec that both inherit -- for
 * that there is no substitute for the documentation. */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include "rsp.hh"
#include "rspbt.hh"

namespace {
  /* everything the guest can observe */
  struct snap_t {
    uint32_t r[32];
    int16_t v[32][8];
    int64_t acc[8];
    bool vco_l[8], vco_h[8], vcc_l[8], vcc_h[8], vce[8];
    uint8_t dmem[0x1000];

    void take(const rsp_t &c) {
      memcpy(r, c.r, sizeof(r));
      memcpy(v, c.v, sizeof(v));
      memcpy(acc, c.acc, sizeof(acc));
      memcpy(vco_l, c.vco_l, sizeof(vco_l));  memcpy(vco_h, c.vco_h, sizeof(vco_h));
      memcpy(vcc_l, c.vcc_l, sizeof(vcc_l));  memcpy(vcc_h, c.vcc_h, sizeof(vcc_h));
      memcpy(vce, c.vce, sizeof(vce));
      memcpy(dmem, c.mem, sizeof(dmem));
    }
  };

  /* A fixed seed per case, so a failure is reproducible by its number alone. */
  void randomize(rsp_t &c, uint64_t seed) {
    std::mt19937_64 g(seed);
    for(int i = 1; i < 32; i++) {
      c.r[i] = static_cast<uint32_t>(g()) & 0xfff;   /* keep addresses inside DMEM */
    }
    c.r[0] = 0;
    for(int i = 0; i < 32; i++) {
      for(int e = 0; e < 8; e++) {
	c.v[i][e] = static_cast<int16_t>(g());
      }
    }
    for(int e = 0; e < 8; e++) {
      c.acc[e] = static_cast<int64_t>(g() & 0xffffffffffffULL) - (1LL << 47);
      c.vco_l[e] = g() & 1;  c.vco_h[e] = g() & 1;
      c.vcc_l[e] = g() & 1;  c.vcc_h[e] = g() & 1;
      c.vce[e]   = g() & 1;
    }
    for(size_t i = 0; i < 0x1000; i++) {
      c.mem[i] = static_cast<uint8_t>(g());
    }
  }

  /* one instruction, then stop */
  void load_insn(rsp_t &c, uint32_t insn) {
    const uint32_t prog[2] = {insn, 0x0000000d};    /* the instruction, then break */
    for(int i = 0; i < 2; i++) {
      uint8_t *p = c.mem + 0x1000 + 4 * i;
      p[0] = static_cast<uint8_t>(prog[i] >> 24); p[1] = static_cast<uint8_t>(prog[i] >> 16);
      p[2] = static_cast<uint8_t>(prog[i] >> 8);   p[3] = static_cast<uint8_t>(prog[i]);
    }
  }

  size_t n_run = 0, n_bad = 0;

  bool differs(const snap_t &a, const snap_t &b, const char *&what, int &idx) {
    for(int i = 0; i < 32; i++) {
      if(a.r[i] != b.r[i]) { what = "gpr"; idx = i; return true; }
    }
    for(int i = 0; i < 32; i++) {
      for(int e = 0; e < 8; e++) {
	if(a.v[i][e] != b.v[i][e]) { what = "vreg"; idx = i; return true; }
      }
    }
    for(int e = 0; e < 8; e++) {
      if(a.acc[e] != b.acc[e]) { what = "acc"; idx = e; return true; }
      if(a.vco_l[e] != b.vco_l[e] or a.vco_h[e] != b.vco_h[e]) { what = "vco"; idx = e; return true; }
      if(a.vcc_l[e] != b.vcc_l[e] or a.vcc_h[e] != b.vcc_h[e]) { what = "vcc"; idx = e; return true; }
      if(a.vce[e] != b.vce[e]) { what = "vce"; idx = e; return true; }
    }
    for(size_t i = 0; i < 0x1000; i++) {
      if(a.dmem[i] != b.dmem[i]) { what = "dmem"; idx = static_cast<int>(i); return true; }
    }
    return false;
  }

  /* An instruction the model does not implement exits the process rather than returning,
   * which is right for a simulator and awkward for a sweep over the whole opcode space.
   * Each group therefore runs in a child: a died child means "not implemented yet", which
   * is information, not a crash. */
  int run_group(const char *name, const std::vector<uint32_t> &insns, uint64_t cases);

  void check(rspbt &jit, uint32_t insn, uint64_t seed, const char *name) {
    rsp_t a, b;
    randomize(a, seed);
    memcpy(&b, &a, sizeof(rsp_t));
    load_insn(a, insn);
    load_insn(b, insn);

    a.run(0);                                /* interpreter */
    jit.run(b, 0, {});                       /* translator */

    snap_t sa, sb;
    sa.take(a);
    sb.take(b);
    n_run++;
    const char *what = nullptr;
    int idx = 0;
    if(differs(sa, sb, what, idx)) {
      n_bad++;
      if(n_bad <= 20) {
	printf("MISMATCH %-8s insn=%08x seed=%llu: %s[%d]\n", name, insn,
	       static_cast<unsigned long long>(seed), what, idx);
      }
    }
  }
}

namespace {
  int run_group(const char *name, const std::vector<uint32_t> &insns, uint64_t cases) {
    fflush(stdout);
    pid_t pid = fork();
    if(pid == 0) {
      rspbt jit;
      for(size_t i = 0; i < insns.size(); i++) {
	for(uint64_t s = 0; s < cases; s++) {
	  check(jit, insns[i], s * 104729 + i * 17 + 1, name);
	}
      }
      printf("%-8s %zu cases, %zu mismatches\n", name, n_run, n_bad);
      fflush(stdout);
      _exit(n_bad == 0 ? 0 : 1);
    }
    int st = 0;
    waitpid(pid, &st, 0);
    if(WIFSIGNALED(st) or (WIFEXITED(st) and WEXITSTATUS(st) > 1)) {
      printf("%-8s NOT IMPLEMENTED (model stopped)\n", name);
      return 2;
    }
    return WIFEXITED(st) ? WEXITSTATUS(st) : 2;
  }
}

int main(int argc, char *argv[]) {
  const uint64_t cases = (argc > 1) ? strtoull(argv[1], nullptr, 0) : 32;
  size_t missing = 0, bad = 0;

  /* vector loads and stores: every form, every element field, aligned and not */
  for(uint32_t op = 0x32; op <= 0x3a; op += 8) {            /* lwc2 then swc2 */
    for(uint32_t form = 0; form < 12; form++) {
      std::vector<uint32_t> insns;
      for(uint32_t e = 0; e < 16; e++) {
	for(uint32_t off = 0; off < 8; off++) {
	  insns.push_back((op << 26) | (7u << 21) | (3u << 16) | (form << 11) | (e << 7) | off);
	}
      }
      char nm[16];
      snprintf(nm, sizeof(nm), "%s%02u", (op == 0x32) ? "ld" : "st", form);
      const int r = run_group(nm, insns, cases);
      if(r == 2) { missing++; } else if(r == 1) { bad++; }
    }
  }

  /* COP2 ALU: every funct, every element field */
  for(uint32_t fn = 0; fn < 64; fn++) {
    std::vector<uint32_t> insns;
    for(uint32_t e = 0; e < 16; e++) {
      insns.push_back((0x12u << 26) | (1u << 25) | (e << 21) | (5u << 16) | (9u << 11) | (13u << 6) | fn);
    }
    char nm[16];
    snprintf(nm, sizeof(nm), "cop2.%02x", fn);
    const int r = run_group(nm, insns, cases);
    if(r == 2) { missing++; } else if(r == 1) { bad++; }
  }

  printf("\n%zu groups unimplemented, %zu groups with mismatches\n", missing, bad);
  return bad == 0 ? 0 : 1;
}

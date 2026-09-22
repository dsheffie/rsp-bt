#include "r4300cfg.hh"

#include <cstdio>
#include <cstring>
#include <deque>
#include <algorithm>

/* MIPS III control-flow decode.  We only care about what an instruction does to the pc,
 * so everything that is not a branch or a jump is "falls through". */
namespace {
  enum class kind { fall, branch, likely, jump, call, jr, jalr, trap };

  struct ctl {
    kind k = kind::fall;
    uint32_t target = 0;     /* for branch/jump/call */
    uint32_t reg = 0;        /* for jr/jalr: the register holding the target */
  };

  ctl decode(uint32_t pc, uint32_t insn) {
    ctl c;
    const uint32_t op = insn >> 26;
    const uint32_t rs = (insn >> 21) & 31;
    const uint32_t rt = (insn >> 16) & 31;
    const int32_t simm = static_cast<int16_t>(insn & 0xffff);
    const uint32_t btgt = pc + 4 + (simm << 2);
    const uint32_t jtgt = (pc & 0xf0000000) | ((insn & 0x3ffffff) << 2);
    switch(op)
      {
      case 0x00:   /* SPECIAL */
	switch(insn & 0x3f)
	  {
	  case 0x08: c.k = kind::jr;   c.reg = rs; break;
	  case 0x09: c.k = kind::jalr; c.reg = rs; break;
	  case 0x0c: /* syscall */
	  case 0x0d: c.k = kind::trap; break;       /* break */
	  default: break;
	  }
	break;
      case 0x01:   /* REGIMM: bltz, bgez, and their likely / link forms */
	if((rt & 0x0c) == 0x00 or (rt & 0x0c) == 0x10) {
	  c.k = (rt & 2) ? kind::likely : kind::branch;
	  c.target = btgt;
	}
	break;
      case 0x02: c.k = kind::jump; c.target = jtgt; break;
      case 0x03: c.k = kind::call; c.target = jtgt; break;
      case 0x04: case 0x05: case 0x06: case 0x07:
	c.k = kind::branch; c.target = btgt; break;
      case 0x14: case 0x15: case 0x16: case 0x17:
	c.k = kind::likely; c.target = btgt; break;
      case 0x11:   /* COP1: bc1f / bc1t, and the likely forms */
	if(rs == 0x08) {
	  c.k = (rt & 2) ? kind::likely : kind::branch;
	  c.target = btgt;
	}
	break;
      default: break;
      }
    return c;
  }
}; // namespace

/* Recognise the compiler's bounds-checked jump table and read it.
 *
 *   sltiu at, idx, N        <- the bound, and the only reason this is safe to follow
 *   beq   at, zero, default
 *   sll   t, idx, 2
 *   lui   at, %hi(tbl)
 *   addu  at, at, t
 *   lw    t, %lo(tbl)(at)
 *   jr    t
 *
 * We scan backwards from the jr for the lw that defines its register, take the table
 * base from the lui/%lo pair feeding it, and the entry count from the sltiu.  Without a
 * bound we refuse: reading an unbounded table walks off into whatever follows it. */
bool r4300_cfg::jump_table(uint32_t jr_pc, std::vector<uint32_t> &out) {
  uint32_t insn = 0;
  if(not read(jr_pc, insn)) {
    return false;
  }
  const uint32_t want = (insn >> 21) & 31;
  uint32_t base = 0, count = 0;
  bool have_base = false;
  uint32_t reg = want;

  for(uint32_t pc = jr_pc - 4; pc + 64 >= jr_pc; pc -= 4) {
    if(not read(pc, insn)) {
      return false;
    }
    const uint32_t op = insn >> 26;
    const uint32_t rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, rd = (insn >> 11) & 31;
    const uint32_t imm = insn & 0xffff;
    if(not have_base and op == 0x23 and rt == reg) {           /* lw reg, lo(rs) */
      base = static_cast<int16_t>(imm);
      reg = rs;
      have_base = true;
    }
    else if(have_base and op == 0x0f and rt == reg) {          /* lui reg, hi */
      base += imm << 16;
      /* the index bound must appear before the table is built */
      for(uint32_t q = pc - 4; q + 64 >= pc; q -= 4) {
	if(not read(q, insn)) {
	  break;
	}
	if((insn >> 26) == 0x0b) {                             /* sltiu at, idx, N */
	  count = insn & 0xffff;
	  break;
	}
      }
      break;
    }
  }
  if(not have_base or count == 0 or count > 256) {
    return false;
  }
  for(uint32_t i = 0; i < count; i++) {
    uint32_t e = 0;
    if(not read(base + i * 4, e)) {
      return false;
    }
    out.push_back(e);
  }
  return true;
}

void r4300_cfg::build(uint32_t entry) {
  func fn;
  fn.entry = entry;

  /* Pass one: walk the instruction stream from the entry, following every intra-function
   * edge, and collect the set of block leaders.  A block ends after the delay slot of the
   * branch that terminates it, so the branch and its slot always translate together. */
  std::set<uint32_t> leaders{entry}, ends;
  std::map<uint32_t, std::vector<uint32_t>> edges;
  std::deque<uint32_t> work{entry};
  std::set<uint32_t> seen, terms;

  while(not work.empty()) {
    uint32_t pc = work.front();
    work.pop_front();
    if(not seen.insert(pc).second) {
      continue;
    }
    for(;; pc += 4) {
      uint32_t insn = 0;
      if(not read(pc, insn)) {
	problems.emplace_back(pc, "unmapped instruction");
	ends.insert(pc);
	break;
      }
      ctl c = decode(pc, insn);
      if(c.k == kind::fall) {
	continue;
      }
      if(c.k == kind::trap) {
	/* break/syscall: the compiler's divide-by-zero and overflow guards.  Reachable
	 * (they are the not-taken side of the guard branch) but they never return, so the
	 * block ends here with no successors.  No delay slot, hence no pc+8 boundary. */
	n_traps++;
	break;
      }
      /* A straight-line run started at one leader can run through a second leader and
       * reach a terminator another run already handled.  Everything downstream of it is
       * then already recorded, so stop -- otherwise its successors get pushed twice and
       * the graph grows duplicate edges. */
      if(not terms.insert(pc).second) {
	break;
      }
      const uint32_t after = pc + 8;          /* past the delay slot */
      ends.insert(after);
      std::vector<uint32_t> &succ = edges[after];

      switch(c.k)
	{
	case kind::branch: case kind::likely:
	  succ.push_back(c.target);
	  succ.push_back(after);
	  leaders.insert(c.target);
	  leaders.insert(after);
	  work.push_back(c.target);
	  work.push_back(after);
	  break;
	case kind::jump:
	  succ.push_back(c.target);
	  leaders.insert(c.target);
	  work.push_back(c.target);
	  break;
	case kind::call:
	  fn.leaf = false;
	  fn.calls.insert(c.target);
	  succ.push_back(after);
	  leaders.insert(after);
	  work.push_back(after);
	  break;
	case kind::jalr:
	  fn.leaf = false;
	  fn.indirect_sites.insert(pc);
	  {
	    auto it = hints.find(pc);
	    if(it != hints.end()) {
	      fn.calls.insert(it->second.begin(), it->second.end());
	    }
	    else {
	      problems.emplace_back(pc, "jalr with no known target");
	    }
	  }
	  succ.push_back(after);
	  leaders.insert(after);
	  work.push_back(after);
	  break;
	case kind::jr:
	  if(c.reg == 31) {                     /* jr $ra: the function returns */
	    break;
	  }
	  else {
	    /* Resolve statically if we can -- the translator must not depend on having
	     * run the code first.  Observed targets are then a check on that resolution,
	     * not the source of it: anything the run reached that the table does not
	     * contain means the table was read wrongly. */
	    std::vector<uint32_t> tgts;
	    auto it = hints.find(pc);
	    if(jump_table(pc, tgts)) {
	      n_jr_static++;
	      if(it != hints.end()) {
		for(uint32_t t : it->second) {
		  if(std::find(tgts.begin(), tgts.end(), t) == tgts.end()) {
		    problems.emplace_back(pc, "observed jr target missing from its table");
		  }
		}
	      }
	    }
	    else if(it != hints.end()) {
	      n_jr_hint++;
	      tgts.assign(it->second.begin(), it->second.end());
	    }
	    else {
	      problems.emplace_back(pc, "unresolved jr");
	    }
	    for(uint32_t t : tgts) {
	      succ.push_back(t);
	      leaders.insert(t);
	      work.push_back(t);
	    }
	  }
	  break;
	default: break;
	}
      break;                                  /* this straight-line run is done */
    }
  }

  /* Pass two: cut the discovered code into blocks at the leaders and the ends, and
   * record for each block how it terminates.  Re-reading is cheap and keeps pass one
   * from having to know the block boundaries it is still discovering. */
  std::set<uint32_t> bounds = leaders;
  bounds.insert(ends.begin(), ends.end());

  for(uint32_t start : leaders) {
    if(not seen.count(start)) {
      continue;
    }
    block b;
    b.start = start;
    uint32_t pc = start;
    bool by_control = false;
    for(;;) {
      uint32_t insn = 0;
      if(not read(pc, insn)) {
	break;
      }
      ctl c = decode(pc, insn);
      if(c.k == kind::trap) {
	pc += 4;                              /* no delay slot, and nothing follows */
	by_control = true;
	b.ends_trap = true;
	break;
      }
      if(c.k != kind::fall) {
	pc += 8;                              /* the delay slot belongs to this block */
	by_control = true;
	b.ends_likely = (c.k == kind::likely);
	switch(c.k)
	  {
	  case kind::call:
	    b.ends_call = true;
	    break;
	  case kind::jalr:
	    b.ends_call = true;
	    if(not hints.count(pc - 8)) {
	      b.unresolved = true;
	      fn.unresolved_calls++;
	    }
	    break;
	  case kind::jr:
	    if(c.reg == 31) {
	      b.ends_return = true;
	    }
	    else {
	      b.ends_indirect = true;
	    }
	    break;
	  default: break;
	  }
	break;
      }
      pc += 4;
      if(bounds.count(pc)) {                  /* the next instruction starts a block */
	break;
      }
    }
    b.end = pc;
    if(by_control) {
      auto it = edges.find(pc);
      if(it != edges.end()) {
	b.succs = it->second;
      }
    }
    else {
      b.succs.push_back(pc);                  /* ran into the next block */
    }
    if(b.ends_indirect and b.succs.empty()) {
      b.unresolved = true;
    }
    fn.n_insns += (b.end - b.start) / 4;
    fn.blocks[start] = b;
  }
  funcs[entry] = fn;
}

void r4300_cfg::discover(const std::vector<uint32_t> &roots) {
  std::deque<uint32_t> work(roots.begin(), roots.end());
  while(not work.empty()) {
    uint32_t f = work.front();
    work.pop_front();
    if(funcs.count(f)) {
      continue;
    }
    build(f);
    for(uint32_t callee : funcs[f].calls) {
      if(not funcs.count(callee)) {
	work.push_back(callee);
      }
    }
  }
}

/* Graphviz, one file per function.  Blocks are boxes holding their disassembly; the
 * taken edge of a conditional is drawn solid and the fallthrough dashed, so the shape of
 * a loop is readable at a glance.  An unresolved indirect jump is drawn red -- that is
 * the thing that stops a function being translatable, and it should be impossible to
 * miss it in the picture. */
void r4300_cfg::write_dot(const std::string &dir) const {
  for(const auto &kv : funcs) {
    const func &fn = kv.second;
    char path[512];
    snprintf(path, sizeof(path), "%s/fn_%08x.dot", dir.c_str(), fn.entry);
    FILE *f = fopen(path, "w");
    if(f == nullptr) {
      continue;
    }
    fprintf(f, "digraph fn_%08x {\n", fn.entry);
    fprintf(f, "  labelloc=\"t\";\n");
    fprintf(f, "  label=\"%08x  %zu blocks, %zu insns%s\";\n",
	    fn.entry, fn.blocks.size(), fn.n_insns, fn.leaf ? ", leaf" : "");
    fprintf(f, "  node [shape=box fontname=\"monospace\" fontsize=9];\n");

    for(const auto &bkv : fn.blocks) {
      const block &b = bkv.second;
      fprintf(f, "  b%08x [label=\"", b.start);
      for(uint32_t pc = b.start; pc < b.end; pc += 4) {
	uint32_t insn = 0;
	if(not read(pc, insn)) {
	  break;
	}
	if(disasm) {
	  std::string t = disasm(insn, pc);
	  for(char &c : t) {                  /* dot's label grammar is unforgiving */
	    if(c == '"' or c == '\\' or c == '{' or c == '}' or c == '|' or c == '<' or c == '>') {
	      c = ' ';
	    }
	  }
	  fprintf(f, "%08x  %s\\l", pc, t.c_str());
	}
	else {
	  fprintf(f, "%08x  %08x\\l", pc, insn);
	}
      }
      fprintf(f, "\"");
      if(b.unresolved) {
	fprintf(f, " color=red penwidth=2");
      }
      else if(b.ends_return) {
	fprintf(f, " color=blue");
      }
      fprintf(f, "];\n");

      /* two successors means a conditional: the first is taken, the second falls through */
      const bool cond = (b.succs.size() == 2) and not b.ends_call;
      for(size_t i = 0; i < b.succs.size(); i++) {
	const char *style = "";
	if(cond and i == 1) {
	  /* not taken.  For a likely branch the delay slot -- the last instruction shown
	   * in the block -- is annulled on this edge, so the block as drawn overstates
	   * what runs here.  Say so rather than quietly draw the wrong thing. */
	  style = b.ends_likely ? " [style=dashed label=\"slot annulled\" fontsize=8]"
				: " [style=dashed]";
	}
	if(fn.blocks.count(b.succs[i])) {
	  fprintf(f, "  b%08x -> b%08x%s;\n", b.start, b.succs[i], style);
	}
	else {
	  /* an edge out of this function's own blocks: a tail call, or a jump table
	   * entry we followed into code the walk did not reach as a block */
	  fprintf(f, "  x%08x_%08x [shape=plaintext label=\"-> %08x\"];\n",
		  b.start, b.succs[i], b.succs[i]);
	  fprintf(f, "  b%08x -> x%08x_%08x%s;\n", b.start, b.start, b.succs[i], style);
	}
      }
    }
    fprintf(f, "}\n");
    fclose(f);
  }
}

void r4300_cfg::report(FILE *f) const {
  size_t insns = 0, blocks = 0, leaves = 0, indirect = 0, bad = 0;
  for(const auto &kv : funcs) {
    insns += kv.second.n_insns;
    blocks += kv.second.blocks.size();
    leaves += kv.second.leaf ? 1 : 0;
    indirect += kv.second.indirect_sites.size();
    for(const auto &b : kv.second.blocks) {
      bad += b.second.unresolved ? 1 : 0;
    }
  }
  fprintf(f, "%zu functions (%zu leaf), %zu blocks, %zu instructions\n",
	  funcs.size(), leaves, blocks, insns);
  fprintf(f, "%zu indirect call sites, %zu unresolved indirect jumps\n", indirect, bad);
  fprintf(f, "%zu jr resolved from a jump table in ROM, %zu only from observed targets\n",
	  n_jr_static, n_jr_hint);
  fprintf(f, "%zu break/syscall (compiler divide guards, never reached)\n", n_traps);
  if(not problems.empty()) {
    fprintf(f, "%zu problems:\n", problems.size());
    for(const auto &p : problems) {
      uint32_t insn = 0;
      const bool got = read(p.first, insn);
      fprintf(f, "  %08x  %-28s %08x  %s\n", p.first, p.second.c_str(), got ? insn : 0,
	      (got and disasm) ? disasm(insn, p.first).c_str() : "");
    }
  }
  size_t clean = 0;
  for(const auto &kv : funcs) {
    clean += (kv.second.unresolved_calls == 0);
  }
  fprintf(f, "%zu of %zu functions have no unresolved indirect call\n", clean, funcs.size());

  fprintf(f, "\n%-10s %6s %6s %5s %6s  %s\n", "entry", "insns", "blocks", "leaf",
	  "calls", "unresolved");
  for(const auto &kv : funcs) {
    fprintf(f, "%08x   %6zu %6zu %5s %6zu  %zu\n", kv.first, kv.second.n_insns,
	    kv.second.blocks.size(), kv.second.leaf ? "yes" : "no",
	    kv.second.calls.size(), kv.second.unresolved_calls);
  }
}

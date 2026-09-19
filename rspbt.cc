#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <chrono>

#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/raw_ostream.h"

#include "rspbt.hh"
#include "rspFunc.hh"
#include "rspInstruction.hh"
#include "rsp.hh"

#pragma GCC diagnostic ignored "-Winvalid-offsetof"

/* ---- callbacks from translated code into the interpreter's device/slow paths ---- */
extern "C" {
  uint32_t rspbt_mfc0(rsp_t *c, uint32_t rd) { return c->mfc0(rd); }
  void rspbt_mtc0(rsp_t *c, uint32_t rd, uint32_t x) { c->mtc0(rd, x); }
  void rspbt_lwc2(rsp_t *c, uint32_t insn) { c->lwc2(insn); }
  void rspbt_swc2(rsp_t *c, uint32_t insn) { c->swc2(insn); }
  uint32_t rspbt_rd(rsp_t *c, uint32_t a, uint32_t n) { return (n == 2) ? c->rd16(a) : c->rd32(a); }
  void rspbt_wr(rsp_t *c, uint32_t a, uint32_t x, uint32_t n) {
    if(n == 2) {
      c->wr16(a, static_cast<uint16_t>(x));
    }
    else {
      c->wr32(a, x);
    }
  }
}

typedef uint32_t (*rsp_fn_t)(rsp_t *, uint32_t);

struct image_t {
  rsp_fn_t fn = nullptr;
  uint8_t imem[0x1000];            /* what this translation was made from */
  std::set<uint32_t> roots;        /* everything a jr may land on */
  bool stale = true;
  unsigned generation = 0;
};

struct rspbt::impl {
  std::unique_ptr<llvm::orc::LLJIT> jit;
  std::unique_ptr<llvm::TargetMachine> tm;
  std::map<uint64_t, image_t> cache;
  image_t *last = nullptr;         /* tasks nearly always reuse the previous image */

  impl();
  void compile(rspbt &owner, image_t &img, uint64_t hash, const uint8_t *imem);
};

static void die(const char *what, llvm::Error err) {
  fprintf(stderr, "rspbt: %s: %s\n", what, llvm::toString(std::move(err)).c_str());
  exit(-1);
}

rspbt::impl::impl() {
  llvm::InitializeNativeTarget();
  llvm::InitializeNativeTargetAsmPrinter();
  auto jtmb = llvm::orc::JITTargetMachineBuilder::detectHost();   /* host CPU + features */
  if(not(jtmb)) {
    die("detectHost", jtmb.takeError());
  }
  jtmb->setCodeGenOptLevel(llvm::CodeGenOptLevel::Aggressive);
  auto t = jtmb->createTargetMachine();
  if(not(t)) {
    die("createTargetMachine", t.takeError());
  }
  tm = std::move(*t);
  auto j = llvm::orc::LLJITBuilder().setJITTargetMachineBuilder(*jtmb).create();
  if(not(j)) {
    die("LLJIT", j.takeError());
  }
  jit = std::move(*j);

  llvm::orc::SymbolMap syms;
  auto add = [&](const char *name, uintptr_t p) {
    syms[jit->mangleAndIntern(name)] = llvm::orc::ExecutorSymbolDef(llvm::orc::ExecutorAddr(p), llvm::JITSymbolFlags::Exported);
  };
  add("rspbt_mfc0", reinterpret_cast<uintptr_t>(&rspbt_mfc0));
  add("rspbt_mtc0", reinterpret_cast<uintptr_t>(&rspbt_mtc0));
  add("rspbt_lwc2", reinterpret_cast<uintptr_t>(&rspbt_lwc2));
  add("rspbt_swc2", reinterpret_cast<uintptr_t>(&rspbt_swc2));
  add("rspbt_rd", reinterpret_cast<uintptr_t>(&rspbt_rd));
  add("rspbt_wr", reinterpret_cast<uintptr_t>(&rspbt_wr));
  if(auto err = jit->getMainJITDylib().define(llvm::orc::absoluteSymbols(syms))) {
    die("define helpers", std::move(err));
  }
}

static uint32_t fetch(const uint8_t *imem, uint32_t a) {
  a &= 0xffc;
  return (static_cast<uint32_t>(imem[a]) << 24) | (imem[a+1] << 16) | (imem[a+2] << 8) | imem[a+3];
}

static uint64_t fnv1a64(const uint8_t *p, size_t n) {
  uint64_t h = 0xcbf29ce484222325ULL;
  for(size_t i = 0; i < n; i++) {
    h = (h ^ p[i]) * 0x100000001b3ULL;
  }
  return h;
}

void rspbt::impl::compile(rspbt &owner, image_t &img, uint64_t hash, const uint8_t *imem) {
  /* ---- static walk.  Pass 1 finds every reachable instruction and every block
   * leader; roots grow as jal/jalr return sites are found. ---- */
  std::map<uint32_t, std::unique_ptr<rspInsn>> insns;
  auto at = [&](uint32_t a) -> rspInsn* {
    a &= 0xffc;
    auto it = insns.find(a);
    if(it == insns.end()) {
      it = insns.emplace(a, std::unique_ptr<rspInsn>(rspInsn::decode(fetch(imem, a), a))).first;
    }
    return it->second.get();
  };
  /* a control instruction is translatable only with a plain instruction in its delay slot */
  auto translatable = [&](uint32_t a) -> bool {
    rspInsn *i = at(a);
    if(i == nullptr) {
      return false;
    }
    if(i->isControl()) {
      rspInsn *d = at(a + 4);
      return d != nullptr and not(d->isControl()) and not(d->isBreak());
    }
    return true;
  };

  /* the codegen context owns the call-string table, so it exists from the start */
  auto ctx = std::make_unique<llvm::LLVMContext>();
  char name[64];
  snprintf(name, sizeof(name), "rsp_%016lx_g%u", hash, img.generation++);
  auto mod = std::make_unique<llvm::Module>(name, *ctx);
  llvm::IRBuilder<> builder(*ctx);
  rspFunc f(*ctx, mod.get(), &builder);

  typedef rspFunc::blockKey key_t;               /* (address, call-string context) */
  std::set<key_t> leaders, walked;
  std::vector<key_t> work;
  bool rootsChanged = true;
  while(rootsChanged) {
    rootsChanged = false;
    for(uint32_t r : img.roots) {
      if(leaders.insert(key_t(r, 0)).second) {
	work.push_back(key_t(r, 0));
      }
    }
    while(not(work.empty())) {
      uint32_t a = work.back().first & 0xffc, cx = work.back().second;
      work.pop_back();
      while(true) {
	if(walked.count(key_t(a, cx))) {
	  leaders.insert(key_t(a, cx));          /* fell into code already seen: a join point */
	  break;
	}
	walked.insert(key_t(a, cx));
	if(not(translatable(a))) {
	  break;
	}
	rspInsn *i = at(a);
	if(i->isBreak()) {
	  break;
	}
	if(i->isControl()) {
	  std::vector<key_t> succ;
	  std::vector<uint32_t> tgts;
	  i->staticTargets(tgts);
	  uint32_t link = (a + 8) & 0xffc;
	  for(uint32_t t : tgts) {
	    succ.push_back(key_t(t, i->isCall() ? f.pushCtx(cx, link) : cx));
	  }
	  if(i->fallsThrough()) {
	    succ.push_back(key_t(link, cx));
	  }
	  if(i->isIndirect() and cx != 0) {      /* a return: back to the caller's context */
	    succ.push_back(key_t(f.contexts[cx].back(), f.popCtx(cx)));
	  }
	  if(i->makesLinkRoot() and img.roots.insert(link).second) {
	    rootsChanged = true;
	  }
	  for(const key_t &k : succ) {
	    if(leaders.insert(k).second) {
	      work.push_back(k);
	    }
	  }
	  break;
	}
	a = (a + 4) & 0xffc;
      }
    }
  }
  /* a root that turned out to be data is not a legal target after all */
  for(auto it = img.roots.begin(); it != img.roots.end(); ) {
    it = translatable(*it) ? std::next(it) : img.roots.erase(it);
  }

  /* ---- pass 2: one LLVM function for the image ---- */
  mod->setDataLayout(tm->createDataLayout());
  mod->setTargetTriple(tm->getTargetTriple().str());
  f.roots.assign(img.roots.begin(), img.roots.end());

  llvm::FunctionType *fty = llvm::FunctionType::get(f.i32, {f.ptr, f.i32}, false);
  f.fn = llvm::Function::Create(fty, llvm::Function::ExternalLinkage, name, mod.get());
  /* the accumulators are <8 x i64>: make sure 512-bit vectors are legal, not split */
  f.fn->addFnAttr("min-legal-vector-width", "512");
  f.fn->addFnAttr("prefer-vector-width", "512");
  f.vState = f.fn->getArg(0);

  llvm::BasicBlock *entryBB = llvm::BasicBlock::Create(*ctx, "entry", f.fn);
  f.exitBB = llvm::BasicBlock::Create(*ctx, "exit", f.fn);
  builder.SetInsertPoint(entryBB);
  for(int i = 1; i < 32; i++) {
    f.gpr[i] = builder.CreateAlloca(f.i32);
  }
  for(int i = 0; i < 32; i++) {
    f.vec[i] = builder.CreateAlloca(f.v16);
  }
  f.acc = builder.CreateAlloca(f.v64);
  for(int i = 0; i < rspFunc::NUM_FLAGS; i++) {
    f.flag[i] = builder.CreateAlloca(f.v1);
  }
  f.retSlot = builder.CreateAlloca(f.i32);
  const size_t flagOffs[rspFunc::NUM_FLAGS] = {offsetof(rsp_t, vco_l), offsetof(rsp_t, vco_h), offsetof(rsp_t, vcc_l),
					      offsetof(rsp_t, vcc_h), offsetof(rsp_t, vce)};
  for(int i = 1; i < 32; i++) {
    builder.CreateStore(builder.CreateLoad(f.i32, f.statePtr(offsetof(rsp_t, r) + 4*i)), f.gpr[i]);
  }
  for(int i = 0; i < 32; i++) {
    builder.CreateStore(builder.CreateAlignedLoad(f.v16, f.statePtr(offsetof(rsp_t, v) + 16*i), llvm::MaybeAlign(2)), f.vec[i]);
  }
  builder.CreateStore(builder.CreateAlignedLoad(f.v64, f.statePtr(offsetof(rsp_t, acc)), llvm::MaybeAlign(8)), f.acc);
  for(int i = 0; i < rspFunc::NUM_FLAGS; i++) {
    llvm::Value *bytes = builder.CreateAlignedLoad(f.vb8, f.statePtr(flagOffs[i]), llvm::MaybeAlign(1));
    builder.CreateStore(builder.CreateICmpNE(bytes, llvm::Constant::getNullValue(f.vb8)), f.flag[i]);
  }
  f.curCtx = 0;
  f.indirectJump(builder.CreateAnd(f.fn->getArg(1), f.c32(0xffc)));   /* enter like a dispatch jr */

  for(const key_t &leader : leaders) {
    f.curCtx = leader.second;
    builder.SetInsertPoint(f.blockFor(leader.first, leader.second));
    uint32_t a = leader.first;
    while(true) {
      if(not(translatable(a))) {
	f.exitToInterp(a);
	break;
      }
      rspInsn *i = at(a);
      if(i->isControl()) {
	i->generateIR(f, at(a + 4));
	break;
      }
      i->generateIR(f, nullptr);
      if(i->isBreak()) {
	break;
      }
      a = (a + 4) & 0xffc;
      if(leaders.count(key_t(a, f.curCtx))) {
	builder.CreateBr(f.blockFor(a));
	break;
      }
    }
  }
  f.curCtx = 0;
  /* blocks named by a branch but never walked cannot exist: every static target was
   * made a leader above.  Anything blockFor() created lazily for a root is a leader too. */

  builder.SetInsertPoint(f.exitBB);
  for(int i = 1; i < 32; i++) {
    builder.CreateStore(builder.CreateLoad(f.i32, f.gpr[i]), f.statePtr(offsetof(rsp_t, r) + 4*i));
  }
  for(int i = 0; i < 32; i++) {
    builder.CreateAlignedStore(builder.CreateLoad(f.v16, f.vec[i]), f.statePtr(offsetof(rsp_t, v) + 16*i), llvm::MaybeAlign(2));
  }
  builder.CreateAlignedStore(builder.CreateLoad(f.v64, f.acc), f.statePtr(offsetof(rsp_t, acc)), llvm::MaybeAlign(8));
  for(int i = 0; i < rspFunc::NUM_FLAGS; i++) {
    llvm::Value *bytes = builder.CreateZExt(builder.CreateLoad(f.v1, f.flag[i]), f.vb8);
    builder.CreateAlignedStore(bytes, f.statePtr(flagOffs[i]), llvm::MaybeAlign(1));
  }
  builder.CreateRet(builder.CreateLoad(f.i32, f.retSlot));

  std::string verr;
  llvm::raw_string_ostream vos(verr);
  if(llvm::verifyFunction(*f.fn, &vos)) {
    fprintf(stderr, "rspbt: generated IR does not verify:\n%s\n", vos.str().c_str());
    exit(-1);
  }
  auto dump = [&](const char *tag) {
    if(owner.dump_ir) {
      std::error_code ec;
      llvm::raw_fd_ostream os(std::string(name) + tag + ".ll", ec);
      mod->print(os, nullptr);
    }
  };
  dump("_in");

  llvm::LoopAnalysisManager lam;
  llvm::FunctionAnalysisManager fam;
  llvm::CGSCCAnalysisManager cgam;
  llvm::ModuleAnalysisManager mam;
  llvm::PassBuilder pb(tm.get());
  pb.registerModuleAnalyses(mam);
  pb.registerCGSCCAnalyses(cgam);
  pb.registerFunctionAnalyses(fam);
  pb.registerLoopAnalyses(lam);
  pb.crossRegisterProxies(lam, fam, cgam, mam);
  pb.buildPerModuleDefaultPipeline(llvm::OptimizationLevel::O2).run(*mod, mam);
  dump("_opt");

  if(auto err = jit->addIRModule(llvm::orc::ThreadSafeModule(std::move(mod), std::move(ctx)))) {
    die("addIRModule", std::move(err));
  }
  auto sym = jit->lookup(name);
  if(not(sym)) {
    die("lookup", sym.takeError());
  }
  img.fn = sym->toPtr<rsp_fn_t>();
  img.stale = false;
  owner.n_compiles++;
  fprintf(stderr, "rspbt: image %016lx: %zu instructions in %zu call-string contexts, %zu blocks, %zu dispatch roots\n",
	  hash, walked.size(), f.contexts.size(), leaders.size(), img.roots.size());
}

rspbt::rspbt() : p(new impl()) {}

rspbt::~rspbt() {
  delete p;
}

void rspbt::run(rsp_t &c, uint32_t pc, const std::vector<uint32_t> &hint_roots) {
  const uint8_t *imem = c.mem + 0x1000;
  /* memoized per instruction-memory image.  A memcmp against the image used last time
   * is far cheaper than hashing 4 KB on every task, and is also what makes a hash
   * collision harmless for the common case. */
  uint64_t hash = 0;
  if(p->last == nullptr or memcmp(p->last->imem, imem, 0x1000) != 0) {
    hash = fnv1a64(imem, 0x1000);
    p->last = &p->cache[hash];
    memcpy(p->last->imem, imem, 0x1000);
  }
  image_t &img = *p->last;
  if(img.roots.insert(pc & 0xffc).second) {
    img.stale = true;
  }
  for(uint32_t h : hint_roots) {
    if(img.roots.insert(h & 0xffc).second) {
      img.stale = true;
    }
  }
  if(img.stale) {
    auto t0 = std::chrono::steady_clock::now();
    p->compile(*this, img, fnv1a64(imem, 0x1000), imem);
    compile_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  c.halted = false;
  uint32_t next = img.fn(&c, pc);
  if(next != 0xffffffffu) {
    /* a jr nobody anticipated (or an untranslatable word): finish in the interpreter
     * and make the target a root for next time */
    n_fallbacks++;
    if(img.roots.insert(next & 0xffc).second) {
      img.stale = true;
    }
    c.run(next);
  }
}

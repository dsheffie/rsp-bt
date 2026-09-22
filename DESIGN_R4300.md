# Translating the R4300 side

rsp-bt translates RSP microcode: one image, 4 KB of it, no calls worth the name, and
one LLVM function per image.  The R4300 side of the same project -- GoldenEye's copy
of libultra's audio library -- is a different shape, and this is the design for it.

Everything below is measured, from GoldenEye's own code, reachable from `alAudioFrame`.

## What we are translating

| | |
|---|---|
| Instructions reachable | 7,200 (28 KB) |
| Functions | 256 |
| Distinct instructions executed | 54 |
| Leaf functions | 148, median 17 instructions |
| Non-leaf functions | 179, all of which save the return address |

So: a few hundred small functions, not one big loop.  That single fact decides the
architecture.

## One host function per guest function

`jal` becomes a real call and `jr $ra` a real return.  The alternative -- one big
region with the return address compared against a list of possible callers -- was
tried on paper and is badly suited here:

- 68 return sites with 155 distinct targets between them;
- 30 of those sites are polymorphic, one with twelve callers.

Those are shared helpers.  Specialising by call string would clone each of them once
per caller; real calls cost nothing and the hardware's return predictor handles them.
The code also respects the o32 ABI throughout (179 of 179 non-leaves save `ra`), so
the mapping onto host calls is honest rather than a guess.

Leaves are emitted as ordinary internal functions and left to LLVM's inliner.  Doing
it by hand would mean 3.5x code growth taken unconditionally; the inliner makes the
same decision per site with a cost model, and collapses a 17-instruction helper called
from 23 places without our help.

## Guest state does not escape

Measured, from `alAudioFrame` over forty frames: the static walk covers 7,202
instructions, 3,157 distinct pcs actually execute, and **none of them is outside the
walk**.  Three more are inside our own DMA stub, which we wrote and can model
directly rather than leaving for.

So the translated call graph is closed, and guest state never has to exist in the
interpreter's layout while inside it.  Registers become ordinary SSA values flowing
through the whole graph; `state_t` is materialised only where state genuinely
escapes:

- the return to the harness, once per frame -- 33,000 instructions of work;
- an interpreter bailout, for a path this workload never takes;
- a guest exception, which cannot happen here (interrupts are off).

One materialisation per frame makes its cost irrelevant, which is why internal
functions can use any convention we like -- arguments in registers, `v0` as a return
value, whatever the host prefers.

Closure is a property of this workload, not a theorem.  The static walk deliberately
covers more than any trace, and a path into the unresolved frontier would escape.
Materialisation must therefore be correct and available at every bailout, even though
it should essentially never fire.  The escape points are few enough to enumerate and
test individually.

## Pointers into the function

`void fn_<addr>(state_t *s, uint8_t *noalias ram)`.

The second argument is the decision that is hard to undo.  The executed mix is about
37% memory operations -- 242k loads and 236k stores out of 1.3M.  Reached only
through `s`, a guest store might overlap `s->gpr` as far as LLVM can tell, so every
store would force the guest registers to be reloaded and mem2reg could not keep them
in host registers across the thing this code does most.  Guest RAM is a separate
mapping from the `state_t` object, so declaring them disjoint is true rather than
convenient.

Do **not** pass `gpr` and `cpr1` as separate `noalias` pointers alongside `s`: they
are members of `state_t`, so the promise would be false and LLVM is entitled to
miscompile on it.  The offset from `s` is free anyway -- it folds into the addressing
mode -- and passing the RAM base directly also removes a dependent load, since
reaching it otherwise costs `s->mem` and then `sparse_mem::mem`.

## Indirect control flow

Three kinds, all resolvable, none needing a profile:

**`jalr` -- 28 sites.**  27 are monomorphic and the worst has two targets.  Emit a
compare against the expected address and a direct call, with an indirect path behind
it.  The pointers live in RAM, written once during initialisation and never touched
during the frame loop (verified by watching every store), so the expected value is
read once after init.  A write to one of those slots must invalidate the translation;
the same store watch is the mechanism.

**`jr` to a jump table -- 5 sites.**  Each is `sltiu` against a bound, then a load
from a table that lives in the ROM's own data segment.  The tables hold between 5 and
16 distinct targets and are readable before anything executes.  They are
intra-procedural -- switch statements, not calls -- so they become an LLVM `switch`
inside the function and never interact with the call design.  Read the table; do not
observe it.  Every table contains strictly more targets than any trace exercised.

**`jr $ra` -- 68 sites.**  Returns.  Nothing to do.

## The interpreter is the floor, not a fallback of last resort

Any call we cannot resolve -- an unresolved `jalr`, a callee not yet translated --
becomes a call to a helper that interprets one guest function and returns.  That
bounds the damage to a function rather than poisoning a region, and it is why the
convention above keeps all state in memory.

This matters more than it first appears: 11 of 14 known indirect targets contain
their own `jalr`, so resolving the frontier is iterative and may not close at all for
paths a given workload never takes.  The translator must be correct when it stops
early.

## Discovery

A worklist, seeded with the entry points, that alternates between:

1. walking a function's control flow to its returns, following direct branches and
   `jal`;
2. resolving indirect sites -- tables from the image, `jalr` pointers from RAM after
   init -- and adding what they reach.

Run to a fixed point or a budget.  Anything still unresolved exits to the interpreter.

Do not seed this from an execution trace.  A trace is a lower bound and mistaking it
for the truth cost this project two wrong answers already; every jump table found
this way contained targets no trace had reached.  Traces are a cross-check.

### What the walk actually finds

`r4300cfg.cc` implements the above; `gemusic/tests/r4300_cfg` drives it and dumps one
Graphviz file per function.  Walking from `alAudioFrame`, with `jalr` pointers taken from
a run over all 63 tunes:

| | |
|---|---|
| functions | 86 (36 leaf) |
| basic blocks | 1512 |
| instructions | 7208 |
| `jr` resolved by reading its table out of ROM | 6 of 6 |
| `jr` needing an observed target | 0 |
| `jalr` sites | 52, of which 27 are ever taken |
| functions with no unresolved indirect call | 76 of 86 |
| `break`/`syscall` | 16, all compiler divide guards on a never-taken edge |

The closed-world claim is checked directly rather than assumed: the tool records every pc
a real run executes and asserts each one falls inside the static walk.  **0 outside**, for
every tune and at 2, 30, 40 and 120 frames -- including runs reaching 3641 distinct pcs,
more than the walk was ever shown.  The 25 `jalr` sites no tune takes are dead paths in
libaudio; they live in 10 functions and are where those functions exit to the interpreter.

Every `jr` resolves from a table in ROM with no dynamic help, which is what makes this a
static translator rather than a trace recorder.  Observed targets are used only to check
those tables, and none disagreed.

### Branch-likely

229 of the 7208 instructions are branch-likely (`beql`, `bnel`, ...), and **not one of
them has a nop in the delay slot** -- every slot carries real work.  A likely branch
annuls its slot when not taken, so the slot cannot simply be emitted into the block that
contains it.

Do not push the slot into a block on the taken edge.  Most of these taken edges are loop
back-edges, so that means inserting a new predecessor on a loop header and fixing up its
phis, 229 times, to no benefit.  **If-convert on the branch condition instead** (the
21264 compiler writer's guide scheme): execute the slot unconditionally and select its
destination.

What the slots actually are:

| | |
|---|---|
| loads (`lw` 118, `lbu` 13, `lh` 6, `lwc1` 3) | 140 |
| register-only (ALU, `mfc1`/`mtc1`/`cfc1`) | 79 |
| **stores** (`sw` 9, `sb` 1) | **10** |

- **Register-only** and **loads**: compute, then `select` the destination on the branch
  condition.  A squashed load needs no guarding beyond that -- guest addresses are masked
  into the flat backing store, so it cannot fault the host.
- **Stores** are the only case needing a safe address, and there are ten of them:
  `store val, ram + select(cond, offset, BITBUCKET)`.

**The bit bucket belongs in guest RAM, not in a host `alloca`.**  Selecting between
`ram + off` and a host object yields a pointer that may point into either, which forces
LLVM to treat those stores as aliasing anything -- undoing exactly the escape analysis
above that lets `s->gpr` stay in registers.  Reserving a guest address instead keeps the
select on the *offset*, so the store is unambiguously a store into `ram`, in the same
alias class as every other guest store.  `RAM_BITBUCKET` at 0x80700b00 (free between
`RAM_RETURN` and `RAM_CSPLAYER`).

Speculating the FP slots is safe here, which was checked rather than assumed: the code
does read FCSR (`cfc1 $t8, $31`), so a squashed op polluting a sticky flag would diverge.
interp_mips models no IEEE sticky flags at all -- its only FCSR write is the
Unimplemented-Op bit on the E-trap path, which covers `add`/`sub`/`mul`/`div`/`sqrt`, and
**none of those appear in any delay slot**.  The four FP slots are converts
(`trunc.w.d`, `trunc.w.s`, `cvt.s.d`, `cvt.d.s`) plus moves, which go through `_fp2int`
and cannot trap.  Select the destination; nothing else is needed.

The dumped graphs label the not-taken edge `slot annulled` so the block, which shows the
slot because that is where it sits in memory, cannot be misread as running it on both
paths.

## First slice: alAudioFrame

`r4300bt.cc` translates one guest function at a time to `uint32_t fn(state_t *, uint8_t *ram)`,
returning 0 if it reached its own `jr $ra` and otherwise the pc the interpreter resumes at.
GPRs are allocas (all 32 promoted by mem2reg: 566 loads become 156); the FP file stays in
`state_t`; every call goes out to the interpreter through `r4300bt_call`.

**Result: byte-identical audio on all 63 tunes**, translated against interpreted at the
same settings.  Do not check against the stored `music/wav/` renders -- looping tunes need
a `-t` limit, so those files only match the tune that ends on its own.

On its own this was not faster: alAudioFrame's body is about 1% of the dynamic
instruction count -- it is a driver, and the callees do the work -- so spilling the GPR
file at its 4,696 call sites cost about what the translated body saved.

## All of it

Translating the whole call graph is the same machinery with three additions: every
function goes into **one module**, a `jal` to a translated function becomes a **direct
call**, and a `jalr` compares its runtime target against the entries discovery saw there
and calls those directly.

The spill/reload around a call is still emitted naively.  That is deliberate: with the
call graph visible in one module the inliner takes the small callees, and the callee's
entry reload then sits directly on top of the caller's spill, where GVN deletes both.
What survives is exactly the calls that were not inlined, which are the ones that really
did need the transfer.

**Discovery needs to be told where the indirect calls go.**  A static walk with no
observed targets finds **4 functions out of 86** -- libaudio is reached almost entirely
through handler pointers.  So `ge_enable_r4300bt` runs 20 frames on a throwaway machine
first and watches the `jalr`s (a throwaway because doing it on the real machine would
advance the sequence; the code image is identical either way, which is what makes the
observations transferable).  One frame reaches 78 functions, 20 reaches all 86.

**Bails compose.**  A translated callee that gives up returns the pc it stopped at, and
the caller has the interpreter finish that callee until control returns to its own link
address.  So a bail anywhere in the call graph costs correctness nothing and only that
subtree's speed -- which is what made it practical to grow the opcode coverage by
measuring which bails were hot rather than by implementing MIPS III exhaustively:

| interpreted instructions per tune | after |
|---|---|
| 9,971,770 | alAudioFrame only |
| 1,997,856 | all 86 functions |
| 889,504 | + FP compare / `bc1`, `mult`/`multu`, `mfhi`/`mflo` |
| 179,336 | + `div`/`divu` |

**Result on 600 s of audio: the R4300 side goes from 10.47 s to 0.61 s, about 17x** over
the same 679,403,431 guest instructions.  End to end, with the RSP translated too,
11.58 s to 2.70 s including a one-time 1.02 s of warm-up, discovery and JIT -- 52x real
time to 222x.  All 63 tunes stay byte-identical.

Quote the ratio, not instructions per second.  The 17x is a fair number because it is the
same scalar instruction stream run two ways, but an absolute rate invites comparison with
the RSP's count, and those do not compare: 55% of executed RSP instructions are vector
ops, each doing eight 16-bit lanes, so the RSP performs roughly 13x the element-level work
of the R4300 while appearing to execute only 4x the instructions.  `-DRSP_WORK_COUNTERS`
in rsp-bt counts vector ALU and vector load/store to make that visible; it is off by
default because it sits in the interpreter's innermost loop.

### The FP file stays in memory (measured, not assumed)

The GPRs are allocas that mem2reg promotes; the FP file is left in `state_t`.  That looks
inconsistent, so it was tried the other way: 32 `i64` allocas, one per `cpr1` slot, with
the three access views (direct for arithmetic, the `(r&1)` half for `mtc1`/`lwc1`, the
even slot for `ldc1`) reading and writing that array.  The FR=0 pairing *is* the alloca
structure, so it is expressible, and it produced byte-identical audio.

It was worse on both counts:

| optimized IR | FP in memory | FP in SSA |
|---|---|---|
| gpr traffic | 10,635 | 10,603 |
| **cpr1 traffic** | **359** | **9,874** |
| fcr1 traffic | 15 | 311 |
| IR lines | 47,645 | 64,079 |
| LLVM O2 | 0.47 s | 0.71 s |
| render, 600 s of audio | 1.67 s | 1.88 s |

Promoting the FP file made its traffic **27x worse**.  The reason is that the cost is not
where the registers are read and written, it is at the call boundaries: leaving the file
in memory means a call needs no FP transfer at all, while promoting it adds 33 stores and
33 reloads to every call site the inliner did not eat, and there are far more of those
than there are FP operations.  Static counts had said FP was 2.7% of the surviving
traffic, and that was the right signal after all.

The lever worth pulling is the 10,635 GPR accesses, which are the same call boundaries.
That is a calling-convention question -- pass the live registers as arguments instead of
materialising all 31 in both directions -- not an SSA-construction one.

Two host traps have to be avoided rather than selected away, because unlike a wrong value
they cannot be discarded after the fact: a zero divisor (the divisor is substituted, the
result selected) and `INT_MIN / -1` (signed division is done in 64 bits, as interpret.cc
does).  The `mfc0` in libultra's interrupt disable/restore is left to the interpreter on
purpose -- CP0 writes have side effects worth more than the 3,382 calls they cost.

Two things worth knowing before extending it:

- **FP registers are FR=0.**  `interpret.cc` reaches the FP file two ways: arithmetic,
  `cvt` and `trunc` index `cpr1` directly, while the GPR<->FPR moves and FP loads/stores
  go through `fpr_hw()`/`fpr_dreg()`, which honour SR.FR.  Under FR=0 a single lives in
  the `(r&1)` half of slot `(r&~1)`, so `mtc1 $t,$f5` writes the **high half of $f4** --
  that pairing is how the compiler builds double constants.  Getting this wrong is what
  made the first attempt diverge seven frames in.  `ge_enable_r4300bt` refuses to
  translate if SR.FR is set.
- **Two interpreter behaviours are not the architectural ones** and are mirrored
  deliberately, because the interpreter is the reference: `addi` is a full 64-bit
  non-trapping add (it behaves like `daddiu`), and `jalr` always links `$31` whatever its
  `rd` field says.  `add`/`sub` do trap on overflow, so they are left to the interpreter
  rather than folded in with `addu`/`subu`.

## Memory and floating point

Addresses: kseg0/kseg1 mask, plus the game's one 4 MB TLB alias at 0x70000000 mapping
to physical 0.  Both are compile-time constant here -- the guest never writes the TLB
after that first entry -- so translate inline and call out only for anything else.

Floating point is FR=0: 32-bit registers, doubles in even/odd pairs.  This is not
optional.  The code builds double constants by writing halves to a register pair, and
an FR=1 model faults on the first one.  interp_mips's user-mode FP view already has
this layout; the full-system view does not.

## Verifying it

The bar is byte-identical audio against the interpreter, across all 59 tunes, which
is the same bar the RSP translator and the wasm build already meet.  Anything less is
not evidence.  A per-function differential harness -- run one function both ways from
the same state, compare registers and memory -- is worth having before trusting the
whole path.

## What this does not address

wasm, where there is no JIT.  The same static walk could feed an ahead-of-time
emitter, as with the microcode, but libaudio is larger and reaches through pointers
that only exist after initialisation, so that is a separate design.

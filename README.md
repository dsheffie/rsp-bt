# rsp-bt

An interpreter and an LLVM-based binary translator for the Nintendo 64 RSP: a 32-bit
MIPS integer subset plus a COP2 vector unit (8 x 16-bit lanes, a 48-bit accumulator per
lane) running from 4 KB of instruction memory against 4 KB of data memory.  "Microcode"
is ordinary MIPS machine code for that core.

**Provenance.**  This is modeled on David Sheffield's
[mips32-bt](https://github.com/dsheffie/mips32-bt), a profile-driven LLVM binary
translator for 32-bit MIPS, and was hacked together by Claude (Anthropic's AI model).  It
borrows that project's architecture and naming - one class per instruction with a
`generateIR()` method, a codegen context in the role of `regionCFG` - but no code was
carried over; the implementation here is new.  It does two things mips32-bt does not:
it translates *statically*, walking a whole instruction-memory image ahead of time
instead of forming regions from an execution profile, and it handles the RSP's bespoke
COP2 vector instructions (the 8 x 16-bit lanes, the 48-bit accumulators and their
clamping rules, the carry/compare/clip flags, and the byte-addressed vector loads and
stores).  See mips32-bt for the original design.

- `rsp.cc`, `rsp.hh` - the interpreter and the machine state (`rsp_t`).  Implements what
  rspboot and the stock audio microcode execute, and dies loudly on anything else.
- `rspInstruction.*` - one class per instruction with a `generateIR()` method, the shape
  of the `Insn` hierarchy in mips32-bt.  A control transfer is handed its delay slot.
- `rspFunc.*` - the codegen context (the role `regionCFG` plays in mips32-bt).
- `rspbt.*` - static CFG walk, function construction, O2 pipeline, ORC JIT, image cache.

## Design

**Static control flow.**  The walk starts from a set of roots and follows branches.  The
only indirect jump the RSP has is `jr`, and both of its uses are devirtualized:

- *Returns.*  Code is specialized by call string: `jal T` at `A` translates `T` in a
  context tagged with the return address `A+8`, and a `jr` inside that context gets a
  one-case switch back to the caller's context.  Inside the clone `r31` is one constant,
  so LLVM folds the switch into a direct branch, including the nested-call idiom
  `addi r5,r31,0 ... jr r5`.  For the audio microcode 15 of 17 switches fold away.
- *Dispatch.*  A `jr` outside any call context is a switch over the known roots: the
  entry, caller-supplied hints (e.g. a command table read from the microcode's data) and
  targets observed at run time.

Every switch defaults to leaving for the interpreter, which finishes the task and makes
the target a root for the next compile.  Hints and guesses therefore affect speed, never
correctness, and a translation depends only on instruction memory, never on data.

**One function per image, state in SSA.**  The function operates directly on `rsp_t`, so
interpreter and translated code are interchangeable mid-task.  GPRs, the 32 vector
registers (`<8 x i16>`), the accumulators (`<8 x i64>`) and the five flag sets
(`<8 x i1>`) are allocas promoted by mem2reg; with a static whole-image CFG hand-placed
phis would buy nothing.  Dead accumulator, flag and clamp work is then ordinary DCE.  The
IR is target independent: on AVX-512 it becomes `vpmuldq`/`vpmovsqw`/k-masks, elsewhere
whatever the back end has.

**Memory.**  Data accesses wrap at 4 KB and may be unaligned.  Each load/store has an
inline path for the case that stays inside DMEM and a call into `rsp.cc` for the rest.
Vector loads/stores are expressed on bytes: a fixed-size form is a load and a blend; an
unaligned `lqv`/`lrv` pair is two 16-byte windows selected by a byte mask.

**Memoization.**  Translations are keyed by a hash of instruction memory, with a memcmp
against the previous image as the fast path.

## Status

Audio microcode only (17 vector ops, `lsv/llv/ldv/lqv/lrv`, `ssv/slv/sdv/sqv`).  Driven by
`~/GoldenEye/gemusic --rsp llvm`: byte-identical output to the interpreter on all 63
GoldenEye sequences with zero interpreter fallbacks, about 21x the interpreter's speed.
Graphics microcode needs roughly 25 more vector operations, the packed and transposed
load/store forms, `cfc2`/`ctc2`, and invalidation when an overlay is DMA'd into IMEM
mid-task (the image key already covers the rest).

Builds against LLVM 18 (`make`, `LLVM_CONFIG=` to override).  Little-endian hosts only.

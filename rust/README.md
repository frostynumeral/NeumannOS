# NeumannOS (Rust port)

This directory holds a Rust port of the MINIX 3.1.0 sources in the rest of
this repository (see the top-level `README.md` for the original C
codebase's history). It is a **starting skeleton**, not a finished port: a
real microkernel, its servers (`pm`, `fs`, `rs`, ...), and its drivers are a
multi-month undertaking on their own. What's here boots in QEMU, sets up
exception handling and a heap, schedules kernel tasks with real,
asynchronously preemptive hardware-timer-driven quantum accounting
(including a task that runs in ring 3, in its own genuinely isolated
address space), exercises blocking message-passing IPC between them, has
its first real kernel calls (a genuine cross-address-space `sys_vircopy`;
`sys_setalarm` waking a blocked task with a real notification instead of
it polling), and can `sys_fork` a real child process with a *deep-copied*,
independent address space (verified by writing to the child's copy and
confirming the parent's is untouched, not just aliasing the same physical
memory), a real `fs` server backing genuine open/read/write requests
with an in-memory filesystem over IPC (not a fixed-reply stand-in) and a
real, if shallow, directory hierarchy on top (`mkdir`, and `open`
enforcing the usual parent-directory rules), and a second ring-3 task
that runs a *real, statically linked ELF64 binary*
(parsed and mapped by this port's own minimal ELF loader, not
hand-assembled bytes poked into a fixed page), a real `int 0x80` syscall
gate with genuine call-number/register dispatch (call number in `rax`,
arguments in `rdi`/`rsi`, a hand-written trap frame preserving every
other register) rather than a fixed action performed regardless of what
the caller asked for — including a ring-3 task genuinely blocking on a
real kernel call (`SYS_SET_ALARM`/`SYS_WAIT_ALARM`, wrapping the same
`sys_setalarm`/notification `CLOCK` already uses) and resuming exactly
where it left off in ring 3 once the alarm actually fires, not just
one-shot calls that always return immediately — and, the first step
toward the BeOS/Haiku-flavored desktop-OS direction noted below, real
VGA graphics (a static, LCARS-style panel of flat-colored rounded-rectangle
bars and buttons, no text, painted into a real linear framebuffer) and a
real PS/2 keyboard driver (hardware IRQ1, scancodes read and translated to
ASCII, asynchronously -- even waking the CPU from `IDLE`'s `hlt`), now
connected to the panel: pressing a digit key highlights the matching
button on screen, a real (if minimal) input-to-output loop. `rs` is also
now a real reincarnation server: a ring-3 task that deliberately crashes
(`flaky`, its one instruction being `ud2`) is isolated -- the CPU
exception it raises kills just that one process, not the whole
machine -- and `rs` restarts it, up to a bounded number of times, exactly
MINIX's signature self-healing behavior. The syscall ABI now also reaches
`fs` for real: a ring-3 task opens and writes a real file over a genuine
`int 0x80` -> syscall dispatch -> IPC -> `fs` round trip, verified by
reading the same file back through a completely independent, kernel-side
path afterward. The keyboard now drives a real line discipline too: typed
characters build up a line in `crate::keyboard`, and pressing Enter wakes
a real, scheduled `console` task (not just a direct function call) that
writes the completed line to a real file via `fs` — a keypress now
reaches a real process doing real IPC, not just `vga::select_button`.
The syscall ABI's `SYS_READ_LINE` closes that loop the other direction:
a real ring-3 task blocks *inside its own trap* for however long it
takes a human to actually type a line and press Enter, then resumes in
ring 3 with the typed bytes — a real keypress, reaching a real process,
by way of a real syscall — enough to build the rest of the system on
top of. `fork`'s other half is real now too: `exec()` (`SYS_EXEC`), where
a ring-3 task names a program by path, and the very trap it made returns
into a *different binary* — loaded out of `fs`, mapped into a brand-new
address space, resumed at its own entry point on its own fresh stack,
with the process itself (its number, priority, kernel stack, open
descriptors) carrying straight through. Verified from outside the
process, not from its own logs: the new image's `.data` reads back
through the *caller's* process slot, while the old image's `.data` page
is no longer mapped there at all — replaced, not merely added to. A
deliberately-failed exec of a non-ELF file is checked the same way,
since "a failed exec leaves the caller exactly as it was" is the part
that is easiest to get wrong.

**Porting philosophy:** the C tree is kept as the behavioral spec — process
numbers, message/call numbers, the device-driver protocol
(`DEV_OPEN`/`DEV_READ`/...), and the `pm`/`fs`/`rs` division of
responsibilities are carried over faithfully, so every design question
already has a settled answer and the existing `test/` POSIX suite is a
real target to eventually validate against. What is *not* carried over is
the C implementation style: no transliterated `message` unions, privilege
bitmasks, or `PUBLIC`/`PRIVATE` macros — each subsystem is rebuilt in
idiomatic, type-safe Rust (enums, `Result`, ownership) around the same
external contract.

**License:** unlike the MINIX C tree this directory ports (top-level
`LICENSE`, Prentice Hall's original BSD-style license), this Rust port is
new code and is dual-licensed under your choice of the
[MIT license](LICENSE-MIT) or the
[Apache License, Version 2.0](LICENSE-APACHE) — the convention most of
the Rust ecosystem (rustc itself, `serde`, `tokio`, ...) uses.

## What's implemented

- `src/com.rs` — process numbers and notification types, ported from
  `include/minix/com.h`.
- `src/table.rs` — the boot image (fixed process list), ported from
  `kernel/table.c`'s `image[]`.
- `src/ipc.rs` — `Message` (ported from the `mess_*`/`message` union in
  `include/minix/ipc.h`) plus `send`/`receive`/`notify`, standing in for
  MINIX's `SEND`/`RECEIVE`/`NOTIFY` kernel calls. These genuinely block now
  (see `src/proc.rs`): a `send`/`receive` that can't complete immediately
  context-switches to another runnable task instead of returning an error.
  Also `send_receive`, standing in for `SENDREC`: `send` a request then
  `receive` its reply, the "call a server, block for the answer" pattern
  every server client (`src/fs.rs`'s `open`/`read`/`write` stubs) uses.
- `src/proc.rs` — the process table and scheduler, ported from
  `kernel/proc.h`/`kernel/proc.c`: `NR_SCHED_QUEUES` priority-ordered ready
  queues, `enqueue`/`dequeue`/`sched`/`pick_proc`, and the rendezvous IPC
  algorithm (`mini_send`/`mini_receive`/`mini_notify`) that `ipc.rs` calls
  into. Also has no C equivalent of its own: the low-level `switch_to`
  (save/restore callee-saved registers, `RFLAGS`, and the stack pointer)
  and the `trampoline` a freshly spawned task's stack is primed to land in
  on its first run, standing in for the register save/restore and
  initial-frame setup that `kernel/mpx386.s` and `kernel/main.c` handle in
  C MINIX. `reschedule()` -- called after every IPC operation and, crucially,
  from inside the timer interrupt handler -- is what makes preemption
  genuinely asynchronous rather than just cooperative; see its doc comment
  and `switch_to`'s for how saving `RFLAGS` per task is what makes calling
  the same switch code from both places sound. `kill(proc_nr, reason)`
  is the newest addition: dequeue `proc_nr` (if it was ready) and mark it
  `rts::DEAD` -- permanently off every ready queue until a fresh `spawn()`
  overwrites the slot -- then deliver a `com::proc_died` notification to
  `RS` (`crate::rs`). Called from `crate::interrupts` when a ring-3 task
  takes a CPU exception; this port's stand-in for real MINIX turning a
  user-process fault into a signal and `PM` reporting the exit to `RS`.
  `kill` is now one of *two* ways a process stops existing, sharing
  `Scheduler::terminate` with the other: `exit_now`, reached from ring 3
  through `crate::syscall`'s `SYS_EXIT`, where the process asked. Ported
  in spirit from `servers/pm/forkexit.c`'s `do_exit`, which real MINIX
  likewise reaches either from an `exit()` call or from the kernel's
  `SYS_SIG` notification about a fatal signal. `terminate` takes the
  slot off every queue, detaches its address space and memory map, and
  then decides what happens to the exit *status*, which is the part that
  needs a policy: a parent already blocked in `wait_for_child` gets it
  handed over directly and is woken (no zombie ever exists); a live
  parent that isn't waiting yet leaves the slot a zombie
  (`rts::ZOMBIE`, `mp_flags & ZOMBIE`) holding the status, keeping the
  process number allocated for exactly as long as that takes -- which is
  what a zombie *is*; and no live parent frees the slot outright, since
  nothing can ever collect a status nobody is related to.
  `wait_for_child` (`do_waitpid`) is the collecting side: it blocks in
  `rts::WAITING` until a child terminates, or picks up a zombie that
  terminated earlier and releases its slot. That release is the only
  thing in this port that ever reclaims a dynamic process number --
  before it, `alloc_proc_nr`'s pool drained monotonically. A process
  with no children at all is told so rather than blocked forever
  (POSIX's `ECHILD`). `exit_now` keeps interrupts off from the
  scheduler-lock section through its final `reschedule()`, which is
  load-bearing rather than tidy: `terminate` can have released this very
  slot, and `switch_to` has not yet written the dead task's stack
  pointer into it, so a timer tick switching to a task that forks could
  otherwise be handed this number and have its brand-new process's `rsp`
  overwritten.
  Two of `struct proc`'s neighbours from `servers/pm/mproc.h` live here
  too, since `fork` is a kernel call in this port and the kernel is
  therefore who needs them: `Proc::mem_map` (`mp_seg[]`, which pages this
  process owns -- see `src/memory.rs`'s `MemMap`) and `Proc::parent`
  (`mp_parent`, who forked it). They travel with the page table as one
  `AddressSpace` value rather than as parallel arguments, so the map and
  the `CR3` it describes cannot get out of step -- `exec`
  (`set_address_space`) replaces both together, and a process whose map
  was left behind would have a later `fork` copying pages that no longer
  exist. `alloc_proc_nr`/`release_proc_nr` hand out and take back the
  process numbers above `com::FIRST_DYNAMIC_PROC_NR`, which is what lets
  a process be created because something asked for one at runtime rather
  than because `crate::com` reserved a number for it in advance
  (`servers/pm/forkexit.c` scans `mproc[]` for a free slot and calls the
  same condition `EAGAIN`). The claim happens up front, marking the slot
  `rts::RESERVED`, precisely because the following `fork_current` is
  several frame-allocating steps away: a timer tick landing in that gap
  can switch to a task that forks too, and an allocator that merely
  *looked* would hand it the same number and have it overwrite a
  half-built process. `child_of` is the inverse of `parent`, and is how
  anything outside a runtime-created process finds it now that no
  constant names it.
- `src/gdt.rs` — Global Descriptor Table and Task State Segment, ported from
  the segment/TSS setup in `kernel/protect.c`. Its only real job right now
  is giving the double-fault handler a dedicated stack (via the TSS's
  Interrupt Stack Table), so a fault during fault delivery — e.g. a kernel
  stack overflow — is reported instead of triple-faulting the CPU.
- `src/interrupts.rs` — the IDT and CPU exception handlers, ported from the
  single vector-indexed dispatcher in `kernel/exception.c`. The C version
  turns a fault into a POSIX signal for a user process or panics for a
  kernel task; `#BP` (breakpoint) reports and returns (exercised by a
  self-test in `main.rs`), and `#DE`/`#UD`/`#GP`/`#PF` (divide error,
  invalid opcode, general protection fault, page fault) now go through
  `recover_or_halt`: a fault in a *ring-3* task calls `crate::proc::kill`
  and `reschedule` instead of halting (this port's stand-in for turning
  the fault into a fatal signal for that one process), while a fault in
  kernel-trusted code (RPL 0) still reports and halts the whole machine,
  same as before -- a kernel bug means the kernel's own state might
  already be corrupted, so continuing isn't safe even by "just" killing
  one task. See `src/rs.rs` below for what actually exercises the
  ring-3-recovery path.
- `src/pic.rs` — 8259 PIC remap, ported from `kernel/i8259.c`'s
  `intr_init()`: moves hardware IRQs off the CPU-exception vector range and
  masks everything except IRQ0 (the timer), since nothing else has a
  handler yet.
- `src/pit.rs` — 8253/8254 PIT programming, ported from the timer-setup
  half of `kernel/clock.c`'s `init_clock()`: channel 0 fires at `HZ` (60,
  matching `include/minix/const.h`).
- `src/interrupts.rs` additionally handles IRQ0 (the timer), calling
  `proc::clock_tick` on every tick -- standing in for the
  `hwint00`/`clock_handler` pair in `kernel/mpx386.s`/`kernel/clock.c`.
- `src/memory.rs` + `src/allocator.rs` — paging and a heap allocator. No
  direct C equivalent: 2005-era i386 MINIX uses segment-based protection
  (`kernel/kernel.h`'s `struct mem_map`, translated by kernel calls like
  `sys_umap`/`sys_vircopy` in `kernel/system/`), not paging, but x86_64 has
  no non-paged protected mode at all, so paging here is mandatory
  groundwork rather than a ported feature. Building on the page table the
  `bootloader` crate already installed, `memory.rs` adds a physical frame
  allocator over the usable regions of the boot-time memory map, and
  `allocator.rs` uses it to map and initialize a 1 MiB heap (via the
  `linked_list_allocator` crate) so `alloc`-crate types (`Box`, `Vec`, ...)
  work -- exercised by a self-test in `main.rs`.
- `src/memory.rs`'s `new_address_space` clones the currently-active PML4
  into a freshly allocated frame -- sharing every existing mapping (kernel
  code/data, the heap, the physical-memory window) by aliasing the same
  lower-level tables, and only becoming genuinely private wherever
  something is mapped into a PML4 slot the original table didn't already
  use. `new_address_space_from` is the same thing with the source table
  named explicitly instead of read out of `CR3`: invisible to every
  caller that runs in the kernel's own address space, but load-bearing
  for `exec` (`src/calls.rs`'s `sys_exec` below), which runs inside a
  *ring-3* caller's trap -- where the active table is the one exec is
  supposed to be throwing away, and copying it would quietly hand the new
  image its predecessor's pages.
  Frames come back now, too. `BootInfoFrameAllocator` was a pure bump
  allocator; it keeps the bump cursor for never-yet-used memory but adds
  a free list threaded *through the free frames themselves* (each one's
  first eight bytes hold the next one's physical address). That shape is
  forced by boot order rather than chosen for elegance --
  `allocator::init_heap` allocates frames in order to map the heap, so
  the frame allocator cannot depend on a heap existing. `allocate_frame`
  pops the free list before bumping, and every frame is zeroed on the
  way out: once frames are recycled a fresh page is no longer untouched
  memory but some *other process's* former stack, and several callers
  map a page without writing all of it. `free_address_space` hands a
  whole address space back -- its user pages, the page-table levels
  leading to them, and the PML4 -- walking exactly the slots used here
  and unused in the base table, the same private/shared distinction
  `pml4_slots_unused` tests; descending into a shared slot would feed
  the kernel's own page tables to the allocator. `frame_stats` exposes
  the accounting, which is the only observable signature "no leak" has
  in a kernel with no process accounting.
  `pml4_slots_unused` is the check that
  makes the aliasing caveat above enforceable rather than a comment: it
  reports whether every PML4 slot an address range falls in is unused in
  a given table, which is exactly the precondition "mapping this will be
  private" depends on. `crate::elf`'s `validate` uses it on every
  segment of an image `exec` was handed. `crate::usermode` is the first thing to use this, to give the
  ring-3 demo task real isolation instead of just a CPU privilege level.
  `page_table_for` and `copy_between_address_spaces` build on it: given
  any process's PML4 frame, translate a virtual address through *that*
  table (via the physical-memory window, without switching `CR3` to it)
  and copy page-at-a-time between two such address spaces -- this is
  where `sys_umap`/`sys_vircopy`'s actual job (translating between address
  spaces) applies; see `src/calls.rs` below for the kernel-call wrapper.
  `MemMap` is the per-process memory map: a small, fixed-capacity list
  of `Segment`s (a base address and a page count) naming the pages an
  address space holds that are *the process's own*, as opposed to the
  kernel mappings every address space shares. It is the Rust counterpart
  of `include/minix/type.h`'s `struct mem_map`, three of which
  (`mp_seg[T]`/`[D]`/`[S]`) describe a MINIX process's whole layout in
  `servers/pm/mproc.h` -- minus the physical base, since here the page
  tables already record where each page lives. Whoever *builds* an
  address space fills it in, because that is where the answer is known:
  `usermode::build_ring3_address_space` has just mapped its two demo
  pages, and `elf::load_image` has just walked an image's program
  headers (a compile-time assertion in `src/elf.rs` keeps `MAX_SEGMENTS`
  big enough for every `PT_LOAD` segment `validate` will accept plus the
  stack). The process table carries it from there
  (`proc::AddressSpace`/`Proc::mem_map`), which is what makes `fork` a
  general call rather than a special case per caller -- see the
  `src/syscall.rs` bullet.
  `fork_address_space` is what reads it: it builds a fresh address space
  derived from the *kernel's* PML4 and maps every page in the map into
  it at the *parent's own frame* -- no page is copied at fork time.
  Read-only pages (a program's text) are simply shared; writable ones
  become copy-on-write on both sides: the `WRITABLE` bit comes off, the
  software-defined `COW` bit (`BIT_9`, one of the three the architecture
  leaves to the OS) goes on, and when the parent is the active address
  space (every ring-3 `SYS_FORK`, which runs inside the parent's own
  trap) its TLB entries for those pages are flushed, since a stale,
  still-writable translation would let it write straight into a frame
  its child now shares. A fork costs the child's page tables and nothing
  else. `pml4_slots_unused` gates every page, the same rule `exec` is
  held to, and a failure anywhere tears the half-built address space
  back down and reports `None`.
  What makes sharing sound is a frame reference count: `SHARERS`, a
  sparse map from frame to how many *extra* address spaces map it (a
  frame with no entry has exactly one mapping -- every frame nobody has
  forked). `free_address_space` drops a reference per leaf
  (`release_frame`) and only returns a frame to the allocator with its
  last one. The first write to a `COW` page -- a ring-3 store, or a
  kernel write through a user pointer, which faults too because `init`
  turns on `CR0.WP` -- lands in `interrupts::page_fault_handler`, which
  calls `resolve_cow_fault` and re-runs the instruction: a frame still
  shared gets copied into a private one (`CowResolution::Copied`); a
  frame whose other sharers have all let go (exited, exec'd, or copied
  already) just gets its write permission back, no copy
  (`CowResolution::Reclaimed`), which is what usually happens to
  `shell` after its child execs. `copy_between_address_spaces` breaks
  `COW` on its destination first, since it writes through the
  physical-memory window and a read-only mapping wouldn't stop it.
  `crate::main`'s `cow_self_test` checks all of this at boot on a real
  ELF address space -- including a genuine ring-0 write fault taken with
  `CR3` switched into the child -- and the full boot exercises every
  path for real (see the `[cow]` log lines).
  Sharing was tried once before, without reference counts, and was
  unsound: the first `free_address_space` of either side handed the
  other process's live page back to the allocator, and duplicating
  table levels per page leaked a table frame per level. Copying every
  page replaced it as a correctness fix; `SHARERS` is what made sharing
  safe to bring back.
  The physical frame allocator is now a
  global, lock-protected resource (`GlobalFrameAllocator`,
  `init_frame_allocator`) rather than a value threaded through
  `kernel_main`'s locals, since a kernel call like `sys_fork` needs to
  allocate memory from whichever task happens to call it, not just during
  boot setup.
- `src/gdt.rs` also now sets up user-mode (ring 3) code/data segments and
  a TSS `RSP0` (the stack the CPU switches to automatically on *any*
  ring-3-to-ring-0 transition) -- `set_rsp0`, called from `crate::proc` on
  every task switch (see below), points it at whichever task is now
  current's *own* dedicated kernel stack, which is what makes it safe for
  more than one task to spend time in ring 3.
- `src/interrupts.rs` adds a ring-3-callable `int 0x80` gate
  (`SYSCALL_VECTOR`), handled by `crate::syscall::entry` (see below).
- `src/usermode.rs` uses all of the above to run a real,
  scheduler-integrated ring-3 task with its own address space:
  `create_address_space` builds a new, private page table (via
  `memory::new_address_space`) and maps a code and a stack page into *it*
  (with `USER_ACCESSIBLE`, without which the CPU refuses to execute or
  touch them at CPL 3 at all -- a `#PF`, not a `#GP`) -- never into the
  kernel's own mapper. `ring3_task_entry`, an ordinary `crate::proc` task
  body, then jumps to CPL 3. The ring-3 code (`USER_CODE`, hand-assembled
  one instruction at a time -- see its own doc comment) calls
  `crate::syscall`'s `SYS_GET_UPTIME` three times and then
  `SYS_BLOCK_FOREVER`, trapping into the kernel and back repeatedly --
  ordinary, repeatable trap entry/exit, not a one-shot trick -- and can be
  asynchronously preempted by the timer while in ring 3 exactly like any
  other task.
- `src/syscall.rs` — the real `int 0x80` gate: call-number/register
  dispatch (call number in `rax`, up to four arguments in
  `rdi`/`rsi`/`rdx`/`rcx`, return value in `rax`), replacing a fixed action
  performed regardless of what the caller asked for. Ported in spirit
  from `kernel/system.c`'s kernel-call dispatch table and the trap gate
  that reaches it (`kernel/mpx386.s`'s `s_call`), though real MINIX
  dispatches kernel calls through the same message-passing rendezvous as
  everything else (a `SENDREC` to `SYSTEM`), not a raw register
  convention -- this port's is closer to Linux's `int 0x80` than MINIX's
  own, since there's no in-kernel message-passing entry point reachable
  from ring 3 yet, and building that needs this register-level plumbing
  first regardless. `entry` is a hand-written naked trap gate (installed
  via `Entry::set_handler_addr`, not `set_handler_fn`, since the
  `x86-interrupt` calling convention doesn't expose the caller's
  general-purpose registers, only the hardware-pushed
  `InterruptStackFrame`): it saves all 15 general-purpose registers the
  CPU didn't already save, calls `dispatch` with the caller's original
  `rax`/`rdi`/`rsi`/`rdx`/`rcx`, writes the `u64` result back into the
  saved `rax` slot, restores everything else unchanged, and `iretq`s.
  `dispatch` implements sixteen calls (the two newest, `SYS_FS_OPEN_EXISTING` and `SYS_CONSOLE_WRITE`, are described under "Ring-3 programs in Rust" below): `SYS_GET_UPTIME` (returns `proc::uptime_ticks()`),
  `SYS_WRITE_LINE` (reads a caller-supplied `(ptr, len)` string and prints
  it -- a genuine cross-ring pointer argument, safe to dereference
  directly because entering a trap gate never switches `CR3`, so
  `dispatch` runs with the *caller's own* address space still active),
  `SYS_SET_ALARM`/`SYS_WAIT_ALARM` (the first of `crate::calls`' own
  kernel calls reachable from ring 3: `SYS_SET_ALARM` calls the same
  `calls::sys_setalarm` `CLOCK` itself uses, and `SYS_WAIT_ALARM` blocks
  the caller in `ipc::receive` *inside this very trap* until the real
  `SYN_ALARM` notification arrives, then returns normally -- proving a
  ring-3 task can genuinely block on a kernel call, with the rest of the
  system (other tasks, the scheduler, the timer) continuing normally
  while it's blocked, and later resume ring-3 execution right where it
  left off, not just make one-shot calls that always return immediately),
  `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ` (a real IPC round trip to
  `fs` -- not just another kernel-internal call -- copying a caller's
  buffer into a local kernel-stack buffer first, since (unlike
  `SYS_WRITE_LINE`) `fs` is a *different task* that may see a different
  `CR3` by the time it actually dereferences anything; see `src/fs.rs`
  above), `SYS_READ_LINE` (the same kernel-stack-buffer trick, in the
  *input* direction: blocks in `keyboard::read_line` for however long it
  takes a human to actually type a line and press Enter -- unbounded, and
  genuinely inside this trap, the same shape as `SYS_WAIT_ALARM` but
  driven by a keypress instead of a timer -- then copies the result into
  the caller's own buffer once this task's own `CR3` is active again; see
  `src/keyboard.rs` above), `SYS_VIRCOPY` (the first call needing a fourth
  argument, hence the ABI's growth to `rdi`/`rsi`/`rdx`/`rcx`: exposes
  `crate::calls::sys_vircopy` -- already proven kernel-side, see
  `vircopy_demo` below -- directly to a ring-3 caller, reading `len` bytes
  from another process's memory into *this task's own* buffer. The
  destination pointer is never dereferenced by `dispatch` itself, unlike
  `SYS_WRITE_LINE`'s or `SYS_FS_WRITE`'s pointers -- it's handed to
  `memory::copy_between_address_spaces`, which reaches it by walking the
  caller's own page tables through the physical-memory offset window, the
  same mechanism a kernel-task caller like `CLOCK` already relies on,
  regardless of which `CR3` happens to be active. No privilege check
  gates which process a caller can read this way, unlike real MINIX's
  IPC-bitmask-gated kernel calls -- see "known simplifications in the
  kernel calls" below), `SYS_FORK` (a fifth register, `r9`, carries
  `frame_ptr` -- not a normal argument, but the address of the 15
  registers `entry` just pushed for *this* trap, captured via `lea r9,
  [rsp]` before the other `mov`s start reusing those registers for the
  call to `dispatch`. Exposes `crate::calls::sys_fork_from_frame`, which
  hands `proc::fork_current` a snapshot of that trap -- `rax` zeroed --
  instead of a fixed entry point, so the new task resumes at the exact
  ring-3 instruction its parent trapped from, not somewhere fixed; see the
  `src/proc.rs` bullet below for how), `SYS_EXEC` (the other half of
  `fork`: same `frame_ptr` mechanism, opposite effect -- instead of
  copying this trap into a *new* process, it overwrites this trap's own
  saved registers so that the `iretq` at the end of `entry` resumes a
  *different program* rather than returning to the caller at all. Reads
  the path out of ring 3 into a kernel-stack buffer first, since the
  caller's own pointer stops meaning anything the moment the address
  space is swapped; loads the image out of `fs` by path
  (`elf::read_file`, blocking on real IPC round trips the whole time,
  still on the old address space, so a missing file changes nothing);
  then `calls::sys_exec` and `TrapFrame::exec_into`. Takes a real
  `argv` and `envp` too (`rdx`/`rcx`, C-style NULL-terminated arrays of
  NUL-terminated strings, `0` for an empty one), copied out of the
  caller *first* -- before the file is even looked up -- by walking the
  caller's page tables rather than dereferencing (`copy_from_caller`),
  and only from PML4 slots the kernel's own address space leaves empty:
  the strings end up back in ring 3 on the new stack, so a pointer into
  the kernel heap would otherwise be a way to read kernel memory out.
  That's `ERR_BAD_ARG_PTR` (`EFAULT`); more than `MAX_EXEC_VECTOR`
  entries or `elf::MAX_START_ARGS_BYTES` of layout is `ERR_ARGS_TOO_BIG`
  (`E2BIG`). Forwards `fs`'s own
  error codes unchanged when the path is the problem, `ERR_BAD_ELF` when
  the file is, and refuses outright -- `ERR_EXEC_NOT_RING3` -- if the
  caller's saved `CS` says it wasn't in ring 3, since exec'ing a kernel
  task would swap its address space out from under it and `iretq` it into
  a ring-3 entry point with a kernel `CS`), `SYS_EXIT`/`SYS_WAIT` (the
  end of a process's life, and the only pair here where one call's
  *absence* of a return is the point: `SYS_EXIT` reaches
  `proc::exit_now`, which never comes back, because the process the
  `iretq` would have returned to has stopped existing by then;
  `SYS_WAIT` blocks in `proc::wait_for_child` until one of the caller's
  children terminates, then returns that child's `proc_nr` and writes
  its status through a caller-supplied pointer -- written directly,
  unlike `SYS_READ_LINE`'s, because no other task ever touches it: the
  status comes back through the process table, and by the time it is
  written this task is running again with its own `CR3`. Both refuse a
  non-ring-3 caller -- `ERR_NOT_RING3` -- for the same reason `SYS_EXEC`
  does, and more sharply: `exit` would tear a *kernel* task's slot down
  from under it mid-trap), and `SYS_BLOCK_FOREVER` (calls
  `ipc::receive(ANY)` directly from inside the trap, never returning --
  the same "nothing sends to this
  proc again" pattern the demo tasks that have to stay readable
  afterwards end with, `SYS_EXIT` being what a process that is simply
  *done* now uses instead).
- `src/proc.rs`'s `reschedule`/`start` call `gdt::set_rsp0` *and* switch
  `CR3` on every switch, pointing both at whichever task just became
  current: `RSP0` at that task's own dedicated kernel stack (the same one
  `crate::proc::stack_top` already used to build its initial `switch_to`
  frame), `CR3` at its own address space if it has one distinct from the
  kernel's (`Proc::cr3`), or the kernel's own otherwise. Per-task `RSP0`
  is what makes running ring-3 code as a real task safe at all (with one
  shared `RSP0`, two tasks both spending time in ring 3 could clobber each
  other's saved state the moment either faulted or was preempted);
  per-task `CR3` is what makes that isolation *real* rather than just a
  CPU privilege level.
  `src/proc.rs` also gains `fork_current` and its `TrapFrame` type,
  `SYS_FORK`'s real counterpart to `spawn`: instead of the ordinary
  single "six callee-saved registers + `RFLAGS` + a return address into
  `trampoline`" frame every other task starts from, a forked child's
  kernel stack gets *two* frames stacked back to back -- that same outer
  `switch_to`-compatible one (pointing at a new `fork_child_resume` stub
  instead of `trampoline`), immediately followed by a full `TrapFrame`
  copy (`repr(C)`, laid out to exactly match what `crate::syscall::entry`
  pushes/pops, so a raw pointer into a live trap can be read as one
  directly) with `rax` zeroed. The first time the child is switched to,
  `switch_to`'s ordinary `ret` lands in `fork_child_resume`, which pops
  that inner frame and `iretq`s with it -- indistinguishable, from ring
  3's side, from the parent's own trap returning normally, except `rax`
  reads `0`. `fork_child_resume` deliberately duplicates `entry`'s own pop
  sequence rather than jumping into it (naked functions have no
  addressable internal label another function's `sym` can reach) --
  a small, self-contained stub kept in sync with `entry`'s tail by hand.
  `SYS_EXEC`'s counterparts are smaller but sit in the same place:
  `set_address_space` points a process at a different PML4 and, when
  that process is the one currently running (always, for exec, which
  happens inside the caller's own trap), reloads `CR3` immediately rather
  than waiting for the next `reschedule` -- with interrupts off across
  both, so a timer tick can't land between the table write and the `CR3`
  load. `set_address_space` also frees the address space it replaces,
  and `kill` frees the dead process's -- both only after `CR3` has been
  moved off the tables in question (`kill` switches to the kernel's own
  address space first, since a process usually dies from the CPU
  exception its own code raised, with its own page tables still
  loaded). `TrapFrame::exec_into` builds the frame that turns a trap return
  into a program *start*: caller's `cs`/`ss` (same ring), the new entry
  point and stack, a fresh `RFLAGS` of `0x202` (deliberately not
  inherited -- a new program shouldn't start with the direction flag its
  predecessor left set), and every general-purpose register zeroed.
  `set_address_space` and `fork_current` both take that one
  `AddressSpace` value now, rather than a bare PML4 frame.
- `src/calls.rs` — the first kernel calls, ported from
  `kernel/system/do_copy.c` (`sys_vircopy`), `do_setalarm.c`
  (`sys_setalarm`), and `kernel/proc.c`'s `do_fork()` (`sys_fork`). Real
  MINIX dispatches these by call number out of a message a process sends
  to `SYSTEM`; this port has no such dispatch yet (see "known
  simplifications" below), so for now they're just ordinary Rust functions
  any kernel task can call directly -- the same way `crate::ipc`'s
  `send`/`receive`/`notify` started out, before anything needed them from
  ring 3. `sys_vircopy` is a thin wrapper resolving two process numbers to
  address spaces (`proc::cr3_of`) and delegating to
  `memory::copy_between_address_spaces`; `sys_setalarm` delegates to
  `proc::set_alarm`, and `proc::clock_tick` is what actually notices an
  alarm's deadline and delivers the `SYN_ALARM` notification for it (using
  a `Scheduler::try_deliver_notification` helper shared with
  `mini_notify`, since `clock_tick` is already inside the scheduler lock
  and can't re-enter it); `sys_fork` resolves a parent process number to
  an address space *and its memory map* (`proc::mem_map_of`), deep-copies
  every page the map names into a brand new one
  (`memory::fork_address_space`), and spawns a task into it
  (`proc::spawn`) -- bundling what real MINIX splits into a kernel call
  (duplicate the memory) and a separate scheduling step, since nothing in
  this port needs them separated yet. Which pages to copy used to be an
  argument, passed in by whoever happened to know; both fork entry points
  share one `fork_child_address_space` helper now that reads it out of
  the process table instead, so "fork this process" is a complete
  request. `sys_fork_from_frame` is `sys_fork`'s
  real-fork-semantics sibling: identical deep-copy step, but hands
  `proc::fork_current` a `proc::TrapFrame` snapshot instead of a fixed
  entry point (see the `src/proc.rs`/`src/syscall.rs` bullets above) --
  reachable from ring 3 (`crate::syscall`'s `SYS_FORK`), unlike `sys_fork`
  itself, which only `pm`'s own kernel-side demo calls.
  `sys_exec` is `fork`'s opposite number, ported in spirit from the pair
  MINIX splits exec across -- `servers/pm/exec.c`'s `do_exec` (find the
  file, lay out and load the image) and the `SYS_EXEC` kernel call it
  makes, `kernel/system/do_exec.c` (point the process at the new image
  and set its saved `pc`/`sp`). This does the first part plus the
  address-space swap: `elf::load_image` into a brand-new address space
  derived from the *kernel's* PML4 (so the new image starts with an empty
  user address space, not its predecessor's mappings), then
  `proc::set_address_space`. `elf::load_image_with_args` also lays the
  caller's `argv`/`envp` out on the new stack page in the System V
  x86-64 start-up form (`write_initial_stack`: `argc` at `rsp`, then
  `argv[]`/NULL, `envp[]`/NULL, an empty auxiliary vector, strings
  above; `rsp` 16-byte aligned) -- the part `do_exec` does by copying
  the caller's prepared stack into `mbuf` and relocating its pointers
  (`patch_ptr`). Images started at boot get the same block with
  `argc == 0`. Writing the new entry point and stack into
  the caller's live trap frame stays in `crate::syscall`, which is the
  only code holding that frame -- the same split `do_exec.c` has by being
  the only code holding `rp->p_reg`. Nothing is mutated until the image
  has been fully validated and mapped, so a failed exec leaves the caller
  running exactly as it was, which is what POSIX requires and what
  `user/shell.s` deliberately checks.
- `src/fs.rs` — a real, in-memory file server, replacing `fs`'s ping-pong
  stand-in. No single C file to port: real `servers/fs` is a whole
  subsystem (`open.c`/`read.c`/`write.c`/`path.c`, an inode/block-cache
  layer over a real block device) this port has no device driver or
  on-disk layout to back yet, so this is the minimal slice of its
  *external behavior* -- an open/read/write request-reply protocol over
  `crate::ipc`, backing files that live in heap-allocated `Vec<u8>`s
  instead of on disk. `InMemoryFs::serve` is the server loop (`receive`
  from anyone, dispatch on `m_type`, `send` a reply), mirroring
  `servers/fs/main.c`'s `while (TRUE) { get_work(); ...; reply(...); }`
  shape; `open`/`write`/`read`/`mkdir` are client-side stubs (using
  `ipc`'s new `send_receive`) that any task can call to talk to it,
  mirroring `src/calls.rs`'s kernel-call wrappers in shape even though
  these cross a real IPC round trip rather than a direct function call.
  `open_existing` is a second client stub added alongside `open`: same
  request, but with a no-create flag, so a caller can ask *whether* a
  file is there. `open`'s create-on-open behavior (see "known
  simplifications in `fs`" below) is fine for a writer and actively
  wrong for a looker-up -- `exec` of a path that doesn't exist would
  otherwise succeed at opening nothing, report `ERR_BAD_ELF` as if the
  program were malformed rather than absent, and leave a stray empty
  file behind at whatever path ring 3 named.
  `InMemoryFs` also now enforces a real directory hierarchy, not just a
  flat, exact-match namespace: `directories` is a flat list of known
  directory paths (the root, `"/"`, always is, implicitly), and
  `open_path`/`mkdir` both check a path's parent the way real
  `servers/fs/path.c`'s `lookup()` does -- `ENOENT` if the parent doesn't
  exist, `ENOTDIR` if it exists but is a file, `EISDIR` if the path
  itself is a directory being opened as a file, `EEXIST` if `mkdir`
  targets a path that already exists. Every client stub now sets the
  request's `source` to `proc::current_proc_nr()` -- the real caller --
  rather than a hardcoded `PM_PROC_NR`: `fs` addresses its reply using
  that field, so a wrong one would silently misdeliver the reply to
  whoever the field named instead of the actual, blocked-waiting caller.
  This is what makes it safe for `crate::syscall`'s `SYS_FS_OPEN`/
  `SYS_FS_WRITE`/`SYS_FS_READ` to call these stubs on behalf of whichever
  ring-3 task is trapped in, not just kernel tasks like `pm`.
- `src/elf.rs` — a minimal ELF64 loader. No direct MINIX C equivalent:
  2005-era MINIX 3.1 loads a program via `execve`'s a.out-format path
  (`servers/pm/exec.c`), not ELF; this is a step up from
  `crate::usermode`'s demo (which pokes a hand-assembled four-byte loop
  directly into one fixed page), loading a real, statically linked
  binary (`user/hello.elf`, built from `user/hello.s` -- see that file's
  header for the exact `as`/`ld` invocation, since there's no cross
  toolchain wired into this build to assemble it automatically) the way
  a real loader must: `load` parses the ELF64 header and program header
  table, maps every `PT_LOAD` segment into a fresh address space
  (`memory::new_address_space`) at the addresses and with the
  read/write/execute permissions the file itself specifies (not one
  hardcoded page), and zero-fills each segment's `p_memsz - p_filesz`
  tail (real BSS semantics) rather than assuming the file image and the
  mapped size are the same thing. `task_entry` reuses
  `usermode::enter_ring3` to jump to the parsed `e_entry` (not a fixed
  constant) on a mapped stack.
  The loader is now split in two, because `exec` made its inputs
  untrusted. `load_image` does the real work and returns a `LoadedImage`
  (`{ pml4, entry, stack_top }`) or an `ElfError`, after a `validate`
  pass that runs *before* a single frame is allocated: magic/class/
  endianness, a program header table that actually fits inside the file,
  per-segment `p_offset + p_filesz` within the image and `p_filesz <=
  p_memsz`, a bound on how many pages the whole image may occupy
  (`p_memsz` needs no file bytes behind it, so without that a
  hundred-byte file can ask for terabytes and walk the frame allocator
  dry), no two `PT_LOAD` segments sharing a page, nothing overlapping
  the fixed stack page, and -- the one that matters most -- every
  segment landing in a PML4 slot the base address space does **not**
  already use (`memory::pml4_slots_unused`).
  That last check is what keeps a chosen file off the kernel, and it is
  deliberately about the *slot*, not the address. An earlier version of
  this bounded segments against `USER_SPACE_END` and called that "can't
  map over the kernel"; that was simply wrong. This port has no
  user/kernel address boundary to bound against -- the kernel image sits
  near `0x20_0000`, the heap at `allocator::HEAP_START`
  (`0x4444_4444_0000`), and the bootloader's physical-memory window at a
  low free PML4 index, all far *below* `USER_SPACE_END` and interleaved
  with the addresses user images legitimately use. Since a new address
  space copies only the top-level table, a segment landing in an
  already-used slot gets no private mapping at all: `map_to` descends
  into the kernel's own lower-level tables and edits them. A crafted ELF
  naming `HEAP_START` was accepted and panicked the kernel from ring 3
  (`PageAlreadyMapped`); one naming an unmapped address in the same slot
  was accepted and installed a user-accessible page into the kernel's
  live page tables. `USER_SPACE_END` remains, but only as the
  canonicality bound it actually is.
  Belt and braces on top: `load_segment` and the stack mapping return
  `ElfError::MappingFailed` rather than `.expect()`-ing, so being wrong
  again about what `validate` guarantees costs a ring-3 caller its
  `exec` instead of costing the machine its kernel. And
  `elf::validator_self_test` (run from `kernel_main`) feeds the loader
  eleven hostile images -- a segment on the kernel heap, one on the
  kernel image, two segments sharing a page, one over the stack page,
  4 GiB of BSS from a 120-byte file, one past the canonical boundary, a
  truncated program header table, an image with no `PT_LOAD` at all, a
  shell script, a 32-bit ELF, and a file too short to hold a header --
  and requires each to come back as the specific right `ElfError`; the rejections are the only part of this a working boot
  cannot demonstrate. `user/echo.s` then does the same thing the other
  way round, from ring 3 and for real: it writes a 120-byte hand-built
  ELF targeting `HEAP_START` into `fs` and execs it, and `exec_verify`
  checks the `ERR_BAD_ELF` it got back. The kernel-side self-test says
  the validator rejects that image; only this one says an unprivileged
  process cannot get it loaded. Header reads go through `read_unaligned` rather than a reference
  cast, since an image `sys_exec` loads is a heap `Vec<u8>` assembled
  from `fs` reads with no alignment guarantee at all. `load` is the thin
  wrapper the boot path still uses: it keeps the old "a built-in image is
  known good, so panic" contract and stashes the entry/stack in
  `ELF_TASK_PARAMS` for a task that hasn't started yet. `read_file`
  (factored out of `spawn_from_fs`) is the other half of "get a program
  off the filesystem", shared with `sys_exec`. `spawn_from_fs` moved off
  `load` and onto `load_image` in the process: its image comes out of
  `fs` too, so a malformed one is now an `ENOEXEC` returned to whoever
  asked for the launch (`crate::rs`) rather than a kernel panic.
- `src/vga.rs` — VGA mode 13h (320x200, 256-color) graphics. No MINIX C
  equivalent (2005-era MINIX has no graphics stack at all); this is the
  first concrete step toward the BeOS/Haiku-flavored desktop-OS direction
  noted at the top of this file, not a ported feature. Mode 13h itself is
  set by the `bootloader` crate's `vga_320x200` feature (a real-mode
  `int 0x10` call before the jump to long mode); this module programs the
  256-color palette via the VGA DAC ports (`0x3C8`/`0x3C9`, 6 bits per
  channel) and writes pixels into the resulting linear framebuffer at
  physical `0xA0000` -- reached through the same physical-memory offset
  window `crate::memory`/`crate::calls` already use for everything else,
  since `map_physical_memory` maps the *entire* physical address range
  (MMIO holes included), not just RAM-typed regions. `fill_rect` is the
  flat-fill primitive; `fill_rounded_rect` builds real rounded rectangles
  on top of it (a pixel in one of the four corner boxes is only painted
  if it falls within a quarter circle of the chosen radius, exactly the
  standard rounded-rectangle construction) -- every bar and button in
  `draw_demo_panel` is one of these, not a plain rectangle. `kernel_main`
  calls `init_palette`/`draw_demo_panel` as early as possible (before
  paging/heap/scheduler setup), so the panel stays on screen even if
  something later in boot panics. `select_button` is the panel's one
  piece of live state: it records which of the four buttons (if any) is
  "selected" (a `spin::Mutex<Option<usize>>`) and immediately redraws the
  whole panel, painting a white `HIGHLIGHT`-colored frame behind that
  button -- reaching the framebuffer itself via
  `crate::memory::physical_memory_offset` (now `pub(crate)`) rather than
  requiring a caller (in practice, the keyboard interrupt handler below)
  to have a pointer to it on hand.
- `src/keyboard.rs` + `src/interrupts.rs`'s `keyboard_interrupt_handler` —
  a real PS/2 keyboard driver. No MINIX C kernel equivalent: 2005-era
  MINIX handles the keyboard in a driver process
  (`drivers/tty/keyboard.c`), not the kernel proper, reached the normal
  device-driver protocol way; this port has no real `tty` server yet to
  hand scancodes to (see `rust/README.md`'s roadmap), so this is the
  minimal first slice, proving the hardware event itself works. `pic.rs`
  now unmasks IRQ1 alongside IRQ0; `keyboard_interrupt_handler` reads the
  scancode byte off port `0x60` (`keyboard::read_scancode`), translates
  it via a scancode-set-1-to-ASCII table (`keyboard::translate`, unshifted
  keys only, release ("break") codes -- bit 7 set -- ignored), and prints
  it -- entirely asynchronous and hardware-driven, the same way
  `timer_interrupt_handler` is for IRQ0, including waking the CPU from
  `IDLE`'s `hlt` the instant a key is pressed. Digit keys `1`-`4` go
  further: `keyboard_interrupt_handler` maps them to `vga::select_button`,
  so pressing one visibly highlights the matching panel button -- an
  input-to-output loop, but still a direct function call from the
  interrupt handler, not a real input event delivered to a process.
  `keyboard.rs` now also has a real line discipline on top: `on_char`
  (called for every translated character) appends to a shared `LINE`
  buffer, and a newline marks it ready and `crate::ipc::notify`s
  `console_task` -- the same interrupt-handler-notifies-a-real-task shape
  `crate::proc::clock_tick` already uses for `SYN_ALARM`, just triggered
  by a keypress. `keyboard_interrupt_handler` now sends the IRQ's EOI
  *before* calling `on_char` (which can trigger that reschedule), mirroring
  `timer_interrupt_handler`'s own EOI-before-switch ordering. `console_task`
  (`com::CONSOLE_PROC_NR`, a new process slot) blocks in `ipc::receive`,
  takes the completed line, and appends it to a real file
  (`/console.log`) via `crate::fs` -- a keypress now drives a real,
  scheduled task doing real IPC, not just `vga::select_button`. Verified
  interactively via QEMU's QMP `send-key`, sent well after boot (with
  `IDLE` already halted): typing `h`, `i`, `ret` and then `w`, `o`, `r`,
  `l`, `d`, `ret` produces `[console] received line: "hi"` and
  `[console] received line: "world"` as two distinct lines. `console_task`
  now also serves real `CONSOLE_READ_LINE` requests (a genuine `send`/
  reply, not a fire-and-forget notification like `LINE_READY`) from
  `read_line` (`crate::syscall`'s `SYS_READ_LINE`): if a caller is already
  waiting when a line completes, `deliver_line` copies it straight into a
  pointer the caller provided and replies with the length -- safe because
  that pointer is always a kernel-stack buffer (mapped identically in
  every address space), never a ring-3 pointer directly, the same
  reasoning `crate::fs`'s `SYS_FS_*` calls already rely on. Only one
  pending reader is tracked at a time (a known simplification, below);
  fine for this port's one caller (`tty`, see `user/hello.s`).
  A completed line is also checked for a `"run <name>"` command
  (`dispatch_run`): if it matches, `console_task` sends a real
  `com::RS_LAUNCH_REQUEST` `send`/reply to `rs` asking it to launch that
  service by name, and logs the reply code -- verified via QMP by typing
  `"run hello"`, which produces `[rs] launch request for "hello" -> 0`
  and a second, independent instance of the ELF-loaded task running under
  its own process number (see the `src/rs.rs` bullet below).
- `src/rs.rs` — a real reincarnation server, replacing `rs`'s busy-loop
  stand-in. Ported in spirit from `servers/rs/manager.c`'s crash-handling
  path -- real MINIX gets there via `PM` noticing a process's unexpected
  exit and telling `RS`, which looks up that service's startup parameters
  in its own table and re-execs it; this port has neither signals nor
  `PM`'s exit path yet, so `crate::proc::kill` (called directly from a
  ring-3 task's own exception handler) delivers a `com::proc_died`
  notification straight to `RS`, and restart parameters are hardcoded
  here rather than looked up in a real service table. `task` is `rs`'s
  main loop: block in `ipc::receive(ANY)`, and if the notification decodes
  (`com::proc_died_slot`) to `flaky` (this milestone's demo service --
  see below), restart it, up to `MAX_RESTARTS` (`3`) times before giving
  up -- a bounded retry policy, mirroring real `RS`'s own restart limits,
  so a service that crashes instantly every time doesn't get restarted
  forever. `flaky` is a real ring-3 task (its own address space, built by
  the same `usermode::build_ring3_address_space` `crate::usermode`'s own
  demo uses) whose entire code is `ud2` -- x86's guaranteed-`#UD` opcode --
  so it crashes the instant it runs, deterministically, giving the
  crash-isolation/restart pipeline something real (and reproducible) to
  prove itself against.
  `rs` now also has a real (if minimal, one-entry) service table
  (`SERVICES`), generalizing the restart path beyond the hardcoded
  `flaky` case: each entry names a real ELF binary seeded into `fs`
  (`elf::HELLO_ELF` at `/bin/hello`, `main.rs`'s `seed_bin_hello`) and
  loadable on demand (`elf::spawn_from_fs`) via a new
  `com::RS_LAUNCH_REQUEST` message -- `crate::keyboard`'s console line
  discipline is the only current sender, triggered by typing
  `run <name>` (see the `src/keyboard.rs` bullet above). A launched
  service is tracked the same way `flaky` is (a bounded restart count per
  entry, `SERVICE_STATE`) and restarted with the same policy if it
  crashes, not just relaunched fresh -- `task`'s death-notification
  branch now looks a died process's `proc_nr` up in `SERVICES` instead of
  only special-casing `FLAKY_PROC_NR`. Verified by typing `run hello` over
  QMP well after boot: `[rs] launch request for "hello" -> 0`, followed by
  a second, independent instance of the same ELF image (a distinct
  process, `APP1_PROC_NR`) running its own full lifecycle (uptime
  queries, a real alarm, `SYS_WRITE_LINE`, `fs` open/write, then blocking
  on its own `SYS_READ_LINE`) completely separately from `tty`'s
  boot-time instance of that binary.
- `src/serial.rs` + `src/main.rs` — boot entry point (via the `bootloader`
  crate): loads the GDT/IDT, runs a breakpoint self-test, sets up paging
  and the heap, builds the ring-3 demo task's and the ELF-loaded task's
  address spaces and maps their pages into them, spawns the kernel tasks,
  programs the PIC/PIT and enables interrupts, and hands off to the
  scheduler. Spawns `IDLE` and `CLOCK` as real kernel tasks (same as the
  boot image), a real `fs` server (see `src/fs.rs` above) and a real `rs`
  server (see `src/rs.rs` above), and temporary stand-in bodies in the
  `pm`/`memory`/`driver`/`tty` process table slots: `pm`/`fs` ping-pong
  three blocking messages back and forth (exercising the rendezvous IPC),
  after which `fs` becomes a real file server
  (`fs::InMemoryFs::serve`) and `pm` exercises it: opens a file, writes to
  it, reopens it fresh (a distinct file descriptor with its own cursor)
  and reads the bytes back, checking they round-trip, then reads once
  more past end of file and checks that comes back empty. `pm` also
  `sys_fork`s the ring-3 task's code page into a brand new `init` process,
  then overwrites *just the child's copy* with a canary value and reads
  back both copies to prove they've genuinely diverged, not aliased the
  same physical page -- and `memory` spins in a tight, CPU-bound loop with
  no `yield_now()`/IPC call anywhere in it (proving the timer interrupt
  truly preempts a task asynchronously, mid-loop, rather than only ever
  switching at cooperative checkpoints). `rs` is spawned at a strictly
  higher priority than `flaky` (see `src/rs.rs` above) specifically so it
  reaches its first blocking `receive` before `flaky` ever gets a chance
  to crash -- `proc::kill`'s notification to `RS` is fire-and-forget, like
  every other notification in this port, so it would otherwise be a race.
  `console` (`keyboard::console_task`, see `src/keyboard.rs` above) is
  spawned at the same priority as `rs`: it just blocks in `receive`
  immediately, waiting for `on_char`'s notification, so unlike `flaky`
  there's nothing for it to race against.
  `driver` runs the ring-3 demo task
  described above (three real `SYS_GET_UPTIME` syscalls, then
  `SYS_BLOCK_FOREVER`), and `tty` runs `elf::task_entry` (`elf::load`
  builds its address space in `kernel_main`, the same way `driver`'s is
  built) -- a real ELF64 binary (`user/hello.elf`) executing its own
  counter-increment/`SYS_GET_UPTIME` loop in ring 3 five times, then a real
  `SYS_FORK`: `tty` forks itself into a genuine child process (at a
  process number allocated on the spot -- `proc::alloc_proc_nr`) that
  resumes at that *exact* point too,
  diverging only in `SYS_FORK`'s own return value -- the parent (seeing
  its child's nonzero `proc_nr`) falls through to continue the sequence
  below unchanged, while the child (seeing `0`) writes a canary into its
  own copy of `vircopy_buf` and a distinguishing message to
  `/from_fork_child.txt` via its own, separately-scheduled
  `SYS_FS_OPEN`/`SYS_FS_WRITE`, then blocks for good, never touching
  `SET_ALARM`/`READ_LINE` (see `src/proc.rs`/`src/syscall.rs` below for
  how the child's resumption actually works, and `fork_child_verify` in
  `src/main.rs` for how both the canary and the file get checked back).
  The parent continues with a real `SYS_SET_ALARM`/`SYS_WAIT_ALARM`
  (genuinely blocking and later resuming in ring 3), a `SYS_WRITE_LINE`, a
  real `SYS_FS_OPEN`/`SYS_FS_WRITE` round trip to `fs` (opening
  `/from_ring3.txt` and writing a message to it, all the way from ring 3),
  a real `SYS_VIRCOPY` reading a range of `driver`'s own private memory
  straight from ring 3 (see `src/syscall.rs` below), a second,
  deliberately-invalid `SYS_VIRCOPY` exercising the ABI's distinct error
  codes, and finally `SYS_BLOCK_FOREVER` -- see `src/syscall.rs` below
  for what each of those actually does. `IDLE` (below) reads
  `/from_ring3.txt` and `/from_fork_child.txt` back afterward to confirm
  the content genuinely landed in `fs`. (`tty` also used to block in
  `SYS_READ_LINE` for one typed line and write it to
  `/from_console.txt`; the shell, `sh`, is the keyboard's reader now, and
  with lines handed to readers first-come first-served a second reader
  parked for good would have taken every other command typed at it.) `CLOCK` now genuinely
  calls `sys_setalarm` and blocks in `receive` -- exactly real MINIX's
  `while (TRUE) receive(HARDWARE, &m)` -- instead of polling
  `uptime_ticks()`, and also exercises `sys_vircopy`: reading the ring-3
  demo task's code page back out of *its* address space into a local
  buffer, proving the copy really goes through a different process's page
  table (`CLOCK` itself never leaves the kernel's own address space).
  `IDLE` (below) does the equivalent check on `tty`, once it's genuinely
  safe to: reading `tty`'s own `.data` counter back out of *its* address
  space and checking it reads `5`, proving the loaded ELF binary's own
  code genuinely executed and wrote through to physical memory, not just
  that it trapped the expected number of times. This check used to run
  from `CLOCK` instead, right alongside the code-page read above -- but
  that raced `tty`'s actual progress against `CLOCK`'s own independent
  3-tick alarm, and real timing variance eventually lost that race (a
  reproducible boot panic, not a hypothetical one); see
  `elf_counter_demo`'s doc comment in `src/main.rs` for the full story.
  `CLOCK` then dynamically spawns a brand new task (`log`) at runtime --
  with the scheduler already running other tasks, not during
  `kernel_main`'s boot-time setup -- proving `proc::spawn` works as a
  genuine "start a new process now" primitive: the exact thing real `rs`
  needs to bring services up on demand, and now genuinely does, every
  time it restarts `flaky` (`src/rs.rs` above).
  Also runs an isolation self-test right after building `driver`'s
  address space: translating the ring-3 code page's address through the
  *kernel's own* page table returns `None`, proving the mapping really is
  private and not merely inaccessible-by-privilege-level.

### Ring-3 programs in Rust: `rust/user/` and the shell

`rust/user/` is a separate crate (excluded from the kernel's workspace,
with its own `.cargo/config.toml` and linker script -- see its
`README.md`) holding the first ring-3 programs written in Rust rather
than hand-assembled, and `neumann_rt`, the runtime they're written on:
this port's first real step on roadmap item 12, the libc-equivalent.

- `neumann_rt` (`src/lib.rs`, `src/sys.rs`): a naked `_start` that hands
  `main` the `argc`/`argv`/`envp` block `exec` built on the stack
  (`Args`, with `env()`/`var()`), turns `main`'s return value into
  `SYS_EXIT`, `print!`/`println!` through `SYS_CONSOLE_WRITE`, a panic
  handler exiting with 101, and one wrapper per system call (`sys`).
  No heap: nothing backs one yet.
- `sh` (`src/bin/sh.rs`, `com::SH_PROC_NR`, started at boot with
  `argv = {"sh"}` and `envp = {"PATH=/bin", "HOME=/"}`): the interactive
  shell. It reads a line (`SYS_READ_LINE`), echoes it (nothing else
  does), splits it into words, and runs it the way every Unix shell
  does -- `fork`, the child `exec`s `/bin/<word>` with the words as its
  `argv` and the shell's own environment, the parent `wait`s and reports
  a non-zero status (`[exit 1]`, `[exit 127]` for a missing command).
  Built in: `exit [status]` and `help`.
- `echo`, `cat`, `ptrtest` -- what it runs. `cat` uses the new
  `SYS_FS_OPEN_EXISTING` (`open` without `O_CREAT`; plain `SYS_FS_OPEN`
  creates what it opens, so `cat missing` used to leave an empty file
  behind). `ptrtest` hands the kernel bad pointers (above).

Everything a real program launch needs was already here; what this adds
is the first thing that *uses* it interactively. `pm`'s `seed_bin`
installs the programs at `/bin` at boot (`elf::RUST_PROGRAMS`, embedded
from `kernel/user/bin/`, which `rust/user/install.sh` fills). The
hand-assembled exec test that used to live at `/bin/echo` moved to
`/bin/exectest` so the real `echo` could have the name.

Kernel changes it needed: every ELF image now gets an 8-page stack
(`elf::STACK_PAGES`; compiled Rust wanted more than the one page the
assembly programs never came near), with `argv`/`envp` in the top page;
`SYS_CONSOLE_WRITE` (a program's standard output, as opposed to
`SYS_WRITE_LINE`'s prefixed log line); `SYS_FS_OPEN_EXISTING`; and
`console_task` queueing readers and lines (see "known simplifications in
the keyboard driver"). Verified interactively over QMP `send-key`:
`help`, `echo`, `cat` of files that exist and don't, an unknown command,
two commands typed back to back without waiting, `run hello` (which goes
to `rs` and not to the shell), `ptrtest`, and `exit 3`, with no faults.

Building it turned up a real copy-on-write bug that nothing before had
exercised: a forked child that ended up *sole owner* of a copy-on-write
page (its parent having copied first) got write permission back on the
page-table leaf only, while the tables above it -- created by `map_to`
with permissions derived from a read-only COW leaf -- stayed read-only,
so the child's next write faulted again, found nothing COW about the
page, and killed it. It was the shell's fifth command. `share_pages_into`
now spells out permissive table flags, and `cow_self_test` has a case
that fails without that fix. A review of copy-on-write and of
`exec`'s `argv` found more, all fixed alongside: the frame allocator
and heap locks are now taken only with interrupts off (the page-fault
handler takes both, and could otherwise spin on a lock held by a
preempted task); fork runs with interrupts off (a kernel task forking a
*different*, runnable process could lose isolation if the source ran
mid-fork); COW resolution translates and acts in one uninterruptible
step; a kernel write into a page shared *without* COW (program text)
now gets a private copy instead of rewriting every sharer's code;
`copy_between_address_spaces` no longer panics stepping past the top of
the lower half; and the syscall layer's pointer handling described in
"known simplifications in the syscall ABI".

Known gaps: `fs` has no `close`, so every `cat` leaks an open-file slot
in `fs`'s table (which grows without bound); output goes to COM1 only,
interleaved with the kernel's own log (the on-screen console is the
next milestone); no quoting, pipes, redirection, variables, job control
or current directory in `sh`.

### A real bug found and fixed by a multi-agent review

A workflow-based review (five independent reviewers each reading a
different slice of `rust/kernel/src`, every finding then re-verified from
scratch by an independent skeptic before being trusted) surfaced a genuine
race condition in `proc::reschedule`/`start`, found independently by three
of the five reviewers and confirmed by three separate verifiers: the
scheduler-lock-guarded part of the switch (`with_scheduler`, which sets
`sched.current = next`) released the lock -- and, for an ordinary
task-level caller, re-enabled interrupts -- *before* `switch_to` had
actually performed the low-level stack/`CR3` swap. A timer tick landing in
that gap ran `clock_tick` against a `sched.current` that didn't yet match
who was physically executing; if that expired the (wrong) task's quantum
and changed `next_ptr` again, the nested `reschedule` the timer handler
calls would `switch_to` using the not-yet-switched-to task's process-table
slot as the "outgoing" side -- overwriting its saved `rsp` with the real
outgoing task's live stack pointer and permanently corrupting it. Fixed by
disabling interrupts for the whole decide-and-switch sequence (not just
the lock-guarded part) and restoring them explicitly on resume, rather
than relying on `switch_to`'s own saved `RFLAGS` (see `reschedule`'s doc
comment for the full explanation). The same review also caught real
duplication between `reschedule`/`start` (now factored into
`Scheduler::cr3_for`) and a stale doc comment in `com.rs`; a fourth
finding (a `PAGE_SIZE` constant duplicating an existing named constant in
`usermode.rs`) was checked and correctly refuted as not a live bug.

### Known simplifications in the scheduler/IPC/timer port

- **Preemption latency for a newly-woken task is bounded by one timer
  tick, not fully instantaneous.** `reschedule()` runs after every IPC
  operation (so a `send`/`notify` that wakes a higher-priority task
  preempts immediately) and after every timer tick (so quantum expiry
  preempts within one tick, currently 1/60s) -- but real MINIX's
  `restart()` (`kernel/mpx386.s`) checks on *every* trap/interrupt return,
  which in a system handling real I/O happens far more often than once per
  tick. This is a latency/precision gap, not a correctness one.
- **No pending-notification bitmap**: `mini_notify` on a task that isn't
  blocked in `receive` at that exact instant is simply dropped, rather
  than queued in a per-process bitmap for later delivery like
  `kernel/proc.c`'s `s_notify_pending` does. (This is also why the timer
  handler manipulates the ready queues directly instead of routing through
  `mini_notify(CLOCK, ...)` the way `kernel/clock.c`'s interrupt handler
  notifies the `CLOCK` task in C MINIX: `mini_notify` locks the same
  scheduler state `clock_tick` is already holding, which would deadlock.)
- **SEND/SEND deadlock panics** instead of returning `ELOCKED`: there's no
  error-propagating IPC API yet for a caller to recover with.

### Known simplifications in the ring-3 task

- **Only three tasks have their own address space** (the ring-3 demo, its
  `sys_fork`ed child, and the ELF-loaded `tty` task); every other task
  still shares the kernel's. That's deliberate for now -- kernel tasks
  (`IDLE`, `CLOCK`) and the `pm`/`rs`/`memory` stand-ins all run
  kernel-trusted code today, so there's nothing to isolate them *from*
  yet -- but it means there's no isolation between, say, `pm` and `fs`
  either. That only starts to matter once real, mutually distrusting
  user-mode servers exist.
- **A new address space is a full clone of the kernel's page table**,
  not a minimal one built from scratch. This is simple and correct (every
  kernel mapping the task might need -- code, the heap, the
  physical-memory window -- is guaranteed present), and ring-3 code still
  can't actually *touch* any of it directly: those entries were never
  marked `USER_ACCESSIBLE`, so the CPU's own permission check (not
  anything this port adds) faults on an attempt from CPL 3, same as it
  would for any other address the task hasn't been given a `USER_ACCESSIBLE`
  mapping for.

### Known simplifications in the syscall ABI

- **Sixteen calls exist** (`SYS_FS_OPEN_EXISTING` and
  `SYS_CONSOLE_WRITE` are the newest -- see the `rust/user/` section
  above), and all but the unrecognized-call-number
  fallback reach real server/kernel-call logic, including `SYS_FORK`
  from any ring-3 caller now (see the `src/syscall.rs`/`src/proc.rs`
  bullets above). Real enough to
  write a genuine ring-3 program against (a demo user program can now get
  the time, print, sleep, do real file I/O, block for real keyboard
  input, read another process's memory, fork itself, replace its own
  image, terminate with a status, and collect a child's), but still a
  hand-picked set proving the dispatch mechanism works, not a real
  syscall surface.
- **Five arguments, not a full calling convention.** `rdi`/`rsi`/`rdx`/
  `rcx` carry the four ordinary caller-supplied arguments; `r9` carries a
  fifth, `SYS_FORK`-only one (`frame_ptr`, not something a caller passes
  intentionally -- see the `src/syscall.rs` bullet above) that isn't part
  of the ABI a normal call sees. A real syscall ABI (or MINIX's own
  message-based one) would want a real, uniform argument convention
  instead of one call quietly reaching around it.
- **A handful of distinct error codes, not a real `errno` set.**
  `dispatch` used to return a single `ERROR: u64 = u64::MAX` sentinel for
  every failure; it now returns one of five small negative-`i64`-as-`u64`
  codes (`ERR_BAD_LENGTH`/`ERR_BAD_UTF8`/`ERR_VIRCOPY_FAILED`/
  `ERR_UNKNOWN_CALL`/`ERR_FORK_UNSUPPORTED_CALLER`, mirroring the
  convention `crate::calls`/`crate::fs` already use for their own
  failures) -- one per *kind* of mistake this dispatch layer itself
  detects, not one per underlying cause the way a real `errno` would
  distinguish (e.g. every `SYS_VIRCOPY` failure from
  `calls::sys_vircopy` itself, whatever the reason, collapses to the same
  `ERR_VIRCOPY_FAILED`). Verified reaching a real ring-3 caller's `rax`,
  not just computed correctly inside `dispatch`: `tty` deliberately makes
  an invalid `SYS_VIRCOPY` call (`user/hello.s`) and a kernel task reads
  its raw return value back out afterward, checking it's exactly
  `ERR_BAD_LENGTH` (see `vircopy_error_from_ring3_verify` in
  `src/main.rs`). `SYS_FS_*` separately forward `fs`'s own real error
  codes through unchanged on top of these, since those are a distinct,
  already-real-`errno`-shaped failure mode with no need for a stand-in.
- **Pointers are validated by copying, one call's worth at a time.**
  Every pointer argument is read with `copy_from_caller` or written with
  `copy_to_caller` (after `check_writable`), both walking the caller's
  page tables and confined to PML4 slots the kernel leaves empty, so a
  bad pointer comes back as `ERR_BAD_ARG_PTR` instead of a ring-0 page
  fault. This used to be a raw dereference, and any ring-3 program could
  halt the machine with one bad pointer, read the kernel heap through
  `SYS_WRITE_LINE`, or write it through `SYS_FS_READ`; `SYS_VIRCOPY`
  could name a kernel task as its source (reading kernel memory) or the
  heap as its destination. `/bin/ptrtest` (run at boot by
  `crate::main`'s `ptr_safety_check`) holds seventeen such calls to
  refusing. What's still coarse: the copies go through one bounded
  kernel-stack buffer per call (256 bytes), and a write checked before a
  call blocks could in principle find the page gone afterwards (nothing
  here unmaps a page from a live process, so it can't today).
- **The error codes overlap.** `ERR_*` are numbered from -1 with no
  regard to the POSIX-numbered `fs` codes forwarded alongside them
  (`ERR_BAD_UTF8` and `ENOENT` are both -2, `ERR_NO_FREE_PROC` and
  `ENOEXEC` both -8), so a code only means one thing in the context of
  the call that returned it. `neumann_rt::sys::strerror` names the reading
  most calls mean; renumbering the ABI would fix it properly.
- **`entry`'s register save list is fixed and total** (all 15
  general-purpose registers, every call), rather than saving only what a
  real syscall convention requires (e.g. SysV's syscall-clobbered set) or
  varying by call number. Correct and simple; a hair more work per trap
  than strictly necessary.
- **`SYS_WAIT_ALARM` doesn't check that a `SYS_SET_ALARM` actually
  preceded it.** It just calls `ipc::receive(CLOCK)`, the same as
  `crate::proc::clock_task` does directly -- a caller that waits with no
  alarm pending simply blocks forever, since nothing will ever notify it.
  There's also no way for a caller to wait for more than one kind of
  event at once, or to time out.

### Known simplifications in the kernel calls

- **`sys_setalarm`, `sys_vircopy`, and `sys_fork` (via its `sys_fork_from_frame`
  sibling) are all reachable from ring 3 now** (via `crate::syscall`'s
  `SYS_SET_ALARM`/`SYS_WAIT_ALARM`, `SYS_VIRCOPY`, and `SYS_FORK`, a real
  register-based call-number dispatch -- see "known simplifications in
  the syscall ABI" above). None of these are entries in a call-number
  dispatch table reached via a message to `SYSTEM` (`kernel/system.c`'s
  `map(SYS_xxx, do_xxx)`) the way real MINIX's kernel calls are --
  `crate::syscall`'s register-based dispatch stands in for that here,
  same as it does for everything else this port reaches from ring 3.
- **Two `sys_fork`s, not one.** `calls::sys_fork` (used by `pm`'s own
  kernel-side demo, the `pm`/`init` canary test in `main.rs`) still starts
  its child at a fixed entry point, not "wherever the parent was" --
  real, working process creation with genuinely independent memory, just
  not full POSIX continuation semantics. `sys_fork_from_frame`
  (`SYS_FORK`'s ring-3 handler) is the sibling that actually achieves
  that: it hands `proc::fork_current` a full `proc::TrapFrame` snapshot
  of the caller's own trap instead of a `fn() -> !`, so the new task
  resumes at the caller's *exact* trapped instruction, in ring 3, seeing
  `0` where the parent sees the child's `proc_nr` -- genuine `fork()`
  semantics, verified by `tty` (`user/hello.s`) forking itself and the
  child immediately diverging (a canary write proving its data page is a
  real, independent copy -- not aliased with the parent's -- plus its own
  `SYS_FS_OPEN`/`SYS_FS_WRITE` to a distinct file, all checked back from
  `IDLE` afterward; see `fork_child_verify` in `src/main.rs`).
- **Which pages to copy is read out of the process table now**, not
  passed in by whoever happens to know. `Proc::mem_map`
  (`crate::memory::MemMap`, filled in by whoever built the address space
  -- see the `src/memory.rs` bullet above) is this port's `mp_seg[]`, so
  `fork` is a call any process with an address space of its own can
  make: `crate::syscall`'s dispatch no longer keeps a hardcoded page list
  per *caller*, and `ERR_FORK_UNSUPPORTED_CALLER` now means the much
  narrower "a kernel task asked to fork, and its memory is the kernel's"
  rather than "this isn't the one task we know about". Child process
  numbers are allocated at runtime too (`proc::alloc_proc_nr`,
  `com::FIRST_DYNAMIC_PROC_NR`), so there is no single reserved child
  slot for a second fork to overwrite. `fork` is copy-on-write now (see
  the `src/memory.rs` bullet), with two rough edges: a kernel-mode COW
  fault that finds no free frame to copy into (inside `SYS_FS_READ`,
  say) halts the machine, like any ring-0 fault, rather than failing
  that one syscall with `EFAULT`; and `SHARERS` is a heap `BTreeMap`
  behind a lock taken with interrupts off, fine on one CPU and one of
  the first things SMP would have to redo. There's also
  a real process *lifecycle* beyond fork/exit/wait: there is no process
  group, session, or controlling terminal; no signals, so no way to ask
  another process to stop (only `kill`, which the kernel does to a
  process that faulted, not something anyone can request); no `waitpid`,
  so a parent can only wait for *some* child rather than a named one,
  and can't poll (`WNOHANG`); and an orphan is not reparented to `init`
  the way a real system does it -- `terminate` frees the slot of a
  process whose parent is already gone, on the grounds that nothing can
  ever collect its status, which is the right outcome but by a different
  route. A process blocked trying to `send` to one that then terminates
  stays blocked forever, too, where MINIX would fail it with
  `EDEADSRCDST`; nothing in this port sends to a process that exits.
- **`sys_vircopy` skips validation** real MINIX's `do_copy` does first:
  resolving `SELF` to the caller's own process number, and rejecting
  invalid process numbers. Every current caller already knows both real
  process numbers, so this hasn't mattered yet.
- **`sys_setalarm` is notification-only.** Real MINIX also lets a caller
  register an in-kernel watchdog callback (a function pointer run directly
  by `do_clocktick`, no message involved) instead of a `SYN_ALARM`
  notification; nothing in this port has an in-kernel watchdog to
  register yet, so only the notification path exists.
- **`clock_tick` scans every process table slot every tick** to check for
  an expired alarm, instead of keeping a sorted timer queue and checking
  only `next_timeout <= realtime` like `kernel/clock.c` does. Fine at
  `NR_PROCS` scale; would need the sorted-queue approach at real scale.

### Known simplifications in `fs`

- **A real but shallow directory hierarchy.** `mkdir`/`open` check a
  path's immediate parent (`ENOENT`/`ENOTDIR`/`EISDIR`/`EEXIST`, see the
  `src/fs.rs` bullet above), but there's still no `readdir`/listing, no
  `unlink`/`rmdir`, no `stat`/permissions/inode-number concept, and no
  relative paths or `.`/`..` -- every path is a full, absolute string
  compared exactly, not a real walk through directory-entry blocks the
  way `servers/fs/path.c`'s `lookup()` does.
- **No cross-address-space copy.** Request/reply args carry raw pointers
  valid in the caller's address space directly, since `fs` and every
  current caller (`pm`) still share the kernel's own address space (see
  "known simplifications in the ring-3 task" above); a real, isolated
  `fs` server would need a `sys_vircopy`-style copy for every buffer, the
  same way `sys_vircopy` itself does for `sys_fork`'d tasks.
- **`open` still always creates** (`open_existing` is opt-in, not the
  default, and there is no `O_CREAT`-style flag set for a caller to
  express anything finer), and there's no `close`: a file descriptor is
  never freed once allocated (`InMemoryFs::open`'s slot table only ever
  grows), and repeated opens of the same name return independent
  descriptors with independent cursors rather than sharing or refusing
  based on any open-file-table policy.
- **In-memory only.** There's no backing device, so nothing here survives
  a reboot -- there's no block layer, block cache, or on-disk layout at
  all (`kernel/kernel.h`'s device abstractions, `servers/fs`'s
  buffer cache), just a `Vec<(String, Vec<u8>)>` that lives as long as
  the kernel does.
- **Still missing some error variety.** `FS_READ`/`FS_WRITE` only ever
  return `EBADF` for any failure (bad descriptor and unrecognized
  request alike); there's still no `EACCES`, `ENOSPC`, or a distinct
  error for a `read`/`write` on a directory's own descriptor (not
  possible yet, since directories can't be `open`ed at all).

### Known simplifications in the ELF loader

- **The binary is checked in pre-built, not assembled by this build.**
  `user/hello.elf` is produced by a manual `as`/`ld` invocation (see
  `user/hello.s`'s header comment), not a build-script step -- there's no
  cross toolchain wired into `cargo build` yet to assemble a fresh user
  program automatically (see the libc-equivalent roadmap item).
- **No dynamic linking, an empty auxv, no `PT_INTERP`.** `load` only
  understands `PT_LOAD` segments. The initial stack a libc's `_start`
  expects is there now (`argc`/`argv`/`envp` in the System V layout, see
  `write_initial_stack`), but its auxiliary vector is only the `AT_NULL`
  terminator -- no `AT_PHDR`/`AT_ENTRY`/`AT_PAGESZ`/`AT_RANDOM`, which a
  real libc reads -- and there's no dynamic linker to name in a
  `PT_INTERP` segment. Only exec'd images get arguments at all; images
  started at boot or by `rs` (`spawn_from_fs`) get `argc == 0`, not even
  their own name as `argv[0]`.
- **One fixed stack address, one page of it.** Every image gets its
  stack at a hardcoded `STACK_ADDR`, one 4 KiB page, the same way
  `usermode.rs`'s demo uses fixed constants. Harmless while each image
  has its own address space, but not a general "load any binary
  anywhere" API (no ASLR, no picking an unused range, no stack growth).
- **Validation covers safety, not program sanity.** `validate` rejects
  everything that could hurt the kernel or another process (see the
  `src/elf.rs` bullet above), and `elf::validator_self_test` holds it to
  that. What it still doesn't check is the class of thing that only
  produces a *broken program*: `p_align` disagreeing with `p_vaddr`, an
  entry point that doesn't land in an executable segment, or a stack
  that is one page whether the program wanted more or not. An earlier
  version of this bullet listed "`PT_LOAD` segments overlapping each
  other" here as merely cosmetic -- it wasn't; it panicked the kernel
  from ring 3, and it is now a rejected case with a self-test behind
  it.
- **Three images, all built out of band.** `HELLO_ELF`, `SHELL_ELF`, and
  `ECHO_ELF` are `include_bytes!`d from hand-assembled files; the last of
  those only so `pm` has something to install at `/bin/exectest`. Loading by
  path out of `fs` is real (`read_file`), but what's *in* `fs` still
  ultimately comes from the kernel binary.

### Known simplifications in `exec()`

- **`argv`/`envp` are real, but small.** `SYS_EXEC` copies both vectors
  out of the address space it's about to destroy and reconstructs them
  on the new image's stack, as a real `execve` does (`user/shell.s`
  execs `/bin/exectest` with four arguments and one environment string, and
  `user/echo.s` genuinely echoes them; `crate::main`'s `argv_verify`
  reads the start-up block back out of the child's stack and checks the
  layout word by word). What's small is the budget: everything has to
  fit in half of the one stack page (`elf::MAX_START_ARGS_BYTES`, 2 KiB,
  where MINIX's own `ARG_MAX` is 16 KiB), and at most
  `MAX_EXEC_VECTOR` (64) entries per vector. The copy-in reads strings
  64 bytes at a time through a page-table walk -- correct and bounded,
  not fast.
- **The replaced image *is* freed now** (`memory::free_address_space`,
  called from `proc::set_address_space` once the new `CR3` is loaded),
  so `exec` no longer costs an address space per call. What is still
  missing is the rest of a real teardown: `fs` descriptors the old image
  opened stay open (there is no `close`), and a partially-mapped image
  rejected mid-way leaves the frames it already took behind, because
  `load_image` frees nothing on the error path -- it only avoids
  allocating in the first place, which covers every case `validate`
  catches but not a `MappingFailed` in the middle.
- **The whole image is read into the heap first.** `read_file` assembles
  the entire program into a `Vec<u8>` over repeated `fs` round trips,
  then copies it page by page into the new address space -- two full
  copies of every byte, and a hard dependency on the heap being big
  enough for the largest program. Real systems demand-page an executable
  straight from the file cache.
- **No permission, ownership, or `#!` handling.** Anything can exec
  anything readable: no execute bit (`fs` has no mode bits), no set-uid,
  no `EACCES`, and no interpreter line (a shell script comes back as a
  malformed image, which is one of the self-test's cases). File
  descriptors survive the call, since `fs` has no close-on-exec flag --
  which happens to match what POSIX does for descriptors *without* the
  flag set, so it's a missing feature rather than wrong behavior.
- **Per-process state that should be reset isn't.** A pending
  `sys_setalarm` deadline survives exec, and so would a signal
  disposition if this port had signals (real `exec` resets handled
  signals to their default). The process's priority, quantum, and
  scheduling history carry over too, which real `exec` also does -- but
  here that's because nothing touches them, not because it was decided.
- **A failed `exec` is a no-op, and that is load-bearing rather than
  incidental.** Nothing is mutated until the image is fully validated
  and mapped, the path is resolved with `fs::open_existing` so a missing
  program creates nothing, and the caller keeps running its own image --
  checked from both sides (`user/shell.s` survives a deliberately-failed
  exec and writes the error code to a file afterward;
  `crate::main`'s `missing_program_check` confirms a missing path
  reports `ENOENT` and leaves no file behind). What is *not* guaranteed:
  an image rejected *after* mapping began (`ElfError::MappingFailed`)
  leaves its already-mapped frames behind -- `validate` is what makes
  that unreachable in practice, not an unwind path.
- **The path pointer from ring 3 is dereferenced without validation**,
  like every other pointer argument in `crate::syscall` (`dispatch`
  bounds the *length* against `MAX_FS_BUF` but takes `arg1` on trust and
  reads it directly). A caller passing an unmapped or non-canonical
  pointer faults inside the kernel, and `interrupts::recover_or_halt`
  only converts faults with a *ring-3* saved `CS` into `proc::kill` --
  a fault in `dispatch` has a ring-0 `CS` and halts the machine. Closing
  this needs a real `copy_from_user` with fault fixup (a fault handler
  that can resume at a recovery address), which this port has nowhere
  yet; it is a pre-existing property of the whole syscall surface rather
  than of `exec`, but `exec` is a conspicuous place to meet it.
- **It composes with `fork` for real now.** `sys_exec` was always
  general -- replacing an address space needs no per-caller knowledge --
  but the demo used to have `shell` exec over *itself*, which proves exec
  and makes a nonsense shell (the process that asks for a program is the
  one that stops existing). `user/shell.s` forks first now, and the
  child is what execs, which is the shape every real program launch has.
  That combination is also what first made `fork`'s old page-sharing
  unsound in practice rather than in theory: the child frees the image it
  inherited the moment it execs. `crate::main`'s `exec_verify` checks
  both address spaces afterward -- the exec'd image's `.data` is readable
  in the child and unmapped in the parent, and `shell`'s own marker page
  is the reverse -- so "exec reached the right process" is checked, not
  assumed.

### Known simplifications in the VGA graphics

- **One fixed 256-color mode (320x200), not VBE/a linear high-resolution
  framebuffer.** Mode 13h is the simplest possible real VGA graphics
  mode to drive (no bank switching, no mode-setting protocol beyond one
  `int 0x10` call) and is exactly what the `bootloader` crate's
  `vga_320x200` feature already sets up; a real desktop UI would want a
  much higher resolution and color depth (the `bootloader` crate has no
  VBE/framebuffer feature of its own to build on for that -- see the
  long-term direction note above).
- **A single, fixed, hardcoded layout.** `draw_demo_panel` always draws
  the same bars/buttons at the same coordinates; there's no generic
  "layout a panel of N elements" API or windowing yet -- this is a
  static image with one piece of live state (`select_button`'s highlight),
  not a UI. Selection is keyboard-driven only (digit keys `1`-`4`, see
  `crate::keyboard` below) -- there's no mouse, no hit-testing against
  button bounds, and no notion of "clicking" a button (as opposed to
  just selecting/highlighting it).
- **No double buffering.** `draw_demo_panel` writes directly to the
  live, currently-displayed framebuffer; fine for one paint that never
  changes again, but a real UI redrawing every frame would need to draw
  off-screen and flip, to avoid visible tearing.
- **`fill_rounded_rect`'s corner circles are computed per pixel, every
  call**, rather than precomputed into a reusable mask or drawn via a
  midpoint-circle algorithm. Fine at 320x200 and a handful of shapes;
  would matter at a much bigger framebuffer or redrawn every frame.

### Known simplifications in the keyboard driver

- **No shift/caps-lock/ctrl/alt state tracking.** `keyboard::translate`
  always uses the unshifted mapping (lowercase letters, unshifted
  punctuation) regardless of which modifier keys are actually held, and
  has no translation at all for modifier keys, F-keys, arrows, or any
  other non-alphanumeric key -- real MINIX's keyboard driver tracks this
  state (`drivers/tty/keyboard.c`) to produce the right shifted/control
  character.
- **Scancode set 1 only**, and only the alphanumeric/punctuation block
  (`0x02`-`0x39`); no handling for scancode set 2/3, the `E0`-prefixed
  extended keys (arrows, the right-hand Ctrl/Alt, ...), or PS/2
  controller configuration beyond what the BIOS/QEMU already leaves in
  place at boot.
- **Digit keys `1`-`4` still bypass IPC entirely**: `select_button` is a
  direct function call from the interrupt handler, unlike `on_char`'s
  real notify-a-task path. Two different keys of the same keypress take
  two genuinely different routes to their effect, which is a little
  inconsistent, if harmless (both are legitimate ways a keypress can have
  an effect; this port just hasn't unified them).
- **`console_task` is not a real `tty`/line discipline.** There's no
  echo (the typed character isn't drawn anywhere -- only the *effect*,
  the completed line reaching `fs`, is observable), no editing
  (backspace, cursor movement), no notion of multiple terminals/sessions,
  and no real `DEV_READ`/`DEV_WRITE` device-driver protocol -- it talks
  to `crate::fs` directly, standing in for a real `tty` server this port
  doesn't have (see `rust/README.md`'s roadmap). `LINE_CAPACITY` (`64`)
  is a hard cutoff with no overflow signal: a character typed past it is
  silently dropped.
- **One shared, un-timestamped line buffer.** `LINE` holds exactly one
  pending line; `on_char` keeps accumulating into it even before
  `console_task` has taken the previous completed one (simple
  typeahead), but there's no queue of *multiple* completed lines --  if
  `console_task` somehow fell behind by more than one full line (it
  doesn't, in practice, since it does no work slow enough to matter),
  a still-unread completed line would simply be overwritten.
- **`/console.log` is opened once, for the task's whole lifetime**, and
  every line is appended through that same descriptor -- correct, but
  only because `crate::fs` has no explicit "append" mode to fall back on
  (see "known simplifications in `fs`" above); re-opening on every line
  would silently overwrite from the start each time instead.
- **Readers and lines are both queued, first-come first-served, and
  bounded.** `console_task` keeps every blocked `SYS_READ_LINE` caller
  (`pending_readers`) and every line typed while nobody was reading
  (`unread`, at most 16), and pairs them oldest-first -- so a command
  typed while the shell is still running the last one waits its turn
  rather than vanishing, and two readers each get their own lines. (The
  first version tracked one reader in a single `Option` and dropped
  lines nobody was waiting for; before that, typing ahead appended new
  keystrokes onto the completed line, merging two lines into one.) The
  keyboard IRQ holds up to 8 completed lines for `console_task`
  (`COMPLETED_LINES`); past either bound a line is dropped, with a log
  line for the `unread` case. There's no notion of a *foreground* reader:
  whoever asked first gets the next line.
- **`run <name>` is the console's, not the shell's.** A line starting
  `run ` goes to `rs` (`dispatch_run`) and is *not* handed to a reader,
  so the shell never sees it -- the one command the console still
  interprets itself.
- **No echo, no editing, no Shift.** The shell prints each line back
  after it's entered (`sh` does that itself); nothing echoes keystrokes
  as they're typed, Backspace isn't handled, and with no modifier state
  there are no capitals or shifted punctuation -- `_` can't be typed, so
  neither can a path containing one.

### Known simplifications in `rs`/crash recovery

- **Only ring-3 faults recover; `flaky` and the one-entry `SERVICES`
  table are the only things with a restart policy.** A fault in
  kernel-trusted code (any kernel task, or any of the stand-in server
  bodies that still share the kernel's own address space) still halts the
  whole machine (see `crate::interrupts`'s doc comment for why that's
  still the right call). `rs::task` restarts `FLAKY_PROC_NR` (hardcoded
  parameters, `rs::spawn_flaky`) and any process number found in
  `SERVICES` (`crate::rs::SERVICES`, currently one entry: `hello`); a real
  `com::proc_died` for anything else is logged and then permanently
  ignored. `SERVICES` is a real, if tiny, version of
  `servers/rs/manager.c`'s `struct rproc` table -- unlike `flaky`'s
  hardcoded parameters, entries are looked up and launched by name
  (`rs::launch_service`) -- but it still has exactly one entry, populated
  by a boot-time demo step (`main.rs`'s `seed_bin_hello`) rather than a
  real installer, and nothing yet adds entries to it at runtime.
- **The "a process died" notification is fire-and-forget**, like every
  other notification in this port (see "known simplifications in the
  scheduler/IPC/timer port" above) -- if `RS` isn't already blocked in
  `receive` at the exact moment `proc::kill` runs, the notification is
  silently dropped and the dead process is never restarted. `rs` is
  spawned at a strictly higher priority than `flaky` specifically to
  avoid this race (see the `src/main.rs` bullet above), rather than the
  notification itself being reliable.
- **Teardown reclaims memory but not IPC state.** `kill` now does free
  a dead process's address space (`memory::free_address_space`, after
  moving `CR3` off it if the process died while current), so `flaky`'s
  restart cycle no longer costs an address space per crash -- each round
  hands nine frames back and the next restart reuses them. It still
  doesn't unwind IPC state: `caller_q` links, in particular, are left as
  they were beyond removing the process from the ready queues. Fine for a demo that only ever kills a lone,
  never-blocked-on-by-anyone-else ring-3 task; not fine for a real,
  general "any process can die at any time" story.
- **A crash is only ever `ud2`.** `flaky`'s address space and stack page
  are correctly set up (so a *real* bug -- a bad pointer dereference, a
  divide by zero -- would be caught exactly the same way, since
  `recover_or_halt` doesn't care which exception vector fired), but
  nothing in this port deliberately exercises `#PF`/`#GP`/`#DE` recovery
  the way it exercises `#UD`; `ud2` was chosen purely because it's the
  one fault the architecture guarantees regardless of memory layout.

## What's not implemented yet (roadmap)

Roughly in the order the original kernel needs them:

1. ~~**GDT/IDT and exception handling**~~ — done (`src/gdt.rs`,
   `src/interrupts.rs`). CPU faults are now reported instead of silently
   triple-faulting.
2. ~~**A real scheduler and process table**~~ — done (`src/proc.rs`).
   Priority ready queues, blocking `send`/`receive`/`notify`, and real
   context switching between kernel tasks.
3. ~~**A timer interrupt**~~ — done (`src/pic.rs`, `src/pit.rs`). Real
   PIC remap and PIT programming; `clock_tick` genuinely decrements
   `ticks_left` and reorders the ready queues on hardware ticks.
4. ~~**Asynchronous preemption**~~ — done. The timer interrupt handler
   calls `reschedule()` directly, which can and does switch stacks from
   inside an interrupt handler; the `rs`/`memory` demo tasks in `main.rs`
   prove a task that never blocks or yields gets preempted mid-loop and
   later resumes exactly where it left off. See "known simplifications"
   above for the (latency-only) gap that's left.
5. ~~**A physical frame allocator and heap**~~ — done (`src/memory.rs`,
   `src/allocator.rs`).
6. ~~**Prove the ring-3 transition mechanism**~~ — done (`src/usermode.rs`,
   plus the user segments/`RSP0`/`SYSCALL_VECTOR` additions to `src/gdt.rs`
   /`src/interrupts.rs`).
7. ~~**Per-task `RSP0` and scheduler-integrated user-mode tasks**~~ — done.
   `proc::reschedule`/`start` call `gdt::set_rsp0` on every switch; the
   ring-3 demo task now runs as an ordinary, asynchronously-preemptible
   `crate::proc` task making repeated `int 0x80` round trips, rather than
   a one-shot excursion. See "known simplifications" above for what's
   still fixed/hardcoded about it.
8. ~~**Per-process page tables**~~ — done (`memory::new_address_space`,
   used by `usermode::create_address_space`). The ring-3 demo task now
   runs in a genuinely separate address space, confirmed by an isolation
   self-test in `main.rs` (translating its code page through the kernel's
   own page table returns `None`). See "known simplifications" above for
   what's still narrow about it: only this one task has its own address
   space, and it's a full clone of the kernel's rather than a minimal one.
9. ~~**Kernel calls**~~ — started (`src/calls.rs`): `sys_vircopy` (backed
   by `memory::copy_between_address_spaces`), `sys_setalarm` (backed by
   `proc::set_alarm`/`clock_tick`), and `sys_fork` (backed by
   `memory::fork_address_space` + `proc::spawn`). `CLOCK` uses the first
   two for real instead of polling; `pm` uses the third to create a real
   child process with a genuinely independent, deep-copied address space
   (verified with a canary write). `sys_vircopy` and `sys_fork` are now
   both also reachable from ring 3 (`crate::syscall`'s `SYS_VIRCOPY`/
   `SYS_FORK`, see the `src/syscall.rs`/`src/proc.rs` bullets above): `tty`
   reads a byte range out of `driver`'s own private memory straight from
   its own ring-3 code (verified by `IDLE` reading the result back out of
   *`tty`'s* memory afterward with a second, independent `sys_vircopy`
   call), and forks itself into a genuine child process that resumes at
   the exact same ring-3 instruction rather than a fixed entry point --
   real `fork()` semantics this time, not `calls::sys_fork`'s own
   fixed-entry-point shape (verified by the child's canary write and its
   own independent `SYS_FS_OPEN`/`SYS_FS_WRITE`, both checked back from
   `IDLE`). See "known simplifications" above for what's not implemented
   yet (`sys_umap`, more of `kernel/system/do_*.c`). `fork` is
   copy-on-write now: the child shares every page, and a write fault
   copies it on demand (`memory::resolve_cow_fault`). `exit`/`wait` are real now too
   (`proc::exit_now`/`wait_for_child`, `crate::syscall`'s
   `SYS_EXIT`/`SYS_WAIT`, ported in spirit from
   `servers/pm/forkexit.c`), which is what finally makes a dynamic
   process number reclaimable: collecting a child's status is what
   releases its slot. Which pages a
   fork copies and which process number its child gets are no longer
   hardcoded per caller, though: both come from real bookkeeping now
   (`crate::memory::MemMap` in `Proc::mem_map`, and
   `proc::alloc_proc_nr`), so `SYS_FORK` works for any ring-3 process
   rather than the one it was written for.
10. ~~**A minimal ELF loader**~~ — done (`src/elf.rs`). Parses a real,
    statically linked ELF64 binary (`user/hello.elf`) and maps its
    `PT_LOAD` segments into a fresh address space at their own specified
    addresses and permissions (not one hand-placed page), zero-filling
    BSS; a second ring-3 task (`tty`) runs it, making real syscalls
    (`src/syscall.rs`) through the same gate `driver`'s own demo uses, and
    `IDLE` reads its `.data` counter back via `sys_vircopy` afterward to
    confirm the loaded code genuinely executed and wrote through to
    physical memory, not just that it made the syscalls it logged. See
    "known simplifications in the ELF
    loader" below for what's missing (no dynamic linking, only an
    empty auxiliary vector, the binary is checked in pre-built rather
    than assembled by this build).
11. **The servers themselves**: `pm` (process manager), `fs` (file system),
    `rs` (reincarnation server), `tty`, `memory`, in roughly that dependency
    order, matching `servers/` and `drivers/` in the C tree -- replacing the
    temporary stand-ins in `main.rs`. Four slices done: `proc::spawn` is
    proven safe to call from an already-running task, not just
    `kernel_main`'s boot-time setup (`clock_task` dynamically spawns `log`
    at runtime); `sys_fork` gives a task a real way to create a child with
    its own independent memory (`pm`'s `init` demo); `fs` (`src/fs.rs`) is
    now a real, in-memory file server answering genuine open/read/write
    requests over IPC, not a fixed-reply stand-in (`pm`'s open/write/reopen/
    read/read-past-EOF demo), with a real (if shallow) directory hierarchy
    on top (`mkdir`, and `open` enforcing `ENOENT`/`ENOTDIR`/`EISDIR`/
    `EEXIST` against a path's parent -- `pm`'s directory demo); and `rs`
    (`src/rs.rs`) is now a real reincarnation server that detects a crashed
    ring-3 process (`crate::proc::kill`, called from a CPU exception
    handler) and restarts it, up to a bounded number of times, exactly
    MINIX's signature self-healing behavior (`flaky`, a task that
    deliberately crashes via `ud2` every time it runs). `rs` now also has
    a real (if minimal, one-entry) service table (`crate::rs::SERVICES`)
    deciding what to start and restart by name rather than only a single
    hardcoded demo, launchable on demand via a console `run <name>`
    command (see the `src/rs.rs`/`src/keyboard.rs` bullets above).
    `sys_fork`'s "child resumes at the parent's exact call site" gap is
    closed (`crate::syscall`'s `SYS_FORK`/`sys_fork_from_frame`, see item
    9 above), and so is its other half: `exec()` is real
    (`crate::calls::sys_exec`, `crate::syscall`'s `SYS_EXEC`) -- a
    process loads a *different* program out of `fs` by path and replaces
    its own image with it, keeping its process number, priority, kernel
    stack and open descriptors, and resuming in ring 3 at the new
    binary's entry point on a freshly mapped stack, in an address space
    that no longer contains a single page of what it used to be running
    (proven by `crate::main`'s `exec_verify`, which requires reading the
    old image's `.data` back to *fail*). A deliberately-failed exec of a
    non-ELF file is checked too, since "a failed exec leaves the caller
    exactly as it was" is the guarantee most easily broken here.
    The two now compose the way a shell composes them, which is the point
    of having both: `shell` (`user/shell.s`) *forks*, and its child is
    what execs `/bin/exectest`, while the parent goes on being `shell`. That
    needed the last two things keeping `fork` from being a general call
    -- a per-process memory map saying which pages a fork must copy
    (`crate::memory::MemMap`, in `Proc::mem_map`), and a process number
    handed out when the fork happens rather than reserved per caller in
    `crate::com` (`proc::alloc_proc_nr`) -- so `SYS_FORK` is now
    something any ring-3 process can call, as many times as there are
    slots, instead of something one hardcoded task could do once. The
    rest of the cycle is there as well: `shell`'s other two children
    `SYS_EXIT` with statuses of their own and `shell` collects both with
    `SYS_WAIT` -- one handed over directly because the parent was
    already blocked waiting, the other collected out of a zombie slot it
    had been sitting in since it terminated. That is the whole
    fork/exec/exit/wait cycle a real program launch is made of, running
    in ring 3 -- and the exec carries a real `argv`/`envp` across, laid
    out on the new stack the way the System V ABI has it, which
    `/bin/exectest` then actually echoes. Still
    missing: `fs`
    growing `readdir` and a real backing store (see "known simplifications in
    `fs`" above) rather than a flat, in-memory, single-address-space
    file/directory table. Ring-3 callers can now reach `fs` for real
    (`crate::syscall`'s `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ`), but
    only because the *syscall layer* copies buffers through a
    kernel-stack buffer around the IPC call -- `fs` itself still has no
    `sys_vircopy`-style cross-address-space copy of its own for a caller
    that reaches it some other way.
12. **A libc-equivalent** for whatever runs in user mode, mirroring `lib/`
    -- started: `rust/user/`'s `neumann_rt` (entry point and `argv`/`envp`,
    a wrapper per system call, formatted console output, a panic
    handler), and the first programs written on it, an interactive shell
    among them (see "Ring-3 programs in Rust" above). Missing: a heap,
    and everything a heap would enable.
13. ~~**Real graphics output**~~ — started (`src/vga.rs`): VGA mode 13h
    (320x200, 256-color), a real palette (VGA DAC ports) and a real
    linear framebuffer, `fill_rect`/`fill_rounded_rect` drawing
    primitives, and a static LCARS-style demo panel painted on boot. This
    is the first concrete step toward the BeOS/Haiku-flavored desktop-OS
    direction noted above, not part of MINIX-fidelity roadmap items 1-12.
    See "known simplifications in the VGA graphics" above for what's
    missing (a higher-resolution/color-depth mode, a real layout/windowing
    system, input handling, double buffering) before this is a UI rather
    than a static image.
14. ~~**Real keyboard input**~~ — done (`src/keyboard.rs`): IRQ1
    unmasked, a real PS/2 scancode read off port `0x60` and translated to
    ASCII (unshifted alphanumeric/punctuation only), proven asynchronous
    and hardware-driven -- it wakes the CPU from `IDLE`'s `hlt` the
    instant a key is pressed, the same way the timer already does every
    tick. Digit keys `1`-`4` are wired all the way through to
    `vga::select_button`, so a keypress visibly changes the framebuffer
    (a real, if minimal, input-to-output loop) -- verified headlessly via
    QMP `send-key` followed by a `screendump` showing the matching
    button's highlight frame move. A real line discipline sits on top
    now too: every translated character feeds a shared line buffer, and
    a newline notifies `console_task`, a real scheduled task that writes
    the completed line to a real file via `fs` -- the "big one" gap this
    item used to cite (anything reaching a real process, not just calling
    straight into `crate::vga`) is closed, verified interactively by
    typing `"hi"` and `"world"` (each terminated with Enter) well after
    boot, once `IDLE` had already halted. `console_task` now also answers
    real `SYS_READ_LINE` requests from ring 3 (`crate::syscall`): `tty`
    blocks for an entire line of real, human-timed keyboard input from
    inside its own trap -- first proven by `tty`, now what the shell
    (`sh`, `rust/user/`) reads every command with, readers and typed-ahead
    lines both queued first-come first-served. `console_task`'s
    line discipline can now also dispatch a `"run <name>"` command to `rs`
    (see item 11 above), not just log/echo the line. See "known
    simplifications in the keyboard driver" above for what's still missing
    (modifier-key state, extended scancodes, echo/editing, no notion of a
    foreground reader, and a real `tty` server this port's
    `console_task` stands in for by talking to `fs` directly).
15. ~~**A real syscall ABI**~~ — done (`src/syscall.rs`). Replaced
    `usermode`'s old fixed-action, count-and-cut-off `int 0x80` handler
    with genuine call-number/register dispatch: a hand-written naked trap
    gate saves every general-purpose register, reads the caller's `rax`
    (call number) and `rdi`/`rsi`/`rdx`/`rcx` (up to four arguments, since
    grown from three -- see the `src/syscall.rs` bullet above),
    dispatches to one of sixteen calls (`SYS_GET_UPTIME`; `SYS_WRITE_LINE`
    -- a real cross-ring pointer argument, read directly since entering a
    trap gate never switches `CR3`; `SYS_SET_ALARM`/`SYS_WAIT_ALARM` --
    the first of `crate::calls`' own kernel calls reachable from ring 3,
    and the first syscall here that genuinely blocks the caller and
    resumes it later rather than always returning immediately;
    `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ` -- a real IPC round trip to
    `fs`, copying pointer arguments through a kernel-stack buffer first
    since `fs` runs with a potentially different `CR3` by the time it
    dereferences anything; `SYS_READ_LINE` -- the same kernel-stack-buffer
    trick in the input direction, blocking for however long it takes a
    human to actually type a line (`crate::keyboard::read_line`); and
    `SYS_BLOCK_FOREVER`), and writes the
    result back into `rax` before `iretq`. Both ring-3 tasks (`usermode`'s
    hand-assembled demo and `crate::elf`'s loaded ELF binary) now
    exercise real syscalls, each ending on its own terms
    (`SYS_BLOCK_FOREVER`) instead of being cut off by a kernel-side
    iteration counter; `tty` (the ELF-loaded one) additionally sets a
    real 3-tick alarm and blocks waiting for it, verified in QEMU to
    resume in ring 3 exactly where it left off once the alarm fires,
    with the rest of the system (`CLOCK`'s own alarm, the dynamically
    spawned `log` task, `rs`/`memory`'s preemption) continuing normally
    the whole time it's blocked, and then opens and writes a real file
    over `SYS_FS_OPEN`/`SYS_FS_WRITE`, verified by `IDLE` reading it back
    afterward through a completely independent path, and finally blocks
    on `SYS_READ_LINE` for a real, human-timed line of keyboard input
    before writing *that* to a second file -- verified end to end by
    typing a line over QMP well after boot. `tty` also now calls
    `SYS_VIRCOPY`, reading a range of `driver`'s own private memory
    straight from ring 3 into its own buffer, verified by `IDLE` reading
    that buffer back afterward with a second, independent `sys_vircopy`
    call (see item 9 above), then deliberately calls it again with an
    invalid length to exercise the ABI's now-distinct error codes
    (`ERR_BAD_LENGTH`/`ERR_BAD_UTF8`/`ERR_VIRCOPY_FAILED`/
    `ERR_UNKNOWN_CALL`, replacing a single undifferentiated `u64::MAX`
    sentinel), verified the same way -- a kernel task reads the raw
    return value `tty` stashed back out and checks it's exactly
    `ERR_BAD_LENGTH`. Finally, `tty` calls `SYS_FORK` -- growing the ABI
    to five registers (`rdi`/`rsi`/`rdx`/`rcx` plus `frame_ptr` in `r9`,
    the caller's full trap frame rather than a normal argument, see the
    `src/syscall.rs`/`src/proc.rs` bullets above) -- and genuinely forks
    itself: the parent sees the new child's `proc_nr` in `rax` and carries
    on unchanged, while a real, separately-scheduled child resumes at that
    *exact same* ring-3 instruction seeing `0` instead, verified by the
    child's canary write (proving its memory is a real, independent copy)
    and its own `SYS_FS_OPEN`/`SYS_FS_WRITE`, both checked back from
    `IDLE`. See "known simplifications in the syscall ABI" above for what's
    not a real syscall surface yet (sixteen calls, one code per *kind* of
    dispatch-level mistake rather than a real per-cause `errno` set).
    `SYS_FORK` is no longer restricted to one known caller with one
    reserved child slot: the pages to copy come out of the caller's own
    memory map (`crate::memory::MemMap`) and the child's process number
    is allocated when the call happens (`crate::proc::alloc_proc_nr`),
    so `shell` forks too -- three distinct forked processes in one boot,
    where one reserved slot would have had the second overwrite the
    first. The twelfth is `SYS_EXEC`, which uses the same `frame_ptr`
    plumbing `SYS_FORK` introduced for the opposite purpose: rather than
    copying the caller's trap into a new process, it overwrites the
    caller's own saved registers, so the trap returns into a different
    program entirely (`shell` becomes `/bin/exectest` mid-syscall). See item
    11 above and "known simplifications in `exec()`".

## Building

Requires a nightly Rust toolchain (for `-Z build-std`, needed because this
targets bare metal and has no prebuilt `core`/`alloc` for it) and the
`bootimage` cargo subcommand:

```
rustup toolchain install nightly
rustup component add rust-src llvm-tools-preview --toolchain nightly
cargo install bootimage
```

Then, from `rust/kernel/`:

```
rustup override set nightly    # only needed once per checkout
cargo build                    # compiles the kernel binary
cargo bootimage                # wraps it in a bootable BIOS disk image
```

This produces
`rust/target/x86_64-unknown-none/debug/bootimage-neumannos-kernel.bin`.

The target (`x86_64-unknown-none`) and build flags are pinned in
`kernel/.cargo/config.toml`. Notably, `rustflags` forces
`relocation-model=static`: the builtin target defaults to a
position-independent executable, which the `bootloader` 0.9 crate's simple
ELF loader does not relocate, causing the kernel to jump into garbage on
boot if left at the default. The kernel's `Cargo.toml` also enables the
`bootloader` crate's `map_physical_memory` feature, which `src/memory.rs`
depends on to reach arbitrary physical frames.

## Running

```
qemu-system-x86_64 -m 256 -drive format=raw,file=target/x86_64-unknown-none/debug/bootimage-neumannos-kernel.bin -serial stdio
```

(or `cargo run`, which invokes `bootimage runner` per `.cargo/config.toml`
and does the same thing, though without the explicit `-m 256` -- pass it
via `QEMU_ARGS` if the default memory size turns out too small for the
heap plus everything else once more of this grows). The VGA display
itself (mode 13h, the LCARS demo panel from `src/vga.rs`) needs an actual
display backend, so `-display none` (used for headless/CI runs, e.g. no
`DISPLAY` available) won't show it -- drop `-display none` for a real
window, or capture it headlessly via QEMU's QMP `screendump` command
(`-qmp unix:/tmp/qmp.sock,server,nowait -display none -vga std`, then a
QMP client sends `{"execute": "screendump", "arguments": {"filename":
"/tmp/screen.ppm"}}` after the capabilities handshake). Note the
resulting image is double the mode's logical 320x200 (640x400): real
mode 13h hardware double-scans it, and QEMU's screendump reflects that,
so multiply logical panel coordinates by 2 before sampling a pixel from
the dump. The keyboard driver can be exercised the same headless way:
with the same `-qmp` socket from above, send
`{"execute": "send-key", "arguments": {"keys": [{"type": "qcode", "data": "a"}]}}`
after the capabilities handshake, and COM1 should print
`[kbd] key: 'a' (scancode 0x1e)` -- this works even after `IDLE` has
logged that it's halting, since the keyboard IRQ wakes the CPU straight
out of `hlt`. To see a digit key's effect on the panel, send a `qcode`
of `"1"`-`"4"` instead and take a `screendump` afterward -- **wait a beat
(e.g. `time.sleep(0.5)`) between the two QMP commands first**: `send-key`
returns as soon as the key event is queued, not once the guest has
actually taken the interrupt and redrawn, and a `screendump` fired
immediately back-to-back can (and, observed once during development,
did) capture the frame from just before the highlight was painted. The
resulting image should show a white rounded-rect frame behind the
button matching the digit pressed (button `N` for digit `N`), and
nowhere else. The line discipline can be exercised the same way: send a
sequence of letter `qcode`s followed by `"ret"` (e.g. `"h"`, `"i"`,
`"ret"`), and COM1 should print `[console] received line: "hi"` --
this too works well after boot, once `IDLE` has already halted. That
line goes to the shell: a plain boot ends with `sh` (`proc_nr` 12)
printing `NeumannOS sh -- type `help` for help` and `$ `, then sitting at
`[syscall] proc 12: SYS_READ_LINE, blocking for a real keypress`
indefinitely -- a shell waiting at a prompt, not a hang. Type a command
(`"e"`, `"c"`, `"h"`, `"o"`, `"spc"`, `"h"`, `"i"`, `"ret"` for
`echo hi`) and COM1 shows the command echoed back, `sh` forking, the
child exec'ing `/bin/echo` (`[syscall] proc 13: SYS_EXEC("/bin/echo",
argc 2, envc 2) -> ...`), `hi`, the child's `SYS_EXIT(0)`, `sh`'s
`SYS_WAIT` collecting it, and a fresh `$ `. `help`, `cat /console.log`,
`cat /nope` (`cat: /nope: no such file or directory`, `[exit 1]`),
`nosuch` (`sh: nosuch: no such file or directory`, `[exit 127]`),
`ptrtest` and `exit 3` are all worth trying; `qmp_type.py`-style
scripts that send one `send-key` per character with a short pause
between commands work well. Typing `"run hello"` instead (`"r"`, `"u"`, `"n"`,
`"spc"`, `"h"`, `"e"`, `"l"`, `"l"`, `"o"`, `"ret"`) exercises `rs`'s
on-demand launch path: COM1 shows `[console] received line: "run hello"`,
`[rs] launch request for "hello" -> 0`, `[console] run "hello" -> 0`,
then a second, independent instance of the ELF-loaded task (a new
process, `APP1_PROC_NR`) running through its own full lifecycle --
`SYS_GET_UPTIME`, a real alarm, `SYS_WRITE_LINE`, `fs` open/write, and
finally blocking on its own `SYS_READ_LINE` -- entirely separately from
`tty`'s boot-time run of the same binary. That includes its own
`SYS_FORK` (`[syscall] proc 10: SYS_FORK -> child proc_nr 14`), which is
the clearest demonstration of what the dynamic process-number pool
bought: with one reserved child slot, this second instance's fork would
have overwritten `tty`'s child rather than becoming a fourth process.
A repeat `run hello` answers `-> -2` (`RS_ALREADY_RUNNING`) -- `rs`
tracks one instance per service, which is unrelated and unchanged.
Expected output on COM1: a line confirming the LCARS demo panel
was painted (`vga: painted the LCARS demo panel ...`, printed as early as
possible -- before paging/heap/scheduler setup -- so the panel is on
screen even if something later panics), a heap self-test (`Box`/`Vec` both actually work),
eleven `[elf] rejected ...` lines from `elf::validator_self_test` (each
naming a hostile image and the `ElfError` it earned) followed by
`[elf] validator self-test: all 11 hostile images rejected` -- these come
*after* the heap self-test, not at the very top, because the synthetic
images are built in the heap -- then `[memory] frame reclaim self-test:
N frames in use before and N after five address-space build/teardown
cycles`, an isolation
self-test (the ring-3 demo's code page translates to `None` through the
kernel's own page table -- it only exists in that task's private address
space), the boot image table, the `pm`/`fs` demo tasks ping-ponging three
messages back and forth (blocking `send`/`receive`), after which `fs`
becomes a real file server and `pm` opens a file, writes to it, reopens it
fresh, reads the bytes back (checking they round-trip through actual
in-memory storage, not a fixed echo), and reads once more past end of
file (checking that returns `0` instead of repeating data), then exercises
the directory hierarchy: opening a path under a directory that doesn't
exist yet fails with `ENOENT`, `mkdir`ing that directory succeeds,
`mkdir`ing it again fails with `EEXIST`, opening the directory itself as
a file fails with `EISDIR`, opening a path under the now-existing
directory succeeds, and opening a path under an ordinary *file* (not a
directory) fails with `ENOTDIR`, then `pm`
`sys_fork`ing a real child (`init`) and proving its copy of the ring-3
task's code page has genuinely diverged (a canary written to the child's
copy doesn't show up in the original), then `flaky` immediately crashing
in ring 3 (`EXCEPTION: INVALID OPCODE`, `code_segment` reporting `Ring3`)
and `[proc] flaky (crash demo) (proc_nr 8) killed: invalid opcode` --
the kernel does *not* halt -- followed by `[rs] flaky (proc_nr 8) died --
restarting it (attempt 1/3)`, then two more identical crash/restart
cycles, and finally `[rs] flaky (proc_nr 8) died again -- already
restarted it 3 times, giving up` after the fourth crash, `driver` (the hand-assembled demo)
and `tty` (a real ELF64 binary loaded by `crate::elf`) each making real,
register-dispatched syscalls through `crate::syscall` --
`[syscall] proc P: SYS_GET_UPTIME -> N` a few times each, `tty`
additionally calling `SYS_FORK` right after its own loop
(`[syscall] proc 5: SYS_FORK -> child proc_nr 15`) and genuinely forking
itself: a real, separately-scheduled child (`proc_nr` 15 -- a number
allocated at the moment of the fork, not reserved for it, so it depends
on what has forked already) resumes at that
same point seeing `0` instead, writes a canary into its own copy of
`vircopy_buf`, and makes its own independent `SYS_FS_OPEN`/`SYS_FS_WRITE`
(`[syscall] proc 15: SYS_FS_OPEN("/from_fork_child.txt") -> N`) before
blocking for good -- while `tty` itself (seeing its child's nonzero
`proc_nr`) falls straight through to `SYS_SET_ALARM`/`SYS_WAIT_ALARM`
(`[syscall] proc 5: SYS_SET_ALARM(3 ticks)`, then, after genuinely
blocking through several other tasks' output in between,
`[syscall] proc 5: SYS_WAIT_ALARM woken by a real SYN_ALARM notification
(uptime N)`) and then `SYS_WRITE_LINE` (`[syscall] proc 5: SYS_WRITE_LINE:
"hello from the ELF-loaded ring-3 task, after waiting for a real alarm!"`,
a real cross-ring pointer argument, read directly out of `tty`'s own
still-active address space), then `SYS_FS_OPEN`/`SYS_FS_WRITE`
(`/from_ring3.txt`) and, just before blocking on `SYS_READ_LINE`, a real
`SYS_VIRCOPY` (`[syscall] proc 5: SYS_VIRCOPY(28 bytes from proc 6) ->
ok`) reading a range of `driver`'s own private memory straight from ring
3 into `tty`'s own buffer -- before each non-blocked task ends itself
with `[syscall] proc P: SYS_BLOCK_FOREVER, blocking for good`, `CLOCK`
waking from its own real `sys_setalarm`-driven `SYN_ALARM` notification
and using `sys_vircopy` (kernel-side) to read the ring-3 demo task's own
code bytes back out of its address space (proving a genuine
cross-address-space copy, since `CLOCK` never leaves the kernel's), then
dynamically spawning a brand new `log` task at runtime (watch it appear
interleaved with `memory`'s output, proof the scheduler was already
running other tasks when it showed up), `memory`'s demo task spinning
through many quanta purely because the timer forces it to keep yielding
and resuming (asynchronous preemption -- watch its counter resume from
exactly where it left off every time), and finally, once everything else
has blocked, `IDLE` running twenty independent checks against processes
that have long since gone quiet: `sys_vircopy`-ing `tty`'s `.data` counter
back out and confirming it reads `5` (proving the loaded ELF binary's own
code genuinely ran, not just that it trapped the right number of times --
this check used to run from `CLOCK` instead, racing `tty`'s actual
progress against `CLOCK`'s own independent alarm; it lives here now
because that race was real, not just theoretical (see the
`idle_task`/`elf_counter_demo` doc comments in `src/main.rs` for the full
story), reading back `/from_ring3.txt` (`[idle] read back "written
from ring 3 via a real syscall, IPC, and fs!" from /from_ring3.txt ...`)
to confirm `tty`'s `SYS_FS_OPEN`/`SYS_FS_WRITE` calls genuinely reached
`fs` through a completely independent, kernel-side path,
`sys_vircopy`-ing `tty`'s `vircopy_buf` back out to confirm *its* ring-3
`SYS_VIRCOPY` call actually landed the right bytes, doing the same for
`err_result` to confirm the deliberately-invalid second `SYS_VIRCOPY`
came back exactly `ERR_BAD_LENGTH`, and finally two checks on the forked
child specifically: `sys_vircopy`-ing *its* `vircopy_buf` (targeting the
child's own `proc_nr`, found via `proc::child_of(TTY_PROC_NR)` rather
than a constant, since nothing reserves a number for it) to confirm it
holds the canary and not `tty`'s
own content -- proof fork's copy was genuinely independent, not
aliased -- and reading back `/from_fork_child.txt` to confirm the
child's own file write reached `fs` too, and finally eight checks on the
fork/exec demo: reading back `/from_exec.txt` (written by the *exec'd*
image, `user/echo.s`), reading `/exec_error.bin` and confirming it holds
exactly `ERR_BAD_ELF` -- which `shell` could only have written after
surviving its own deliberately-failed exec of a non-ELF file --
reading `/evil_exec_error.bin` and confirming it too holds
`ERR_BAD_ELF` -- that one written by the exec'd image after it tried,
from ring 3, to `exec` a hand-built ELF asking to be mapped onto the
kernel's own heap (`[syscall] proc 13: SYS_EXEC("/evil") -> bad image:
SegmentInSharedSlot` appears earlier in the log), which the first
version of this loader's validation accepted -- then finding `shell`'s
forked child in the process table and cross-checking it against
`/shell_fork.bin`, the `proc_nr` `SYS_FORK` actually handed back to
`shell` in ring 3 (`[idle] read back 13 from /shell_fork.bin -- the
proc_nr SYS_FORK returned to shell in ring 3 (process table says 13)`),
and then four reads that together pin down *which* address space
changed: the exec'd image's `.data` counter reads `1` in the child and
is `Err(SrcNotMapped)` in `shell`, while `shell`'s own marker page still
reads `0xfeedface` in `shell` and is `Err(SrcNotMapped)` in the child
(`[idle] reading shell's marker page at 0x555555580000 out of the
*child* instead: Err(SrcNotMapped) ...`). The last of those is the
strongest single line in the boot: the child had that page a moment
ago -- it inherited a copy from the fork -- and `exec` freed 11 frames
of that inherited image, so `shell`'s own copy still reading
`0xfeedface` afterwards is also direct evidence that teardown didn't
reach into the parent. Then the other end of a process's life:
`/from_exited_child.txt` (written by a forked child that then
terminated with `SYS_EXIT`) and the two `(proc_nr, status)` pairs
`shell` collected with `SYS_WAIT`, in `/shell_wait1.bin` and
`/shell_wait2.bin`, which have to read `42` and `7`. Both children get
the *same* process number, which is the point: collecting a status is
what releases a slot, so the second fork gets the first child's number
back. Watch for the two different collection paths in the log --
`[proc] proc_nr 11 collected child proc_nr 14 (status 42) handed over
directly -- it was still blocked here when the child exited` for the
first, and `... (status 7) out of a zombie slot, which is now free
again` for the second, which `shell` deliberately lets terminate before
asking for it. Before all of that, watch `shell`
(`proc_nr` 11) announce itself, fail an exec on purpose
(`[syscall] proc 11: SYS_EXEC("/not_a_program") -> bad image: NotElf`),
carry on regardless, and then fork
(`[syscall] proc 11: SYS_FORK -> child proc_nr 13`), then fork twice
more and wait for each of those in turn
(`[syscall] proc 14: SYS_EXIT(42)` from a child, then
`[syscall] proc 11: SYS_WAIT -> child proc_nr 14 terminated with
status 42` from the parent), and finally go quiet with
`shell: forked -- my child is becoming /bin/exectest, and I am still shell`,
while its first child stops being that program entirely
(`[syscall] proc 13: SYS_EXEC("/bin/exectest", argc 4, envc 1) -> replaced
its own image, entering at 0x666666660000 on a fresh stack`), with every
line after that from `proc 13` coming from a completely different
program. If
the child runs before `pm` has installed `/bin/exectest`, you'll also see a
few `-> fs error -2` (`ENOENT`) attempts a tick apart first: that's
`user/shell.s`'s retry loop, not a failure. `IDLE` then exercises the
dynamic process-number pool's own two edges, which no successful boot
reaches on its own (`[idle] process-number pool: claimed the 4 remaining
dynamic slot(s) ([14, 16, 17, 18]), then it correctly refused`): it
claims every number left until the pool refuses, hands them all back,
and checks the first one comes out again. It also asks
`proc::wait_for_child` for a child it doesn't have, which has to come
back `None` (POSIX `ECHILD`) rather than block -- a case no ring-3
program here reaches, and one that would hang the whole system rather
than fail quietly if it were wrong. Finally `IDLE` reports that
it's halting. Just before that it reports the final frame accounting and
re-runs three more address-space build/teardown cycles
(`[idle] frames in use: N (free list M) -- unchanged after three more
address-space cycles: N ...`) -- the same balance check the boot-time
one makes, but at the end of a real workload, once `flaky`'s kills have
actually put frames on the free list and `exec` has returned an image.
Watch too for `[proc] reclaimed 9 frames from flaky (crash demo)` after
each of its four crashes and `[proc] forked child 1 (proc_nr 12)
replaced its address space, freed 11 frames of the old one`; both of
those were permanent leaks until the allocator learned to take memory
back. Finally `IDLE` reports it is halting (with the accumulated tick
count).

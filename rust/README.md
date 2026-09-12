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
with an in-memory filesystem over IPC (not a fixed-reply stand-in), and
a second ring-3 task that runs a *real, statically linked ELF64 binary*
(parsed and mapped by this port's own minimal ELF loader, not
hand-assembled bytes poked into a fixed page) — enough to build the rest
of the system on top of.

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
  the same switch code from both places sound.
- `src/gdt.rs` — Global Descriptor Table and Task State Segment, ported from
  the segment/TSS setup in `kernel/protect.c`. Its only real job right now
  is giving the double-fault handler a dedicated stack (via the TSS's
  Interrupt Stack Table), so a fault during fault delivery — e.g. a kernel
  stack overflow — is reported instead of triple-faulting the CPU.
- `src/interrupts.rs` — the IDT and CPU exception handlers, ported from the
  single vector-indexed dispatcher in `kernel/exception.c`. The C version
  turns a fault into a POSIX signal for a user process or panics for a
  kernel task; with no user processes yet, every handler here takes the
  "kernel task" branch (report and halt), except `#BP` (breakpoint), which
  reports and returns — exercised by a self-test in `main.rs`.
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
  use. `crate::usermode` is the first thing to use this, to give the
  ring-3 demo task real isolation instead of just a CPU privilege level.
  `page_table_for` and `copy_between_address_spaces` build on it: given
  any process's PML4 frame, translate a virtual address through *that*
  table (via the physical-memory window, without switching `CR3` to it)
  and copy page-at-a-time between two such address spaces -- this is
  where `sys_umap`/`sys_vircopy`'s actual job (translating between address
  spaces) applies; see `src/calls.rs` below for the kernel-call wrapper.
  `fork_address_space` goes one step further than `new_address_space`
  alone can: for each address in an explicit list, it walks that
  address's *entire* page-table path fresh, duplicating any level still
  shared with the source (so touching it can never modify the original)
  and deep-copying the leaf page's contents into a newly allocated frame
  -- what real `fork()` needs for every page the parent already had, not
  just ones mapped after the fork. The physical frame allocator is now a
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
  (`SYSCALL_VECTOR`); its handler prints the CPU-captured `CS` selector's
  RPL, which is what actually proves a caller was in ring 3 -- not
  something the kernel side merely asserts.
- `src/usermode.rs` uses all of the above to run a real,
  scheduler-integrated ring-3 task with its own address space:
  `create_address_space` builds a new, private page table (via
  `memory::new_address_space`) and maps a code and a stack page into *it*
  (with `USER_ACCESSIBLE`, without which the CPU refuses to execute or
  touch them at CPL 3 at all -- a `#PF`, not a `#GP`) -- never into the
  kernel's own mapper. `ring3_task_entry`, an ordinary `crate::proc` task
  body, then jumps to CPL 3. The ring-3 code loops `int 0x80`, trapping
  into the kernel and back repeatedly -- ordinary, repeatable trap
  entry/exit, not a one-shot trick -- and can be asynchronously preempted
  by the timer while in ring 3 exactly like any other task.
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
  an address space, deep-copies the given pages into a brand new one
  (`memory::fork_address_space`), and spawns a task into it
  (`proc::spawn`) -- bundling what real MINIX splits into a kernel call
  (duplicate the memory) and a separate scheduling step, since nothing in
  this port needs them separated yet.
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
  shape; `open`/`write`/`read` are client-side stubs (using `ipc`'s new
  `send_receive`) that any task can call to talk to it, mirroring
  `src/calls.rs`'s kernel-call wrappers in shape even though these cross
  a real IPC round trip rather than a direct function call.
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
- `src/serial.rs` + `src/main.rs` — boot entry point (via the `bootloader`
  crate): loads the GDT/IDT, runs a breakpoint self-test, sets up paging
  and the heap, builds the ring-3 demo task's and the ELF-loaded task's
  address spaces and maps their pages into them, spawns the kernel tasks,
  programs the PIC/PIT and enables interrupts, and hands off to the
  scheduler. Spawns `IDLE` and `CLOCK` as real kernel tasks (same as the
  boot image), a real `fs` server (see `src/fs.rs` above), and temporary
  stand-in bodies in the `pm`/`rs`/`memory`/`driver`/`tty` process table
  slots: `pm`/`fs` ping-pong three blocking messages back and forth
  (exercising the rendezvous IPC), after which `fs` becomes a real file
  server
  (`fs::InMemoryFs::serve`) and `pm` exercises it: opens a file, writes to
  it, reopens it fresh (a distinct file descriptor with its own cursor)
  and reads the bytes back, checking they round-trip, then reads once
  more past end of file and checks that comes back empty. `pm` also
  `sys_fork`s the ring-3 task's code page into a brand new `init` process,
  then overwrites *just the child's copy* with a canary value and reads
  back both copies to prove they've genuinely diverged, not aliased the
  same physical page -- and `rs`/`memory` each spin in a tight, CPU-bound
  loop with no
  `yield_now()`/IPC call anywhere in it (proving the timer interrupt truly
  preempts a task asynchronously, mid-loop, rather than only ever
  switching at cooperative checkpoints). `driver` runs the ring-3 demo task
  described above, and `tty` runs `elf::task_entry` (`elf::load` builds
  its address space in `kernel_main`, the same way `driver`'s is built) --
  a real ELF64 binary (`user/hello.elf`) executing its own `incl`/`int
  0x80` loop in ring 3, its five round trips counted independently of
  `driver`'s. `CLOCK` now genuinely
  calls `sys_setalarm` and blocks in `receive` -- exactly real MINIX's
  `while (TRUE) receive(HARDWARE, &m)` -- instead of polling
  `uptime_ticks()`, and also exercises `sys_vircopy` twice: reading the
  ring-3 demo task's code page back out of *its* address space into a
  local buffer, and, after `tty` has run its five iterations and blocked,
  reading `tty`'s own `.data` counter back out of *its* address space and
  checking it reads `5` -- proving both that the copy really goes through
  a different process's page table (`CLOCK` itself never leaves the
  kernel's own address space) and that the loaded ELF binary's own code
  genuinely executed and wrote through to physical memory, not just that
  it trapped the expected number of times.
  `CLOCK` then dynamically spawns a brand new task (`log`) at runtime --
  with the scheduler already running other tasks, not during
  `kernel_main`'s boot-time setup -- proving `proc::spawn` works as a
  genuine "start a new process now" primitive: the actual thing real `rs`
  needs to bring services up on demand, which is why "the servers
  themselves" (the next roadmap item) needed this first.
  Also runs an isolation self-test right after building `driver`'s
  address space: translating the ring-3 code page's address through the
  *kernel's own* page table returns `None`, proving the mapping really is
  private and not merely inaccessible-by-privilege-level.

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

- **No real syscall dispatch.** `SYSCALL_VECTOR`'s handler always performs
  the same fixed action (count the call, print, and eventually block the
  caller for good) regardless of which register values the caller set up;
  there's no argument-passing convention or call-number dispatch yet,
  since there's nothing to call. The fixed iteration count
  (`interrupts::SYSCALL_COUNTS`, one per process slot so `driver` and
  `tty` -- see `src/elf.rs` -- don't race to the same threshold) exists
  purely so a demo user program's code can stay a trivial loop instead of
  needing a counter encoded by hand.
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

### Known simplifications in the kernel calls

- **No real dispatch.** `sys_vircopy`/`sys_setalarm`/`sys_fork` are
  ordinary Rust functions any kernel task calls directly, not entries
  reached via a message to `SYSTEM` and a call-number dispatch table
  (`kernel/system.c`'s `map(SYS_xxx, do_xxx)`). That dispatch has nowhere
  to live yet: there's no real syscall argument-passing convention (the
  one `int 0x80` gate that exists is `usermode`'s fixed demo action, not a
  general call mechanism -- see its own known-simplifications section).
- **`sys_fork`'s child starts at a fixed entry point, not "wherever the
  parent was."** Real `fork()` gives the child an exact copy of the
  parent's *entire* address space and resumes both sides from the same
  call site (the child's `fork()` returns `0`, the parent's returns the
  child's pid). This port's tasks are built around a fixed `fn() -> !`
  entry point (`crate::proc::spawn`) instead, so `sys_fork`'s child starts
  fresh at whatever entry point the caller supplies -- a real, working
  process-creation primitive with genuinely independent memory (see the
  `pm`/`init` canary demo in `main.rs`), just not full POSIX continuation
  semantics.
- **`private_pages` must be listed explicitly**, rather than discovered
  by walking the parent's entire user-accessible page-table range. There's
  no per-process memory-map bookkeeping yet (`kernel/kernel.h`'s
  `struct mem_map`) to read it back out of.
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

- **No real filesystem hierarchy.** Files are looked up by an exact-match
  flat name (no directories, no `/`-separated path resolution -- see
  `servers/fs/path.c`'s `lookup()` for what the real thing does), and
  there's no `unlink`/`stat`/permissions/inode-number concept at all yet.
- **No cross-address-space copy.** Request/reply args carry raw pointers
  valid in the caller's address space directly, since `fs` and every
  current caller (`pm`) still share the kernel's own address space (see
  "known simplifications in the ring-3 task" above); a real, isolated
  `fs` server would need a `sys_vircopy`-style copy for every buffer, the
  same way `sys_vircopy` itself does for `sys_fork`'d tasks.
- **`open` always creates**, and there's no `close`: a file descriptor is
  never freed once allocated (`InMemoryFs::open`'s slot table only ever
  grows), and repeated opens of the same name return independent
  descriptors with independent cursors rather than sharing or refusing
  based on any open-file-table policy.
- **In-memory only.** There's no backing device, so nothing here survives
  a reboot -- there's no block layer, block cache, or on-disk layout at
  all (`kernel/kernel.h`'s device abstractions, `servers/fs`'s
  buffer cache), just a `Vec<(String, Vec<u8>)>` that lives as long as
  the kernel does.
- **No error variety.** Every failure (`EBADF` alone) covers "bad
  descriptor" and "unrecognized request" both; real `fs` distinguishes
  many more `errno` values (`ENOENT`, `EACCES`, `ENOSPC`, ...).

### Known simplifications in the ELF loader

- **The binary is checked in pre-built, not assembled by this build.**
  `user/hello.elf` is produced by a manual `as`/`ld` invocation (see
  `user/hello.s`'s header comment), not a build-script step -- there's no
  cross toolchain wired into `cargo build` yet to assemble a fresh user
  program automatically (see the libc-equivalent roadmap item).
- **No dynamic linking, no `argv`/`envp`/auxv, no `PT_INTERP`.** `load`
  only understands `PT_LOAD` segments; a real `execve` also sets up the
  initial stack contents a libc's `_start` expects (argument/environment
  vectors, the auxiliary vector) and can invoke a dynamic linker named by
  a `PT_INTERP` segment. `user/hello.s` is freestanding and asks for
  none of that, so this hasn't mattered yet.
- **One fixed binary, one fixed stack address.** `elf::load` always loads
  `HELLO_ELF` at a hardcoded `STACK_ADDR`, the same way `usermode.rs`'s
  demo uses fixed constants -- fine for this port's one ELF task, but not
  a general "load any binary, anywhere" API yet (no ASLR, no picking an
  unused address range).
- **No `PT_LOAD` overlap/validation.** A real loader also has to guard
  against a hostile or malformed file (overlapping segments, addresses
  that alias kernel-reserved ranges, `p_align` mismatches); `load` trusts
  `user/hello.elf` is well-formed, since this port only ever loads a
  binary it built itself.

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
   (verified with a canary write). See "known simplifications" above for
   what's not implemented yet (real call dispatch, `sys_umap`, more of
   `kernel/system/do_*.c`, full POSIX fork continuation semantics).
10. ~~**A minimal ELF loader**~~ — done (`src/elf.rs`). Parses a real,
    statically linked ELF64 binary (`user/hello.elf`) and maps its
    `PT_LOAD` segments into a fresh address space at their own specified
    addresses and permissions (not one hand-placed page), zero-filling
    BSS; a second ring-3 task (`tty`) runs it, its five `int 0x80` round
    trips counted independently of `driver`'s own demo (see "known
    simplifications in the ring-3 task" above -- the syscall counter is
    now per-process), and `CLOCK` reads its `.data` counter back via
    `sys_vircopy` afterward to confirm the loaded code genuinely executed
    and wrote through to physical memory, not just that it trapped in the
    expected number of times. See "known simplifications in the ELF
    loader" below for what's missing (no dynamic linking, no `argv`/
    `envp`, the binary is checked in pre-built rather than assembled by
    this build).
11. **The servers themselves**: `pm` (process manager), `fs` (file system),
    `rs` (reincarnation server), `tty`, `memory`, in roughly that dependency
    order, matching `servers/` and `drivers/` in the C tree -- replacing the
    temporary stand-ins in `main.rs`. Three slices done: `proc::spawn` is
    proven safe to call from an already-running task, not just
    `kernel_main`'s boot-time setup (`clock_task` dynamically spawns `log`
    at runtime); `sys_fork` gives a task a real way to create a child with
    its own independent memory (`pm`'s `init` demo); and `fs` (`src/fs.rs`)
    is now a real, in-memory file server answering genuine open/read/write
    requests over IPC, not a fixed-reply stand-in (`pm`'s open/write/reopen/
    read/read-past-EOF demo). Still missing: a real `rs` that decides
    *what* to start and *why* (crash detection/restart policy,
    `servers/rs/manager.c`), full POSIX fork/exec semantics (the child
    resuming from the parent's exact call site, and using the ELF loader
    above to load a program image instead of starting at a fixed entry
    point), and `fs` growing a real directory hierarchy, cross-address-space
    copies, and a backing store (see "known simplifications in `fs`" above)
    rather than a flat, in-memory, single-address-space file table.
12. **A libc-equivalent** for whatever runs in user mode, mirroring `lib/`.

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
heap plus everything else once more of this grows). Expected output on
COM1: a heap self-test (`Box`/`Vec` both actually work), an isolation
self-test (the ring-3 demo's code page translates to `None` through the
kernel's own page table -- it only exists in that task's private address
space), the boot image table, the `pm`/`fs` demo tasks ping-ponging three
messages back and forth (blocking `send`/`receive`), after which `fs`
becomes a real file server and `pm` opens a file, writes to it, reopens it
fresh, reads the bytes back (checking they round-trip through actual
in-memory storage, not a fixed echo), and reads once more past end of
file (checking that returns `0` instead of repeating data), then `pm`
`sys_fork`ing a real child (`init`) and proving its copy of the ring-3
task's code page has genuinely diverged (a canary written to the child's
copy doesn't show up in the original), five ring-3 round trips each from
`driver` (the hand-assembled demo) and `tty` (a real ELF64 binary loaded
by `crate::elf`) (`[syscall] iteration N from Ring3 ... proc P`, printed
from inside the syscall handler using the CPU-captured selector -- not
something the kernel side merely claims, and counted separately per
process) before each task blocks for good, `CLOCK` waking from a real
`sys_setalarm`-driven `SYN_ALARM` notification and then using
`sys_vircopy` twice: once to read the ring-3 demo task's code bytes back
out of its own address space (proving a genuine cross-address-space copy,
since `CLOCK` never leaves the kernel's), and once to read `tty`'s `.data`
counter back out and confirm it reads `5` (proving the loaded ELF
binary's own code genuinely ran, not just that it trapped the right
number of times), then dynamically spawning a brand new
`log` task at runtime (watch it appear interleaved with `memory`'s output,
proof the scheduler was already running other tasks when it showed up),
the `rs`/`memory` demo tasks trading off every quantum purely because the
timer forces it (asynchronous preemption -- watch `memory`'s counter
resume from exactly where it left off after `rs` gets a turn), and finally
`IDLE` reporting that it's halting (with the accumulated tick count) once
everything else has blocked.

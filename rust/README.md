# NeumannOS (Rust port)

This directory holds a Rust port of the MINIX 3.1.0 sources in the rest of
this repository (see the top-level `README.md` for the original C
codebase's history). It is a **starting skeleton**, not a finished port: a
real microkernel, its servers (`pm`, `fs`, `rs`, ...), and its drivers are a
multi-month undertaking on their own. What's here boots in QEMU, sets up
exception handling and a heap, schedules kernel tasks with real,
asynchronously preemptive hardware-timer-driven quantum accounting
(including a task that runs in ring 3, in its own genuinely isolated
address space), exercises blocking message-passing IPC between them, and
has its first two real kernel calls (a genuine cross-address-space
`sys_vircopy`, and `sys_setalarm` waking a blocked task with a real
notification instead of it polling) — enough to build the rest of the
system on top of.

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
- `src/calls.rs` — the first two kernel calls, ported from
  `kernel/system/do_copy.c` (`sys_vircopy`) and `do_setalarm.c`
  (`sys_setalarm`). Real MINIX dispatches these by call number out of a
  message a process sends to `SYSTEM`; this port has no such dispatch yet
  (see "known simplifications" below), so for now they're just ordinary
  Rust functions any kernel task can call directly -- the same way
  `crate::ipc`'s `send`/`receive`/`notify` started out, before anything
  needed them from ring 3. `sys_vircopy` is a thin wrapper resolving two
  process numbers to address spaces (`proc::cr3_of`) and delegating to
  `memory::copy_between_address_spaces`; `sys_setalarm` delegates to
  `proc::set_alarm`, and `proc::clock_tick` is what actually notices an
  alarm's deadline and delivers the `SYN_ALARM` notification for it (using
  a `Scheduler::try_deliver_notification` helper shared with
  `mini_notify`, since `clock_tick` is already inside the scheduler lock
  and can't re-enter it).
- `src/serial.rs` + `src/main.rs` — boot entry point (via the `bootloader`
  crate): loads the GDT/IDT, runs a breakpoint self-test, sets up paging
  and the heap, builds the ring-3 demo task's address space and maps its
  pages into it, spawns the kernel tasks, programs the PIC/PIT and enables
  interrupts, and hands off to the scheduler. Spawns `IDLE` and `CLOCK` as
  real kernel tasks (same as the boot image), plus temporary stand-in
  bodies in the `pm`/`fs`/`rs`/`memory`/`driver` process table slots:
  `pm`/`fs` ping-pong three blocking messages back and forth (exercising
  the rendezvous IPC), `rs`/`memory` each spin in a tight, CPU-bound loop
  with no `yield_now()`/IPC call anywhere in it (proving the timer
  interrupt truly preempts a task asynchronously, mid-loop, rather than
  only ever switching at cooperative checkpoints), and `driver` runs the
  ring-3 demo task described above -- the only one of these with its own
  address space rather than sharing the kernel's. `CLOCK` now genuinely
  calls `sys_setalarm` and blocks in `receive` -- exactly real MINIX's
  `while (TRUE) receive(HARDWARE, &m)` -- instead of polling
  `uptime_ticks()`, and also exercises `sys_vircopy` by reading the
  ring-3 task's code page back out of *its* address space into a local
  buffer, proving the copy really goes through a different process's page
  table (`CLOCK` itself never leaves the kernel's own address space).
  `CLOCK` then dynamically spawns a brand new task (`log`) at runtime --
  with the scheduler already running other tasks, not during
  `kernel_main`'s boot-time setup -- proving `proc::spawn` works as a
  genuine "start a new process now" primitive: the actual thing real `rs`
  needs to bring services up on demand, which is why "the servers
  themselves" (the next roadmap item) needed this first.
  Also runs an isolation self-test right after building that address
  space: translating the ring-3 code page's address through the
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
  (`interrupts::SYSCALL_COUNT`) exists purely so `usermode::USER_CODE`'s
  hand-assembly can stay a trivial two-instruction loop instead of needing
  a counter encoded by hand.
- **Only one task has its own address space.** `Proc::cr3` supports it
  per-task, but only the ring-3 demo actually gets one; every other task
  still shares the kernel's. That's deliberate for now -- kernel tasks
  (`IDLE`, `CLOCK`) and the `pm`/`fs`/`rs`/`memory` stand-ins all run
  kernel-trusted code today, so there's nothing to isolate them *from*
  yet -- but it means there's no isolation between, say, `pm` and `fs`'s
  demo bodies either. That only starts to matter once real, mutually
  distrusting user-mode servers exist.
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

- **No real dispatch.** `sys_vircopy`/`sys_setalarm` are ordinary Rust
  functions any kernel task calls directly, not entries reached via a
  message to `SYSTEM` and a call-number dispatch table
  (`kernel/system.c`'s `map(SYS_xxx, do_xxx)`). That dispatch has nowhere
  to live yet: there's no real syscall argument-passing convention (the
  one `int 0x80` gate that exists is `usermode`'s fixed demo action, not a
  general call mechanism -- see its own known-simplifications section).
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
   by `memory::copy_between_address_spaces`) and `sys_setalarm` (backed by
   `proc::set_alarm`/`clock_tick`). `CLOCK` now uses both for real instead
   of polling. See "known simplifications" above for what's not
   implemented yet (real call dispatch, `sys_umap`, more of
   `kernel/system/do_*.c`).
10. **The servers themselves**: `pm` (process manager), `fs` (file system),
    `rs` (reincarnation server), `tty`, `memory`, in roughly that dependency
    order, matching `servers/` and `drivers/` in the C tree -- replacing the
    temporary stand-ins in `main.rs`. First slice done: `proc::spawn` is
    now proven safe to call from an already-running task, not just
    `kernel_main`'s boot-time setup (`clock_task` dynamically spawns `log`
    at runtime) -- the actual primitive real `rs` needs to bring services
    up on demand. Still missing: a real `rs` that decides *what* to start
    and *why* (crash detection/restart policy, `servers/rs/manager.c`),
    and everything `pm`/`fs` actually need to do their jobs (process
    creation with copy-on-fork-like semantics rather than a fixed
    `fn() -> !` entry point, and a real filesystem, respectively).
11. **A libc-equivalent** for whatever runs in user mode, mirroring `lib/`.

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
messages back and forth (blocking `send`/`receive`), five ring-3 round
trips (`[syscall] iteration N from Ring3 ...`, printed from inside the
syscall handler using the CPU-captured selector -- not something the
kernel side merely claims) before that task blocks for good, `CLOCK`
waking from a real `sys_setalarm`-driven `SYN_ALARM` notification and then
using `sys_vircopy` to read the ring-3 task's code bytes back out of its
own address space (proving a genuine cross-address-space copy, since
`CLOCK` never leaves the kernel's), then dynamically spawning a brand new
`log` task at runtime (watch it appear interleaved with `memory`'s output,
proof the scheduler was already running other tasks when it showed up),
the `rs`/`memory` demo tasks trading off every quantum purely because the
timer forces it (asynchronous preemption -- watch `memory`'s counter
resume from exactly where it left off after `rs` gets a turn), and finally
`IDLE` reporting that it's halting (with the accumulated tick count) once
everything else has blocked.

# NeumannOS (Rust port)

This directory holds a Rust port of the MINIX 3.1.0 sources in the rest of
this repository (see the top-level `README.md` for the original C
codebase's history). It is a **starting skeleton**, not a finished port: a
real microkernel, its servers (`pm`, `fs`, `rs`, ...), and its drivers are a
multi-month undertaking on their own. What's here boots in QEMU, sets up
exception handling and a heap, schedules kernel tasks with real,
asynchronously preemptive hardware-timer-driven quantum accounting
(including a task that runs in ring 3, in its own genuinely isolated
address space), and exercises blocking message-passing IPC between them —
enough to build the rest of the system on top of.

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
  ring-3 demo task real isolation instead of just a CPU privilege level --
  this is where `sys_umap`/`sys_vircopy`'s actual job (translating between
  address spaces) starts to apply, though neither is ported yet.
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
  address space rather than sharing the kernel's. Also runs an isolation
  self-test right after building that address space: translating the
  ring-3 code page's address through the *kernel's own* page table
  returns `None`, proving the mapping really is private and not merely
  inaccessible-by-privilege-level.

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
9. **Kernel calls** (`kernel/system.c`, `kernel/system/do_*.c`) — the
   privileged operations servers need (`sys_vircopy`, `sys_setalarm`, etc.).
10. **The servers themselves**: `pm` (process manager), `fs` (file system),
    `rs` (reincarnation server), `tty`, `memory`, in roughly that dependency
    order, matching `servers/` and `drivers/` in the C tree -- replacing the
    temporary stand-ins in `main.rs`.
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
kernel side merely claims) before that task blocks for good, the
`rs`/`memory` demo tasks trading off every quantum purely because the
timer forces it (asynchronous preemption -- watch `memory`'s counter
resume from exactly where it left off after `rs` gets a turn), and
finally `IDLE` reporting that it's halting (with the accumulated tick
count) once everything else has blocked.

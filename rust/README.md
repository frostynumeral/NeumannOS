# NeumannOS (Rust port)

This directory holds a Rust port of the MINIX 3.1.0 sources in the rest of
this repository (see the top-level `README.md` for the original C
codebase's history). It is a **starting skeleton**, not a finished port: a
real microkernel, its servers (`pm`, `fs`, `rs`, ...), and its drivers are a
multi-month undertaking on their own. What's here boots in QEMU, sets up
exception handling, schedules kernel tasks with real hardware-timer-driven
quantum accounting, and exercises blocking message-passing IPC between
them — enough to build the rest of the system on top of.

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
  (save/restore callee-saved registers and the stack pointer) and the
  `trampoline` a freshly spawned task's stack is primed to land in on its
  first run, standing in for the register save/restore and initial-frame
  setup that `kernel/mpx386.s` and `kernel/main.c` handle in C MINIX.
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
- `src/serial.rs` + `src/main.rs` — boot entry point (via the `bootloader`
  crate): loads the GDT/IDT, runs a breakpoint self-test, programs the
  PIC/PIT and enables interrupts, prints the boot image table over COM1,
  spawns the kernel tasks, and hands off to the scheduler. Spawns `IDLE`
  and `CLOCK` as real kernel tasks (same as the boot image), plus
  temporary stand-in bodies in the `pm`/`fs` process table slots that
  ping-pong three blocking messages back and forth, to exercise the
  scheduler and rendezvous IPC end to end before the real servers exist.

### Known simplifications in the scheduler/IPC/timer port

- **Quantum accounting is real; asynchronous preemption isn't, yet.** The
  timer interrupt (`proc::clock_tick`) genuinely decrements the running
  task's `ticks_left` and reorders the ready queues on real hardware
  ticks, same as `kernel/clock.c`'s `clock_handler`. But real MINIX's
  `restart()` (`kernel/mpx386.s`) unconditionally switches to whichever
  process `pick_proc` picked on *every* trap/interrupt return, so a
  higher-priority process preempts immediately. This port's timer handler
  doesn't force a switch, because resuming a task interrupted mid-
  execution needs a full trap-frame `iretq`, not the plain `ret` that
  `switch_to` uses for a task that's voluntarily blocked mid-function-call.
  So the switch still only happens the next time *some* task calls
  `reschedule()` (by blocking in IPC or calling `yield_now()`) -- a task
  that never does either keeps running past a quantum expiry
  uninterrupted. Closing that gap (extending `switch_to` to also resume
  via a saved trap frame) is next on the roadmap below.
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
   `ticks_left` and reorders the ready queues on hardware ticks. See
   "known simplifications" above for the gap that's left: it doesn't yet
   force a switch away from a task that never blocks or yields on its own.
4. **Asynchronous preemption**: extend `switch_to`/task state so a task
   interrupted mid-execution by the timer can be resumed later via a full
   trap-frame `iretq` (not just the plain-`ret` cooperative resume used
   today), and have `clock_tick` actually perform the switch on quantum
   expiry instead of only reordering the ready queues.
5. **User-mode processes and address-space isolation** — right now
   everything (including the `pm`/`fs` stand-ins) runs in kernel context.
6. **Kernel calls** (`kernel/system.c`, `kernel/system/do_*.c`) — the
   privileged operations servers need (`sys_vircopy`, `sys_setalarm`, etc.).
7. **The servers themselves**: `pm` (process manager), `fs` (file system),
   `rs` (reincarnation server), `tty`, `memory`, in roughly that dependency
   order, matching `servers/` and `drivers/` in the C tree -- replacing the
   temporary ping-pong stand-ins in `main.rs`.
8. **A libc-equivalent** for whatever runs in user mode, mirroring `lib/`.

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
boot if left at the default.

## Running

```
qemu-system-x86_64 -drive format=raw,file=target/x86_64-unknown-none/debug/bootimage-neumannos-kernel.bin -serial stdio
```

(or `cargo run`, which invokes `bootimage runner` per `.cargo/config.toml`
and does the same thing). Expected output on COM1: the boot image table,
`CLOCK` reporting a handful of real PIT ticks it observed while waiting,
the `pm`/`fs` demo tasks ping-ponging three messages back and forth
(proving real blocking `send`/`receive` and context switching), and
finally `IDLE` reporting that it's halting (with the accumulated tick
count) once everything else has blocked.

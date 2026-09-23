//! Process numbers and kernel notification types.
//!
//! Rust port of `include/minix/com.h`. This is the shared vocabulary that
//! the process table and scheduler (`crate::proc`) and the IPC layer
//! (`crate::ipc`) are built around: every process in the system image has
//! one of these fixed numbers, and every kernel notification carries one of
//! these message types.

/// Magic process numbers (mirrors `ANY`/`NONE`/`SELF` in com.h).
pub const ANY: i32 = 0x7ace;
pub const NONE: i32 = 0x6ace;
pub const SELF: i32 = 0x8ace;

/// Kernel tasks. These run with full kernel privilege in the kernel's own
/// context, unlike the user-space servers below, which will eventually run
/// as separate, unprivileged processes once address-space isolation lands
/// (see the roadmap in `rust/README.md`). `crate::proc` schedules IDLE and
/// CLOCK as real, runnable tasks; SYSTEM and KERNEL are reserved process
/// numbers (kernel calls and hardware interrupts, respectively) rather than
/// schedulable tasks in their own right, matching the real kernel.
pub const IDLE: i32 = -4;
pub const CLOCK: i32 = -3;
pub const SYSTEM: i32 = -2;
pub const KERNEL: i32 = -1;
pub const HARDWARE: i32 = KERNEL;

pub const NR_TASKS: usize = 4;

/// User-space servers and drivers that make up the rest of the boot image.
/// None of these are the *real* servers yet -- `crate::proc` temporarily
/// schedules these slots with stand-in bodies to exercise the scheduler
/// and IPC end to end, and only `DRVR_PROC_NR` (as
/// `crate::usermode::ring3_task_entry`) actually runs in ring 3 with its
/// own address space so far (see `rust/README.md`); the rest still share
/// the kernel's. The numbers are reserved so that code can be written
/// against the final shape of the system before the real servers exist.
pub const PM_PROC_NR: i32 = 0;
pub const FS_PROC_NR: i32 = 1;
pub const RS_PROC_NR: i32 = 2;
pub const MEM_PROC_NR: i32 = 3;
pub const LOG_PROC_NR: i32 = 4;
pub const TTY_PROC_NR: i32 = 5;
pub const DRVR_PROC_NR: i32 = 6;
pub const INIT_PROC_NR: i32 = 7;
/// `crate::rs`'s demo service: a real ring-3 task whose only instruction
/// deterministically crashes it, so `rs`'s restart policy has something
/// real to prove itself against.
pub const FLAKY_PROC_NR: i32 = 8;
/// `crate::keyboard`'s real line-discipline consumer (`console_task`):
/// blocks for a completed line, then writes it to `fs`.
pub const CONSOLE_PROC_NR: i32 = 9;
/// Reserved slot for `crate::rs`'s one runtime-launchable app
/// (`crate::elf::spawn_from_fs`), started on demand via a
/// `RS_LAUNCH_REQUEST` rather than at boot. A fixed, pre-reserved number
/// rather than a dynamically allocated one, same reasoning as
/// `FLAKY_PROC_NR`/`CONSOLE_PROC_NR`: there's no free-list/dynamic
/// process-number allocator in this port yet.
pub const APP1_PROC_NR: i32 = 10;
/// The `exec()` demo (`crate::calls::sys_exec`): a ring-3 task that
/// forks and whose *child*, a moment later, is running `/bin/exectest`
/// instead of the image it inherited -- the fork/exec pair a real shell
/// is built out of (`user/shell.s`). The child's own process number
/// isn't here, because it isn't reserved at compile time any more: see
/// `FIRST_DYNAMIC_PROC_NR`.
pub const SHELL_PROC_NR: i32 = 11;
/// `sh`, the interactive shell (`rust/user/src/bin/sh.rs`): the first
/// ring-3 program written in Rust rather than assembly, started at boot
/// and waiting for commands typed at the keyboard.
pub const SH_PROC_NR: i32 = 12;

/// The first process number handed out at *runtime* rather than nailed
/// down here (`crate::proc::alloc_proc_nr`), and how many of them there
/// are. Every number above is a fixed member of the system image, the
/// same way `kernel/table.c`'s `image[]` fixes MINIX's own; these are
/// the slots left over for processes that only exist because something
/// asked for one while the system was running -- which, in this port,
/// means a `fork()` (`crate::syscall`'s `SYS_FORK`).
///
/// Two reserved slots used to stand in for this: `FORK_CHILD_PROC_NR`,
/// "the one outstanding forked child this port supports at a time", and
/// before it the same trick for every other runtime-created process. A
/// fixed number per *caller* is what made `SYS_FORK` refuse anyone but
/// one known task; a small pool plus `alloc_proc_nr` is what makes it a
/// real call any process can make, as many times as there are slots.
pub const FIRST_DYNAMIC_PROC_NR: i32 = SH_PROC_NR + 1;
/// Sixteen (it was six, before threads: every thread is a slot too) is
/// headroom rather than a measurement: `user/shell.s` has at
/// most two children alive at once, `tty` one, and a console-launched
/// service one, so four would do -- but two of `shell`'s children exit
/// and are reaped (`crate::proc::wait_for_child`), and sizing the pool
/// to the exact peak would make the difference between "reaped" and
/// "still allocated" the difference between working and not.
pub const NR_DYNAMIC_PROCS: usize = 16;

/// A name for each dynamically allocated slot. `crate::proc::Proc::name`
/// is a `&'static str` -- fine for a fixed system image, where every
/// name is a literal, but a process created at runtime has no literal of
/// its own -- so the names are pre-written here rather than composed
/// when the process appears. Real MINIX has the same problem and solves
/// it the same way in reverse: `p_name` is a fixed-size char array
/// copied into, not a pointer.
pub const fn dynamic_proc_name(proc_nr: i32) -> &'static str {
    match proc_nr - FIRST_DYNAMIC_PROC_NR {
        0 => "forked child 1",
        1 => "forked child 2",
        2 => "forked child 3",
        3 => "forked child 4",
        4 => "forked child 5",
        5 => "forked child 6",
        6 => "forked child 7",
        7 => "forked child 8",
        8 => "forked child 9",
        9 => "forked child 10",
        10 => "forked child 11",
        11 => "forked child 12",
        12 => "forked child 13",
        13 => "forked child 14",
        14 => "forked child 15",
        15 => "forked child 16",
        _ => "forked child",
    }
}

/// How many process-table slots there are in total: every fixed process
/// number above, plus the dynamic range. Not "boot processes" -- several
/// of the fixed numbers (`FLAKY_PROC_NR`, `APP1_PROC_NR`) belong to
/// processes started well after boot, and the dynamic range belongs to
/// ones that have no number until they exist.
pub const NR_PROC_SLOTS: usize = NR_TASKS + FIRST_DYNAMIC_PROC_NR as usize + NR_DYNAMIC_PROCS;

/// Map a process number to a dense array index, for the process table
/// (`crate::proc`) and the IPC mailboxes it used to have on its own
/// (`crate::ipc`). Kernel tasks use negative numbers (`IDLE`..`KERNEL`);
/// boot-image servers use small non-negative numbers. Both ranges are
/// folded into one 0-based index here.
pub const fn slot(proc_nr: i32) -> usize {
    (proc_nr + NR_TASKS as i32) as usize
}

/// Notification message types, analogous to `NOTIFY_FROM(p_nr)` in com.h.
pub const NOTIFY_MESSAGE: i32 = 0x1000;

pub const fn notify_from(p_nr: i32) -> i32 {
    NOTIFY_MESSAGE | (p_nr + NR_TASKS as i32)
}

pub const SYN_ALARM: i32 = notify_from(CLOCK);
pub const SYS_SIG: i32 = notify_from(SYSTEM);
pub const HARD_INT: i32 = notify_from(HARDWARE);

/// Marker bit for "a process died" notifications (`crate::proc::kill`).
/// No real MINIX message type is quite this: `kernel/proc.c`'s
/// `cause_sig`/PM's exit path deliver a real signal or exit status, not a
/// bare notification, and this port has neither yet (see `crate::rs`'s
/// doc comment). The low bits carry the dead process's *slot* (`slot`),
/// not its process number, so `RS` learns *which* process died without
/// needing a separate payload/args field -- `proc_died`/`proc_died_slot`
/// are the encode/decode pair.
pub const PROC_DIED: i32 = 0x4000;

pub const fn proc_died(p_nr: i32) -> i32 {
    PROC_DIED | slot(p_nr) as i32
}

/// Decode a `proc_died` notification's `m_type` back to a slot index, or
/// `None` if it isn't one (every other notification type in this file
/// uses the `NOTIFY_MESSAGE` (`0x1000`) bit instead, so the two never
/// collide).
pub const fn proc_died_slot(m_type: i32) -> Option<usize> {
    if m_type & PROC_DIED != 0 {
        Some((m_type & !PROC_DIED) as usize)
    } else {
        None
    }
}

/// The inverse of `slot`: which process number lives in a given process-
/// table index. Only `crate::rs` needs this so far, to turn a
/// `proc_died_slot` result back into a real process number.
pub const fn proc_nr_of_slot(slot: usize) -> i32 {
    slot as i32 - NR_TASKS as i32
}

/// A real `send`/reply message type (not a fire-and-forget notification):
/// `crate::keyboard`'s `console_task` sends this to `RS_PROC_NR` to ask it
/// to launch a named service (`crate::rs`'s service table), mirroring
/// `crate::keyboard`'s own `CONSOLE_READ_LINE` request/reply shape.
pub const RS_LAUNCH_REQUEST: i32 = 401;

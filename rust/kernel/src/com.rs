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
/// None of these are real, independent processes yet -- no user mode, no
/// address-space isolation (see `rust/README.md`) -- but `crate::proc`
/// temporarily schedules a couple of these slots with stand-in bodies to
/// exercise the scheduler and IPC end to end; the numbers are reserved so
/// that code can be written against the final shape of the system before
/// the real servers exist.
pub const PM_PROC_NR: i32 = 0;
pub const FS_PROC_NR: i32 = 1;
pub const RS_PROC_NR: i32 = 2;
pub const MEM_PROC_NR: i32 = 3;
pub const LOG_PROC_NR: i32 = 4;
pub const TTY_PROC_NR: i32 = 5;
pub const DRVR_PROC_NR: i32 = 6;
pub const INIT_PROC_NR: i32 = 7;

pub const NR_BOOT_PROCS: usize = NR_TASKS + INIT_PROC_NR as usize + 1;

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

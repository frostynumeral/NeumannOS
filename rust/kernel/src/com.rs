//! Process numbers and kernel notification types.
//!
//! Rust port of `include/minix/com.h`. This is the shared vocabulary that
//! the kernel's process table (`crate::table`) and the IPC layer
//! (`crate::ipc`) are built around: every process in the system image has
//! one of these fixed numbers, and every kernel notification carries one of
//! these message types.

/// Magic process numbers (mirrors `ANY`/`NONE`/`SELF` in com.h).
pub const ANY: i32 = 0x7ace;
pub const NONE: i32 = 0x6ace;
pub const SELF: i32 = 0x8ace;

/// Kernel tasks. These run with full kernel privilege and, in this port,
/// simply exist as reserved process-table slots until the scheduler and
/// address-space isolation land (see the roadmap in `rust/README.md`).
pub const IDLE: i32 = -4;
pub const CLOCK: i32 = -3;
pub const SYSTEM: i32 = -2;
pub const KERNEL: i32 = -1;
pub const HARDWARE: i32 = KERNEL;

pub const NR_TASKS: usize = 4;

/// User-space servers and drivers that make up the rest of the boot image.
/// None of these are implemented yet; the numbers are reserved so that the
/// IPC and process-table code can be written against the final shape of the
/// system before the servers themselves exist.
pub const PM_PROC_NR: i32 = 0;
pub const FS_PROC_NR: i32 = 1;
pub const RS_PROC_NR: i32 = 2;
pub const MEM_PROC_NR: i32 = 3;
pub const LOG_PROC_NR: i32 = 4;
pub const TTY_PROC_NR: i32 = 5;
pub const DRVR_PROC_NR: i32 = 6;
pub const INIT_PROC_NR: i32 = 7;

pub const NR_BOOT_PROCS: usize = NR_TASKS + INIT_PROC_NR as usize + 1;

/// Notification message types, analogous to `NOTIFY_FROM(p_nr)` in com.h.
pub const NOTIFY_MESSAGE: i32 = 0x1000;

pub const fn notify_from(p_nr: i32) -> i32 {
    NOTIFY_MESSAGE | (p_nr + NR_TASKS as i32)
}

pub const SYN_ALARM: i32 = notify_from(CLOCK);
pub const SYS_SIG: i32 = notify_from(SYSTEM);
pub const HARD_INT: i32 = notify_from(HARDWARE);

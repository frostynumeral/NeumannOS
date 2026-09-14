//! The boot image: the fixed list of processes that make up the system.
//!
//! Rust port of `kernel/table.c`'s `image[]`. In C MINIX this table also
//! carries scheduling parameters and privilege bitmasks (which traps, IPC
//! targets, and kernel calls each process may use); those fields are kept
//! here as inert metadata for now, until process creation, address-space
//! separation, and the privilege-checking code in `kernel/system.c` have a
//! Rust equivalent.

use crate::com::*;

pub struct BootImageEntry {
    pub proc_nr: i32,
    pub name: &'static str,
}

pub static BOOT_IMAGE: [BootImageEntry; NR_BOOT_PROCS] = [
    BootImageEntry { proc_nr: IDLE, name: "IDLE" },
    BootImageEntry { proc_nr: CLOCK, name: "CLOCK" },
    BootImageEntry { proc_nr: SYSTEM, name: "SYSTEM" },
    BootImageEntry { proc_nr: HARDWARE, name: "KERNEL" },
    BootImageEntry { proc_nr: PM_PROC_NR, name: "pm" },
    BootImageEntry { proc_nr: FS_PROC_NR, name: "fs" },
    BootImageEntry { proc_nr: RS_PROC_NR, name: "rs" },
    BootImageEntry { proc_nr: TTY_PROC_NR, name: "tty" },
    BootImageEntry { proc_nr: MEM_PROC_NR, name: "memory" },
    BootImageEntry { proc_nr: LOG_PROC_NR, name: "log" },
    BootImageEntry { proc_nr: DRVR_PROC_NR, name: "driver" },
    BootImageEntry { proc_nr: INIT_PROC_NR, name: "init" },
    BootImageEntry { proc_nr: FLAKY_PROC_NR, name: "flaky" },
    BootImageEntry { proc_nr: CONSOLE_PROC_NR, name: "console" },
    BootImageEntry { proc_nr: APP1_PROC_NR, name: "hello (app slot, not started at boot)" },
];

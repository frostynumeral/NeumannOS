//! `ptrtest`: hands the kernel deliberately bad pointers through every
//! system call that takes one, and checks each is refused with an error
//! code rather than faulting. Before the kernel copied user memory
//! through the caller's page tables (`crate::syscall`'s
//! `copy_from_caller`/`copy_to_caller`), most of these halted the whole
//! machine: a ring-0 page fault is fatal. Run at boot by the kernel's
//! own verification (`crate::main`'s `ptr_safety_check`), which reads
//! the verdict back out of `/ptrtest.out`; runnable from `sh` too.

#![no_std]
#![no_main]

use neumann_rt::sys::{self, syscall4};
use neumann_rt::{println, Args};

neumann_rt::main!(main);

/// The kernel's heap (`crate::allocator::HEAP_START`): mapped in every
/// address space, so "is it mapped?" alone would let ring 3 read and
/// write it.
const KERNEL_HEAP: u64 = 0x4444_4444_0000;
/// In this program's own PML4 slot but never mapped.
const UNMAPPED: u64 = 0x3000_0080_0000;
/// This program's own text: mapped, user-accessible, read-only.
const OWN_TEXT: u64 = 0x3000_0000_0000;

const ERR_VIRCOPY_FAILED: i64 = -3;
/// `sh`, the interactive shell (the kernel's `com::SH_PROC_NR`).
const SH_PROC_NR: u64 = 12;

fn main(_args: Args) -> i32 {
    let fd = sys::open(b"/ptrtest.tmp").unwrap_or(-1);
    let mut status_word = 0i32;
    let cases: [(&str, i64, i64); 17] = unsafe {
        [
            ("SYS_WRITE_LINE from the kernel heap", syscall4(sys::SYS_WRITE_LINE, KERNEL_HEAP, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_WRITE_LINE from unmapped memory", syscall4(sys::SYS_WRITE_LINE, UNMAPPED, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_CONSOLE_WRITE from the kernel heap", syscall4(sys::SYS_CONSOLE_WRITE, KERNEL_HEAP, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_CONSOLE_WRITE from unmapped memory", syscall4(sys::SYS_CONSOLE_WRITE, UNMAPPED, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_FS_OPEN of a path on the kernel heap", syscall4(sys::SYS_FS_OPEN, KERNEL_HEAP, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_FS_OPEN_EXISTING of an unmapped path", syscall4(sys::SYS_FS_OPEN_EXISTING, UNMAPPED, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_FS_WRITE from the kernel heap", syscall4(sys::SYS_FS_WRITE, fd as u64, KERNEL_HEAP, 8, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_FS_READ into this program's read-only text", syscall4(sys::SYS_FS_READ, fd as u64, OWN_TEXT, 8, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_FS_READ into the kernel heap", syscall4(sys::SYS_FS_READ, fd as u64, KERNEL_HEAP, 8, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_READ_LINE into this program's read-only text", syscall4(sys::SYS_READ_LINE, OWN_TEXT, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_WAIT writing its status into read-only text", syscall4(sys::SYS_WAIT, OWN_TEXT, 0, 0, 0), sys::ERR_BAD_ARG_PTR),
            ("SYS_EXEC of a path on the kernel heap", syscall4(sys::SYS_EXEC, KERNEL_HEAP, 8, 0, 0), sys::ERR_BAD_ARG_PTR),
            // src_proc -4 is IDLE, a kernel task: its address space is the kernel's.
            ("SYS_VIRCOPY out of the kernel heap", syscall4(sys::SYS_VIRCOPY, (-4i64) as u64, KERNEL_HEAP, &mut status_word as *mut i32 as u64, 4), ERR_VIRCOPY_FAILED),
            // The source must be really mapped here, or the copy fails on
            // the source and the destination check is never what's tested:
            // `sh` (process 12, running since boot) is linked at the same
            // address as this program, so its text is there.
            ("SYS_VIRCOPY into the kernel heap", syscall4(sys::SYS_VIRCOPY, SH_PROC_NR, OWN_TEXT, KERNEL_HEAP, 4), ERR_VIRCOPY_FAILED),
            // Process numbers that name no slot used to index the process
            // table out of bounds -- a kernel panic.
            ("SYS_VIRCOPY from process 1000", syscall4(sys::SYS_VIRCOPY, 1000, OWN_TEXT, &mut status_word as *mut i32 as u64, 4), ERR_VIRCOPY_FAILED),
            ("SYS_VIRCOPY from process -5", syscall4(sys::SYS_VIRCOPY, (-5i64) as u64, OWN_TEXT, &mut status_word as *mut i32 as u64, 4), ERR_VIRCOPY_FAILED),
            // Zero bytes to a non-canonical address: nothing to do, and
            // must not panic building a `VirtAddr` out of it.
            ("SYS_VIRCOPY of 0 bytes to a non-canonical address", syscall4(sys::SYS_VIRCOPY, SH_PROC_NR, OWN_TEXT, 0x8000_0000_0000_0000, 0), 0),
        ]
    };

    let mut failed = 0;
    for (what, got, want) in cases.iter() {
        if got != want {
            println!("ptrtest: FAIL {}: got {}, expected {}", what, got, want);
            failed += 1;
        }
    }
    let verdict: &[u8] = if failed == 0 { b"ok" } else { b"FAIL" };
    if failed == 0 {
        println!("ptrtest: all {} bad-pointer calls refused with the right error", cases.len());
    }
    if let Ok(out) = sys::open(b"/ptrtest.out") {
        let _ = sys::write(out, verdict);
    }
    if failed == 0 { 0 } else { 1 }
}

//! Kernel calls: privileged operations servers need but can't do
//! themselves.
//!
//! Rust port of a first couple of entries in `kernel/system.c`'s
//! kernel-call dispatch table. Real MINIX dispatches these by call number
//! out of a message a process sends to `SYSTEM`; this port has no such
//! dispatch yet (see `rust/README.md` -- there's no real syscall argument
//! marshaling, just the one fixed `int 0x80` gate `crate::usermode` and
//! `crate::interrupts` use for their demo), so for now these are just
//! ordinary Rust functions any kernel task can call directly -- exactly
//! how `crate::ipc`'s `send`/`receive`/`notify` started out too, before
//! anything needed them from ring 3.

use crate::com;
use crate::memory::{self, CopyError};
use crate::proc;
use x86_64::VirtAddr;

/// `sys_vircopy()`/`sys_physcopy()`: copy `len` bytes from `src_addr` in
/// `src_proc`'s address space to `dst_addr` in `dst_proc`'s. Ported from
/// `kernel/system/do_copy.c`'s `do_copy()`, which handles both kernel
/// calls with one handler and dispatches to `virtual_copy()`; this port
/// only has the paged-address-space equivalent of that (`crate::memory`'s
/// `copy_between_address_spaces`), so there's no `SYS_PHYSCOPY`/segment
/// distinction to make here -- a virtual address is a virtual address
/// regardless of which process's page table it's resolved through.
///
/// Simplification: the C version accepts `SELF` for either process number
/// (meaning "the calling process") and validates process numbers before
/// touching them; neither is implemented here yet, since every current
/// caller already knows its own and the other side's real process number.
pub fn sys_vircopy(
    src_proc: i32,
    src_addr: VirtAddr,
    dst_proc: i32,
    dst_addr: VirtAddr,
    len: usize,
) -> Result<(), CopyError> {
    let (src_cr3, _) = proc::cr3_of(src_proc);
    let (dst_cr3, _) = proc::cr3_of(dst_proc);
    memory::copy_between_address_spaces(src_cr3, src_addr, dst_cr3, dst_addr, len)
}

/// `sys_setalarm()`: ask `CLOCK` to notify the calling task
/// (`com::SYN_ALARM`) once `delay_ticks` real timer ticks have elapsed.
/// Ported from `kernel/system/do_setalarm.c`; thin wrapper over
/// `proc::set_alarm`, which is where the actual bookkeeping and delivery
/// live (`crate::proc::clock_tick`) since they need direct access to the
/// process table.
///
/// Simplification: the C version lets the caller supply a callback
/// function pointer (for in-kernel watchdogs) or ask for a signal-style
/// notification depending on caller; this only implements the
/// notification path, since nothing in this port has an in-kernel
/// watchdog callback to register yet.
pub fn sys_setalarm(delay_ticks: u64) {
    proc::set_alarm(delay_ticks);
}

/// Process number `SYN_ALARM` notifications appear to come from --
/// re-exported here so callers of `sys_setalarm` don't need to reach into
/// `crate::com` just to match against it.
pub const SYN_ALARM: i32 = com::SYN_ALARM;

/// `sys_fork()`: create a new task (`child_proc_nr`) whose address space
/// starts as an independent copy of `src_proc`'s -- not just a structural
/// clone that still aliases the parent's existing pages
/// (`memory::new_address_space` alone), but a real, deep copy of each
/// address in `private_pages`, so a write to one side is invisible to the
/// other. Ported in spirit from `kernel/proc.c`'s `do_fork()` (called via
/// `PM_PROC_NR`'s `SYS_FORK`), which duplicates the parent's memory map
/// for the real thing; bundles what real MINIX splits across a kernel call
/// (duplicate the memory) and a separate scheduling step (make it
/// runnable), since nothing in this port needs them separated yet.
///
/// Simplification: this version starts the child at a fixed `fn() -> !`
/// entry point (see `crate::proc::spawn`), not "wherever the caller was" --
/// fine for `pm`'s own demo (a plain kernel-side child), but not real
/// `fork()` semantics. `sys_fork_from_frame` below is the ring-3-reachable
/// sibling that actually resumes at the caller's exact trapped
/// instruction. `private_pages` also has to be passed explicitly rather
/// than discovered by walking the parent's entire user-accessible range,
/// since there's no per-process memory-map bookkeeping (`kernel/kernel.h`'s
/// `struct mem_map`) to read it back out of yet.
#[allow(clippy::too_many_arguments)]
pub fn sys_fork(
    src_proc: i32,
    private_pages: &[VirtAddr],
    child_proc_nr: i32,
    name: &'static str,
    entry: fn() -> !,
    priority: u8,
    quantum: i32,
    preemptible: bool,
) {
    let (src_cr3, _) = proc::cr3_of(src_proc);
    let child_pml4 = memory::fork_address_space(src_cr3, private_pages);
    proc::spawn(child_proc_nr, name, entry, priority, quantum, preemptible, Some(child_pml4));
}

/// `sys_fork`'s real-fork-semantics sibling: same deep-copy-then-schedule
/// shape, but hands `frame` (a snapshot of the caller's own trap, taken by
/// `crate::syscall`'s `SYS_FORK` handler with `rax` already zeroed) to
/// `proc::fork_current` instead of a fixed entry point, so the new task
/// resumes exactly where its parent was, in ring 3, seeing `0` where the
/// parent sees this function's return value (the child's `proc_nr`) --
/// genuine `fork()` semantics, reachable from ring 3 through the syscall
/// ABI (`crate::syscall::SYS_FORK`).
#[allow(clippy::too_many_arguments)]
pub fn sys_fork_from_frame(
    src_proc: i32,
    private_pages: &[VirtAddr],
    child_proc_nr: i32,
    name: &'static str,
    priority: u8,
    quantum: i32,
    preemptible: bool,
    frame: &proc::TrapFrame,
) -> i32 {
    let (src_cr3, _) = proc::cr3_of(src_proc);
    let child_pml4 = memory::fork_address_space(src_cr3, private_pages);
    proc::fork_current(child_proc_nr, name, priority, quantum, preemptible, child_pml4, frame);
    child_proc_nr
}

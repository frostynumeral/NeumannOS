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
use crate::elf;
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
/// (`memory::new_address_space` alone), but a copy-on-write one: every
/// page the parent owns is shared until either side writes it, at which
/// point the writer gets its own copy, so a write to one side is still
/// invisible to the other. Ported in spirit from `kernel/proc.c`'s `do_fork()` (called via
/// `PM_PROC_NR`'s `SYS_FORK`), which duplicates the parent's memory map
/// for the real thing; bundles what real MINIX splits across a kernel call
/// (duplicate the memory) and a separate scheduling step (make it
/// runnable), since nothing in this port needs them separated yet.
///
/// Which pages to copy is no longer an argument: it comes from
/// `src_proc`'s own memory map (`proc::mem_map_of`, filled in by whoever
/// built that address space), so this works for any process with an
/// address space of its own rather than only for a caller someone has
/// hardcoded a page list for.
///
/// Simplification: this version starts the child at a fixed `fn() -> !`
/// entry point (see `crate::proc::spawn`), not "wherever the caller was" --
/// fine for `pm`'s own demo (a plain kernel-side child), but not real
/// `fork()` semantics. `sys_fork_from_frame` below is the ring-3-reachable
/// sibling that actually resumes at the caller's exact trapped
/// instruction.
///
/// `false` if the address space couldn't be built (see
/// `memory::fork_address_space`); no process is created in that case.
#[allow(clippy::too_many_arguments)]
pub fn sys_fork(
    src_proc: i32,
    child_proc_nr: i32,
    name: &'static str,
    entry: fn() -> !,
    priority: u8,
    quantum: i32,
    preemptible: bool,
) -> bool {
    match fork_child_address_space(src_proc) {
        Some(space) => {
            proc::spawn(child_proc_nr, name, entry, priority, quantum, preemptible, Some(space));
            true
        }
        None => false,
    }
}

/// The address space half of both `fork` entry points: a fresh,
/// independent copy of `src_proc`'s, described by the same memory map.
///
/// The new address space is derived from the *kernel's* PML4, not the
/// parent's, for the same reason `sys_exec` is (see
/// `memory::new_address_space_from`): the only things a child should
/// inherit are the kernel mappings every address space shares, plus its
/// own copy of the pages the map names. Deriving from the parent would
/// additionally leave the child aliasing whatever else happened to be in
/// the parent's top-level slots.
fn fork_child_address_space(src_proc: i32) -> Option<proc::AddressSpace> {
    let map = proc::mem_map_of(src_proc);
    let (src_cr3, _) = proc::cr3_of(src_proc);
    let pml4 = memory::fork_address_space(src_cr3, proc::kernel_cr3(), &map)?;
    Some(proc::AddressSpace { pml4, map })
}

/// `sys_fork`'s real-fork-semantics sibling: same copy-then-schedule
/// shape, but hands `frame` (a snapshot of the caller's own trap, taken by
/// `crate::syscall`'s `SYS_FORK` handler with `rax` already zeroed) to
/// `proc::fork_current` instead of a fixed entry point, so the new task
/// resumes exactly where its parent was, in ring 3, seeing `0` where the
/// parent sees this function's return value (the child's `proc_nr`) --
/// genuine `fork()` semantics, reachable from ring 3 through the syscall
/// ABI (`crate::syscall::SYS_FORK`).
///
/// `None` -- with no process created and the caller left exactly as it
/// was -- if the child's address space couldn't be built.
#[allow(clippy::too_many_arguments)]
pub fn sys_fork_from_frame(
    src_proc: i32,
    child_proc_nr: i32,
    name: &'static str,
    priority: u8,
    quantum: i32,
    preemptible: bool,
    frame: &proc::TrapFrame,
) -> Option<i32> {
    let space = fork_child_address_space(src_proc)?;
    // The child's parent is the caller's *team*: a thread that forks
    // makes a child of its process, collected by whichever thread waits.
    proc::fork_current(proc::team_of(src_proc), child_proc_nr, name, priority, quantum, preemptible, space, frame);
    Some(child_proc_nr)
}

/// `sys_exec()`: throw away everything `proc_nr` was running and give it
/// `image` instead, keeping the process itself -- its process number, its
/// priority, its kernel stack, its place in the ready queue, every other
/// process's right to send to it. `fork` makes a second process that is
/// the caller; `exec` keeps the one process and replaces what it is.
/// Together they're how every process after `init` comes to exist on a
/// real system.
///
/// Ported in spirit from the pair MINIX splits this across:
/// `servers/pm/exec.c`'s `do_exec` (find the file, work out the memory
/// layout, load the image) and the `SYS_EXEC` kernel call it then makes,
/// `kernel/system/do_exec.c` (point the process at the new image and set
/// its saved `pc`/`sp` so it resumes there). This does the first half and
/// the address-space swap; the second half -- writing the new entry point
/// and stack pointer into the caller's *live* trap frame -- belongs to
/// `crate::syscall`'s `SYS_EXEC` handler, which is the only code holding
/// that frame, exactly as `do_exec.c` is the only code holding
/// `rp->p_reg`. The returned `LoadedImage` is what it needs for that.
///
/// The new address space is built from the kernel's PML4, not the
/// caller's (`memory::new_address_space_from`), so the new image starts
/// with a genuinely empty user address space rather than inheriting the
/// mappings of the program it replaced. Nothing frees those old mappings
/// -- see `proc::set_address_space` for that caveat.
///
/// `args` is the new image's `argv`/`envp`, already copied out of the
/// caller's memory (`crate::syscall`'s `SYS_EXEC` does that, since the
/// caller's pointers stop meaning anything the moment this swaps address
/// spaces) and laid out on the new stack by the loader
/// (`elf::StartArgs`) -- the part `do_exec` does by copying the caller's
/// prepared stack image into `mbuf` and relocating its pointers.
///
/// Notably absent compared to real `exec`: any notion of file
/// permissions or a set-uid bit, and closing file descriptors marked
/// close-on-exec (`fs` has no such flag, and descriptors here survive the
/// call, as they would for a plain POSIX `exec` without it).
pub fn sys_exec(
    proc_nr: i32,
    image: &[u8],
    args: &elf::StartArgs,
) -> Result<elf::LoadedImage, elf::ElfError> {
    let loaded = elf::load_image_with_args(proc::kernel_cr3(), image, args)?;
    proc::set_address_space(proc_nr, proc::AddressSpace { pml4: loaded.pml4, map: loaded.map });
    Ok(loaded)
}

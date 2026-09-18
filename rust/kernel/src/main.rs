//! NeumannOS kernel entry point.
//!
//! This is the Rust port's `kernel/main.c` equivalent: it boots, sets up
//! the GDT/IDT so CPU faults are reported instead of triple-faulting
//! (`crate::gdt`, `crate::interrupts`), sets up paging and a heap
//! allocator (`crate::memory`, `crate::allocator`), builds the ring-3 demo
//! task's own address space and maps its pages into it
//! (`crate::usermode`, `crate::memory::new_address_space`), spawns the
//! kernel tasks (`crate::proc`) -- including that ring-3 task, which now
//! runs in genuine isolation from the kernel's own address space, and can
//! still be asynchronously preempted and take repeated traps like any
//! other task, since each task has its own dedicated `RSP0`
//! (`crate::gdt::set_rsp0`) -- programs the PIC/PIT and enables interrupts
//! so the timer starts driving real, asynchronously-preemptive
//! scheduling, prints the boot image (the process table MINIX would load
//! into memory at this point), and hands off to the scheduler -- just as
//! `kernel/main.c` ends by calling `restart()`. See `rust/README.md` for
//! what's implemented versus planned.
#![no_std]
#![no_main]
#![feature(abi_x86_interrupt)]

extern crate alloc;

mod allocator;
mod calls;
mod com;
mod elf;
mod fs;
mod gdt;
mod interrupts;
mod ipc;
mod keyboard;
mod memory;
mod pic;
mod pit;
mod proc;
mod rs;
mod serial;
mod syscall;
mod table;
mod usermode;
mod vga;

use alloc::boxed::Box;
use alloc::vec::Vec;
use bootloader::{entry_point, BootInfo};
use core::panic::PanicInfo;
use x86_64::structures::paging::PhysFrame;
use x86_64::VirtAddr;

entry_point!(kernel_main);

fn kernel_main(boot_info: &'static BootInfo) -> ! {
    serial::init();
    serial_println!("NeumannOS kernel booting (Rust port of MINIX 3.1.0)");
    serial_println!();

    gdt::init();
    interrupts::init_idt();
    serial_println!("GDT/IDT loaded");
    // Self-test: deliberately trip a breakpoint exception. The handler
    // prints and returns, proving the IDT is wired up and that a fault
    // doesn't crash the kernel.
    x86_64::instructions::interrupts::int3();
    serial_println!("survived breakpoint exception");

    let phys_mem_offset = VirtAddr::new(boot_info.physical_memory_offset);

    // Draw the LCARS-style demo panel as early as possible: it only
    // needs the physical-memory window the bootloader's own
    // `map_physical_memory` feature already set up (see vga.rs), not
    // paging/heap/scheduler setup below, so it stays visible on screen
    // even if something later in boot panics.
    vga::init_palette();
    vga::draw_demo_panel(vga::framebuffer(phys_mem_offset));
    serial_println!("vga: painted the LCARS demo panel (mode 13h, 320x200)");

    let mut mapper = unsafe { memory::init(phys_mem_offset) };
    unsafe { memory::init_frame_allocator(&boot_info.memory_map) };
    allocator::init_heap(&mut mapper).expect("heap initialization failed");
    serial_println!(
        "heap mapped: {} KiB at {:#x}",
        allocator::HEAP_SIZE / 1024,
        allocator::HEAP_START
    );
    // Self-test: exercise the global allocator through both a Box (single
    // fixed-size allocation) and a growing Vec (multiple reallocations),
    // proving the heap actually works rather than merely compiling.
    let boxed = Box::new(41u32 + 1);
    let mut vec: Vec<u32> = Vec::new();
    for i in 0..100u32 {
        vec.push(i);
    }
    serial_println!(
        "heap self-test: boxed={}, vec.len()={}, vec.sum()={}",
        boxed,
        vec.len(),
        vec.iter().sum::<u32>()
    );
    drop(boxed);
    drop(vec);

    // Build the ring-3 demo task's own address space and map its
    // code/stack pages into it now; the task itself (spawned below) does
    // the actual jump to ring 3 once the scheduler runs it.
    let ring3_address_space = usermode::create_address_space(phys_mem_offset);
    // Same idea, but loading a real ELF binary's segments (crate::elf)
    // instead of hand-placing a fixed byte array.
    let elf_address_space = elf::load(elf::HELLO_ELF, com::TTY_PROC_NR);
    // And a third: the first half of the exec() demo (`user/shell.s`),
    // which replaces itself with a *different* program the moment it
    // runs (crate::calls::sys_exec). Loaded here exactly like any other
    // ELF task -- exec is what makes it interesting, not how it starts.
    let shell_address_space = elf::load(elf::SHELL_ELF, com::SHELL_PROC_NR);
    // Self-test: the loader's *rejections*, which are the only part of
    // its validation that a working boot can't demonstrate. See
    // `elf::validator_self_test` for why this is here and not assumed.
    elf::validator_self_test();
    // Self-test: this is the actual proof of isolation, not just that
    // things still work. The demo pages were never mapped into *this*
    // (the kernel's own) page table -- only into `ring3_address_space` --
    // so translating that address here must fail.
    use x86_64::structures::paging::Translate;
    serial_println!(
        "isolation self-test: ring-3 code page at {:#x} in the kernel's own address space: {:?}",
        usermode::USER_CODE_ADDR,
        mapper.translate_addr(VirtAddr::new(usermode::USER_CODE_ADDR)),
    );
    serial_println!();

    serial_println!("boot image:");
    for entry in table::BOOT_IMAGE.iter() {
        serial_println!("  proc_nr={:>3}  name={}", entry.proc_nr, entry.name);
    }
    serial_println!();

    // Tasks must be spawned (and so enqueued as ready) before interrupts
    // are enabled: the timer can start firing as soon as the PIT is
    // programmed, and its handler calls `reschedule()` unconditionally
    // (see `proc.rs`'s module doc comment) -- with no task enqueued yet,
    // there would be nothing for it to pick.
    spawn_tasks(ring3_address_space, elf_address_space, shell_address_space);
    serial_println!("tasks spawned");

    pic::init();
    pit::init();
    x86_64::instructions::interrupts::enable();
    serial_println!("PIC remapped, PIT programmed for {} Hz, interrupts enabled", pit::HZ);
    serial_println!("handing off to the scheduler");
    serial_println!();
    proc::start()
}

/// Spawn the kernel tasks. `IDLE` and `CLOCK` are real kernel tasks, same
/// as in the boot image; `pm`/`memory`/`driver` don't exist as real
/// servers yet (see `rust/README.md`), so their process table slots run
/// small stand-in bodies instead: `pm` exercises blocking `send`/
/// `receive` (and now real IPC-served file I/O via `fs`), `memory` (as
/// `busy_task`) exercises asynchronous preemption, and `driver` (as
/// `usermode::ring3_task_entry`) runs in its own address space
/// (`ring3_address_space`, built in `kernel_main`) -- one of three tasks
/// here that isn't sharing the kernel's. `fs` is now a real, in-memory
/// file server (`fs::InMemoryFs::serve`). `tty` (as `elf::task_entry`)
/// is another task with its own address space (`elf_address_space`): a
/// real ELF binary (`elf::HELLO_ELF`), loaded and run in ring 3 rather
/// than a hand-assembled demo. `rs` is now a real reincarnation server
/// (`rs::task`), monitoring `flaky` (`rs::spawn_flaky`, the third and
/// last ring-3 task here) -- a service that crashes the instant it runs
/// (on purpose, via `ud2`), letting `rs` prove it can restart a dead
/// process rather than the whole machine going down with it. Priorities,
/// quantum sizes, and preemptibility match `kernel/table.c`'s image
/// entries (`IDL_F`/`TSK_F`/`SRV_F` flags) where a real counterpart
/// exists.
fn spawn_tasks(
    ring3_address_space: PhysFrame,
    elf_address_space: PhysFrame,
    shell_address_space: PhysFrame,
) {
    proc::spawn(com::IDLE, "IDLE", idle_task, proc::IDLE_Q, 8, true, None);
    proc::spawn(com::CLOCK, "CLOCK", clock_task, proc::TASK_Q, 64, false, None);
    proc::spawn(com::PM_PROC_NR, "pm (demo)", demo_pm_task, 3, 32, true, None);
    proc::spawn(com::FS_PROC_NR, "fs (demo)", demo_fs_task, 4, 32, true, None);
    // Priority 5, one step above the other demo tasks (6): `rs` needs to
    // already be blocked in `ipc::receive` before `flaky` gets a chance to
    // crash, since `proc::kill`'s notification to `RS` is fire-and-forget,
    // exactly like every other notification in this port (see "known
    // simplifications" in rust/README.md) -- silently dropped if `RS`
    // isn't receiving yet. A strictly higher priority guarantees `rs`
    // gets the CPU (and reaches that first blocking `receive`) before any
    // priority-6 task, including `flaky`, regardless of spawn order.
    proc::spawn(com::RS_PROC_NR, "rs", rs::task, 5, 16, true, None);
    proc::spawn(com::MEM_PROC_NR, "memory (demo)", busy_task, 6, 16, true, None);
    proc::spawn(
        com::DRVR_PROC_NR,
        "driver (ring3 demo)",
        usermode::ring3_task_entry,
        6,
        16,
        true,
        Some(ring3_address_space),
    );
    proc::spawn(
        com::TTY_PROC_NR,
        "tty (elf demo)",
        elf::task_entry,
        6,
        16,
        true,
        Some(elf_address_space),
    );
    // The exec() demo, at the same priority as the other ring-3 tasks:
    // it starts out running `elf::SHELL_ELF` and ends up running
    // /bin/echo, without ever ceasing to be this process.
    proc::spawn(
        com::SHELL_PROC_NR,
        "shell (exec demo)",
        elf::task_entry,
        6,
        16,
        true,
        Some(shell_address_space),
    );
    rs::spawn_flaky();
    proc::spawn(com::CONSOLE_PROC_NR, "console", keyboard::console_task, 5, 16, true, None);
}

/// Real MINIX's idle task just halts, waking on the next interrupt; ported
/// as-is. It's still preemptible (matching `IDL_F`), though with nothing
/// else runnable that mostly just means its own ticks get charged to it.
///
/// `IDLE` only ever becomes current once *everything* else has blocked
/// (it's the lowest-priority queue, `proc::IDLE_Q`) -- `memory`'s
/// busy-loop alone keeps that from happening until around tick 30 (see
/// `busy_task`), comfortably after `tty`'s entire sequence (its counter
/// loop, its `SYS_VIRCOPY`, and its alarm-triggered
/// `SYS_FS_OPEN`/`SYS_FS_WRITE`, `user/hello.s`) has had time to run --
/// so this, unlike `CLOCK`'s own fixed-alarm wakeup, is a genuinely safe
/// "everything that's going to happen already has" checkpoint, not a
/// race against `tty`'s actual progress. `elf_counter_demo` used to run
/// from `clock_task` instead, racing `tty`'s counter loop against
/// `CLOCK`'s own independent 3-tick alarm -- "true in practice" (per its
/// old doc comment) until real timing variance proved otherwise (an
/// observed, reproducible boot panic, not a hypothetical one); it and
/// `vircopy_from_ring3_verify` both belong here instead, for the same
/// reason `fs_from_ring3_verify` already does.
fn idle_task() -> ! {
    elf_counter_demo();
    fs_from_ring3_verify();
    vircopy_from_ring3_verify();
    vircopy_error_from_ring3_verify();
    fork_child_verify();
    exec_verify();
    serial_println!("[idle] no other task is ready, halting (uptime: {} ticks)", proc::uptime_ticks());
    halt_loop()
}

/// Proves `tty`'s `SYS_FS_OPEN`/`SYS_FS_WRITE` calls (`user/hello.s`, via
/// `crate::syscall`) genuinely reached `fs` over IPC and landed in real
/// storage: opens the same path *fresh* (a distinct file descriptor, its
/// own cursor at `0`, reached the ordinary kernel-side way -- `crate::fs`'s
/// client stubs, not a syscall) and reads back exactly the bytes `tty`
/// wrote from ring 3. Not just "the syscall didn't crash" -- the actual
/// content, read back through a completely independent path.
fn fs_from_ring3_verify() {
    let expected = b"written from ring 3 via a real syscall, IPC, and fs!";
    let fd = fs::open("/from_ring3.txt");
    assert!(fd >= 0, "fs::open(\"/from_ring3.txt\") failed: {}", fd);
    let mut buf = [0u8; 64];
    let n = fs::read(fd, &mut buf);
    serial_println!(
        "[idle] read back {:?} from /from_ring3.txt (written by tty from ring 3 via SYS_FS_OPEN/SYS_FS_WRITE)",
        core::str::from_utf8(&buf[..n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(n, expected.len() as i64, "wrong length read back from /from_ring3.txt");
    assert_eq!(&buf[..n as usize], expected, "fs content doesn't match what tty wrote via a real syscall");
}

/// Proves `tty`'s ring-3 `SYS_VIRCOPY` call (`user/hello.s`, via
/// `crate::syscall`) genuinely copied `driver`'s private memory into
/// `tty`'s *own* address space, not just that the syscall returned
/// success: reads `tty`'s `vircopy_buf` (`elf::VIRCOPY_BUF_ADDR`) back out
/// with a second, independent `sys_vircopy` call (kernel-side, the same
/// mechanism `vircopy_demo` above already proved) and checks it matches
/// `usermode::USER_CODE` -- the exact bytes `driver`'s own code page
/// holds. Same "everything that's going to happen already has" checkpoint
/// as `fs_from_ring3_verify` above, for the same reason: `tty`'s
/// `SYS_VIRCOPY` runs well before its own `SYS_READ_LINE` blocks, which is
/// itself well before `IDLE` ever becomes current.
fn vircopy_from_ring3_verify() {
    let mut buf = [0u8; usermode::USER_CODE.len()];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::VIRCOPY_BUF_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading tty's vircopy_buf");
    serial_println!(
        "[idle] read back {:?} from tty's own vircopy_buf (copied there by tty itself via ring-3 SYS_VIRCOPY)",
        buf
    );
    assert_eq!(
        buf, usermode::USER_CODE,
        "tty's ring-3 SYS_VIRCOPY didn't actually copy driver's code page"
    );
}

/// Proves the syscall ABI's distinct error codes (`crate::syscall`'s
/// `ERR_BAD_LENGTH`/`ERR_BAD_UTF8`/`ERR_VIRCOPY_FAILED`/
/// `ERR_UNKNOWN_CALL`) genuinely reach a ring-3 caller's `rax`, not just
/// that `dispatch` computes the right value internally: `tty` makes a
/// second, deliberately-invalid `SYS_VIRCOPY` call (an oversized `len`,
/// `user/hello.s`) and stashes the raw return value in `err_result`
/// (`elf::ERR_RESULT_ADDR`); this reads it back the same way
/// `vircopy_from_ring3_verify` reads `vircopy_buf` and checks it's
/// exactly `syscall::ERR_BAD_LENGTH`, not merely "some nonzero failure
/// code."
fn vircopy_error_from_ring3_verify() {
    let mut buf = [0u8; 8];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::ERR_RESULT_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading tty's err_result");
    let result = u64::from_le_bytes(buf);
    serial_println!(
        "[idle] read back {:#x} from tty's own err_result (expected ERR_BAD_LENGTH {:#x})",
        result,
        syscall::ERR_BAD_LENGTH
    );
    assert_eq!(
        result, syscall::ERR_BAD_LENGTH,
        "tty's deliberately-invalid ring-3 SYS_VIRCOPY didn't come back ERR_BAD_LENGTH"
    );
}

/// Proves `tty`'s ring-3 `SYS_FORK` call (`user/hello.s`, via
/// `crate::syscall`) genuinely created an independent child process, not
/// just that the syscall returned a plausible-looking proc_nr: checks two
/// things a shared-page aliasing bug couldn't fake. First, the forked
/// child's own `vircopy_buf` reads back the canary (`0xcafebabe`) the
/// child's ring-3 code wrote into *its own* copy right after forking --
/// read via a kernel-side `sys_vircopy` targeting
/// `com::FORK_CHILD_PROC_NR` specifically, not `tty`. Second, `tty`'s own
/// `vircopy_buf` (already checked by `vircopy_from_ring3_verify` above)
/// still holds `driver`'s code bytes, not the child's canary -- if
/// `memory::fork_address_space` had left the data page merely aliased
/// instead of genuinely deep-copied, the child's later write would have
/// clobbered the parent's copy too, and that earlier check would already
/// have failed. Finally, reads back the distinguishing file the child
/// wrote (`/from_fork_child.txt`, a real `SYS_FS_OPEN`/`SYS_FS_WRITE` from
/// the child's own, separately-scheduled ring-3 execution) the same way
/// `fs_from_ring3_verify` does for `tty`'s own file.
fn fork_child_verify() {
    let mut canary_buf = [0u8; 4];
    calls::sys_vircopy(
        com::FORK_CHILD_PROC_NR,
        x86_64::VirtAddr::new(elf::VIRCOPY_BUF_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(canary_buf.as_mut_ptr() as u64),
        canary_buf.len(),
    )
    .expect("sys_vircopy failed reading the forked child's vircopy_buf");
    let canary = u32::from_le_bytes(canary_buf);
    serial_println!(
        "[idle] read back {:#x} from the forked child's own vircopy_buf (expected canary 0xcafebabe)",
        canary
    );
    assert_eq!(
        canary, 0xcafebabe,
        "the forked child's vircopy_buf doesn't hold its own canary -- fork may have left it aliased with tty's"
    );

    let expected = b"hello from the forked child, running independently in ring 3!";
    let fd = fs::open("/from_fork_child.txt");
    assert!(fd >= 0, "fs::open(\"/from_fork_child.txt\") failed: {}", fd);
    let mut buf = [0u8; 96];
    let n = fs::read(fd, &mut buf);
    serial_println!(
        "[idle] read back {:?} from /from_fork_child.txt (written by the forked child from its own ring-3 execution)",
        core::str::from_utf8(&buf[..n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(n, expected.len() as i64, "wrong length read back from /from_fork_child.txt");
    assert_eq!(&buf[..n as usize], expected, "fs content doesn't match what the forked child wrote");
}

/// Proves `shell`'s ring-3 `SYS_EXEC` call (`user/shell.s`, via
/// `crate::syscall`) genuinely *replaced* that process's program rather
/// than starting a second one alongside it. Five independent checks, in
/// the order they run, each ruling out a different way a fake (or
/// half-done) exec could still look real:
///
/// 1. `/from_exec.txt` holds what `user/echo.s` writes -- read back
///    through `fs`'s ordinary kernel-side path, so the exec'd image
///    demonstrably ran and did real work, not just that the syscall
///    logged something. (An exec that loaded the image but never
///    transferred control would fail here.)
/// 2. `/exec_error.bin` holds `syscall::ERR_BAD_ELF`, written by
///    `shell` *after* a deliberately-failed exec of an ordinary text
///    file it created itself (`user/shell.s`). That file existing at all
///    is the real result: a failed exec has to leave the caller running
///    its original image (POSIX requires it, and `sys_exec` discards the
///    old address space only a few lines after building the new one), so
///    `shell` had to still be `shell` -- own `.data`, own stack -- long
///    enough to write it.
/// 3. `/evil_exec_error.bin` holds `syscall::ERR_BAD_ELF`, written by
///    the exec'd image after it tried, from ring 3, to exec a
///    hand-built ELF whose only segment asks to be mapped at
///    `allocator::HEAP_START` -- the kernel's own heap (`user/echo.s`).
///    The first version of this port's validator accepted exactly that
///    image, and mapping it wrote into the kernel's own page tables,
///    because a new address space copies only the top-level table and
///    the check bounded addresses rather than PML4 slots.
///    `elf::validator_self_test` covers the same ground kernel-side;
///    this is the one that proves the path is shut to an actual
///    unprivileged process.
/// 4. `user/echo.s`'s own `.data` counter (`elf::ECHO_COUNTER_ADDR`)
///    reads `1` *in `com::SHELL_PROC_NR`'s address space* -- the new
///    program's instructions ran inside the original process's slot, not
///    somewhere else. (A `spawn`-in-disguise would have put it in a
///    different process.)
/// 5. `user/shell.s`'s `pre_exec_marker` (`elf::SHELL_MARKER_ADDR`), a
///    page that demonstrably *was* mapped in this process a moment ago
///    (`shell` wrote to it before calling exec, and a write to an
///    unmapped page would have faulted), is now unmapped: reading it
///    back has to fail with `CopyError::SrcNotMapped`. (An exec that
///    merely added the new image's mappings to the old address space --
///    the easy way to get checks 1 and 4 to pass -- would fail here.)
///
/// Runs from `idle_task` for the same reason every other check there
/// does; see its doc comment. Check 1 additionally waits rather than
/// assuming: `shell` sleeps between exec attempts if `/bin/echo` hasn't
/// been installed yet (`user/shell.s`), and `IDLE` becoming runnable
/// during one of those naps is exactly the sort of interleaving this
/// port has already been bitten by once.
fn exec_verify() {
    let expected = b"written by the exec'd image, not the one that called exec";
    let mut buf = [0u8; 96];
    let n = read_when_available("/from_exec.txt", &mut buf);
    serial_println!(
        "[idle] read back {:?} from /from_exec.txt (written by the image shell exec'd itself into)",
        core::str::from_utf8(&buf[..n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(n, expected.len() as i64, "wrong length read back from /from_exec.txt");
    assert_eq!(&buf[..n as usize], expected, "fs content doesn't match what the exec'd image wrote");

    let mut err_buf = [0u8; 8];
    let err_n = read_when_available("/exec_error.bin", &mut err_buf);
    assert_eq!(err_n, 8, "wrong length read back from /exec_error.bin");
    let exec_error = u64::from_le_bytes(err_buf);
    serial_println!(
        "[idle] read back {:#x} from /exec_error.bin -- shell's deliberately-failed exec of a non-ELF file (expected ERR_BAD_ELF {:#x}), written by shell itself afterward",
        exec_error,
        syscall::ERR_BAD_ELF
    );
    assert_eq!(
        exec_error,
        syscall::ERR_BAD_ELF,
        "exec'ing a non-ELF file should come back ERR_BAD_ELF to the ring-3 caller"
    );

    let mut evil_buf = [0u8; 8];
    let evil_n = read_when_available("/evil_exec_error.bin", &mut evil_buf);
    assert_eq!(evil_n, 8, "wrong length read back from /evil_exec_error.bin");
    let evil_result = u64::from_le_bytes(evil_buf);
    serial_println!(
        "[idle] read back {:#x} from /evil_exec_error.bin -- a ring-3 exec of an image asking to be mapped onto the kernel heap (expected ERR_BAD_ELF {:#x})",
        evil_result,
        syscall::ERR_BAD_ELF
    );
    assert_eq!(
        evil_result,
        syscall::ERR_BAD_ELF,
        "a ring-3 process was able to exec an image targeting the kernel's own address range"
    );

    let mut counter_buf = [0u8; 4];
    calls::sys_vircopy(
        com::SHELL_PROC_NR,
        x86_64::VirtAddr::new(elf::ECHO_COUNTER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(counter_buf.as_mut_ptr() as u64),
        counter_buf.len(),
    )
    .expect("sys_vircopy failed reading the exec'd image's counter out of shell's address space");
    let counter = i32::from_le_bytes(counter_buf);
    serial_println!(
        "[idle] read back {} from /bin/echo's own .data counter, in shell's process (expected 1)",
        counter
    );
    assert_eq!(counter, 1, "the exec'd image's own code should have incremented its counter once");

    let mut marker_buf = [0u8; 4];
    let stale = calls::sys_vircopy(
        com::SHELL_PROC_NR,
        x86_64::VirtAddr::new(elf::SHELL_MARKER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(marker_buf.as_mut_ptr() as u64),
        marker_buf.len(),
    );
    serial_println!(
        "[idle] reading shell's pre-exec marker page at {:#x} back: {:?} (expected SrcNotMapped -- the old image is gone)",
        elf::SHELL_MARKER_ADDR,
        stale
    );
    assert!(
        matches!(stale, Err(memory::CopyError::SrcNotMapped)),
        "shell's pre-exec .data page is still mapped after exec -- the old image wasn't replaced, only added to"
    );
}

/// Read `path`, waiting until it both exists *and* has content. `fs`
/// starts empty every boot and this port has no "wait for a file"
/// primitive (no `select`, no inotify, nothing), so the wait is a plain
/// poll bounded by a real deadline (see the tick comment below).
///
/// `fs::open_existing`, not `fs::open` -- and that distinction is the
/// whole reason this function works. `fs::open` creates a missing file
/// (`crate::fs`'s `open_path`), so polling with it would succeed on the
/// first attempt no matter what, read zero bytes, and leave behind an
/// empty file at the very path it was waiting for: a "wait" that can
/// never wait, and that destroys the evidence it was checking for. The
/// zero-length retry covers the other half of the same race -- the
/// writer having opened the file but not yet written to it.
fn read_when_available(path: &str, buf: &mut [u8]) -> i64 {
    // Bounded in real ticks, not in attempts. An attempt count is the
    // wrong unit here: `yield_now` from `IDLE` with every other task
    // asleep re-picks `IDLE` and returns immediately, so five hundred
    // attempts can elapse inside a single timer tick -- while the task
    // being waited for is blocked on a one-tick alarm and hasn't had a
    // chance to run at all. (Observed, not hypothetical: with `pm`'s
    // seeding artificially delayed, this gave up before `shell` had
    // retried its exec even twice.) Spinning on `yield_now` until the
    // clock moves is what actually lets the other task make progress --
    // and it has to be a spin rather than a real `sys_setalarm` sleep,
    // because `IDLE` blocking would leave the scheduler with nothing
    // runnable at all.
    const MAX_TICKS: u64 = 300; // 5 seconds at pit::HZ
    let deadline = proc::uptime_ticks() + MAX_TICKS;
    loop {
        let fd = fs::open_existing(path);
        if fd >= 0 {
            // A file that exists but is still empty means the writer got
            // as far as `open` and no further -- the same race, one step
            // later.
            let n = fs::read(fd, buf);
            if n != 0 {
                return n;
            }
        }
        if proc::uptime_ticks() >= deadline {
            panic!("{} never appeared within {} ticks", path, MAX_TICKS);
        }
        proc::yield_now();
    }
}

/// Stand-in for `kernel/clock.c`'s clock task. Now much closer to the real
/// thing than earlier milestones' version: it calls `sys_setalarm`
/// (`crate::calls`) and blocks in `receive`, exactly like real MINIX's
/// `while (TRUE) { receive(HARDWARE, &m); ...; }` waiting on
/// `do_clocktick`'s `lock_notify`, instead of polling
/// `proc::uptime_ticks()` in a spin-yield loop. `crate::proc::clock_tick`
/// is what actually notices the deadline and delivers the `SYN_ALARM`
/// that wakes this up.
///
/// Also demonstrates `sys_vircopy` (see `vircopy_demo` below) before
/// settling down, since `CLOCK` is a convenient, deterministic place to
/// run it: the ring-3 task's code page is populated in `kernel_main`
/// before any task ever runs, so this works regardless of scheduling
/// order, without needing to coordinate with that task directly.
/// (`elf_counter_demo`/`vircopy_from_ring3_verify` below used to run from
/// here too, racing `CLOCK`'s own fixed 3-tick alarm against `tty`'s
/// actual progress through the ready queue -- "true in practice" until it
/// wasn't; see `idle_task` for where they live now and why that's
/// actually safe.) Then demonstrates the other real-servers prerequisite
/// this milestone adds:
/// dynamically spawning a brand new task (`log`) at runtime, with the
/// scheduler already running other tasks -- the primitive real `rs`
/// (starting services on demand, not just at boot) actually needs.
fn clock_task() -> ! {
    calls::sys_setalarm(3);
    let notif = ipc::receive(com::CLOCK);
    assert_eq!(notif.m_type, calls::SYN_ALARM, "expected a SYN_ALARM notification");
    serial_println!(
        "[clock] woken by a real SYN_ALARM notification (m_type {:#x}) after {} real PIT ticks",
        notif.m_type,
        proc::uptime_ticks()
    );

    vircopy_demo();

    serial_println!("[clock] dynamically spawning a new task (log) at runtime");
    proc::spawn(com::LOG_PROC_NR, "log (dynamic)", dynamic_log_task, 5, 24, true, None);

    ipc::receive(com::ANY); // nothing left to receive; parks CLOCK for good
    unreachable!("nothing sends to CLOCK in this demo");
}

/// Spawned at runtime by `clock_task`, well after `kernel_main` has handed
/// off to the scheduler and other tasks are already running -- proving
/// `proc::spawn` genuinely works as a dynamic "start a new process now"
/// primitive, not just as boot-time setup.
fn dynamic_log_task() -> ! {
    serial_println!(
        "[log] dynamically spawned at runtime (proc_nr {}, uptime {} ticks)",
        proc::current_proc_nr(),
        proc::uptime_ticks()
    );
    ipc::receive(com::ANY); // nothing left to receive; parks log for good
    unreachable!("nothing sends to log in this demo");
}

/// Proves `sys_vircopy` (`crate::calls`) genuinely translates through a
/// *different* process's page table rather than the currently-active one:
/// reads the ring-3 demo task's code page back out of its own, private
/// address space (built in `kernel_main`) into a local kernel buffer, and
/// checks the bytes match what `usermode::create_address_space` wrote
/// there. `CLOCK` runs entirely in the kernel's own address space, so this
/// is a genuine cross-address-space copy, not a same-space one in
/// disguise.
fn vircopy_demo() {
    let mut buf = [0u8; usermode::USER_CODE.len()];
    calls::sys_vircopy(
        com::DRVR_PROC_NR,
        x86_64::VirtAddr::new(usermode::USER_CODE_ADDR),
        com::CLOCK,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed");
    serial_println!(
        "[clock] sys_vircopy read back {:?} from the ring-3 task's own address space (expected {:?})",
        buf,
        usermode::USER_CODE
    );
    assert_eq!(buf, usermode::USER_CODE, "sys_vircopy returned the wrong bytes");
}

/// Proves `elf::HELLO_ELF`'s loaded code genuinely ran -- not just that
/// it made the syscalls `crate::syscall::dispatch` logged, but that its
/// own `incl counter(%rip)` instruction, executing out of pages this
/// port's own ELF loader mapped (not the kernel poking bytes in
/// directly, like `usermode`'s demo), actually wrote through to physical
/// memory. Reads `tty`'s copy of its own `.data` counter back into a
/// local buffer via `sys_vircopy` and checks it against the five
/// `SYS_GET_UPTIME` calls `user/hello.s` makes before moving on to
/// `SYS_WRITE_LINE`/`SYS_BLOCK_FOREVER`.
///
/// Runs from `idle_task` (see its doc comment for why that's a genuinely
/// safe checkpoint, not a race) rather than `clock_task`: an earlier
/// version ran this right after `CLOCK`'s own fixed 3-tick alarm fired,
/// racing that independent timer against `tty`'s actual progress through
/// the ready queue -- "true in practice" until a run of real boots under
/// heavier host load reproducibly proved it wasn't (an observed panic:
/// `counter` read back `0`, not `5`), the same class of "two independent
/// timers, assumed-safe interleaving" bug the scheduler's own
/// `reschedule`/`start` race (see "A real bug found and fixed by a
/// multi-agent review" above) already showed up once in this port.
fn elf_counter_demo() {
    let mut buf = [0u8; 4];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::COUNTER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading the ELF task's counter");
    let counter = i32::from_le_bytes(buf);
    serial_println!(
        "[idle] sys_vircopy read back the ELF-loaded task's own .data counter: {} (expected 5)",
        counter
    );
    assert_eq!(
        counter, 5,
        "the loaded ELF binary's own code should have incremented its counter 5 times"
    );
}

/// Temporary stand-in for the `pm` server (see `spawn_tasks`): proves
/// blocking `send`/`receive` by pinging `fs` a few times and printing each
/// reply, then demonstrates `sys_fork` (see `fork_demo` below) -- fitting,
/// since real `pm` is exactly who would call it.
fn demo_pm_task() -> ! {
    for i in 0..3 {
        serial_println!("[pm] sending ping {} to fs", i);
        ipc::send(com::FS_PROC_NR, ipc::Message { source: com::PM_PROC_NR, m_type: 100, args: [i, 0, 0, 0] });
        let reply = ipc::receive(com::FS_PROC_NR);
        serial_println!("[pm] got reply from fs: {:?}", reply);
    }

    seed_bin();
    missing_program_check();
    fork_demo();
    fs_rw_demo();

    serial_println!("[pm] demo finished, blocking for good");
    ipc::receive(com::ANY); // nothing left to receive; parks pm so idle can run
    unreachable!("nothing sends to pm once the demo is done");
}

/// Install the two runnable programs into `fs` -- the "the binaries are
/// on disk" step a real package manager (or an install CD) would
/// otherwise have done. `fs` is in-memory and starts empty every boot, so
/// nothing can be loaded *by path* until a task has put it there:
/// `/bin/hello` is what `crate::rs`'s `SERVICES` table launches on
/// demand (`elf::spawn_from_fs`), and `/bin/echo` is what `shell` execs
/// itself into (`crate::calls::sys_exec`, via `user/shell.s`). Must run
/// from a task, not `kernel_main` directly: `fs::open`/`fs::write` block
/// on a real IPC round trip, which needs a task context to block in.
///
/// Runs as early in `pm`'s demo as it can rather than at the end, since
/// `shell` is waiting on `/bin/echo` to appear before it can get on with
/// its own job. "As early as it can" is immediately after the
/// `pm`/`fs` ping-pong: `fs` only becomes a real file server once those
/// three messages are done with (`demo_fs_task`), and a `mkdir` sent
/// before that gets answered by the ping-pong stand-in instead, which
/// replies to anything at all with `ping + 100`. It doesn't have to be
/// early for correctness -- `user/shell.s` retries rather than assuming
/// an ordering, deliberately (see its header comment) -- but there's no
/// reason to make it sit through pointless retries either.
fn seed_bin() {
    let rc = fs::mkdir("/bin");
    assert_eq!(rc, 0, "fs::mkdir(\"/bin\") failed: {}", rc);
    for (path, image) in [("/bin/hello", elf::HELLO_ELF), ("/bin/echo", elf::ECHO_ELF)] {
        let fd = fs::open(path);
        assert!(fd >= 0, "fs::open({:?}) failed: {}", path, fd);
        let n = fs::write(fd, image);
        serial_println!("[pm] seeded {} with {} bytes", path, n);
        assert_eq!(n, image.len() as i64, "fs::write didn't accept the whole ELF image");
    }
}

/// Proves that looking for a program that isn't there reports `ENOENT`
/// and *changes nothing* -- the second half of "a failed exec is a
/// no-op", and the half that isn't about the caller's own image.
///
/// Worth checking explicitly because `fs::open` creates the file it's
/// asked for (`crate::fs`'s `open_path`), so the obvious implementation
/// of `elf::read_file` had exec of a missing path succeed at opening
/// nothing, read zero bytes, report the *wrong* error (`ERR_BAD_ELF`,
/// as if the program were malformed rather than absent), and leave a
/// stray empty file behind at whatever path ring 3 named -- which a
/// ring-3 caller could repeat to grow `fs` without bound.
fn missing_program_check() {
    const MISSING: &str = "/no_such_program";
    assert_eq!(fs::open_existing(MISSING), fs::ENOENT, "{} exists before the test?", MISSING);
    let result = elf::read_file(MISSING);
    assert!(
        matches!(result, Err(fs::ENOENT)),
        "reading a program that doesn't exist should be ENOENT, got {:?}",
        result.map(|image| image.len())
    );
    assert_eq!(
        fs::open_existing(MISSING),
        fs::ENOENT,
        "looking for a missing program created it -- a failed exec has to change nothing"
    );
    serial_println!("[pm] reading {:?} -> ENOENT, and no empty file left behind", MISSING);
}

/// Proves `fs` (see `spawn_tasks`, now `fs::InMemoryFs::serve` instead of
/// a ping-pong stand-in) genuinely serves open/write/read requests over
/// IPC, backed by real in-memory storage rather than just echoing back
/// whatever it was sent: opens a file, writes to it, reopens it fresh
/// (a distinct file descriptor, its own cursor starting at 0) and reads
/// the bytes back, checking they round-trip -- then reads once more past
/// end of file and checks that comes back empty rather than repeating
/// data or blocking forever. Then exercises the real directory hierarchy
/// (`fs_directory_demo`): a path can't be opened until its parent
/// directory actually exists.
fn fs_rw_demo() {
    let written = b"hello from pm, stored in fs";

    let write_fd = fs::open("/hello.txt");
    assert!(write_fd >= 0, "fs::open failed: {}", write_fd);
    let n = fs::write(write_fd, written);
    serial_println!("[pm] wrote {} bytes to /hello.txt via fs (fd {})", n, write_fd);
    assert_eq!(n, written.len() as i64, "fs::write didn't accept the whole buffer");

    let read_fd = fs::open("/hello.txt");
    assert!(read_fd >= 0, "fs::open (reopen) failed: {}", read_fd);
    assert_ne!(read_fd, write_fd, "reopening the same file should hand back a fresh descriptor");
    let mut buf = [0u8; 64];
    let n = fs::read(read_fd, &mut buf);
    assert_eq!(n, written.len() as i64, "fs::read returned the wrong length");
    let round_tripped = &buf[..n as usize];
    serial_println!(
        "[pm] read back {:?} from fs (expected {:?})",
        core::str::from_utf8(round_tripped).unwrap_or("<invalid utf8>"),
        core::str::from_utf8(written).unwrap_or("<invalid utf8>")
    );
    assert_eq!(round_tripped, written, "fs did not return the bytes that were written");

    let n = fs::read(read_fd, &mut buf);
    serial_println!("[pm] read past end of file returned {} bytes (expected 0)", n);
    assert_eq!(n, 0, "reading past end of file should return 0, not repeat data or error");

    fs_directory_demo();
}

/// Proves `fs`'s directory hierarchy (`fs::InMemoryFs::mkdir`/`open_path`)
/// enforces the usual POSIX parent-directory rules, not just a flat,
/// exact-match namespace: a path under a directory that doesn't exist yet
/// is rejected (`ENOENT`), creating that directory makes the same open
/// succeed, creating it again fails (`EEXIST`), and a path that treats an
/// ordinary file as if it were a directory is rejected (`ENOTDIR`).
fn fs_directory_demo() {
    let missing = fs::open("/logs/today.txt");
    serial_println!("[pm] fs::open(\"/logs/today.txt\") before mkdir -> {} (expected ENOENT)", missing);
    assert_eq!(missing, fs::ENOENT, "opening a path under a nonexistent directory should fail with ENOENT");

    let made = fs::mkdir("/logs");
    serial_println!("[pm] fs::mkdir(\"/logs\") -> {} (expected 0)", made);
    assert_eq!(made, 0, "mkdir on a fresh path under an existing parent (\"/\") should succeed");

    let again = fs::mkdir("/logs");
    serial_println!("[pm] fs::mkdir(\"/logs\") again -> {} (expected EEXIST)", again);
    assert_eq!(again, fs::EEXIST, "mkdir on an already-existing path should fail with EEXIST");

    let as_dir = fs::open("/logs");
    serial_println!("[pm] fs::open(\"/logs\") (a directory) -> {} (expected EISDIR)", as_dir);
    assert_eq!(as_dir, fs::EISDIR, "opening a directory as if it were a file should fail with EISDIR");

    let fd = fs::open("/logs/today.txt");
    serial_println!("[pm] fs::open(\"/logs/today.txt\") after mkdir -> fd {} (expected >= 0)", fd);
    assert!(fd >= 0, "opening a path under a directory that now exists should succeed");

    let under_file = fs::open("/hello.txt/nested.txt");
    serial_println!(
        "[pm] fs::open(\"/hello.txt/nested.txt\") (parent is a file, not a directory) -> {} (expected ENOTDIR)",
        under_file
    );
    assert_eq!(
        under_file,
        fs::ENOTDIR,
        "opening a path under an existing *file* should fail with ENOTDIR, not silently succeed"
    );
}

/// Proves `sys_fork` (`crate::calls`) gives the child a genuinely
/// independent *copy* of the forked page, not just another alias of the
/// same physical memory: forks the ring-3 demo task's code page into a
/// new child (`init`), overwrites the *child's* copy with a canary value,
/// then reads back both copies. If fork only cloned the top-level page
/// table (`memory::new_address_space` alone) rather than deep-copying the
/// page (`memory::fork_address_space`), the original task's page would
/// show the canary too, since both sides would still be the same
/// physical frame.
fn fork_demo() {
    let code_addr = x86_64::VirtAddr::new(usermode::USER_CODE_ADDR);

    serial_println!("[pm] forking the ring-3 task's address space into a new child (init)");
    calls::sys_fork(
        com::DRVR_PROC_NR,
        &[code_addr],
        com::INIT_PROC_NR,
        "init (forked child)",
        forked_child_task,
        5,
        24,
        true,
    );

    let canary: [u8; 4] = [0xAA, 0xBB, 0xCC, 0xDD];
    calls::sys_vircopy(
        com::PM_PROC_NR,
        x86_64::VirtAddr::new(canary.as_ptr() as u64),
        com::INIT_PROC_NR,
        code_addr,
        canary.len(),
    )
    .expect("writing the canary into the forked child's page failed");

    let mut child_copy = [0u8; 4];
    calls::sys_vircopy(
        com::INIT_PROC_NR,
        code_addr,
        com::PM_PROC_NR,
        x86_64::VirtAddr::new(child_copy.as_mut_ptr() as u64),
        4,
    )
    .expect("reading back the child's page failed");

    let mut original_copy = [0u8; 4];
    calls::sys_vircopy(
        com::DRVR_PROC_NR,
        code_addr,
        com::PM_PROC_NR,
        x86_64::VirtAddr::new(original_copy.as_mut_ptr() as u64),
        4,
    )
    .expect("reading back the original task's page failed");

    serial_println!(
        "[pm] after writing a canary into the forked child's copy: child={:?}, original={:?} (unchanged: {:?})",
        child_copy,
        original_copy,
        usermode::USER_CODE
    );
    assert_eq!(child_copy, canary, "the forked child's page didn't receive the write");
    assert_eq!(
        original_copy,
        usermode::USER_CODE[..4],
        "fork did not give the child an independent copy -- the write leaked into the original task's page!"
    );
}

/// The child `sys_fork` creates in `fork_demo`. Its code page was
/// overwritten with a canary value for the divergence test, so it
/// deliberately doesn't try to jump to ring 3 and execute it (it's no
/// longer valid `int 0x80` machine code) -- it exists only to prove the
/// process-table/scheduler side of `sys_fork` works, alongside the memory
/// side `fork_demo` checks directly.
fn forked_child_task() -> ! {
    serial_println!("[init] forked child task running (proc_nr {})", proc::current_proc_nr());
    ipc::receive(com::ANY); // nothing left to receive; parks init for good
    unreachable!("nothing sends to init in this demo");
}

/// `fs` (see `spawn_tasks`): still starts with the same ping/pong
/// `demo_pm_task` opens with (proving plain blocking `send`/`receive`
/// still works), then becomes a real file server -- an
/// `fs::InMemoryFs` that genuinely stores and serves open/read/write
/// requests over IPC (`fs_rw_demo`), rather than a stand-in that only
/// ever echoes a fixed reply.
fn demo_fs_task() -> ! {
    for _ in 0..3 {
        let ping = ipc::receive(com::PM_PROC_NR);
        serial_println!("[fs] received ping from pm: {:?}", ping);
        ipc::send(
            com::PM_PROC_NR,
            ipc::Message { source: com::FS_PROC_NR, m_type: 200, args: [ping.args[0] + 100, 0, 0, 0] },
        );
    }
    serial_println!("[fs] ping/pong done, now serving real open/read/write requests");
    fs::InMemoryFs::new().serve()
}

/// Proof of asynchronous preemption: a tight, CPU-bound loop with no
/// `yield_now()` or IPC call anywhere in it, spun forever until its own
/// timeout. The only way this ever gets interrupted at all is the timer
/// forcibly reordering the ready queue and switching away once its
/// quantum is used up (`proc::clock_tick` + `proc::reschedule`, called
/// from `crate::interrupts::timer_interrupt_handler`) -- with the
/// previous milestone's purely-cooperative scheduler, this would simply
/// never yield the CPU to anything else. (An earlier version of this
/// demo paired two of these -- `rs` and `memory` -- sharing a priority
/// queue, so their forced trade-off was visible in the log; `rs` has
/// since become a real task (`rs::task`), so `memory` now demonstrates
/// this alone -- its counter still resumes from exactly where it left
/// off after every other runnable task, including `rs`'s restart
/// activity, gets its turn.)
fn busy_task() -> ! {
    busy_loop("memory")
}

fn busy_loop(tag: &str) -> ! {
    let start = proc::uptime_ticks();
    let mut n: u64 = 0;
    loop {
        n = n.wrapping_add(1);
        if n % 4_000_000 == 0 {
            let now = proc::uptime_ticks();
            serial_println!("[{}] still spinning, n={} uptime={}", tag, n, now);
            if now >= start + 30 {
                break;
            }
        }
    }
    serial_println!("[{}] demo finished, blocking for good", tag);
    ipc::receive(com::ANY); // nothing left to receive; parks this task so idle can run
    unreachable!("nothing sends to {} once the demo is done", tag);
}

pub fn halt_loop() -> ! {
    loop {
        x86_64::instructions::hlt();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    serial_println!("KERNEL PANIC: {}", info);
    halt_loop()
}

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
    let elf_address_space = elf::load(elf::HELLO_ELF, phys_mem_offset);
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
    spawn_tasks(ring3_address_space, elf_address_space);
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
fn spawn_tasks(ring3_address_space: PhysFrame, elf_address_space: PhysFrame) {
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
    rs::spawn_flaky();
}

/// Real MINIX's idle task just halts, waking on the next interrupt; ported
/// as-is. It's still preemptible (matching `IDL_F`), though with nothing
/// else runnable that mostly just means its own ticks get charged to it.
///
/// `IDLE` only ever becomes current once *everything* else has blocked
/// (it's the lowest-priority queue, `proc::IDLE_Q`) -- `memory`'s
/// busy-loop alone keeps that from happening until around tick 30 (see
/// `busy_task`), comfortably after `tty`'s own alarm-triggered
/// `SYS_FS_OPEN`/`SYS_FS_WRITE` (`user/hello.s`) has had time to run --
/// so this is a safe, "everything that's going to happen already has"
/// checkpoint for `fs_from_ring3_verify` to run at, the same way
/// `clock_task`'s own checks rely on `tty`'s *counter loop* (not its full
/// sequence) having already finished by the time *it* runs.
fn idle_task() -> ! {
    fs_from_ring3_verify();
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
/// order, without needing to coordinate with that task directly. Then
/// demonstrates the other real-servers prerequisite this milestone adds:
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
    elf_counter_demo();

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
/// Relies on `tty` (proc 5) having already run to completion (blocked in
/// `SYS_BLOCK_FOREVER`) by the time this runs -- true in practice, since
/// `tty` and `driver` share the same priority queue and `tty` is
/// enqueued right after `driver` blocks, well before `CLOCK`'s alarm (3
/// ticks) fires -- same kind of scheduling-order dependency
/// `vircopy_demo` above already has on `driver`.
fn elf_counter_demo() {
    let mut buf = [0u8; 4];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::COUNTER_ADDR),
        com::CLOCK,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading the ELF task's counter");
    let counter = i32::from_le_bytes(buf);
    serial_println!(
        "[clock] sys_vircopy read back the ELF-loaded task's own .data counter: {} (expected 5)",
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

    fork_demo();
    fs_rw_demo();

    serial_println!("[pm] demo finished, blocking for good");
    ipc::receive(com::ANY); // nothing left to receive; parks pm so idle can run
    unreachable!("nothing sends to pm once the demo is done");
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

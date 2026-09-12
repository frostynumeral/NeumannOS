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
mod gdt;
mod interrupts;
mod ipc;
mod memory;
mod pic;
mod pit;
mod proc;
mod serial;
mod table;
mod usermode;

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
    let mut mapper = unsafe { memory::init(phys_mem_offset) };
    let mut frame_allocator = unsafe { memory::BootInfoFrameAllocator::init(&boot_info.memory_map) };
    allocator::init_heap(&mut mapper, &mut frame_allocator).expect("heap initialization failed");
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
    // code/stack pages into it now, while `frame_allocator` is handy; the
    // task itself (spawned below) does the actual jump to ring 3 once the
    // scheduler runs it.
    let ring3_address_space = usermode::create_address_space(phys_mem_offset, &mut frame_allocator);
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
    spawn_tasks(ring3_address_space);
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
/// as in the boot image; `pm`/`fs`/`rs`/`memory`/`driver` don't exist as
/// real servers yet (see `rust/README.md`), so their process table slots
/// run small stand-in bodies instead: `pm`/`fs` exercise blocking
/// `send`/`receive`, `rs`/`memory` (as `busy_task_a`/`busy_task_b`)
/// exercise asynchronous preemption, and `driver` (as
/// `usermode::ring3_task_entry`) runs in its own address space
/// (`ring3_address_space`, built in `kernel_main`) -- the only task here
/// that isn't sharing the kernel's. Priorities, quantum sizes, and
/// preemptibility match `kernel/table.c`'s image entries
/// (`IDL_F`/`TSK_F`/`SRV_F` flags).
fn spawn_tasks(ring3_address_space: PhysFrame) {
    proc::spawn(com::IDLE, "IDLE", idle_task, proc::IDLE_Q, 8, true, None);
    proc::spawn(com::CLOCK, "CLOCK", clock_task, proc::TASK_Q, 64, false, None);
    proc::spawn(com::PM_PROC_NR, "pm (demo)", demo_pm_task, 3, 32, true, None);
    proc::spawn(com::FS_PROC_NR, "fs (demo)", demo_fs_task, 4, 32, true, None);
    proc::spawn(com::RS_PROC_NR, "rs (demo)", busy_task_a, 6, 16, true, None);
    proc::spawn(com::MEM_PROC_NR, "memory (demo)", busy_task_b, 6, 16, true, None);
    proc::spawn(
        com::DRVR_PROC_NR,
        "driver (ring3 demo)",
        usermode::ring3_task_entry,
        6,
        16,
        true,
        Some(ring3_address_space),
    );
}

/// Real MINIX's idle task just halts, waking on the next interrupt; ported
/// as-is. It's still preemptible (matching `IDL_F`), though with nothing
/// else runnable that mostly just means its own ticks get charged to it.
fn idle_task() -> ! {
    serial_println!("[idle] no other task is ready, halting (uptime: {} ticks)", proc::uptime_ticks());
    halt_loop()
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

/// Temporary stand-in for the `pm` server (see `spawn_tasks`): proves
/// blocking `send`/`receive` by pinging `fs` a few times and printing each
/// reply.
fn demo_pm_task() -> ! {
    for i in 0..3 {
        serial_println!("[pm] sending ping {} to fs", i);
        ipc::send(com::FS_PROC_NR, ipc::Message { source: com::PM_PROC_NR, m_type: 100, args: [i, 0, 0, 0] });
        let reply = ipc::receive(com::FS_PROC_NR);
        serial_println!("[pm] got reply from fs: {:?}", reply);
    }
    serial_println!("[pm] demo finished, blocking for good");
    ipc::receive(com::ANY); // nothing left to receive; parks pm so idle can run
    unreachable!("nothing sends to pm once the demo is done");
}

/// Temporary stand-in for the `fs` server (see `spawn_tasks`): the other
/// half of the `demo_pm_task` ping/pong.
fn demo_fs_task() -> ! {
    for _ in 0..3 {
        let ping = ipc::receive(com::PM_PROC_NR);
        serial_println!("[fs] received ping from pm: {:?}", ping);
        ipc::send(
            com::PM_PROC_NR,
            ipc::Message { source: com::FS_PROC_NR, m_type: 200, args: [ping.args[0] + 100, 0, 0, 0] },
        );
    }
    serial_println!("[fs] demo finished, blocking for good");
    ipc::receive(com::ANY); // nothing left to receive; parks fs so idle can run
    unreachable!("nothing sends to fs once the demo is done");
}

/// Both proofs of asynchronous preemption: a tight, CPU-bound loop with no
/// `yield_now()` or IPC call anywhere in it. `busy_task_a` and
/// `busy_task_b` share a priority queue (see `spawn_tasks`), so the only
/// way both ever get to run is the timer interrupt forcibly reordering the
/// ready queue and switching away once each one's quantum is used up
/// (`proc::clock_tick` + `proc::reschedule`, called from
/// `crate::interrupts::timer_interrupt_handler`) -- with the previous
/// milestone's purely-cooperative scheduler, whichever of these ran first
/// would simply never yield the CPU to the other.
fn busy_task_a() -> ! {
    busy_loop("rs")
}

fn busy_task_b() -> ! {
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

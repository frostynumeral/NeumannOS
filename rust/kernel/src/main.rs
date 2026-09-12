//! NeumannOS kernel entry point.
//!
//! This is the Rust port's `kernel/main.c` equivalent: it boots, sets up
//! the GDT/IDT so CPU faults are reported instead of triple-faulting
//! (`crate::gdt`, `crate::interrupts`), prints the boot image (the process
//! table MINIX would load into memory at this point), spawns the kernel
//! tasks (`crate::proc`), and hands off to the scheduler -- just as
//! `kernel/main.c` ends by calling `restart()`. There is no user-mode, no
//! MMU-based address-space isolation, and no interrupt-driven preemption
//! yet — see `rust/README.md` for what's implemented versus planned.
#![no_std]
#![no_main]
#![feature(abi_x86_interrupt)]

mod com;
mod gdt;
mod interrupts;
mod ipc;
mod proc;
mod serial;
mod table;

use bootloader::{entry_point, BootInfo};
use core::panic::PanicInfo;

entry_point!(kernel_main);

fn kernel_main(_boot_info: &'static BootInfo) -> ! {
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
    serial_println!();

    serial_println!("boot image:");
    for entry in table::BOOT_IMAGE.iter() {
        serial_println!("  proc_nr={:>3}  name={}", entry.proc_nr, entry.name);
    }
    serial_println!();

    spawn_tasks();
    serial_println!("tasks spawned, handing off to the scheduler");
    serial_println!();
    proc::start()
}

/// Spawn the kernel tasks. `IDLE` and `CLOCK` are real kernel tasks, same
/// as in the boot image; `pm`/`fs` don't exist as real servers yet (there's
/// no user mode to run them in -- see `rust/README.md`), so their process
/// table slots run a small stand-in body instead, purely to exercise
/// blocking `send`/`receive` and the scheduler end to end before the real
/// servers exist. Priorities and quantum sizes match `kernel/table.c`.
fn spawn_tasks() {
    proc::spawn(com::IDLE, "IDLE", idle_task, proc::IDLE_Q, 8);
    proc::spawn(com::CLOCK, "CLOCK", clock_task, proc::TASK_Q, 64);
    proc::spawn(com::PM_PROC_NR, "pm (demo)", demo_pm_task, 3, 32);
    proc::spawn(com::FS_PROC_NR, "fs (demo)", demo_fs_task, 4, 32);
}

/// Real MINIX's idle task just halts, waking on the next interrupt; ported
/// as-is, since even without a timer interrupt yet, halting is the correct
/// thing to do once nothing else is runnable.
fn idle_task() -> ! {
    serial_println!("[idle] no other task is ready, halting");
    halt_loop()
}

/// Stand-in for `kernel/clock.c`'s clock task: real MINIX only actually
/// runs this when a PIT interrupt fires, and is genuinely idle in between
/// (not implemented yet -- see `rust/README.md`). This simulates a handful
/// of ticks via cooperative yielding, then blocks in `receive` for good --
/// deliberately, not just for realism: `CLOCK` has the highest priority of
/// any task here (`TASK_Q`), so if it never blocked and instead looped
/// `yield_now()` forever, `pick_proc` would keep re-picking the only
/// occupant of the highest-priority queue and starve every other task
/// permanently.
fn clock_task() -> ! {
    for tick in 1..=3 {
        serial_println!("[clock] simulated tick {}", tick);
        proc::yield_now();
    }
    ipc::receive(com::ANY); // never actually sent to in this demo; parks CLOCK for good
    unreachable!("nothing sends to CLOCK in this demo");
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

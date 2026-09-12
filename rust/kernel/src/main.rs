//! NeumannOS kernel entry point.
//!
//! This is the Rust port's `kernel/main.c` + `kernel/table.c` equivalent:
//! it boots, sets up the GDT/IDT so CPU faults are reported instead of
//! triple-faulting (`crate::gdt`, `crate::interrupts`), prints the boot
//! image (the process table MINIX would load into memory at this point),
//! and exercises the IPC primitives with a self-test message exchange.
//! There is no scheduler, no user-mode processes, and no MMU-based
//! address-space isolation yet — see `rust/README.md` for what's
//! implemented versus planned.
#![no_std]
#![no_main]
#![feature(abi_x86_interrupt)]

mod com;
mod gdt;
mod interrupts;
mod ipc;
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

    serial_println!("IPC self-test: SEND to pm, RECEIVE as pm, NOTIFY from clock");
    let msg = ipc::Message { source: com::RS_PROC_NR, m_type: 1, args: [42, 0, 0, 0] };
    ipc::send(com::PM_PROC_NR, msg).expect("send to pm failed");
    let received = ipc::receive(com::PM_PROC_NR).expect("receive by pm failed");
    serial_println!("  pm received: {:?}", received);

    ipc::notify(com::TTY_PROC_NR, com::HARD_INT).expect("notify tty failed");
    let notif = ipc::receive(com::TTY_PROC_NR).expect("receive by tty failed");
    serial_println!("  tty received notification: {:?}", notif);

    serial_println!();
    serial_println!("(no scheduler yet: halting here)");
    halt_loop()
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

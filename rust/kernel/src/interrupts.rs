//! CPU exception handling, plus the one hardware interrupt this port
//! currently handles (the timer).
//!
//! The exception handlers are a Rust equivalent of `kernel/exception.c`.
//! The original `exception()` function is a single dispatcher indexed by
//! vector number: for a fault in a user process it converts the fault into
//! a POSIX signal (`SIGFPE`, `SIGSEGV`, ...) delivered to that process; for
//! a fault in a kernel task it panics. This port has no user processes or
//! signals yet (see `rust/README.md`), so every handler here takes the
//! "kernel task" branch: report the fault and halt. What's new compared to
//! the C version is the double-fault handler, which needs its own
//! dedicated stack (set up in `crate::gdt`) purely so that a fault *while
//! already faulting* — e.g. a kernel stack overflow — is reported instead
//! of silently triple-faulting the CPU and resetting the machine.
//!
//! `timer_interrupt_handler` is the IRQ0 handler `crate::pic`/`crate::pit`
//! set up, standing in for the `hwint00`/`clock_handler` pair in
//! `kernel/mpx386.s`/`kernel/clock.c`.

use crate::gdt;
use crate::pic;
use lazy_static::lazy_static;
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode};

lazy_static! {
    static ref IDT: InterruptDescriptorTable = {
        let mut idt = InterruptDescriptorTable::new();
        idt.divide_error.set_handler_fn(divide_error_handler);
        idt.breakpoint.set_handler_fn(breakpoint_handler);
        idt.invalid_opcode.set_handler_fn(invalid_opcode_handler);
        idt.general_protection_fault.set_handler_fn(general_protection_fault_handler);
        idt.page_fault.set_handler_fn(page_fault_handler);
        unsafe {
            idt.double_fault
                .set_handler_fn(double_fault_handler)
                .set_stack_index(gdt::DOUBLE_FAULT_IST_INDEX);
        }
        idt[pic::IRQ0_VECTOR].set_handler_fn(timer_interrupt_handler);
        idt
    };
}

pub fn init_idt() {
    IDT.load();
}

extern "x86-interrupt" fn divide_error_handler(frame: InterruptStackFrame) {
    crate::serial_println!("EXCEPTION: DIVIDE ERROR\n{:#?}", frame);
}

extern "x86-interrupt" fn breakpoint_handler(frame: InterruptStackFrame) {
    crate::serial_println!("EXCEPTION: BREAKPOINT\n{:#?}", frame);
}

extern "x86-interrupt" fn invalid_opcode_handler(frame: InterruptStackFrame) {
    crate::serial_println!("EXCEPTION: INVALID OPCODE\n{:#?}", frame);
    crate::halt_loop();
}

extern "x86-interrupt" fn general_protection_fault_handler(
    frame: InterruptStackFrame,
    error_code: u64,
) {
    crate::serial_println!(
        "EXCEPTION: GENERAL PROTECTION FAULT (error code {:#x})\n{:#?}",
        error_code,
        frame
    );
    crate::halt_loop();
}

extern "x86-interrupt" fn page_fault_handler(
    frame: InterruptStackFrame,
    error_code: PageFaultErrorCode,
) {
    crate::serial_println!(
        "EXCEPTION: PAGE FAULT accessing {:?} ({:?})\n{:#?}",
        x86_64::registers::control::Cr2::read(),
        error_code,
        frame
    );
    crate::halt_loop();
}

extern "x86-interrupt" fn double_fault_handler(
    frame: InterruptStackFrame,
    error_code: u64,
) -> ! {
    // Unlike the other handlers, this one cannot return: the CPU only
    // raises #DF for faults it cannot safely resume from (a fault during
    // fault delivery), so continuing is not an option.
    panic!(
        "EXCEPTION: DOUBLE FAULT (error code {:#x})\n{:#?}",
        error_code, frame
    );
}

extern "x86-interrupt" fn timer_interrupt_handler(_frame: InterruptStackFrame) {
    crate::proc::clock_tick();
    pic::end_of_interrupt(0);
}

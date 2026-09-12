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
//!
//! `syscall_handler`, at `SYSCALL_VECTOR`, is a first, minimal analogue of
//! `kernel/table.c`'s `SYS_VECTOR` (the software-interrupt gate user-mode
//! processes trap into the kernel through) -- see `crate::usermode` for
//! the other half (getting *into* ring 3 in the first place). It doesn't
//! implement any real call yet; it exists to prove the round trip works.

use crate::gdt;
use crate::pic;
use lazy_static::lazy_static;
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode};
use x86_64::PrivilegeLevel;

/// Chosen to sit comfortably above the CPU-exception range (0-31) and the
/// PIC's remapped IRQ vectors (`pic::IRQ0_VECTOR..IRQ0_VECTOR+16`), with no
/// significance beyond that -- unlike real MINIX's `SYS_VECTOR`, this port
/// has no other software-trap vectors yet to stay clear of (see
/// `pic.rs`'s doc comment for the same caveat about its own vector choice).
pub const SYSCALL_VECTOR: u8 = 0x80;

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
        // DPL 3: unlike every other gate here, ring-3 code (crate::usermode)
        // must be allowed to reach this one with a plain `int` instruction.
        // Every other vector keeps the default DPL 0, so user-mode code
        // triggering, say, the timer vector directly would correctly fault
        // instead of being let in.
        idt[SYSCALL_VECTOR]
            .set_handler_fn(syscall_handler)
            .set_privilege_level(PrivilegeLevel::Ring3);
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

extern "x86-interrupt" fn syscall_handler(frame: InterruptStackFrame) {
    // Printing `frame.code_segment` here is the actual proof this all
    // works: it's only reachable via the CPU's own privilege-transition
    // machinery, so an RPL of 3 in it is the CPU itself confirming the
    // interrupted code was genuinely running in ring 3 -- not something
    // `crate::usermode` merely asserts.
    crate::serial_println!(
        "[syscall] reached from {:?} (CS index {})",
        frame.code_segment.rpl(),
        frame.code_segment.index()
    );
    // There's only one caller of this gate right now (crate::usermode's
    // one-shot demo), so unlike every other handler in this file, this one
    // never falls through to the compiler-generated `iretq` that would
    // resume ring 3: it abandons that side entirely and jumps back into
    // `usermode::demo`'s caller instead, as if the whole ring-3 excursion
    // had been an ordinary function call. See `usermode::resume_kernel`'s
    // doc comment. Once real user-mode processes exist and make repeated,
    // varied syscalls, this will need to become a real dispatch that
    // usually *does* just return to resume ring 3.
    unsafe { crate::usermode::resume_kernel() }
}

extern "x86-interrupt" fn timer_interrupt_handler(_frame: InterruptStackFrame) {
    crate::proc::clock_tick();
    // Send EOI before a possible switch, not after: if the switch parks
    // this exact call stack for a while (it may not resume again for a
    // long time, if ever), the PIC still needs to know this IRQ is done so
    // it can keep delivering the next ones in the meantime.
    pic::end_of_interrupt(0);
    // This is what makes preemption asynchronous rather than merely
    // cooperative: control can leave right here, mid-interrupt, and only
    // come back (possibly much later, on a completely different call
    // stack having run in between) once something switches back to
    // whichever task was running when this tick fired. See
    // `proc::switch_to`'s doc comment for what makes that sound.
    crate::proc::reschedule();
}

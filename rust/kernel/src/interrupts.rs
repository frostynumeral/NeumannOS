//! CPU exception handling, plus the hardware interrupts this port
//! currently handles (the timer, and the PS/2 keyboard).
//!
//! The exception handlers are a Rust equivalent of `kernel/exception.c`.
//! The original `exception()` function is a single dispatcher indexed by
//! vector number: for a fault in a user process it converts the fault into
//! a POSIX signal (`SIGFPE`, `SIGSEGV`, ...) delivered to that process; for
//! a fault in a kernel task it panics. This port has no signals yet (see
//! `rust/README.md`), so a fault in a *ring-3* task instead goes straight
//! to `crate::proc::kill` (`recover_or_halt`, below) -- the same
//! "stop scheduling it, tell `RS`" primitive real MINIX ultimately drives
//! through a signal and `PM`'s exit path, collapsed into one direct call.
//! A fault in a kernel task still takes the original "report and halt"
//! branch: kernel code is trusted, so a bug in it means the kernel's own
//! state might already be corrupted. What's new compared to the C
//! version is the double-fault handler, which needs its own dedicated
//! stack (set up in `crate::gdt`) purely so that a fault *while already
//! faulting* — e.g. a kernel stack overflow — is reported instead of
//! silently triple-faulting the CPU and resetting the machine.
//!
//! `timer_interrupt_handler` is the IRQ0 handler `crate::pic`/`crate::pit`
//! set up, standing in for the `hwint00`/`clock_handler` pair in
//! `kernel/mpx386.s`/`kernel/clock.c`. `keyboard_interrupt_handler` is the
//! IRQ1 handler, reading and translating scancodes via `crate::keyboard`.
//!
//! `SYSCALL_VECTOR`'s handler is `crate::syscall::entry`, a real
//! call-number/register dispatch gate (see that module) -- the software-
//! interrupt gate user-mode processes trap into the kernel through (see
//! `crate::usermode`/`crate::elf` for the other half, getting *into*
//! ring 3 in the first place).

use crate::gdt;
use crate::pic;
use lazy_static::lazy_static;
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode};
use x86_64::{PrivilegeLevel, VirtAddr};

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
        idt[pic::IRQ0_VECTOR + 1].set_handler_fn(keyboard_interrupt_handler);
        // DPL 3: unlike every other gate here, ring-3 code (crate::usermode)
        // must be allowed to reach this one with a plain `int` instruction.
        // Every other vector keeps the default DPL 0, so user-mode code
        // triggering, say, the timer vector directly would correctly fault
        // instead of being let in. `set_handler_addr`, not `set_handler_fn`:
        // `crate::syscall::entry` is a hand-written naked trap gate, not an
        // `extern "x86-interrupt" fn` -- see that module's doc comment for
        // why it needs to be.
        unsafe {
            idt[SYSCALL_VECTOR]
                .set_handler_addr(VirtAddr::new(crate::syscall::entry as *const () as u64))
                .set_privilege_level(PrivilegeLevel::Ring3);
        }
        idt
    };
}

pub fn init_idt() {
    IDT.load();
}

/// Shared tail for every CPU-exception handler below that can recover
/// from a *ring-3* fault: kill the faulting task and switch away
/// (`crate::proc::kill`/`reschedule`) -- this port's stand-in for real
/// MINIX turning the fault into a fatal signal for that one process
/// (`kernel/exception.c`) instead of taking down the whole machine. A
/// fault in *kernel*-trusted code (RPL 0 -- every kernel task, and every
/// stand-in server body that still shares the kernel's own address
/// space) still halts everything: kernel code is trusted, so a bug in it
/// means the kernel's own state might already be corrupted, and
/// continuing (even by "just" killing one task) isn't safe. After
/// `kill()`, `reschedule()` is guaranteed to actually switch away (the
/// just-killed task can never be `next_ptr` again), so this call site
/// never really "returns" the way the halting branch never does either --
/// the compiler just can't see that statically.
fn recover_or_halt(frame: &InterruptStackFrame, description: &str) {
    if frame.code_segment.rpl() == PrivilegeLevel::Ring3 {
        let proc_nr = crate::proc::current_proc_nr();
        crate::proc::kill(proc_nr, description);
        crate::proc::reschedule();
    } else {
        crate::halt_loop();
    }
}

extern "x86-interrupt" fn divide_error_handler(frame: InterruptStackFrame) {
    crate::serial_println!("EXCEPTION: DIVIDE ERROR\n{:#?}", frame);
    recover_or_halt(&frame, "divide error");
}

extern "x86-interrupt" fn breakpoint_handler(frame: InterruptStackFrame) {
    crate::serial_println!("EXCEPTION: BREAKPOINT\n{:#?}", frame);
}

extern "x86-interrupt" fn invalid_opcode_handler(frame: InterruptStackFrame) {
    crate::serial_println!("EXCEPTION: INVALID OPCODE\n{:#?}", frame);
    recover_or_halt(&frame, "invalid opcode");
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
    recover_or_halt(&frame, "general protection fault");
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
    recover_or_halt(&frame, "page fault");
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

/// IRQ1 (keyboard): read the scancode the controller just latched, print
/// its ASCII translation (if `crate::keyboard`'s table has one), and
/// acknowledge. No task is woken here yet -- there's no real `tty`/line-
/// discipline layer to hand this to (see `rust/README.md`'s roadmap) --
/// so this is currently just proof the IRQ genuinely fires per keypress,
/// asynchronously, the same way `timer_interrupt_handler` proves IRQ0
/// does.
extern "x86-interrupt" fn keyboard_interrupt_handler(_frame: InterruptStackFrame) {
    let scancode = crate::keyboard::read_scancode();
    if let Some(ascii) = crate::keyboard::translate(scancode) {
        crate::serial_println!("[kbd] key: {:?} (scancode {:#04x})", ascii as char, scancode);
        // Digits '1'-'4' pick one of the VGA demo panel's buttons and
        // redraw it highlighted -- the first real input-to-output loop
        // in this port (crate::keyboard -> crate::vga), not just proof
        // the hardware event fires.
        if ascii.is_ascii_digit() {
            if let Some(index) = (ascii - b'0').checked_sub(1) {
                crate::vga::select_button(index as usize);
            }
        }
    }
    pic::end_of_interrupt(1);
}

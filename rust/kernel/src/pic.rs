//! 8259 Programmable Interrupt Controller.
//!
//! Rust port of `kernel/i8259.c`'s `intr_init()`: reprograms the master and
//! slave PICs so hardware IRQs land on vectors we control instead of the
//! CPU-exception range (0-31), then masks every line except IRQ0 (the
//! timer, wired up in `crate::pit`/`crate::interrupts`) and IRQ1 (the
//! keyboard, `crate::keyboard`/`crate::interrupts`) -- we don't have
//! handlers for anything else yet.
//!
//! One deliberate deviation from the C: real MINIX remaps IRQ0-7 to
//! `0x50`/IRQ8-15 to `0x70` (`include/ibm/interrupt.h`), specifically to
//! stay clear of the extra software-trap vectors it reserves in `0x20`-
//! `0x4F` (e.g. the system-call gate). This port hasn't reserved any of
//! those yet -- there's no user mode to trap in from (see
//! `rust/README.md`) -- so the conventional `0x20`/`0x28` (32/40) remap
//! target is used instead; the reasoning behind MINIX's specific choice
//! doesn't apply yet, and nothing external depends on the exact vector
//! numbers the way it does on, say, process numbers or message layout.

use x86_64::instructions::port::Port;

const MASTER_CMD: u16 = 0x20;
const MASTER_DATA: u16 = 0x21;
const SLAVE_CMD: u16 = 0xA0;
const SLAVE_DATA: u16 = 0xA1;

pub const IRQ0_VECTOR: u8 = 32;
const IRQ8_VECTOR: u8 = IRQ0_VECTOR + 8;

const CASCADE_IRQ: u8 = 2;

const ICW1_INIT_ICW4: u8 = 0x11; // edge triggered, cascade, ICW4 needed
const ICW4_8086: u8 = 0x01; // 8086 mode, normal EOI

const EOI: u8 = 0x20;

/// Reprogram both PICs (ICW1-4, same four steps `intr_init` does), then
/// mask every IRQ line except IRQ0.
pub fn init() {
    let mut master_cmd: Port<u8> = Port::new(MASTER_CMD);
    let mut master_data: Port<u8> = Port::new(MASTER_DATA);
    let mut slave_cmd: Port<u8> = Port::new(SLAVE_CMD);
    let mut slave_data: Port<u8> = Port::new(SLAVE_DATA);

    unsafe {
        // ICW1: start initialization.
        master_cmd.write(ICW1_INIT_ICW4);
        slave_cmd.write(ICW1_INIT_ICW4);
        // ICW2: vector offsets.
        master_data.write(IRQ0_VECTOR);
        slave_data.write(IRQ8_VECTOR);
        // ICW3: master/slave wiring via the cascade line.
        master_data.write(1 << CASCADE_IRQ);
        slave_data.write(CASCADE_IRQ);
        // ICW4: 8086 mode.
        master_data.write(ICW4_8086);
        slave_data.write(ICW4_8086);

        // Mask everything except IRQ0 (timer) and IRQ1 (keyboard) on the
        // master, everything on the slave (we have no handlers for
        // IRQ2-15 yet).
        master_data.write(!0b0000_0011u8);
        slave_data.write(0xFF);
    }
}

/// Acknowledge an IRQ so the PIC will deliver further interrupts. `irq` is
/// 0-15, matching the numbering `crate::interrupts` uses for its handlers.
pub fn end_of_interrupt(irq: u8) {
    let mut master_cmd: Port<u8> = Port::new(MASTER_CMD);
    let mut slave_cmd: Port<u8> = Port::new(SLAVE_CMD);
    unsafe {
        if irq >= 8 {
            slave_cmd.write(EOI);
        }
        master_cmd.write(EOI);
    }
}

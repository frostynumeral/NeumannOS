//! 8253/8254 Programmable Interval Timer.
//!
//! Rust port of the timer-programming half of `kernel/clock.c`'s
//! `init_clock()` (the rest of that function -- registering and enabling
//! the IRQ handler -- is `crate::pic::init` plus the IDT entry in
//! `crate::interrupts`).

use x86_64::instructions::port::Port;

const CHANNEL0: u16 = 0x40;
const MODE_COMMAND: u16 = 0x43;

/// Channel 0, access low-then-high byte, mode 3 (square wave), binary.
const SQUARE_WAVE: u8 = 0x36;

const TIMER_FREQ: u32 = 1_193_182; // the 8253/8254's fixed input frequency

/// Ticks per second. Matches `HZ` in `include/minix/const.h`.
pub const HZ: u32 = 60;

/// Program channel 0 to fire `HZ` times a second. Real MINIX also flips a
/// PS/2-specific acknowledge bit in the clock interrupt handler
/// (`kernel/clock.c`'s `CLOCK_ACK_BIT`); not needed for the platforms QEMU
/// emulates here.
pub fn init() {
    let count = (TIMER_FREQ / HZ) as u16;
    let mut mode: Port<u8> = Port::new(MODE_COMMAND);
    let mut channel0: Port<u8> = Port::new(CHANNEL0);
    unsafe {
        mode.write(SQUARE_WAVE);
        channel0.write((count & 0xFF) as u8);
        channel0.write((count >> 8) as u8);
    }
}

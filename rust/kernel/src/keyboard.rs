//! PS/2 keyboard: read raw scancodes off port `0x60` and translate the
//! common "make" codes (key pressed, not released) to ASCII.
//!
//! No MINIX C equivalent in the kernel itself: 2005-era MINIX handles the
//! keyboard in a driver (`drivers/tty/keyboard.c`), not the kernel proper,
//! reached the normal device-driver-protocol way (`rust/README.md`'s
//! roadmap -- there's no real `tty` server yet for this port to hand
//! scancodes to). This is the minimal first slice: read a scancode,
//! translate it, prove it's a real, hardware-driven, asynchronous event
//! (like `crate::pit`'s timer) rather than a polled one -- the actual
//! `tty`/line-discipline layer is future work.

/// Standard IBM PC/AT "scancode set 1" (what real hardware -- and QEMU's
/// PS/2 emulation -- both still send by default), unshifted-key mapping
/// for the alphanumeric/punctuation block (scancodes `0x02`-`0x39`).
/// Index `0` is unused (there's no scancode `0x00`); everything without
/// an ASCII equivalent here (Ctrl, Alt, F-keys, arrows, ...) is `0` too.
/// Known simplification: no shift/caps-lock state is tracked, so this is
/// always the unshifted mapping (lowercase letters, unshifted
/// punctuation) regardless of which modifier keys are actually held.
const SCANCODE_TO_ASCII: [u8; 0x3A] = {
    let mut table = [0u8; 0x3A];
    table[0x02] = b'1';
    table[0x03] = b'2';
    table[0x04] = b'3';
    table[0x05] = b'4';
    table[0x06] = b'5';
    table[0x07] = b'6';
    table[0x08] = b'7';
    table[0x09] = b'8';
    table[0x0A] = b'9';
    table[0x0B] = b'0';
    table[0x0C] = b'-';
    table[0x0D] = b'=';
    table[0x10] = b'q';
    table[0x11] = b'w';
    table[0x12] = b'e';
    table[0x13] = b'r';
    table[0x14] = b't';
    table[0x15] = b'y';
    table[0x16] = b'u';
    table[0x17] = b'i';
    table[0x18] = b'o';
    table[0x19] = b'p';
    table[0x1A] = b'[';
    table[0x1B] = b']';
    table[0x1C] = b'\n';
    table[0x1E] = b'a';
    table[0x1F] = b's';
    table[0x20] = b'd';
    table[0x21] = b'f';
    table[0x22] = b'g';
    table[0x23] = b'h';
    table[0x24] = b'j';
    table[0x25] = b'k';
    table[0x26] = b'l';
    table[0x27] = b';';
    table[0x28] = b'\'';
    table[0x29] = b'`';
    table[0x2B] = b'\\';
    table[0x2C] = b'z';
    table[0x2D] = b'x';
    table[0x2E] = b'c';
    table[0x2F] = b'v';
    table[0x30] = b'b';
    table[0x31] = b'n';
    table[0x32] = b'm';
    table[0x33] = b',';
    table[0x34] = b'.';
    table[0x35] = b'/';
    table[0x39] = b' ';
    table
};

/// The PS/2 controller's data port: reading it both fetches the byte the
/// keyboard sent *and* tells the controller it's been consumed (so the
/// next IRQ1 can fire) -- there's no separate acknowledgment step.
const DATA_PORT: u16 = 0x60;

/// Read the scancode byte the keyboard just sent. Only valid to call from
/// the IRQ1 handler (`crate::interrupts::keyboard_interrupt_handler`):
/// that's the only time the controller is guaranteed to have a fresh byte
/// waiting.
pub fn read_scancode() -> u8 {
    let mut data: x86_64::instructions::port::Port<u8> = x86_64::instructions::port::Port::new(DATA_PORT);
    unsafe { data.read() }
}

/// Translate a scancode to ASCII, if this port's table has one for it.
/// Bit 7 set means "key released" (a "break" code, the make code with the
/// top bit set) -- real MINIX's keyboard driver uses release events too
/// (to track modifier key state); this port doesn't track any state yet,
/// so every break code simply has no translation.
pub fn translate(scancode: u8) -> Option<u8> {
    if scancode & 0x80 != 0 {
        return None; // key release ("break" code)
    }
    match SCANCODE_TO_ASCII.get(scancode as usize) {
        Some(&0) | None => None,
        Some(&ascii) => Some(ascii),
    }
}

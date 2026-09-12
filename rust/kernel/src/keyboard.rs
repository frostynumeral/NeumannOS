//! PS/2 keyboard: read raw scancodes off port `0x60`, translate the
//! common "make" codes (key pressed, not released) to ASCII, and buffer
//! a line at a time for `console_task` -- a real, scheduled task, not
//! just a direct function call (`crate::vga::select_button`) from the
//! interrupt handler.
//!
//! No MINIX C equivalent in the kernel itself: 2005-era MINIX handles the
//! keyboard in a driver (`drivers/tty/keyboard.c`), not the kernel proper,
//! reached the normal device-driver protocol (`rust/README.md`'s
//! roadmap -- there's still no real `tty` server for this port's
//! `console_task` to be a stand-in for, so it talks to `crate::fs`
//! directly instead of through one). `console_task` is nonetheless this
//! port's first real line discipline: `on_char` (called from
//! `crate::interrupts::keyboard_interrupt_handler`) accumulates
//! translated characters into `LINE` until a newline completes one, then
//! `crate::ipc::notify`s `console_task` -- exactly the same
//! interrupt-handler-notifies-a-real-task shape `crate::proc::clock_tick`
//! already uses for `SYN_ALARM`, just triggered by a keypress instead of
//! a timer tick.

use crate::{com, fs, ipc, serial_println};

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

/// Longest line `on_char`/`console_task` will buffer. A character typed
/// past this is silently dropped -- a real line discipline would likely
/// bell or refuse further input instead; this port doesn't have a way to
/// signal that back to the (nonexistent) terminal yet.
const LINE_CAPACITY: usize = 64;

struct LineBuffer {
    buf: [u8; LINE_CAPACITY],
    len: usize,
    /// Set by `on_char` on a newline, cleared by `take_line` once
    /// `console_task` has consumed it. `on_char` keeps accumulating into
    /// `buf`/`len` for the *next* line even before this one's been taken
    /// (mirroring a real line discipline's typeahead), rather than
    /// blocking further input until `console_task` catches up.
    ready: bool,
}

/// The one pending (or just-completed) line, shared between the keyboard
/// IRQ handler (producer) and `console_task` (consumer). A `spin::Mutex`
/// rather than anything fancier: the producer only ever holds it for a
/// few array writes, never across a block/switch, so there's no
/// deadlock risk against `console_task` (which never holds it across a
/// blocking call either).
static LINE: spin::Mutex<LineBuffer> =
    spin::Mutex::new(LineBuffer { buf: [0; LINE_CAPACITY], len: 0, ready: false });

/// Called from `crate::interrupts::keyboard_interrupt_handler` for every
/// translated character. A newline marks the buffered line ready and
/// wakes `console_task` (`crate::ipc::notify`, which -- like
/// `crate::proc::clock_tick`'s own `SYN_ALARM` delivery -- reschedules
/// immediately if that just woke a higher-priority task); anything else
/// is appended to the line in progress.
pub fn on_char(ascii: u8) {
    if ascii == b'\n' {
        LINE.lock().ready = true;
        crate::ipc::notify(crate::com::CONSOLE_PROC_NR, LINE_READY);
        return;
    }
    let mut line = LINE.lock();
    if line.len < LINE_CAPACITY {
        let len = line.len;
        line.buf[len] = ascii;
        line.len += 1;
    }
}

/// Notification type `on_char` sends `console_task`. Distinct from the
/// `NOTIFY_MESSAGE`-based ones (`com::notify_from`) purely so it can't be
/// confused with one of those; nothing currently sends both to the same
/// task, so this is a stylistic distinction more than a load-bearing one.
const LINE_READY: i32 = crate::com::NOTIFY_MESSAGE | 0x0100;

/// Take the completed line out of `LINE` (if one is ready) and reset it
/// for the next one. Returns the line's bytes and length -- a fixed-size
/// array rather than a slice, so `console_task` can hold it across the
/// `LINE` lock being released.
fn take_line() -> Option<([u8; LINE_CAPACITY], usize)> {
    let mut line = LINE.lock();
    if !line.ready {
        return None;
    }
    let result = (line.buf, line.len);
    line.len = 0;
    line.ready = false;
    Some(result)
}

/// Real message type (not a fire-and-forget notification like
/// `LINE_READY`: a genuine `send`/reply pair) a caller uses to ask
/// `console_task` for the next completed line -- see `read_line` and
/// `crate::syscall`'s `SYS_READ_LINE`, the only current caller.
const CONSOLE_READ_LINE: i32 = 400;

/// A caller blocked waiting for the next line, remembered by
/// `console_task` across however many `LINE_READY` notifications (and
/// whatever else console does in between) it takes for one to actually
/// arrive. `ptr`/`max_len` describe *where* to write the line's bytes,
/// not just *who* to reply to -- see `read_line`'s doc comment for why
/// writing straight into a pointer the caller gave us is safe here in a
/// way it wouldn't be for an arbitrary ring-3 pointer.
struct PendingReader {
    proc_nr: i32,
    ptr: u64,
    max_len: usize,
}

/// Copy up to `reader.max_len` bytes of `line` into `reader.ptr` and
/// reply to `reader.proc_nr` with how many bytes that was.
fn deliver_line(reader: &PendingReader, line: &[u8]) {
    let n = core::cmp::min(line.len(), reader.max_len);
    // Safety: `reader.ptr` was handed to us by `crate::syscall`'s
    // `SYS_READ_LINE`, which always points at a *kernel*-stack buffer
    // (part of the calling task's own per-task kernel stack, mapped
    // identically in every address space -- see `crate::proc`'s
    // `STACKS`) precisely so it stays dereferenceable here regardless of
    // which `CR3` happens to be active when `console_task` (which always
    // runs in the kernel's own address space) gets around to writing it,
    // possibly much later than when the request arrived.
    unsafe { core::ptr::copy_nonoverlapping(line.as_ptr(), reader.ptr as *mut u8, n) };
    ipc::send(
        reader.proc_nr,
        ipc::Message { source: com::CONSOLE_PROC_NR, m_type: CONSOLE_READ_LINE, args: [n as i64, 0, 0, 0] },
    );
}

/// Client-side stub for `crate::syscall`'s `SYS_READ_LINE`: ask
/// `console_task` for the next completed line, blocking until one is
/// typed (there's no "no line available" reply -- a caller that doesn't
/// want to block has nothing to call instead yet). `ptr` must point at
/// memory valid in *this* address space for at least `max_len` bytes --
/// `SYS_READ_LINE` only ever passes a kernel-stack buffer, never a
/// ring-3 pointer directly, for exactly the reason `deliver_line`'s doc
/// comment explains.
pub fn read_line(ptr: u64, max_len: usize) -> i64 {
    let reply = ipc::send_receive(
        com::CONSOLE_PROC_NR,
        ipc::Message {
            source: crate::proc::current_proc_nr(),
            m_type: CONSOLE_READ_LINE,
            args: [ptr as i64, max_len as i64, 0, 0],
        },
    );
    reply.args[0]
}

/// `console`'s task body: a real, if minimal, line-oriented server.
/// Every completed line (`on_char`'s `LINE_READY` notification) gets
/// logged and appended to `/console.log` via `crate::fs` unconditionally
/// (the original proof this task does real IPC, not just a direct
/// function call like `crate::vga::select_button`); if some other task
/// is also waiting for a line (`CONSOLE_READ_LINE`, from
/// `crate::syscall`'s `SYS_READ_LINE`), that same line is delivered to
/// them too, directly into the buffer their request pointed at
/// (`deliver_line`). A `CONSOLE_READ_LINE` request that arrives with no
/// line buffered yet is remembered (`pending_reader`) rather than
/// replied to immediately, and satisfied whenever the next line
/// completes -- proof a ring-3 task can genuinely block waiting for
/// keyboard input and resume once a human (or, in QEMU, a QMP
/// `send-key` sequence) actually types something.
///
/// Opens `/console.log` exactly once, before the loop, and keeps writing
/// through that same descriptor for every subsequent line: `fs::open`
/// always starts a fresh descriptor's cursor at `0` (see `crate::fs`'s
/// known simplifications -- there's no explicit "append" mode), so
/// re-opening on every line would overwrite from the start each time
/// instead of accumulating one line after another.
pub fn console_task() -> ! {
    let fd = fs::open("/console.log");
    if fd < 0 {
        serial_println!("[console] fs::open(\"/console.log\") failed: {}", fd);
    }
    let mut pending_reader: Option<PendingReader> = None;

    loop {
        let msg = ipc::receive(com::ANY);

        if msg.m_type == CONSOLE_READ_LINE {
            // Simplification: only one pending reader is tracked at a
            // time -- a second SYS_READ_LINE arriving before the first
            // is satisfied would overwrite (and so silently starve) it.
            // Fine for this port's one demo caller.
            pending_reader =
                Some(PendingReader { proc_nr: msg.source, ptr: msg.args[0] as u64, max_len: msg.args[1] as usize });
            continue;
        }

        let Some((buf, len)) = take_line() else { continue };
        let line = &buf[..len];
        serial_println!(
            "[console] received line: {:?}",
            core::str::from_utf8(line).unwrap_or("<invalid utf8>")
        );
        if fd >= 0 {
            fs::write(fd, line);
            fs::write(fd, b"\n");
        }
        if let Some(reader) = pending_reader.take() {
            deliver_line(&reader, line);
        }
    }
}

//! Serial port output (COM1), used for boot diagnostics.
//!
//! `bootimage runner` and the QEMU invocation in `rust/README.md` forward
//! COM1 to stdio, so `serial_println!` output shows up directly in the
//! terminal running QEMU without needing a display.

use lazy_static::lazy_static;
use spin::Mutex;
use uart_16550::SerialPort;

lazy_static! {
    static ref SERIAL1: Mutex<SerialPort> = {
        let mut port = unsafe { SerialPort::new(0x3F8) };
        port.init();
        Mutex::new(port)
    };
}

/// Force initialization of the serial port. Not strictly required (the
/// `lazy_static` runs on first use), but called explicitly from `main.rs`
/// so the boot sequence documents the dependency.
pub fn init() {
    lazy_static::initialize(&SERIAL1);
}

#[doc(hidden)]
pub fn _print(args: core::fmt::Arguments) {
    use core::fmt::Write;
    SERIAL1.lock().write_fmt(args).expect("serial write failed");
}

#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => {
        $crate::serial::_print(format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! serial_println {
    () => { $crate::serial_print!("\n") };
    ($fmt:expr) => { $crate::serial_print!(concat!($fmt, "\n")) };
    ($fmt:expr, $($arg:tt)*) => {
        $crate::serial_print!(concat!($fmt, "\n"), $($arg)*)
    };
}

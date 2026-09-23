//! `neumann_rt`: the runtime every NeumannOS ring-3 program written in
//! Rust links against -- this port's first "libc-equivalent" (roadmap
//! item 12, mirroring what `lib/` is to MINIX's C programs), kept as
//! small as it can be while still being something programs are written
//! *on* rather than around.
//!
//! What it provides:
//!
//! - `_start`, the real entry point: it hands the stack pointer the
//!   kernel's `exec` left (pointing at `argc`, then `argv[]`, `envp[]` --
//!   the System V start-up block `crate::elf::write_initial_stack` builds
//!   in the kernel) to the program's `main`, and turns `main`'s return
//!   value into a real `exit` status.
//! - `Args`, a view of that block: `argv`/`envp` as byte strings.
//! - `sys`, one safe wrapper per system call (`int 0x80`, call number in
//!   `rax`, arguments in `rdi`/`rsi`/`rdx`/`rcx`, result in `rax`; see the
//!   kernel's `crate::syscall` for the other side of every one).
//! - `print!`/`println!`, formatted output to the console through
//!   `SYS_CONSOLE_WRITE`.
//! - A panic handler that reports the panic and exits with status 101,
//!   Rust's own convention for "panicked".
//!
//! - A global allocator (`heap`) over `SYS_BRK`, so programs can use
//!   `alloc`'s `Vec`, `String`, `Box` and friends.

#![no_std]

extern crate alloc;

use core::fmt;

pub mod heap;
pub mod os;
pub mod sys;
pub mod thread;

/// The program's arguments and environment, exactly as `exec` laid them
/// out on the stack. Every string is borrowed straight from there, which
/// is why they are `'static`: nothing ever frees the start-up block.
#[derive(Clone, Copy)]
pub struct Args {
    argc: usize,
    argv: *const *const u8,
    envp: *const *const u8,
}

impl Args {
    /// Safety: `sp` must be the stack pointer `_start` was entered with.
    unsafe fn from_stack(sp: *const u64) -> Args {
        let argc = *sp as usize;
        let argv = sp.add(1) as *const *const u8;
        let envp = argv.add(argc + 1);
        Args { argc, argv, envp }
    }

    pub fn len(&self) -> usize {
        self.argc
    }

    pub fn is_empty(&self) -> bool {
        self.argc == 0
    }

    /// `argv[i]`, without its terminating NUL.
    pub fn get(&self, i: usize) -> Option<&'static [u8]> {
        if i >= self.argc {
            return None;
        }
        // Safety: `i < argc`, and `exec` wrote `argc` valid pointers.
        Some(unsafe { cstr(*self.argv.add(i)) })
    }

    /// Every argument, `argv[0]` included.
    pub fn iter(&self) -> impl Iterator<Item = &'static [u8]> + '_ {
        (0..self.argc).filter_map(move |i| self.get(i))
    }

    /// Every environment string (`NAME=value`).
    pub fn env(&self) -> impl Iterator<Item = &'static [u8]> {
        let envp = self.envp;
        (0..)
            // Safety: `envp` is NULL-terminated, and `take_while` stops
            // at the terminator before reading past it.
            .map(move |i| unsafe { *envp.add(i) })
            .take_while(|p| !p.is_null())
            .map(|p| unsafe { cstr(p) })
    }

    /// The value of environment variable `name`, if it's set.
    pub fn var(&self, name: &[u8]) -> Option<&'static [u8]> {
        self.env().find_map(|entry| {
            let rest = entry.strip_prefix(name)?;
            rest.strip_prefix(b"=")
        })
    }
}

/// The bytes of the NUL-terminated string at `p`, NUL excluded.
///
/// Safety: `p` must point at a NUL-terminated string that lives forever.
unsafe fn cstr(p: *const u8) -> &'static [u8] {
    let mut len = 0;
    while *p.add(len) != 0 {
        len += 1;
    }
    core::slice::from_raw_parts(p, len)
}

extern "Rust" {
    /// Supplied by the program through `neumann_rt::main!`.
    fn __neumann_main(args: Args) -> i32;
}

/// Declare a program's `main`: `neumann_rt::main!(main);` with
/// `fn main(args: neumann_rt::Args) -> i32`. The return value is the
/// program's exit status.
#[macro_export]
macro_rules! main {
    ($main:path) => {
        #[no_mangle]
        fn __neumann_main(args: $crate::Args) -> i32 {
            $main(args)
        }
    };
}

/// The entry point `exec` jumps to. Naked, because the one thing it has
/// to do -- capture `rsp` before anything pushes onto it -- is exactly
/// what a compiler-generated prologue would destroy. `rsp` arrives
/// 16-byte aligned (the kernel guarantees it); the `call` then leaves it
/// at 8 mod 16 inside `rt_start`, which is what the ABI expects of any
/// function's entry.
#[unsafe(naked)]
#[no_mangle]
#[link_section = ".text._start"]
pub unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        "mov rdi, rsp",
        "xor ebp, ebp", // mark the outermost frame for anything that walks them
        "call {rt_start}",
        "ud2",
        rt_start = sym rt_start,
    )
}

extern "C" fn rt_start(sp: *const u64) -> ! {
    // Safety: `sp` is `_start`'s entry `rsp`.
    let args = unsafe { Args::from_stack(sp) };
    let status = unsafe { __neumann_main(args) };
    sys::exit(status)
}

/// `print!`/`println!`'s sink. Buffers up to one `SYS_CONSOLE_WRITE`'s
/// worth and flushes whenever it fills, so a formatted line costs one or
/// two system calls rather than one per `write_str` fragment.
pub struct Console {
    buf: [u8; 256],
    len: usize,
}

impl Console {
    pub const fn new() -> Console {
        Console { buf: [0; 256], len: 0 }
    }

    pub fn flush(&mut self) {
        if self.len > 0 {
            sys::console_write(&self.buf[..self.len]);
            self.len = 0;
        }
    }

    pub fn write_bytes(&mut self, mut bytes: &[u8]) {
        while !bytes.is_empty() {
            if self.len == self.buf.len() {
                self.flush();
            }
            let n = bytes.len().min(self.buf.len() - self.len);
            self.buf[self.len..self.len + n].copy_from_slice(&bytes[..n]);
            self.len += n;
            bytes = &bytes[n..];
        }
    }
}

impl Default for Console {
    fn default() -> Console {
        Console::new()
    }
}

impl fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_bytes(s.as_bytes());
        Ok(())
    }
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    let mut console = Console::new();
    let _ = fmt::write(&mut console, args);
    console.flush();
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => { $crate::_print(format_args!($($arg)*)) };
}

#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($($arg:tt)*) => { $crate::_print(format_args!("{}\n", format_args!($($arg)*))) };
}

/// A byte string that prints as text, for `print!` of an argument or a
/// file name: invalid UTF-8 comes out as `?` rather than refusing to
/// print at all.
pub struct Bytes<'a>(pub &'a [u8]);

impl fmt::Display for Bytes<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for chunk in self.0.utf8_chunks() {
            f.write_str(chunk.valid())?;
            if !chunk.invalid().is_empty() {
                f.write_str("?")?;
            }
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("panic: {}", info.message());
    sys::exit(101)
}

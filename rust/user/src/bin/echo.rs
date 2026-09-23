//! `echo`: print the arguments, separated by spaces, then a newline.

#![no_std]
#![no_main]

use neumann_rt::{Args, Console};

neumann_rt::main!(main);

fn main(args: Args) -> i32 {
    let mut out = Console::new();
    for (i, arg) in args.iter().skip(1).enumerate() {
        if i > 0 {
            out.write_bytes(b" ");
        }
        out.write_bytes(arg);
    }
    out.write_bytes(b"\n");
    out.flush();
    0
}

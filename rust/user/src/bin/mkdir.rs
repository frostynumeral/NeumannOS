//! `mkdir`: create each named directory. Its parent has to exist already
//! (no `-p`).

#![no_std]
#![no_main]

use neumann_rt::{println, sys, Args, Bytes};

neumann_rt::main!(main);

fn main(args: Args) -> i32 {
    if args.len() < 2 {
        println!("usage: mkdir DIR...");
        return 2;
    }
    let mut status = 0;
    for dir in args.iter().skip(1) {
        if let Err(code) = sys::mkdir(dir) {
            println!("mkdir: {}: {}", Bytes(dir), sys::strerror(code));
            status = 1;
        }
    }
    status
}

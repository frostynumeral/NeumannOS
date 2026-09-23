//! `cat`: copy each named file to the console. Exit status 1 if any of
//! them couldn't be opened (they're reported, and the rest still get
//! printed), 2 with no arguments -- there's no standard input to fall
//! back to, since the keyboard is line-oriented and has no end of file.

#![no_std]
#![no_main]

use neumann_rt::{println, sys, Args, Bytes};

neumann_rt::main!(main);

fn main(args: Args) -> i32 {
    if args.len() < 2 {
        println!("usage: cat FILE...");
        return 2;
    }
    let mut status = 0;
    for path in args.iter().skip(1) {
        let fd = match sys::open_existing(path) {
            Ok(fd) => fd,
            Err(code) => {
                println!("cat: {}: {}", Bytes(path), sys::strerror(code));
                status = 1;
                continue;
            }
        };
        let mut buf = [0u8; sys::MAX_IO];
        loop {
            match sys::read(fd, &mut buf) {
                Ok(0) => break,
                Ok(n) => sys::console_write(&buf[..n]),
                Err(code) => {
                    println!("cat: {}: {}", Bytes(path), sys::strerror(code));
                    status = 1;
                    break;
                }
            }
        }
    }
    status
}

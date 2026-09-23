//! `ls`: list each named directory (`/` if none is named -- there is no
//! current directory to default to), one entry per line: size, then
//! name, with a trailing `/` on subdirectories. Naming a file lists just
//! that file, as `ls` does.

#![no_std]
#![no_main]

use neumann_rt::{println, sys, Args, Bytes};

neumann_rt::main!(main);

fn main(args: Args) -> i32 {
    let mut status = 0;
    let multiple = args.len() > 2;
    let mut listed_any = false;
    for (i, dir) in args.iter().skip(1).enumerate() {
        listed_any = true;
        if multiple {
            if i > 0 {
                println!();
            }
            println!("{}:", Bytes(dir));
        }
        if !list(dir) {
            status = 1;
        }
    }
    if !listed_any && !list(b"/") {
        status = 1;
    }
    status
}

/// List one directory; `false` if it couldn't be.
fn list(dir: &[u8]) -> bool {
    let mut entry = sys::DirEntry::empty();
    let mut index = 0;
    loop {
        match sys::readdir(dir, index, &mut entry) {
            Ok(true) => {
                let slash = if entry.is_dir() { "/" } else { "" };
                println!("{:>8}  {}{}", entry.size, Bytes(entry.name()), slash);
                index += 1;
            }
            Ok(false) => return true,
            Err(sys::ENOTDIR) => {
                // A file: report it, the way `ls FILE` does.
                return match sys::open_existing(dir) {
                    Ok(fd) => {
                        let _ = sys::close(fd);
                        println!("{}", Bytes(dir));
                        true
                    }
                    Err(code) => {
                        println!("ls: {}: {}", Bytes(dir), sys::strerror(code));
                        false
                    }
                };
            }
            Err(code) => {
                println!("ls: {}: {}", Bytes(dir), sys::strerror(code));
                return false;
            }
        }
    }
}

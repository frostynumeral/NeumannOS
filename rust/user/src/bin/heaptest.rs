//! `heaptest`: the heap, end to end. A `Vec` grown past a megabyte (many
//! `SYS_BRK` growths) with its contents checked, a `String` and a `Box`;
//! the break having moved and stayed inside the heap's range; a `fork`
//! whose child rewrites the heap and exits with a checksum while the
//! parent's copy stays as it was (the heap is shared copy-on-write like
//! any other page); and three threads allocating and freeing at once
//! through the one locked heap. Writes `ok` to `/heaptest.out` if every
//! check passed -- the kernel's boot verification (`crate::main`'s
//! `heap_check`) reads it.

#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;
use core::ptr::addr_of_mut;
use neumann_rt::{println, sys, thread, Args};

neumann_rt::main!(main);

const HEAP_BASE: u64 = 0x3800_0000_0000;
static mut STACKS: [[u8; 16384]; 3] = [[0; 16384]; 3];

fn churn(seed: u64) -> i32 {
    // Allocate and free in a pattern, checking every value on the way.
    let mut sum = 0u64;
    for round in 0..50u64 {
        let v: Vec<u64> = (0..200 + round).map(|i| i * seed).collect();
        let s: u64 = v.iter().sum();
        let n = 200 + round;
        if s != seed * n * (n - 1) / 2 {
            return -1;
        }
        sum = sum.wrapping_add(s);
    }
    (sum % 1000) as i32
}

fn main(_args: Args) -> i32 {
    let mut ok = true;
    let mut fail = |what: &str| {
        println!("heaptest: FAIL {}", what);
        ok = false;
    };

    let start = sys::brk(0).unwrap_or(0);
    let mut big: Vec<u64> = Vec::new();
    for i in 0..200_000u64 {
        big.push(i ^ 0x5a5a);
    }
    if big.iter().enumerate().any(|(i, &v)| v != (i as u64) ^ 0x5a5a) {
        fail("vec contents");
    }
    let end = sys::brk(0).unwrap_or(0);
    if !(start >= HEAP_BASE && end > start + 1_000_000 && end <= HEAP_BASE + 16 * 1024 * 1024) {
        fail("break didn't move as expected");
    }
    let mut s = String::new();
    let _ = write!(s, "{} {}", "neumann", 42);
    if s != "neumann 42" {
        fail("string");
    }
    let b = Box::new([7u8; 1000]);
    if b.iter().any(|&x| x != 7) {
        fail("box");
    }

    // fork: the child's writes to the heap stay in the child.
    match sys::fork() {
        Ok(0) => {
            for v in big.iter_mut() {
                *v = 0;
            }
            big.push(1);
            sys::exit(big.iter().sum::<u64>() as i32) // 1
        }
        Ok(_) => match sys::wait() {
            Ok((_, 1)) => {
                if big.len() != 200_000 || big[199_999] != 199_999 ^ 0x5a5a {
                    fail("the child's heap writes reached the parent");
                }
            }
            other => {
                println!("heaptest: child -> {:?}", other);
                fail("fork child");
            }
        },
        Err(_) => fail("fork"),
    }
    drop(big);

    // Threads sharing the heap.
    let mut handles = Vec::new();
    for i in 0..3 {
        let stack = unsafe { &mut *addr_of_mut!(STACKS[i]) };
        match thread::spawn(stack, churn, i as u64 + 1) {
            Ok(t) => handles.push(t),
            Err(_) => fail("thread spawn"),
        }
    }
    for t in handles {
        if !matches!(t.join(), Ok(status) if status >= 0) {
            fail("a thread's allocations came out wrong");
        }
    }

    println!("heaptest: {} ({} KiB of heap)", if ok { "ok" } else { "FAIL" }, (sys::brk(0).unwrap_or(0) - HEAP_BASE) / 1024);
    if let Ok(out) = sys::open(b"/heaptest.out") {
        let _ = sys::write(out, if ok { b"ok" } else { b"FAIL" });
        let _ = sys::close(out);
    }
    if ok { 0 } else { 1 }
}

//! `porttest`: Haiku's port semantics, checked end to end against
//! `neumann_rt::os` -- the kernel kit's port API with Haiku's names and
//! status codes. Writes `ok` to `/porttest.out` if every check passed;
//! the kernel's boot verification (`crate::main`'s `port_check`) reads it.

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use neumann_rt::os::*;
use neumann_rt::{println, sys, thread, Args};

neumann_rt::main!(main);

static mut STACKS: [[u8; 16384]; 2] = [[0; 16384]; 2];
static mut SHARED_PORT: port_id = -1;

/// Consumer: read 100 messages off `SHARED_PORT`, checking each is the
/// next in order; its status is the sum of the payloads.
fn consumer(_: u64) -> i32 {
    let port = unsafe { SHARED_PORT };
    let mut sum = 0i32;
    for expected in 0..100i32 {
        let mut code = 0;
        let mut buf = [0u8; 4];
        if read_port(port, &mut code, &mut buf) != 4 || code != expected {
            return -1;
        }
        sum += i32::from_le_bytes(buf);
    }
    sum
}

static mut READY_SEM: i64 = -1;

/// Blocks reading a port until it's deleted out from under it. Signals
/// `READY_SEM` first, so the main thread knows it's about to block.
fn doomed_reader(_: u64) -> i32 {
    let mut code = 0;
    let mut buf = [0u8; 4];
    let _ = sys::sem_release(unsafe { READY_SEM });
    read_port(unsafe { SHARED_PORT }, &mut code, &mut buf) as i32
}

fn main(_args: Args) -> i32 {
    let mut failures = 0;
    let mut check = |ok: bool, what: &str| {
        if !ok {
            println!("porttest: FAIL {}", what);
            failures += 1;
        }
    };

    // Create, find.
    let port = create_port(4, "porttest queue");
    check(port > 0, "create_port");
    check(find_port("porttest queue") == port, "find_port by name");
    check(find_port("no such port") == B_NAME_NOT_FOUND, "find_port of a missing name");
    check(create_port(0, "x") == B_BAD_VALUE, "create_port with capacity 0");

    // FIFO order, counts, sizes, info.
    for (code, msg) in [(1, &b"one"[..]), (2, b"two!"), (3, b"three")] {
        check(write_port(port, code, msg) == B_OK, "write_port");
    }
    check(port_count(port) == 3, "port_count");
    check(port_buffer_size(port) == 3, "port_buffer_size of the head");
    let mut info = port_info::new();
    check(get_port_info(port, &mut info) == B_OK, "get_port_info");
    check(info.port == port && info.capacity == 4 && info.queue_count == 3, "port_info counts");
    check(info.name() == b"porttest queue", "port_info name");
    for (want_code, want) in [(1, &b"one"[..]), (2, b"two!"), (3, b"three")] {
        let mut code = 0;
        let mut buf = [0u8; 16];
        let n = read_port(port, &mut code, &mut buf);
        check(n == want.len() as isize && code == want_code && &buf[..n as usize] == want, "read_port order");
    }
    let _ = get_port_info(port, &mut info);
    check(info.total_count == 3 && info.queue_count == 0, "port_info after reading");
    // A message bigger than the buffer: what fits, the rest dropped.
    let _ = write_port(port, 9, b"0123456789");
    let mut code = 0;
    let mut small = [0u8; 4];
    check(read_port(port, &mut code, &mut small) == 4 && &small == b"0123", "read_port truncation");

    // Full port: zero timeout would block, a real timeout expires.
    for i in 0..4 {
        let _ = write_port(port, i, b"x");
    }
    check(write_port_etc(port, 5, b"x", B_RELATIVE_TIMEOUT, 0) == B_WOULD_BLOCK, "zero timeout on a full port");
    let start = system_time();
    check(write_port_etc(port, 5, b"x", B_RELATIVE_TIMEOUT, 50_000) == B_TIMED_OUT, "timeout on a full port");
    check(system_time() - start >= 30_000, "the timeout actually waited");
    let empty = create_port(1, "porttest empty");
    let mut buf = [0u8; 4];
    check(read_port_etc(empty, &mut code, &mut buf, B_RELATIVE_TIMEOUT, 0) == B_WOULD_BLOCK as isize, "zero timeout on an empty port");
    check(port_buffer_size_etc(empty, B_RELATIVE_TIMEOUT, 20_000) == B_TIMED_OUT as isize, "port_buffer_size timeout");
    // B_INFINITE_TIMEOUT with a timeout flag means "wait forever" -- and
    // mustn't overflow the kernel's tick arithmetic (it used to panic it).
    check(write_port_etc(empty, 1, b"inf", B_RELATIVE_TIMEOUT, B_INFINITE_TIMEOUT) == B_OK, "infinite relative timeout");
    check(read_port_etc(empty, &mut code, &mut buf, B_ABSOLUTE_TIMEOUT, B_INFINITE_TIMEOUT) == 3, "infinite absolute timeout");
    let _ = delete_port(empty);

    // Producer/consumer between threads over a 2-deep port: both sides block.
    let pc = create_port(2, "porttest pc");
    unsafe { SHARED_PORT = pc };
    let stack = unsafe { &mut *addr_of_mut!(STACKS[0]) };
    let consumer_thread = thread::spawn(stack, consumer, 0).expect("spawn");
    for i in 0..100i32 {
        let _ = write_port(pc, i, &(i * 2).to_le_bytes());
    }
    check(consumer_thread.join() == Ok(9900), "producer/consumer through a port");

    // Across processes: a forked child writes to our port by name.
    match sys::fork() {
        Ok(0) => {
            let p = find_port("porttest queue");
            let status = if p > 0 { write_port_etc(p, 77, b"from the child", B_RELATIVE_TIMEOUT, 1_000_000) } else { p };
            sys::exit(if status == B_OK { 0 } else { 1 })
        }
        Ok(_) => {
            // Drain the four "x" messages the timeout test left first.
            for _ in 0..4 {
                let _ = read_port(port, &mut code, &mut buf);
            }
            let child = sys::wait();
            let mut msg = [0u8; 32];
            let n = read_port(port, &mut code, &mut msg);
            check(child.map(|(_, s)| s) == Ok(0) && code == 77 && &msg[..n.max(0) as usize] == b"from the child", "a message from another process");
        }
        Err(_) => check(false, "fork"),
    }

    // close_port: no writes, reads drain, then B_BAD_PORT_ID.
    let _ = write_port(port, 1, b"last");
    check(close_port(port) == B_OK, "close_port");
    check(write_port(port, 2, b"no") == B_BAD_PORT_ID, "write to a closed port");
    check(read_port(port, &mut code, &mut buf) == 4, "read what a closed port still holds");
    check(read_port(port, &mut code, &mut buf) == B_BAD_PORT_ID as isize, "read a drained closed port");
    check(delete_port(port) == B_OK, "delete_port");
    check(port_count(port) == B_BAD_PORT_ID as isize, "a deleted port is gone");

    // delete_port wakes a blocked reader with B_BAD_PORT_ID.
    // The reader signals just before it reads; sleeping after that hands
    // it the CPU with nothing to do but block, so the delete below really
    // does find it waiting (rather than it reading an already-dead id).
    let victim = create_port(1, "porttest victim");
    unsafe { SHARED_PORT = victim };
    let ready = thread::Semaphore::new(0).expect("sem");
    unsafe { READY_SEM = ready.id() };
    let stack = unsafe { &mut *addr_of_mut!(STACKS[1]) };
    let reader = thread::spawn(stack, doomed_reader, 0).expect("spawn");
    ready.acquire();
    sys::sleep(5);
    check(port_count(victim) == 0, "the victim port still exists before delete");
    let _ = delete_port(victim);
    check(reader.join() == Ok(B_BAD_PORT_ID), "delete_port wakes a waiting reader");

    // A team's ports die with it. The child reports its port's id over a
    // sync port and waits to be told to go, so the parent can confirm the
    // port really existed before checking it's gone afterwards.
    let sync = create_port(1, "porttest sync");
    match sys::fork() {
        Ok(0) => {
            let mine = create_port(1, "porttest child port");
            let _ = write_port(sync, mine, b"");
            let mut c = 0;
            let _ = read_port_etc(mine, &mut c, &mut [], B_RELATIVE_TIMEOUT, 2_000_000);
            sys::exit(0)
        }
        Ok(_) => {
            let mut child_port = 0;
            let _ = read_port(sync, &mut child_port, &mut []);
            check(child_port > 0 && find_port("porttest child port") == child_port, "the child's port exists while it lives");
            let _ = write_port(child_port, 0, b"go");
            let _ = sys::wait();
            check(find_port("porttest child port") == B_NAME_NOT_FOUND, "a dead team's port was deleted");
        }
        Err(_) => check(false, "fork"),
    }
    let _ = delete_port(sync);

    let ok = failures == 0;
    println!("porttest: {}", if ok { "ok" } else { "FAIL" });
    if let Ok(out) = sys::open(b"/porttest.out") {
        let _ = sys::write(out, if ok { b"ok" } else { b"FAIL" });
        let _ = sys::close(out);
    }
    if ok { 0 } else { 1 }
}

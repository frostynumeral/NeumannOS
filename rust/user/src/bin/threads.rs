//! `threads`: four threads increment one shared counter, each inside a
//! critical section guarded by a semaphore used as a mutex, and the
//! total has to come out exact. The critical section deliberately reads
//! the counter, dawdles, then writes it back -- a window a preempting
//! timer tick will land in -- so without the semaphore, updates would be
//! lost (`threads race` runs it that way, to show the difference).
//!
//! Then it checks what joining returns (each thread's own status), and
//! finishes by leaving one thread running forever and exiting anyway:
//! `exit` from the main thread ends the whole team, which the kernel's
//! log shows. Writes `ok` to `/threads.out` if every check passed --
//! the kernel's boot verification (`crate::main`'s `threads_check`)
//! reads it.

#![no_std]
#![no_main]

use core::ptr::{addr_of, addr_of_mut};
use neumann_rt::thread::{self, Semaphore};
use neumann_rt::{println, sys, Args};

neumann_rt::main!(main);

const THREADS: usize = 4;
/// Enough that each thread runs for several scheduling quanta, so the
/// timer really does preempt them inside the critical section -- with
/// too few, each thread finishes within one quantum and neither the
/// semaphore nor its absence is ever tested.
const ROUNDS: u64 = 1000;

static mut STACKS: [[u8; 8192]; THREADS + 1] = [[0; 8192]; THREADS + 1];
static mut COUNTER: u64 = 0;
static mut LOCK: i64 = -1; // the semaphore's id, or -1 for "race"

fn worker(index: u64) -> i32 {
    for _ in 0..ROUNDS {
        let lock = unsafe { LOCK };
        if lock >= 0 {
            let _ = sys::sem_acquire(lock);
        }
        // Read, dawdle, write: the window a lost update needs.
        let value = unsafe { addr_of!(COUNTER).read_volatile() };
        for _ in 0..2000 {
            core::hint::spin_loop();
        }
        unsafe { addr_of_mut!(COUNTER).write_volatile(value + 1) };
        if lock >= 0 {
            let _ = sys::sem_release(lock);
        }
    }
    100 + index as i32
}

fn forever(_: u64) -> i32 {
    loop {
        core::hint::spin_loop();
    }
}

/// A thread that blocks for a line of keyboard input.
fn reader(_: u64) -> i32 {
    let mut line = [0u8; 64];
    let _ = sys::read_line(&mut line);
    0
}

/// A thread that faults: the whole team has to go with it.
fn crasher(_: u64) -> i32 {
    unsafe { core::arch::asm!("ud2") };
    0
}

fn main(args: Args) -> i32 {
    match args.get(1) {
        // `threads crash`: a thread faults while the main thread waits
        // for it -- fatal to the whole team, as a fatal signal is to a
        // POSIX process, so the shell sees the main thread killed too.
        Some(b"crash") => {
            let stack = unsafe { &mut *addr_of_mut!(STACKS[0]) };
            let t = thread::spawn(stack, crasher, 0).expect("thread spawn");
            let _ = t.join();
            println!("threads: FAIL, still here after a thread crashed");
            return 1;
        }
        // `threads linger`: a thread blocked reading a line when the
        // program exits. It can't be freed mid-call (the console will
        // still reply to it), so it finishes the call first -- the next
        // line typed -- and the program's exit status only reaches the
        // shell after that.
        Some(b"linger") => {
            let stack = unsafe { &mut *addr_of_mut!(STACKS[0]) };
            let _ = thread::spawn(stack, reader, 0).expect("thread spawn");
            sys::sleep(10); // let it block in read_line
            println!("threads: exiting with a thread still reading");
            return 5;
        }
        // `threads exec`: exec while another thread runs is refused.
        Some(b"exec") => {
            let stack = unsafe { &mut *addr_of_mut!(STACKS[0]) };
            let _ = thread::spawn(stack, forever, 0).expect("thread spawn");
            let argv = [b"echo\0".as_ptr(), core::ptr::null()];
            let envp = [core::ptr::null::<u8>()];
            let code = unsafe { sys::exec(b"/bin/echo", argv.as_ptr(), envp.as_ptr()) };
            println!("threads: exec with a thread running -> {}", sys::strerror(code));
            return if code == sys::ERR_MULTITHREADED { 0 } else { 1 };
        }
        _ => {}
    }
    let race = args.get(1) == Some(b"race");
    let lock = if race { None } else { Some(Semaphore::new(1).expect("sem_create")) };
    unsafe { LOCK = lock.as_ref().map_or(-1, |s| s.id()) };

    let mut handles: [Option<thread::Thread>; THREADS] = [const { None }; THREADS];
    for (i, handle) in handles.iter_mut().enumerate() {
        let stack = unsafe { &mut *addr_of_mut!(STACKS[i]) };
        *handle = Some(thread::spawn(stack, worker, i as u64).expect("thread spawn"));
    }
    let mut ok = true;
    for (i, handle) in handles.iter_mut().enumerate() {
        let status = handle.take().unwrap().join();
        if status != Ok(100 + i as i32) {
            println!("threads: join {} -> {:?}", i, status);
            ok = false;
        }
    }
    let total = unsafe { addr_of!(COUNTER).read_volatile() };
    let expected = THREADS as u64 * ROUNDS;
    println!("threads: {} x {} = {} ({})", THREADS, ROUNDS, total, if race { "no lock" } else { "locked" });
    if !race && total != expected {
        println!("threads: FAIL, expected {}", expected);
        ok = false;
    }
    if race {
        println!("threads: {} updates lost to the race", expected - total);
        return 0;
    }

    // Joining something that isn't a thread of this team is an error.
    if sys::thread_join(1).is_ok() {
        println!("threads: FAIL, joined a non-thread");
        ok = false;
    }

    if let Ok(out) = sys::open(b"/threads.out") {
        let _ = sys::write(out, if ok { b"ok" } else { b"FAIL" });
        let _ = sys::close(out);
    }

    // Leave one running and exit anyway: the team goes with the main thread.
    let stack = unsafe { &mut *addr_of_mut!(STACKS[THREADS]) };
    let _ = thread::spawn(stack, forever, 0);
    drop(lock);
    if ok { 0 } else { 1 }
}

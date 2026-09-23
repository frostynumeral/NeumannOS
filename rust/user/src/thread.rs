//! Threads and semaphores: the BeOS-flavored half of the runtime.
//!
//! A thread (`spawn`) is a real kernel thread -- its own process-table
//! slot, scheduled and preempted on its own -- running in this program's
//! address space, sharing its memory and descriptors. `join` waits for
//! one to finish and collects what its function returned. `Semaphore` is
//! a counting semaphore (BeOS's `create_sem`/`acquire_sem`/
//! `release_sem`), the primitive everything else synchronizes with; a
//! semaphore with one unit is a mutex.
//!
//! There's no allocator, so a thread's stack is memory the caller hands
//! over -- a `static` array, in practice -- for as long as the thread
//! runs.

use crate::sys;

/// What a new thread needs to find: its function and argument. Written
/// at the very top of the thread's own stack, so nothing has to outlive
/// `spawn` elsewhere.
#[repr(C)]
struct Start {
    f: fn(u64) -> i32,
    arg: u64,
}

/// Where every thread begins: pick the function out of the start block,
/// run it, and end the thread with what it returned.
extern "C" fn thread_main(start: *const Start) -> ! {
    // Safety: `spawn` put a `Start` here, on this thread's own stack,
    // above anything the thread itself will push.
    let Start { f, arg } = unsafe { start.read() };
    sys::thread_exit(f(arg))
}

/// A running thread, to `join`.
pub struct Thread {
    id: i64,
}

impl Thread {
    pub fn id(&self) -> i64 {
        self.id
    }

    /// Wait for the thread to finish; what its function returned.
    pub fn join(self) -> Result<i32, i64> {
        sys::thread_join(self.id)
    }
}

/// Start `f(arg)` in a new thread, on `stack`.
///
/// `stack` is `'static` and `mut` because the thread owns it until it
/// exits; hand each running thread its own.
pub fn spawn(stack: &'static mut [u8], f: fn(u64) -> i32, arg: u64) -> Result<Thread, i64> {
    let base = stack.as_mut_ptr() as u64;
    let top = (base + stack.len() as u64) & !0xf;
    let start = (top - core::mem::size_of::<Start>() as u64) & !0xf;
    if start < base + 256 {
        return Err(sys::ERR_BAD_ARG_PTR); // too small to be a stack
    }
    // Safety: inside `stack`, which is ours to use.
    unsafe { (start as *mut Start).write(Start { f, arg }) };
    // `rsp` as a function sees it on entry: 16-byte aligned *minus* the
    // return address a `call` would have pushed (there is none; a fake
    // zero sits there, and `thread_main` never returns through it).
    let rsp = start - 8;
    unsafe { (rsp as *mut u64).write(0) };
    let id = unsafe { sys::thread_spawn(thread_main as *const () as u64, rsp, start)? };
    Ok(Thread { id })
}

/// A counting semaphore. Deleted when dropped.
pub struct Semaphore {
    id: i64,
}

impl Semaphore {
    pub fn new(count: i32) -> Result<Semaphore, i64> {
        Ok(Semaphore { id: sys::sem_create(count)? })
    }

    /// Take a unit, waiting for one if necessary.
    pub fn acquire(&self) {
        sys::sem_acquire(self.id).expect("acquire on a deleted semaphore");
    }

    /// Give a unit back.
    pub fn release(&self) {
        sys::sem_release(self.id).expect("release on a deleted semaphore");
    }

    pub fn id(&self) -> i64 {
        self.id
    }
}

impl Drop for Semaphore {
    fn drop(&mut self) {
        let _ = sys::sem_delete(self.id);
    }
}

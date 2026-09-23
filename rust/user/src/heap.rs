//! The global allocator: a linked-list heap (`linked_list_allocator`)
//! over memory the kernel maps on request through `SYS_BRK`. The heap
//! starts empty; the first allocation, and any that doesn't fit, moves
//! the program break up by at least `GROW` and hands the new stretch to
//! the heap. It never gives memory back -- freed blocks are reused, not
//! returned -- which is what most `malloc`s do with `brk` too.
//!
//! A spin lock guards it, since a program's threads share one heap. A
//! thread preempted while holding it makes the others spin until it runs
//! again -- a spin lock can't yield, so a higher-priority spinner can
//! burn a quantum or more before the holder gets back -- but allocation
//! is short, so that's rare. (A semaphore would block instead; it costs
//! a system call per allocation.) `sys::fork` holds it across the call.

use core::alloc::{GlobalAlloc, Layout};
use core::ptr::{self, NonNull};
use linked_list_allocator::Heap;
use spin::Mutex;

use crate::sys;

/// Least the heap grows by at a time: one system call per 64 KiB rather
/// than one per allocation.
const GROW: usize = 64 * 1024;

struct BrkHeap(Mutex<Option<Heap>>);

#[global_allocator]
static ALLOCATOR: BrkHeap = BrkHeap(Mutex::new(None));

impl BrkHeap {
    /// Grow the heap by enough for `layout` (and at least `GROW`).
    fn grow(heap: &mut Option<Heap>, layout: Layout) -> bool {
        let want = (layout.size() + layout.align()).max(GROW);
        let want = (want + 4095) & !4095;
        let Ok(old_end) = sys::brk(0) else { return false };
        if sys::brk(old_end + want as u64).is_err() {
            return false;
        }
        match heap {
            // Safety: `old_end..old_end + want` was just mapped for us by
            // the kernel, and nothing else uses it.
            None => *heap = Some(unsafe { Heap::new(old_end as *mut u8, want) }),
            // Safety: the new stretch starts exactly where the heap ends
            // (the break only moves with us), so it extends it.
            Some(h) => unsafe { h.extend(want) },
        }
        true
    }
}

/// The heap's lock, held across a `fork` (see `sys::fork`); released
/// when dropped.
pub struct ForkGuard(#[allow(dead_code)] spin::MutexGuard<'static, Option<Heap>>);

/// Take the heap's lock for the length of a `fork`.
pub fn lock_for_fork() -> ForkGuard {
    ForkGuard(ALLOCATOR.0.lock())
}

unsafe impl GlobalAlloc for BrkHeap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let mut heap = self.0.lock();
        loop {
            if let Some(h) = heap.as_mut() {
                if let Ok(block) = h.allocate_first_fit(layout) {
                    return block.as_ptr();
                }
            }
            if !Self::grow(&mut heap, layout) {
                return ptr::null_mut();
            }
        }
    }

    unsafe fn dealloc(&self, block: *mut u8, layout: Layout) {
        if let (Some(h), Some(block)) = (self.0.lock().as_mut(), NonNull::new(block)) {
            h.deallocate(block, layout);
        }
    }
}

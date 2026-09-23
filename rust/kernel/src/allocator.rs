//! Heap allocator.
//!
//! No equivalent C file to port: MINIX kernel tasks and servers don't have
//! a general-purpose heap either (their allocations are static tables and
//! process/segment structures, like the ones `crate::proc` still uses for
//! its own process table and stacks), but Rust's `alloc` crate (`Box`,
//! `Vec`, and friends) needs one, and later milestones -- per-process page
//! tables chief among them (see `rust/README.md`) -- are much more
//! natural to write against a heap than against more fixed-size static
//! arrays.

use linked_list_allocator::LockedHeap;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::{
    mapper::MapToError, FrameAllocator, Mapper, Page, PageTableFlags, Size4KiB,
};
use x86_64::VirtAddr;

use crate::memory::GlobalFrameAllocator;

/// Arbitrary, fixed virtual address range for the kernel heap -- there's no
/// user-mode address space yet to conflict with (see `rust/README.md`), so
/// there's no real constraint on where this lives other than "not already
/// mapped by the bootloader".
pub const HEAP_START: usize = 0x_4444_4444_0000;
pub const HEAP_SIZE: usize = 1024 * 1024; // 1 MiB

/// The heap's lock is a spin lock, and code that runs with interrupts
/// already off -- the page-fault handler, and `crate::memory`'s
/// copy-on-write bookkeeping, both of which allocate or free through a
/// `BTreeMap` -- takes it too. If a task holding it with interrupts on
/// were preempted, or a page fault arrived mid-allocation, that code
/// would spin forever on a lock only the parked task can release. So
/// every allocation happens with interrupts off, and nothing can hold
/// the lock across a switch.
struct InterruptSafeHeap(LockedHeap);

unsafe impl core::alloc::GlobalAlloc for InterruptSafeHeap {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        without_interrupts(|| self.0.alloc(layout))
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: core::alloc::Layout) {
        without_interrupts(|| self.0.dealloc(ptr, layout))
    }
}

#[global_allocator]
static ALLOCATOR: InterruptSafeHeap = InterruptSafeHeap(LockedHeap::empty());

/// Map `HEAP_SIZE` bytes at `HEAP_START` and hand that range to the global
/// allocator. Must run once, after `crate::memory::init` and
/// `crate::memory::init_frame_allocator`, before any `alloc`-crate type
/// (`Box`, `Vec`, ...) is used.
pub fn init_heap(mapper: &mut impl Mapper<Size4KiB>) -> Result<(), MapToError<Size4KiB>> {
    let page_range = {
        let heap_start = VirtAddr::new(HEAP_START as u64);
        let heap_end = heap_start + HEAP_SIZE as u64 - 1u64;
        let heap_start_page = Page::containing_address(heap_start);
        let heap_end_page = Page::containing_address(heap_end);
        Page::range_inclusive(heap_start_page, heap_end_page)
    };

    let mut frame_allocator = GlobalFrameAllocator;
    for page in page_range {
        let frame = frame_allocator
            .allocate_frame()
            .ok_or(MapToError::FrameAllocationFailed)?;
        let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE;
        unsafe { mapper.map_to(page, frame, flags, &mut frame_allocator)?.flush() };
    }

    unsafe {
        ALLOCATOR.0.lock().init(HEAP_START as *mut u8, HEAP_SIZE);
    }
    Ok(())
}

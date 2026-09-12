//! Paging and physical frame allocation.
//!
//! Real MINIX ties memory management directly into the process table:
//! `kernel/kernel.h`'s `struct mem_map` gives each process's segments a
//! base and size in physical memory (segment-based protection, no paging
//! -- this is a 2005-era i386 kernel that predates MINIX 3's later switch
//! to a paged VM system), and privileged operations like `sys_umap`/
//! `sys_vircopy` (`kernel/system/do_umap.c`, `do_vircopy.c`) translate
//! between a process's virtual segments and physical addresses on the
//! kernel's behalf.
//!
//! This port targets x86_64, which has no non-paged protected mode at all,
//! so there is no equivalent C file to port here: paging is mandatory
//! groundwork, not a MINIX feature. What's here is the minimum needed to
//! start allocating memory dynamically (`crate::allocator`) ahead of the
//! next roadmap step (`rust/README.md`): per-process page tables for real
//! address-space isolation, which is where this module's C parallel
//! (`sys_umap`/`sys_vircopy`'s job of translating between address spaces)
//! will actually start to apply.

use bootloader::bootinfo::{MemoryMap, MemoryRegionType};
use x86_64::structures::paging::{FrameAllocator, OffsetPageTable, PageTable, PhysFrame, Size4KiB};
use x86_64::{PhysAddr, VirtAddr};

/// Build an `OffsetPageTable` over the page table the bootloader already
/// installed (it maps the kernel plus, thanks to the `map_physical_memory`
/// feature, all physical memory at `physical_memory_offset`). Ours to
/// extend with further mappings from here on (`crate::allocator`'s heap,
/// and eventually a second address space per process).
///
/// # Safety
/// The caller must guarantee that the complete physical memory is mapped
/// at `physical_memory_offset`, and must call this only once (aliasing a
/// `&mut PageTable` twice is undefined behavior).
pub unsafe fn init(physical_memory_offset: VirtAddr) -> OffsetPageTable<'static> {
    let level_4_table = active_level_4_table(physical_memory_offset);
    OffsetPageTable::new(level_4_table, physical_memory_offset)
}

unsafe fn active_level_4_table(physical_memory_offset: VirtAddr) -> &'static mut PageTable {
    use x86_64::registers::control::Cr3;

    let (level_4_table_frame, _) = Cr3::read();
    let phys = level_4_table_frame.start_address();
    let virt = physical_memory_offset + phys.as_u64();
    let page_table_ptr: *mut PageTable = virt.as_mut_ptr();
    &mut *page_table_ptr
}

/// Hands out physical frames from the regions the bootloader's memory map
/// (`BootInfo::memory_map`) marked `Usable`. Analogous in spirit to the
/// free-memory bookkeeping `kernel/main.c` does for `mem_map`/`free_mem`
/// when building the boot-time memory list, just frame-granular instead of
/// segment-granular.
pub struct BootInfoFrameAllocator {
    memory_map: &'static MemoryMap,
    next: usize,
}

impl BootInfoFrameAllocator {
    /// # Safety
    /// The caller must guarantee `memory_map` is valid and that every
    /// frame it marks `Usable` really is unused.
    pub unsafe fn init(memory_map: &'static MemoryMap) -> Self {
        BootInfoFrameAllocator { memory_map, next: 0 }
    }

    fn usable_frames(&self) -> impl Iterator<Item = PhysFrame> {
        let regions = self.memory_map.iter();
        let usable = regions.filter(|r| r.region_type == MemoryRegionType::Usable);
        let addr_ranges = usable.map(|r| r.range.start_addr()..r.range.end_addr());
        let frame_addresses = addr_ranges.flat_map(|r| r.step_by(4096));
        frame_addresses.map(|addr| PhysFrame::containing_address(PhysAddr::new(addr)))
    }
}

unsafe impl FrameAllocator<Size4KiB> for BootInfoFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame> {
        let frame = self.usable_frames().nth(self.next);
        self.next += 1;
        frame
    }
}

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
//! groundwork, not a MINIX feature. Alongside allocating memory
//! dynamically (`crate::allocator`), this module now also builds new,
//! separate address spaces (`new_address_space`) -- see `crate::usermode`
//! for the first thing to actually use one, and `rust/README.md` for how
//! this relates to the still-unported `sys_umap`/`sys_vircopy` (which
//! translate between address spaces in the real kernel; the "which
//! address space" part is what this module adds).

use bootloader::bootinfo::{MemoryMap, MemoryRegionType};
use x86_64::structures::paging::{
    FrameAllocator, OffsetPageTable, PageTable, PhysFrame, Size4KiB, Translate,
};
use x86_64::{PhysAddr, VirtAddr};

/// The offset physical memory is mapped at, captured once by `init` so
/// later code (`page_table_for`, `copy_between_address_spaces`) can build
/// an `OffsetPageTable` over *any* address space's PML4, not just the
/// active one, without every caller having to thread the offset through.
/// Written once, at boot, before any other CPU-visible state depends on
/// it; read-only after.
static mut PHYSICAL_MEMORY_OFFSET: u64 = 0;

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
    PHYSICAL_MEMORY_OFFSET = physical_memory_offset.as_u64();
    let level_4_table = active_level_4_table(physical_memory_offset);
    OffsetPageTable::new(level_4_table, physical_memory_offset)
}

fn physical_memory_offset() -> VirtAddr {
    VirtAddr::new(unsafe { PHYSICAL_MEMORY_OFFSET })
}

unsafe fn active_level_4_table(physical_memory_offset: VirtAddr) -> &'static mut PageTable {
    use x86_64::registers::control::Cr3;

    let (level_4_table_frame, _) = Cr3::read();
    let phys = level_4_table_frame.start_address();
    let virt = physical_memory_offset + phys.as_u64();
    let page_table_ptr: *mut PageTable = virt.as_mut_ptr();
    &mut *page_table_ptr
}

/// Allocate a fresh top-level (PML4) page table that starts out an exact
/// copy of the currently-active one -- so it shares every existing
/// mapping (kernel code/data, the heap, the physical-memory window) by
/// aliasing the same lower-level tables, the same way a real OS starts a
/// new address space as a copy of the kernel's. Mapping a *new* page into
/// it only actually becomes private to this address space if that virtual
/// address's PML4 slot wasn't already in use in the table it was copied
/// from (an empty slot gets fresh, unshared lower-level tables on the
/// first `map_to` into it); mapping into a virtual address whose PML4
/// slot the original address space already used would instead alias --
/// and so modify -- that *shared* lower-level table. `crate::usermode`
/// picks demo addresses far enough apart (each PML4 slot spans 512 GiB)
/// to guarantee this.
pub fn new_address_space(
    physical_memory_offset: VirtAddr,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) -> (PhysFrame, OffsetPageTable<'static>) {
    let new_frame = frame_allocator
        .allocate_frame()
        .expect("out of physical frames for a new address space's PML4");

    let (active_frame, _) = x86_64::registers::control::Cr3::read();
    let active_ptr: *const PageTable =
        (physical_memory_offset + active_frame.start_address().as_u64()).as_ptr();
    let new_ptr: *mut PageTable =
        (physical_memory_offset + new_frame.start_address().as_u64()).as_mut_ptr();
    unsafe {
        core::ptr::copy_nonoverlapping(active_ptr, new_ptr, 1);
        let table = &mut *new_ptr;
        (new_frame, OffsetPageTable::new(table, physical_memory_offset))
    }
}

/// Build a *read-only-in-spirit* `OffsetPageTable` over `pml4_frame`,
/// whether or not it's the currently-active one. Used to translate a
/// virtual address in some *other* process's address space without
/// switching `CR3` to it -- exactly the job `sys_vircopy`
/// (`crate::calls`) needs, and the same job `kernel/system/do_copy.c`'s
/// `virtual_copy()` does for MINIX's segment-based addressing instead.
///
/// # Safety
/// `pml4_frame` must be a frame previously returned by `new_address_space`
/// (or the frame `Cr3::read()` reports), so that it's actually a valid,
/// currently-allocated PML4. Callers must not hold this and separately
/// mutate the same address space's page tables at once (single-core, no
/// real aliasing risk today, but the same caveat `new_address_space` has).
unsafe fn page_table_for(pml4_frame: PhysFrame) -> OffsetPageTable<'static> {
    let offset = physical_memory_offset();
    let ptr: *mut PageTable = (offset + pml4_frame.start_address().as_u64()).as_mut_ptr();
    OffsetPageTable::new(&mut *ptr, offset)
}

/// Copy `len` bytes from `src_addr` in the address space rooted at
/// `src_pml4` to `dst_addr` in the one rooted at `dst_pml4`, without ever
/// switching `CR3`: both ends are reached through the physical-memory
/// window instead. Ported in spirit from `kernel/system/do_copy.c`'s
/// `SYS_VIRCOPY` (`virtual_copy()`), which does the equivalent translate-
/// then-copy for MINIX's segment-based addresses; see `crate::calls` for
/// the process-number-based wrapper this backs.
///
/// Walks a page at a time so a copy that crosses a page boundary in
/// either address space still works, same as `virtual_copy` handles a
/// copy crossing a segment boundary.
pub fn copy_between_address_spaces(
    src_pml4: PhysFrame,
    src_addr: VirtAddr,
    dst_pml4: PhysFrame,
    dst_addr: VirtAddr,
    len: usize,
) -> Result<(), CopyError> {
    let src_table = unsafe { page_table_for(src_pml4) };
    let dst_table = unsafe { page_table_for(dst_pml4) };
    let offset = physical_memory_offset();

    let mut remaining = len;
    let mut src = src_addr;
    let mut dst = dst_addr;
    while remaining > 0 {
        let src_phys = src_table.translate_addr(src).ok_or(CopyError::SrcNotMapped)?;
        let dst_phys = dst_table.translate_addr(dst).ok_or(CopyError::DstNotMapped)?;

        let src_room = 4096 - (src.as_u64() as usize % 4096);
        let dst_room = 4096 - (dst.as_u64() as usize % 4096);
        let chunk = remaining.min(src_room).min(dst_room);

        unsafe {
            let src_ptr: *const u8 = (offset + src_phys.as_u64()).as_ptr();
            let dst_ptr: *mut u8 = (offset + dst_phys.as_u64()).as_mut_ptr();
            core::ptr::copy_nonoverlapping(src_ptr, dst_ptr, chunk);
        }

        remaining -= chunk;
        src += chunk as u64;
        dst += chunk as u64;
    }
    Ok(())
}

#[derive(Debug)]
pub enum CopyError {
    SrcNotMapped,
    DstNotMapped,
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

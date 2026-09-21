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
use spin::Mutex;
use x86_64::structures::paging::mapper::{MappedFrame, TranslateResult};
use x86_64::structures::paging::{
    FrameAllocator, FrameDeallocator, Mapper, OffsetPageTable, Page, PageTable, PageTableFlags,
    PhysFrame, Size4KiB, Translate,
};
use x86_64::{PhysAddr, VirtAddr};

/// The offset physical memory is mapped at, captured once by `init` so
/// later code (`page_table_for`, `copy_between_address_spaces`) can build
/// an `OffsetPageTable` over *any* address space's PML4, not just the
/// active one, without every caller having to thread the offset through.
/// Written once, at boot, before any other CPU-visible state depends on
/// it; read-only after.
static mut PHYSICAL_MEMORY_OFFSET: u64 = 0;

/// The one physical frame allocator, behind a lock so it can be reached
/// from anywhere -- not just `kernel_main`'s boot-time setup, which is all
/// that could allocate memory before this existed. `crate::calls::sys_fork`
/// is the first kernel call that needs to allocate at an arbitrary runtime
/// point, from whichever task happens to call it.
static FRAME_ALLOCATOR: Mutex<Option<BootInfoFrameAllocator>> = Mutex::new(None);

/// Install the global frame allocator. Called once from `kernel_main`,
/// right after the memory map is available.
///
/// # Safety
/// Same requirement as `BootInfoFrameAllocator::init`: the memory map must
/// be valid and every frame it marks `Usable` must really be unused.
pub unsafe fn init_frame_allocator(memory_map: &'static MemoryMap) {
    *FRAME_ALLOCATOR.lock() = Some(BootInfoFrameAllocator::init(memory_map));
}

/// A `FrameAllocator` handle that delegates to the global one -- for
/// passing to APIs (like `Mapper::map_to`) that need a
/// `&mut impl FrameAllocator<Size4KiB>` rather than a bare
/// `allocate_frame()` call.
pub struct GlobalFrameAllocator;

unsafe impl FrameAllocator<Size4KiB> for GlobalFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame> {
        FRAME_ALLOCATOR
            .lock()
            .as_mut()
            .expect("global frame allocator used before init_frame_allocator")
            .allocate_frame()
    }
}

impl FrameDeallocator<Size4KiB> for GlobalFrameAllocator {
    /// # Safety
    /// `frame` must be a frame this allocator handed out, no longer
    /// mapped in any address space and not referenced by any page table.
    unsafe fn deallocate_frame(&mut self, frame: PhysFrame) {
        FRAME_ALLOCATOR
            .lock()
            .as_mut()
            .expect("global frame allocator used before init_frame_allocator")
            .deallocate_frame(frame)
    }
}

/// Give `frame` back to the allocator. Free function rather than only the
/// `FrameDeallocator` impl, since most callers here (`free_address_space`)
/// have a bare frame rather than a `&mut impl FrameDeallocator`.
///
/// # Safety
/// Same as the trait method: the frame must be one this allocator handed
/// out, currently unmapped everywhere. Freeing a frame twice, or one
/// still reachable through some page table, corrupts unrelated memory.
pub unsafe fn deallocate_frame(frame: PhysFrame) {
    GlobalFrameAllocator.deallocate_frame(frame)
}

/// `(frames currently handed out, frames sitting on the free list)`.
/// Exists so a self-test can assert that a sequence of allocate-and-free
/// operations actually balances -- "no leak" is not observable any other
/// way in a kernel with no process accounting.
pub fn frame_stats() -> (usize, usize) {
    let guard = FRAME_ALLOCATOR.lock();
    let allocator = guard.as_ref().expect("frame allocator used before init_frame_allocator");
    (allocator.in_use, allocator.free_count)
}

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

pub(crate) fn physical_memory_offset() -> VirtAddr {
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
) -> (PhysFrame, OffsetPageTable<'static>) {
    let (active_frame, _) = x86_64::registers::control::Cr3::read();
    new_address_space_from(active_frame, physical_memory_offset)
        .expect("out of physical frames for a new address space's PML4")
}

/// `new_address_space`, but copying an explicitly named PML4 rather than
/// whichever one happens to be in `CR3` right now.
///
/// The distinction is invisible to every caller that runs in the kernel's
/// own address space (`kernel_main`, `crate::rs`) -- for them the active
/// table *is* the kernel's. It matters exactly once: `crate::calls::
/// sys_exec` runs inside a ring-3 caller's own trap, so `CR3` is that
/// caller's address space, complete with the user mappings of the image
/// exec is supposed to be throwing away. Copying *that* would hand the
/// new image its predecessor's pages, which is both wrong (exec must
/// start from a clean user address space) and unobservable-until-it-bites
/// (the new program would run fine; only the old image's pages
/// mysteriously surviving would show it). Passing `crate::proc::
/// kernel_cr3()` explicitly makes "which address space is this derived
/// from" a decision rather than an accident.
///
/// Returns `None` when there is no frame left for the new top-level
/// table, rather than panicking like `new_address_space` does. That
/// difference exists for the same reason the explicit `src_pml4` does:
/// the caller may be `sys_exec`, acting for a ring-3 process, and this
/// is the first of three allocations on that path -- the other two
/// (`crate::elf`'s segment and stack mappings) already degrade to
/// `ElfError::MappingFailed`. Frames do come back now
/// (`free_address_space`), but "running out" is still a state the system
/// can genuinely reach, and a ring-3 caller reaching it should lose its
/// `exec`, not the machine.
pub fn new_address_space_from(
    src_pml4: PhysFrame,
    physical_memory_offset: VirtAddr,
) -> Option<(PhysFrame, OffsetPageTable<'static>)> {
    let new_frame = GlobalFrameAllocator.allocate_frame()?;

    let src_ptr: *const PageTable =
        (physical_memory_offset + src_pml4.start_address().as_u64()).as_ptr();
    let new_ptr: *mut PageTable =
        (physical_memory_offset + new_frame.start_address().as_u64()).as_mut_ptr();
    unsafe {
        core::ptr::copy_nonoverlapping(src_ptr, new_ptr, 1);
        let table = &mut *new_ptr;
        Some((new_frame, OffsetPageTable::new(table, physical_memory_offset)))
    }
}

/// Whether every PML4 slot the range `start ..= end_inclusive` falls in
/// is *unused* in the address space rooted at `pml4`.
///
/// This is the precondition `new_address_space_from` describes and that
/// nothing previously checked. A new address space is only a copy of the
/// top-level table, so any slot the base table already uses is shared:
/// mapping into it does not create a private mapping, it reaches down
/// into the base address space's own lower-level tables and edits them.
/// For a hand-picked demo address (`crate::usermode`, `crate::elf`'s
/// built-in images) that was checkable by eye. For `crate::calls::
/// sys_exec`, whose addresses come out of a file a ring-3 caller chose,
/// it has to be checked for real -- and checking the *slot* is the only
/// correct form of the check, because "is this a kernel address?" has no
/// answer in terms of a single boundary here: this port's kernel lives in
/// the lower half (the kernel image near `0x20_0000`, the heap at
/// `crate::allocator::HEAP_START`, and the bootloader's
/// physical-memory window all sit at low PML4 indices), interleaved with
/// the addresses user images legitimately use.
pub fn pml4_slots_unused(pml4: PhysFrame, start: VirtAddr, end_inclusive: VirtAddr) -> bool {
    let offset = physical_memory_offset();
    // Safety: same contract as `page_table_for` -- `pml4` is a frame this
    // module handed out, reachable through the physical-memory window.
    let table: &PageTable = unsafe { &*((offset + pml4.start_address().as_u64()).as_ptr()) };
    let first = u16::from(start.p4_index());
    let last = u16::from(end_inclusive.p4_index());
    (first..=last).all(|i| table[i as usize].is_unused())
}

/// One contiguous run of pages a process privately owns: the unit a
/// `MemMap` is built out of.
///
/// The closest thing in the C tree is `include/minix/type.h`'s
/// `struct mem_map`, three of which (`mp_seg[T]`/`[D]`/`[S]` --  text,
/// data, stack) describe a MINIX process's whole memory layout in
/// `servers/pm/mproc.h`. That version carries a physical base as well as
/// a virtual one, because 2005-era MINIX has no paging: a segment *is* a
/// contiguous run of physical memory. Here the page tables already record
/// where each page physically lives, so a segment only needs to say which
/// virtual pages belong to the process -- the rest is a page-table walk
/// away (`fork_address_space`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Segment {
    pub base: VirtAddr,
    pub pages: usize,
}

/// How many segments one `MemMap` holds. Has to be at least one more
/// than the number of `PT_LOAD` segments `crate::elf` will load (it adds
/// a stack segment of its own on top); `crate::elf` asserts exactly that
/// at compile time, so an image that passes `elf::validate` can never
/// overflow a map.
pub const MAX_SEGMENTS: usize = 17;

/// Which pages a process's address space holds that are *its own* --
/// everything `fork()` has to duplicate and `exec()` throws away, as
/// opposed to the kernel mappings every address space shares.
///
/// This is the bookkeeping whose absence used to make `fork` a
/// special case per caller: `crate::syscall`'s `SYS_FORK` handler
/// hardcoded a list of pages for the one ring-3 task known to call it,
/// and refused anyone else (`ERR_FORK_UNSUPPORTED_CALLER`). Whoever
/// *builds* an address space knows this already -- `crate::elf`'s loader
/// has just walked the program headers, `crate::usermode` has just
/// mapped its two demo pages -- so the map is filled in there and
/// carried in the process table (`crate::proc::Proc::mem_map`) for
/// `fork` to read back.
#[derive(Clone, Copy)]
pub struct MemMap {
    segments: [Segment; MAX_SEGMENTS],
    len: usize,
}

impl MemMap {
    /// The map of a process with no private memory at all: every kernel
    /// task, which runs in the kernel's own address space.
    pub const EMPTY: MemMap =
        MemMap { segments: [Segment { base: VirtAddr::zero(), pages: 0 }; MAX_SEGMENTS], len: 0 };

    /// Record one more run of `pages` pages starting at `base`. `false`
    /// if the map is already full, which callers must treat as a failure
    /// to build the address space rather than ignore -- a map missing a
    /// segment is worse than no map, since `fork` would silently hand a
    /// child a page still shared with its parent.
    pub fn push(&mut self, base: VirtAddr, pages: usize) -> bool {
        if self.len >= MAX_SEGMENTS {
            return false;
        }
        self.segments[self.len] = Segment { base, pages };
        self.len += 1;
        true
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segments[..self.len]
    }

    pub fn total_pages(&self) -> usize {
        self.segments().iter().map(|s| s.pages).sum()
    }

    /// Every page this map covers, one segment after another.
    pub fn pages(&self) -> impl Iterator<Item = Page<Size4KiB>> + '_ {
        self.segments().iter().copied().flat_map(|seg| {
            (0..seg.pages as u64).map(move |i| Page::containing_address(seg.base + i * PAGE_SIZE))
        })
    }
}

const PAGE_SIZE: u64 = 4096;

/// Fork the address space rooted at `src_pml4` into a brand-new,
/// independent one: it shares `base_pml4`'s mappings (the kernel image,
/// the heap, the physical-memory window) the same way every address space
/// here does, and gets a freshly allocated, byte-for-byte copy of every
/// page in `map` -- so a write on either side is invisible to the other,
/// which is the whole substance of `fork()`.
///
/// Ported in spirit from `kernel/proc.c`'s `do_fork()` and the copy
/// `servers/pm/forkexit.c`'s `do_fork()` asks the kernel for: real MINIX
/// duplicates the parent's `mp_seg` memory map and then copies the
/// memory it describes, which (with no paging in that kernel) is exactly
/// a contiguous physical copy. `map` is the `mp_seg` equivalent, and
/// `crate::proc::mem_map_of` is where the caller gets it.
///
/// Copying *every* page in the map, rather than sharing the read-only
/// ones, is deliberate and was a correctness fix rather than a
/// simplification. An earlier version of this function built the child
/// out of the parent's own page tables -- duplicating table levels along
/// the path to each page it was told to copy, and leaving every other
/// leaf entry pointing at the parent's frames. Two things were wrong with
/// that. A page left shared has two address spaces referring to one
/// frame, and nothing in this port counts references, so the first
/// `free_address_space` of either side (an `exec`, a crash,
/// `crate::proc::kill`) would hand the *other* process's live page back
/// to the allocator. And duplicating table levels per page leaked one
/// table frame per level every time two copied pages shared a path --
/// which is every image whose text and data sit in the same 2 MiB
/// region. Sharing read-only pages is worth having back once frames are
/// reference-counted (it is what makes real `fork` cheap, and the first
/// step toward copy-on-write); until then, a full copy is the version
/// that is actually sound.
///
/// `None` if any part of it fails -- no frame left for a page or a page
/// table, a page in `map` that isn't actually mapped in the parent, or a
/// page whose PML4 slot `base_pml4` already uses (the
/// `pml4_slots_unused` rule: mapping there would edit the *base's* own
/// lower-level tables rather than the child's). A failed fork leaves
/// nothing behind: the partially built address space is torn back down
/// before returning.
pub fn fork_address_space(
    src_pml4: PhysFrame,
    base_pml4: PhysFrame,
    map: &MemMap,
) -> Option<PhysFrame> {
    let offset = physical_memory_offset();
    // Safety: same contract as `copy_between_address_spaces`, which
    // likewise holds two of these at once -- both are PML4 frames this
    // module handed out, reached through the physical-memory window, and
    // they are different address spaces (the parent's and a brand-new
    // one), so the `&mut` they each hand out don't alias.
    let src = unsafe { page_table_for(src_pml4) };
    let (child_pml4, mut child) = new_address_space_from(base_pml4, offset)?;

    match copy_pages_into(&src, &mut child, base_pml4, map, offset) {
        Ok(()) => Some(child_pml4),
        Err(()) => {
            // Safety: built here, never loaded into `CR3`, referenced by
            // nothing else -- and derived from `base_pml4`, so the
            // private/shared split `free_address_space` relies on holds.
            unsafe { free_address_space(child_pml4, base_pml4) };
            None
        }
    }
}

/// Give `child` its own copy of every page in `map`, reading the
/// originals out of `src`. Factored out of `fork_address_space` so that
/// one place can tear the half-built address space down on *any*
/// failure, rather than every `?` needing to remember to.
fn copy_pages_into(
    src: &OffsetPageTable<'static>,
    child: &mut OffsetPageTable<'static>,
    base_pml4: PhysFrame,
    map: &MemMap,
    offset: VirtAddr,
) -> Result<(), ()> {
    let mut allocator = GlobalFrameAllocator;
    for page in map.pages() {
        let start = page.start_address();
        if !pml4_slots_unused(base_pml4, start, start + (PAGE_SIZE - 1)) {
            return Err(());
        }
        let (original, flags) = match src.translate(start) {
            TranslateResult::Mapped { frame: MappedFrame::Size4KiB(frame), flags, .. } => {
                (frame, flags)
            }
            // Not mapped, or mapped by a huge page this port never
            // creates: either way the map disagrees with the page tables
            // it claims to describe, which is a bug to report rather than
            // paper over.
            _ => return Err(()),
        };
        let fresh = allocator.allocate_frame().ok_or(())?;
        // Safety: `original` is mapped in `src` (just translated) and
        // `fresh` was handed out by the allocator a line ago; both are
        // reachable through the physical-memory window, and 4 KiB is
        // exactly one frame.
        unsafe {
            core::ptr::copy_nonoverlapping(
                (offset + original.start_address().as_u64()).as_ptr::<u8>(),
                (offset + fresh.start_address().as_u64()).as_mut_ptr::<u8>(),
                PAGE_SIZE as usize,
            );
        }
        // Safety: `page` is unmapped in `child` (a fresh address space
        // whose only non-empty PML4 slots are `base_pml4`'s, which the
        // check above excluded) and `fresh` is not mapped anywhere else.
        // `.ignore()` rather than `.flush()`: this address space isn't in
        // `CR3`, so there's nothing cached to invalidate.
        match unsafe { child.map_to(page, fresh, flags, &mut allocator) } {
            Ok(flush) => flush.ignore(),
            Err(_) => {
                // Safety: allocated just above, never mapped anywhere.
                unsafe { deallocate_frame(fresh) };
                return Err(());
            }
        }
    }
    Ok(())
}

/// Give back every frame that belongs to the address space rooted at
/// `pml4` -- its user pages, the page-table levels that lead to them,
/// and finally the top-level table itself -- and return how many frames
/// that was.
///
/// "Belongs to" is decided by the same rule that makes a mapping private
/// in the first place (`pml4_slots_unused`): a PML4 slot used here but
/// *unused in `base_pml4`* was created by and for this address space, so
/// everything under it is ours to free. A slot the base also uses is
/// shared -- the kernel image, the heap, the physical-memory window --
/// and its lower-level tables are the *kernel's*, so walking into it
/// would hand the kernel's own page tables back to the allocator. That
/// asymmetry is the whole difficulty of freeing an address space in this
/// port, and it is why this takes `base_pml4` rather than working it out
/// alone.
///
/// This is what the old "every `exec` leaks the image it replaces, and
/// `kill` never reclaims a dead process" simplification was waiting on.
///
/// # Safety
/// `pml4` must not be loaded in `CR3` on any CPU, and nothing may reach
/// its pages afterward. Callers switch `CR3` away first
/// (`crate::proc::set_address_space`, `crate::proc::kill`).
pub unsafe fn free_address_space(pml4: PhysFrame, base_pml4: PhysFrame) -> usize {
    let offset = physical_memory_offset();
    let table: &PageTable = &*((offset + pml4.start_address().as_u64()).as_ptr());
    let base: &PageTable = &*((offset + base_pml4.start_address().as_u64()).as_ptr());

    let mut freed = 0;
    for i in 0..512 {
        if table[i].is_unused() || !base[i].is_unused() {
            continue; // empty, or shared with the base address space
        }
        if let Ok(frame) = table[i].frame() {
            freed += free_table(frame, 3);
        }
    }
    deallocate_frame(pml4);
    freed + 1
}

/// Free one page-table frame and everything below it. `level` counts
/// down: 3 = PDPT, 2 = PD, 1 = PT, whose entries are leaf pages.
///
/// # Safety
/// `table_frame` must be a page-table frame private to the address space
/// being torn down (see `free_address_space`), reachable through the
/// physical-memory window and referenced by nothing else.
unsafe fn free_table(table_frame: PhysFrame, level: u8) -> usize {
    let offset = physical_memory_offset();
    let table: &PageTable = &*((offset + table_frame.start_address().as_u64()).as_ptr());

    let mut freed = 0;
    for i in 0..512 {
        let entry = &table[i];
        if entry.is_unused() {
            continue;
        }
        // A huge-page entry is a leaf mapping of 2 MiB or 1 GiB, not a
        // pointer to a table. Nothing in a private address space maps
        // one today (every mapping here goes through 4 KiB `map_to`), and
        // freeing a large region as though it were one 4 KiB frame would
        // be actively wrong -- so skip it rather than guess.
        if entry.flags().contains(PageTableFlags::HUGE_PAGE) {
            continue;
        }
        let frame = match entry.frame() {
            Ok(frame) => frame,
            Err(_) => continue,
        };
        if level > 1 {
            freed += free_table(frame, level - 1);
        } else {
            deallocate_frame(frame); // a leaf: one of the process's own pages
            freed += 1;
        }
    }
    deallocate_frame(table_frame);
    freed + 1
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
    /// How far the bump cursor has advanced into `usable_frames()`. Only
    /// moves forward; frames that come back are reused from `free_list`
    /// instead.
    next: usize,
    /// Head of a free list threaded *through the free frames themselves*
    /// -- each one's first eight bytes hold the physical address of the
    /// next, or zero at the end.
    ///
    /// Storing the list inside the frames rather than in a `Vec` is not
    /// a micro-optimization: `crate::allocator::init_heap` allocates
    /// frames to map the heap, so anything the allocator needs has to
    /// work before a heap exists. A free frame is by definition memory
    /// nothing else is using, which makes it exactly the right place to
    /// keep the bookkeeping.
    free_list: Option<PhysFrame>,
    /// Frames handed out and not yet returned, and the length of
    /// `free_list` -- see `frame_stats`.
    in_use: usize,
    free_count: usize,
}

impl BootInfoFrameAllocator {
    /// # Safety
    /// The caller must guarantee `memory_map` is valid and that every
    /// frame it marks `Usable` really is unused.
    pub unsafe fn init(memory_map: &'static MemoryMap) -> Self {
        BootInfoFrameAllocator { memory_map, next: 0, free_list: None, in_use: 0, free_count: 0 }
    }

    fn usable_frames(&self) -> impl Iterator<Item = PhysFrame> {
        let regions = self.memory_map.iter();
        let usable = regions.filter(|r| r.region_type == MemoryRegionType::Usable);
        let addr_ranges = usable.map(|r| r.range.start_addr()..r.range.end_addr());
        let frame_addresses = addr_ranges.flat_map(|r| r.step_by(4096));
        frame_addresses.map(|addr| PhysFrame::containing_address(PhysAddr::new(addr)))
    }

    /// The `u64` inside `frame` that holds the next free-list link.
    fn link_of(frame: PhysFrame) -> *mut u64 {
        (physical_memory_offset() + frame.start_address().as_u64()).as_mut_ptr()
    }

    /// Wipe a frame before handing it out.
    ///
    /// This became necessary the moment frames started being reused. A
    /// bump allocator only ever returns memory nobody has touched; a
    /// recycling one returns memory that belonged to some *other
    /// process* moments ago -- `flaky`'s stack, the image `exec` just
    /// discarded -- and several callers map a fresh frame without
    /// writing all of it (`crate::elf`'s stack page,
    /// `crate::usermode`'s code page beyond `code.len()`). Left alone,
    /// a new ring-3 task would start life able to read a dead one's
    /// memory. Zeroing here rather than at those call sites makes it an
    /// invariant of the allocator instead of a rule every future caller
    /// has to remember -- and it also erases this allocator's own
    /// free-list link, which would otherwise be the first eight bytes
    /// of every recycled page.
    fn zero(frame: PhysFrame) {
        let ptr: *mut u8 = (physical_memory_offset() + frame.start_address().as_u64()).as_mut_ptr();
        // Safety: the frame is owned by this allocator and about to be
        // handed to a caller; nothing else refers to it.
        unsafe { core::ptr::write_bytes(ptr, 0, 4096) };
    }

    /// # Safety
    /// See `deallocate_frame` in this module.
    unsafe fn deallocate_frame(&mut self, frame: PhysFrame) {
        let next = self.free_list.map_or(0, |f| f.start_address().as_u64());
        Self::link_of(frame).write(next);
        self.free_list = Some(frame);
        self.free_count += 1;
        self.in_use = self.in_use.saturating_sub(1);
    }
}

unsafe impl FrameAllocator<Size4KiB> for BootInfoFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame> {
        // Reuse before bumping, so a workload that frees as much as it
        // allocates (every `exec`, every `rs` restart) runs forever in a
        // fixed footprint instead of marching through physical memory.
        // It also keeps `usable_frames().nth()` -- which is O(n) in the
        // cursor -- from being walked again for a frame already known.
        if let Some(frame) = self.free_list {
            // Safety: the frame is on the free list, so nothing else
            // holds it, and its first eight bytes are the link this
            // allocator wrote in `deallocate_frame`.
            let next = unsafe { Self::link_of(frame).read() };
            self.free_list = if next == 0 {
                None
            } else {
                Some(PhysFrame::containing_address(PhysAddr::new(next)))
            };
            self.free_count -= 1;
            self.in_use += 1;
            Self::zero(frame);
            return Some(frame);
        }

        let frame = self.usable_frames().nth(self.next);
        if let Some(frame) = frame {
            self.next += 1;
            self.in_use += 1;
            Self::zero(frame);
        }
        frame
    }
}

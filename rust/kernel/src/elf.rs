//! A minimal ELF64 loader: map a real, statically linked binary's
//! `PT_LOAD` segments into a fresh address space and run it in ring 3.
//!
//! No direct MINIX C equivalent: 2005-era MINIX 3.1 loads a program via
//! `execve`'s a.out-format path (`servers/pm/exec.c`, `lib/libsys`), not
//! ELF. This is groundwork for "a real, compiled user program can run,"
//! not a ported feature -- and it's a step up from `crate::usermode`'s
//! demo, which pokes a hand-assembled four-byte loop directly into a
//! single fixed page. Here, the code and data come from an actual ELF
//! file (`user/hello.elf`, built from `user/hello.s` -- see that file for
//! the exact `as`/`ld` invocation), parsed and mapped the way a real
//! loader must: possibly more than one segment, at whatever virtual
//! addresses the file specifies, with the file's own per-segment
//! read/write/execute permissions, and zero-filled past `p_filesz` up to
//! `p_memsz` (real BSS semantics) rather than assuming the file image and
//! the mapped size are the same thing.

use crate::memory::{self, GlobalFrameAllocator};
use crate::{com, fs, proc};
use alloc::vec::Vec;
use spin::Mutex;
use x86_64::structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB};
use x86_64::VirtAddr;

/// The demo program, built ahead of time from `user/hello.s` (see that
/// file's header comment for how) rather than assembled at kernel-build
/// time: there's no cross toolchain wired into this build yet to produce
/// a user-mode binary automatically (see `rust/README.md`'s roadmap).
pub static HELLO_ELF: &[u8] = include_bytes!("../user/hello.elf");

/// Fixed virtual address for this demo's one stack page, chosen clear of
/// `crate::usermode`'s own demo addresses (a different address space
/// entirely, so no real collision risk -- just kept distinct for
/// clarity) and of `HELLO_ELF`'s own linked addresses (see `user/hello.s`).
const STACK_ADDR: u64 = 0x_7777_7777_0000;
const PAGE_SIZE: u64 = 4096;

/// Where `user/hello.s`'s `counter` lives (matches its `--section-start`
/// build command). Exposed so a kernel task can `sys_vircopy` it back out
/// after the demo ends and confirm the loaded program's own code
/// genuinely executed and wrote to its mapped `.data` segment -- not just
/// that it trapped into the kernel the expected number of times.
pub const COUNTER_ADDR: u64 = 0x_5555_5556_0000;

const PT_LOAD: u32 = 1;
const PF_X: u32 = 1;
const PF_W: u32 = 2;

/// Mirrors `Elf64_Ehdr` (`/usr/include/elf.h`): fixed 64-byte header at
/// the start of every ELF64 file.
#[repr(C)]
struct Elf64Header {
    e_ident: [u8; 16],
    e_type: u16,
    e_machine: u16,
    e_version: u32,
    e_entry: u64,
    e_phoff: u64,
    e_shoff: u64,
    e_flags: u32,
    e_ehsize: u16,
    e_phentsize: u16,
    e_phnum: u16,
    e_shentsize: u16,
    e_shnum: u16,
    e_shstrndx: u16,
}

/// Mirrors `Elf64_Phdr`: one entry per segment the loader needs to act
/// on. Only `PT_LOAD` entries matter here -- others (`PT_PHDR`,
/// `PT_GNU_STACK`, ...) are informational or don't apply to this loader.
#[repr(C)]
struct Elf64ProgramHeader {
    p_type: u32,
    p_flags: u32,
    p_offset: u64,
    p_vaddr: u64,
    p_paddr: u64,
    p_filesz: u64,
    p_memsz: u64,
    p_align: u64,
}

/// Per-task `(entry, stack_top)`, keyed by `com::slot(proc_nr)`, for
/// `task_entry` to pick up once it's actually running as that task. A side
/// table rather than an argument because `crate::proc::spawn` takes a
/// plain `fn() -> !` with no way to pass per-task data in directly; a table
/// (rather than the single pair of globals this used to be) is what makes
/// more than one ELF-loaded task -- the boot-time demo *and* whatever
/// `spawn_from_fs` launches later -- able to coexist, each looking up its
/// own entry by `proc::current_proc_nr()` once its trampoline runs.
static ELF_TASK_PARAMS: Mutex<[(u64, u64); com::NR_BOOT_PROCS]> =
    Mutex::new([(0, 0); com::NR_BOOT_PROCS]);

/// Parse `image` and map each `PT_LOAD` segment into a freshly built
/// address space (`memory::new_address_space`), plus one stack page.
/// Records `proc_nr`'s entry point/stack top in `ELF_TASK_PARAMS` for
/// `task_entry` to find once it's spawned under that same `proc_nr`, and
/// returns the new address space's top-level page table frame, for
/// `crate::proc::spawn` to record as that task's `CR3` -- same shape as
/// `usermode::create_address_space`.
pub fn load(image: &[u8], proc_nr: i32) -> PhysFrame {
    let physical_memory_offset = memory::physical_memory_offset();
    assert!(image.len() >= core::mem::size_of::<Elf64Header>(), "ELF image too small");
    let header = unsafe { &*(image.as_ptr() as *const Elf64Header) };
    assert_eq!(&header.e_ident[0..4], b"\x7fELF", "not an ELF file");
    assert_eq!(header.e_ident[4], 2, "not a 64-bit ELF file (ELFCLASS64)");
    assert_eq!(header.e_ident[5], 1, "not a little-endian ELF file (ELFDATA2LSB)");

    let (pml4_frame, mut mapper) = memory::new_address_space(physical_memory_offset);
    let mut frame_allocator = GlobalFrameAllocator;

    let ph_entry_size = header.e_phentsize as usize;
    let ph_base = header.e_phoff as usize;
    for i in 0..header.e_phnum as usize {
        let ph_offset = ph_base + i * ph_entry_size;
        let ph = unsafe { &*(image[ph_offset..].as_ptr() as *const Elf64ProgramHeader) };
        if ph.p_type != PT_LOAD {
            continue;
        }
        load_segment(&mut mapper, &mut frame_allocator, image, ph, physical_memory_offset);
    }

    let stack_flags = PageTableFlags::PRESENT
        | PageTableFlags::WRITABLE
        | PageTableFlags::USER_ACCESSIBLE
        | PageTableFlags::NO_EXECUTE;
    let stack_page = Page::containing_address(VirtAddr::new(STACK_ADDR));
    let stack_frame = frame_allocator
        .allocate_frame()
        .expect("out of physical frames for the ELF demo's stack page");
    unsafe {
        mapper
            .map_to(stack_page, stack_frame, stack_flags, &mut frame_allocator)
            .expect("failed to map the ELF demo's stack page")
            .ignore();
    }

    ELF_TASK_PARAMS.lock()[com::slot(proc_nr)] = (header.e_entry, STACK_ADDR + PAGE_SIZE);
    pml4_frame
}

/// Map every page `ph` spans, zero it (so BSS -- the `p_memsz - p_filesz`
/// tail with no file backing -- reads as zero rather than leftover frame
/// contents), then copy in whatever part of `p_filesz` overlaps each
/// page. Done a page at a time rather than byte at a time so the overlap
/// arithmetic (a segment's first and last page are usually only
/// partially covered by its own permissions/content) only has to be
/// worked out once per page, not once per byte.
fn load_segment(
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut GlobalFrameAllocator,
    image: &[u8],
    ph: &Elf64ProgramHeader,
    physical_memory_offset: VirtAddr,
) {
    let flags = PageTableFlags::PRESENT
        | PageTableFlags::USER_ACCESSIBLE
        | if ph.p_flags & PF_W != 0 { PageTableFlags::WRITABLE } else { PageTableFlags::empty() }
        | if ph.p_flags & PF_X == 0 { PageTableFlags::NO_EXECUTE } else { PageTableFlags::empty() };

    let seg_start = ph.p_vaddr;
    let seg_file_end = ph.p_vaddr + ph.p_filesz;
    let seg_mem_end = ph.p_vaddr + ph.p_memsz;

    let start_page = Page::<Size4KiB>::containing_address(VirtAddr::new(seg_start));
    let end_page = Page::<Size4KiB>::containing_address(VirtAddr::new(seg_mem_end - 1));

    for page in Page::range_inclusive(start_page, end_page) {
        let frame = frame_allocator
            .allocate_frame()
            .expect("out of physical frames loading an ELF segment");
        unsafe {
            mapper
                .map_to(page, frame, flags, frame_allocator)
                .expect("failed to map an ELF segment page")
                .ignore();
        }

        let page_ptr = (physical_memory_offset + frame.start_address().as_u64()).as_mut_ptr::<u8>();
        unsafe { core::ptr::write_bytes(page_ptr, 0, PAGE_SIZE as usize) };

        let page_start = page.start_address().as_u64();
        let page_end = page_start + PAGE_SIZE;
        let copy_start = core::cmp::max(seg_start, page_start);
        let copy_end = core::cmp::min(seg_file_end, page_end);
        if copy_end > copy_start {
            let file_offset = (ph.p_offset + (copy_start - seg_start)) as usize;
            let len = (copy_end - copy_start) as usize;
            let page_offset = (copy_start - page_start) as usize;
            unsafe {
                core::ptr::copy_nonoverlapping(
                    image[file_offset..file_offset + len].as_ptr(),
                    page_ptr.add(page_offset),
                    len,
                );
            }
        }
    }
}

/// A `crate::proc` task body: jump to ring 3 at the real entry point
/// `load` recorded for whichever process number this task was spawned
/// under (`ELF_TASK_PARAMS`), on the stack `load` mapped for it. Reuses
/// `usermode::enter_ring3` -- the CPU-level part of "jump to ring 3"
/// doesn't care whether the code being jumped to came from a
/// hand-assembled byte array or a real ELF file, or which of possibly
/// several ELF-loaded tasks this happens to be.
pub fn task_entry() -> ! {
    let (code_sel, data_sel) = crate::gdt::user_selectors();
    let (entry, stack_top) = ELF_TASK_PARAMS.lock()[com::slot(proc::current_proc_nr())];
    unsafe { crate::usermode::enter_ring3(entry, stack_top, code_sel.0 as u64, data_sel.0 as u64) }
}

/// Longest chunk `spawn_from_fs` reads out of `fs` per round trip. Not a
/// limit on the file's own size -- `fs::read` is called in a loop until it
/// returns `<= 0`, so the whole file is assembled regardless of length;
/// this only bounds each individual request.
const READ_CHUNK: usize = 256;

/// Load `path` out of `fs` and spawn it as a fresh ring-3 task under
/// `proc_nr` -- the runtime counterpart to `kernel_main`'s boot-time
/// `load(HELLO_ELF, ...)` call, reusing the same loader and the same
/// `task_entry` trampoline so a launched app and the boot-time demo are
/// indistinguishable once running. `crate::rs` is the only caller: it's
/// the one place in this port that starts services, whether at boot (not
/// applicable here) or on request (`com::RS_LAUNCH_REQUEST`).
pub fn spawn_from_fs(
    path: &str,
    proc_nr: i32,
    name: &'static str,
    priority: u8,
    quantum: i32,
) -> Result<(), i64> {
    let fd = fs::open(path);
    if fd < 0 {
        return Err(fd);
    }
    let mut image: Vec<u8> = Vec::new();
    let mut chunk = [0u8; READ_CHUNK];
    loop {
        let n = fs::read(fd, &mut chunk);
        if n <= 0 {
            break;
        }
        image.extend_from_slice(&chunk[..n as usize]);
    }
    let address_space = load(&image, proc_nr);
    proc::spawn(proc_nr, name, task_entry, priority, quantum, true, Some(address_space));
    Ok(())
}

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

use crate::memory::{self, GlobalFrameAllocator, MemMap};
use crate::{com, fs, proc, serial_println};
use alloc::vec::Vec;
use spin::Mutex;
use x86_64::registers::control::Cr3;
use x86_64::structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB};
use x86_64::VirtAddr;

/// The demo program, built ahead of time from `user/hello.s` (see that
/// file's header comment for how) rather than assembled at kernel-build
/// time: there's no cross toolchain wired into this build yet to produce
/// a user-mode binary automatically (see `rust/README.md`'s roadmap).
pub static HELLO_ELF: &[u8] = include_bytes!("../user/hello.elf");

/// The `exec()` demo's *first* image (`user/shell.s`): a program whose
/// entire job is to replace itself with `ECHO_ELF` (see
/// `crate::calls::sys_exec`). Loaded at boot like `HELLO_ELF`.
pub static SHELL_ELF: &[u8] = include_bytes!("../user/shell.elf");

/// The `exec()` demo's *second* image (`user/echo.s`): what
/// `SHELL_ELF`'s process is running by the time anything looks at it.
/// Unlike the other two, this one is never loaded from a static byte
/// slice at boot -- `crate::main`'s `seed_bin` writes it into `fs` at
/// `/bin/echo`, and `sys_exec` reads it back out of the filesystem by
/// path, the way a real `exec` finds a program. It's `include_bytes!`d
/// here only to have something to install; nothing loads it directly.
pub static ECHO_ELF: &[u8] = include_bytes!("../user/echo.elf");

/// Fixed virtual address for this demo's one stack page, chosen clear of
/// `crate::usermode`'s own demo addresses (a different address space
/// entirely, so no real collision risk -- just kept distinct for
/// clarity) and of `HELLO_ELF`'s own linked addresses (see `user/hello.s`).
/// `pub` so `crate::syscall`'s `SYS_FORK` handler can list it as one of
/// `tty`'s private pages to deep-copy into a forked child (`fork` must
/// give the child its own stack, not one aliased with the parent's).
pub const STACK_ADDR: u64 = 0x_7777_7777_0000;
const PAGE_SIZE: u64 = 4096;

/// Where `user/hello.s`'s `counter` lives (matches its `--section-start`
/// build command). Exposed so a kernel task can `sys_vircopy` it back out
/// after the demo ends and confirm the loaded program's own code
/// genuinely executed and wrote to its mapped `.data` segment -- not just
/// that it trapped into the kernel the expected number of times. Also
/// the page that holds every other symbol below (`vircopy_buf`,
/// `err_result`, ...), all in the same 4 KiB page -- which is why
/// `tty`'s `SYS_FORK` copying it is what makes the forked child's own
/// canary write observable.
pub const COUNTER_ADDR: u64 = 0x_5555_5556_0000;

/// Where `user/hello.s`'s `vircopy_buf` lives -- the destination `tty`'s
/// own ring-3 `SYS_VIRCOPY` call (`crate::syscall`) writes into, and
/// where a kernel task can `sys_vircopy` it back out afterward to confirm
/// the copy genuinely landed in `tty`'s own address space. Read off the
/// built `hello.elf` with `nm` rather than computed, since it sits partway
/// through `.data` (after `counter`/`message`/`path`/... -- see
/// `user/hello.s`), not at the section's own link address like
/// `COUNTER_ADDR` is.
pub const VIRCOPY_BUF_ADDR: u64 = 0x_5555_5556_00de;

/// Where `user/hello.s`'s `err_result` lives -- `tty` stashes its
/// deliberately-invalid second `SYS_VIRCOPY` call's return value here
/// (an oversized `len`, expected to come back `crate::syscall::
/// ERR_BAD_LENGTH`), for a kernel task to `sys_vircopy` back out and
/// check afterward, proving the syscall ABI's distinct error codes
/// actually reach a ring-3 caller's `rax`, not just that `dispatch`
/// computes the right value internally. Read off the built `hello.elf`
/// with `nm`, same reasoning as `VIRCOPY_BUF_ADDR`.
pub const ERR_RESULT_ADDR: u64 = 0x_5555_5556_00fe;

/// Where `user/shell.s`'s `pre_exec_marker` lives (the first thing in its
/// `.data`, matching its `--section-start` build command). `crate::main`'s
/// `exec_verify` reads this address out of *both* processes involved in
/// the fork/exec pair and requires opposite answers: still
/// `0xfeedface` in `shell` itself, which never stopped running its own
/// image, and a failed read in the child, which inherited a copy of that
/// image and then had `exec` throw it away.
pub const SHELL_MARKER_ADDR: u64 = 0x_5555_5558_0000;

/// Where `user/echo.s`'s `counter` lives (the first thing in *its*
/// `.data`). `crate::main`'s `exec_verify` reads this back out of the
/// process that exec'd and checks it's `1` -- the new image's own
/// instructions having run, inside the caller's original process slot --
/// and out of that process's *parent*, where it must not be mapped at
/// all.
pub const ECHO_COUNTER_ADDR: u64 = 0x_6666_6667_0000;

/// Where `user/echo.s`'s `entry_rsp` lives (`ECHO_COUNTER_ADDR + 8`, read
/// off the built `echo.elf` with `nm`): the `rsp` the exec'd image found
/// on its first instruction, which is the address of the `argc` word
/// `write_initial_stack` put there. `crate::main`'s `exec_verify` reads
/// it, then walks the start-up block at that address in the child's own
/// memory.
pub const ECHO_ENTRY_RSP_ADDR: u64 = 0x_6666_6667_0008;

/// Where `user/echo.s`'s `argc_seen` lives (`ECHO_COUNTER_ADDR + 16`):
/// the `argc` the exec'd image itself read off its stack.
pub const ECHO_ARGC_ADDR: u64 = 0x_6666_6667_0010;

/// One past the last canonical lower-half address. A segment above this
/// is rejected outright -- but note this is only a *canonicality* bound,
/// not a user/kernel boundary. This port has no such boundary: the
/// kernel image, the heap (`crate::allocator::HEAP_START`) and the
/// bootloader's physical-memory window all live at low addresses,
/// interleaved with the ones user images use. What actually keeps a
/// loaded image off the kernel is the PML4-slot check in `validate`
/// (`memory::pml4_slots_unused`); see that function for why the slot,
/// not the address, is the thing worth checking.
const USER_SPACE_END: u64 = 0x_0000_8000_0000_0000;

/// Most `PT_LOAD` segments an image may have. Real binaries have a
/// handful; the bound exists so the pairwise overlap check in `validate`
/// stays cheap on a file that claims tens of thousands of them.
const MAX_LOAD_SEGMENTS: usize = 16;

/// Most program header table entries `validate` will walk at all, for
/// the same reason.
const MAX_PROGRAM_HEADERS: usize = 64;

/// Most pages an image (all its segments plus its stack) may occupy --
/// 4 MiB. `p_memsz` needs no file bytes behind it (that's what BSS is),
/// so without this a hundred-byte file can ask for terabytes and walk
/// `load_segment` straight through every free frame in the machine.
const MAX_IMAGE_PAGES: u64 = 1024;

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
static ELF_TASK_PARAMS: Mutex<[(u64, u64); com::NR_PROC_SLOTS]> =
    Mutex::new([(0, 0); com::NR_PROC_SLOTS]);

/// Everything a successfully loaded image consists of. `load` records
/// the last two fields in `ELF_TASK_PARAMS` for a task that hasn't
/// started yet; `crate::calls::sys_exec` writes them straight into a live
/// trap frame instead, which is why `load_image` returns them rather than
/// only stashing them.
pub struct LoadedImage {
    pub pml4: PhysFrame,
    pub entry: u64,
    /// The image's initial `rsp`: not the top of its stack page any more,
    /// but the address of `argc` in the System V start-up block
    /// `write_initial_stack` builds there (see `StartArgs`). Always
    /// 16-byte aligned, as the ABI requires of `rsp` on entry to `_start`.
    pub stack_pointer: u64,
    /// Which pages this image's segments and stack occupy, for the
    /// process table to carry (`crate::proc::AddressSpace`) so that a
    /// later `fork` of whoever runs it knows what to copy. Built here
    /// because this is where it's known: the program headers have just
    /// been walked, and nothing downstream can recover the layout from a
    /// bare PML4 without re-walking the page tables.
    pub map: MemMap,
}

/// A map has to be able to hold every `PT_LOAD` segment `validate` will
/// accept, plus the stack segment `load_image` adds on top -- otherwise
/// an image could load successfully with a segment missing from its map,
/// and a later `fork` would hand the child a page still shared with its
/// parent. Checked here, at compile time, rather than as a runtime
/// error `load_image` would have to report and a self-test would have to
/// cover.
const _: () = assert!(MAX_LOAD_SEGMENTS + 1 <= memory::MAX_SEGMENTS);

/// Why an image was rejected. Every variant is a check `load` used to
/// make with `assert!` (or not at all): fine when the only images in the
/// system were two `include_bytes!`d files the build produced, but
/// `crate::calls::sys_exec` now loads whatever bytes happen to be at a
/// path a *ring-3* caller named, so a malformed or hostile file has to
/// come back as an error to that caller rather than panicking the whole
/// kernel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElfError {
    /// Shorter than a single ELF64 header.
    TooSmall,
    /// No `\x7fELF` magic.
    NotElf,
    /// Not `ELFCLASS64`/`ELFDATA2LSB` -- some other ELF flavor entirely.
    NotElf64,
    /// The program header table (or one entry of it) falls outside the
    /// file, or `e_phentsize` is too small to hold an `Elf64_Phdr`.
    BadProgramHeaders,
    /// Nothing to load: no `PT_LOAD` segment, or one with `p_memsz == 0`.
    NoLoadableSegments,
    /// A segment's file contents (`p_offset + p_filesz`) run past the end
    /// of the image, or claim more file bytes than memory bytes.
    SegmentOutOfFile,
    /// A segment (or the entry point) asks to live past the canonical
    /// lower-half boundary (`USER_SPACE_END`).
    SegmentOutsideUserSpace,
    /// A segment lands in a PML4 slot the base address space already
    /// uses (`memory::pml4_slots_unused`). Such a mapping would not be
    /// private to the new address space at all: it would reach into the
    /// *kernel's* own lower-level page tables and modify them, since a
    /// new address space is only a copy of the top-level table. This is
    /// the check that actually keeps a chosen file off the kernel --
    /// `SegmentOutsideUserSpace` does not, because this port's kernel
    /// mappings are in the lower half too.
    SegmentInSharedSlot,
    /// A segment overlaps the fixed stack page this loader maps
    /// (`STACK_ADDR`), which would leave the image's own contents and its
    /// stack fighting over the same frame.
    SegmentOverlapsStack,
    /// Two `PT_LOAD` segments want pages that overlap. Not just a broken
    /// program: the second `map_to` of the same page fails, and this
    /// loader has no way to merge them.
    SegmentsOverlap,
    /// More `PT_LOAD` segments (or program headers) than this loader
    /// will consider -- see `MAX_LOAD_SEGMENTS`/`MAX_PROGRAM_HEADERS`.
    TooManySegments,
    /// The image's segments and stack would occupy more than
    /// `MAX_IMAGE_PAGES` pages. `p_memsz` needs no file bytes behind it,
    /// so this bound is what stops a tiny file from draining the frame
    /// allocator.
    ImageTooLarge,
    /// `argv`/`envp` (strings, their terminating NULs and the pointer
    /// vectors pointing at them) wouldn't fit in `MAX_START_ARGS_BYTES`.
    /// Checked before anything is allocated, like every other rejection
    /// here. POSIX's name for it is `E2BIG`.
    ArgsTooLarge,
    /// Mapping a page failed even though validation passed -- the frame
    /// allocator is empty, or the page turned out to be mapped already.
    /// Should be unreachable after `validate`; it exists so that being
    /// wrong about that returns an error to the caller instead of
    /// panicking the kernel.
    MappingFailed,
}

/// What a freshly started image finds on its stack: the argument and
/// environment vectors a real `execve` hands a program's `_start`. Each
/// string is given *without* its terminating NUL -- `write_initial_stack`
/// adds one -- and must not contain one either, since a C-style reader
/// would stop there (`crate::syscall`'s copy-in can't produce one: it
/// stops *at* the NUL).
///
/// `EMPTY` is what the images this port starts at boot get: `argc == 0`
/// and empty vectors, still laid out in full, so a program's `_start`
/// can read `argc` unconditionally whoever started it.
#[derive(Clone, Copy)]
pub struct StartArgs<'a> {
    pub argv: &'a [&'a [u8]],
    pub envp: &'a [&'a [u8]],
}

impl StartArgs<'_> {
    pub const EMPTY: StartArgs<'static> = StartArgs { argv: &[], envp: &[] };

    /// Bytes the start-up block occupies at the top of the stack page,
    /// before alignment padding: the strings (plus NULs), then `argc`,
    /// `argv[]` and `envp[]` (each NULL-terminated), then an auxiliary
    /// vector holding only its `AT_NULL` terminator (two words). `None`
    /// on overflow, which a caller treats the same as too large.
    fn block_size(&self) -> Option<usize> {
        let strings = self
            .argv
            .iter()
            .chain(self.envp.iter())
            .try_fold(0usize, |acc, s| acc.checked_add(s.len())?.checked_add(1))?;
        let words = 1usize
            .checked_add(self.argv.len())?
            .checked_add(1)?
            .checked_add(self.envp.len())?
            .checked_add(1)?
            .checked_add(2)?;
        strings.checked_add(words.checked_mul(8)?)
    }
}

/// Most of the one stack page (`STACK_ADDR`) the start-up block may
/// take. Half: whatever the arguments don't use is all the stack the
/// program has, and a program handed a page of arguments and no stack
/// would fault on its first `call`. Real systems call this `ARG_MAX`
/// (MINIX 3.1's is `ARG_MAX` in `include/limits.h`, 16 KiB, bounded the
/// same way by `servers/pm/exec.c`'s fixed-size `mbuf`).
pub const MAX_START_ARGS_BYTES: usize = (PAGE_SIZE as usize) / 2;

/// Lay out `args` at the top of the stack page whose kernel-visible
/// (physical-memory-window) address is `page`, as the System V x86-64
/// ABI's process start-up convention has it, and return the new image's
/// initial `rsp` -- the counterpart of what `servers/pm/exec.c`'s
/// `do_exec` builds in `mbuf` and `patch_ptr`s before copying it to the
/// top of the new stack. From `rsp` upward:
///
/// ```text
///   argc
///   argv[0] .. argv[argc-1], NULL
///   envp[0] .. envp[envc-1], NULL
///   AT_NULL, 0                 (an empty auxiliary vector)
///   (padding)
///   argv strings, envp strings, each NUL-terminated
/// ```
///
/// Every pointer is a *user* address (inside the page at `STACK_ADDR`),
/// computed rather than copied, because the page is being written
/// through a kernel mapping that the program will never see.
/// `rsp % 16 == 0`, the ABI's requirement at `_start`.
///
/// The caller has already checked `args.block_size()` against
/// `MAX_START_ARGS_BYTES`, so everything here fits.
fn write_initial_stack(page: *mut u8, args: &StartArgs) -> u64 {
    // Offsets from the page's start; the user address of offset `o` is
    // `STACK_ADDR + o`. The strings occupy the top of the page in the
    // order they're listed (argv, then envp), so the start of each one is
    // known from the lengths of those before it, and the vectors can be
    // filled in the same pass that copies the strings -- no scratch
    // storage for pointers, on a kernel stack that has little to spare.
    let strings: usize = args.argv.iter().chain(args.envp.iter()).map(|s| s.len() + 1).sum();
    let words = 1 + (args.argv.len() + 1) + (args.envp.len() + 1) + 2;
    let strings_start = PAGE_SIZE as usize - strings;
    let rsp_offset = (strings_start - words * 8) & !0xf;

    // Safety (both closures): every offset written lies inside the page,
    // since the whole block fits (see the doc comment).
    let put_word = |i: usize, value: u64| unsafe {
        (page.add(rsp_offset + i * 8) as *mut u64).write_unaligned(value)
    };
    let mut next_string = strings_start;
    let mut put_string = |s: &[u8]| -> u64 {
        let at = next_string;
        unsafe {
            core::ptr::copy_nonoverlapping(s.as_ptr(), page.add(at), s.len());
            // The frame was zeroed on hand-out, but the layout shouldn't
            // lean on that.
            *page.add(at + s.len()) = 0;
        }
        next_string += s.len() + 1;
        STACK_ADDR + at as u64
    };

    let mut i = 0;
    put_word(i, args.argv.len() as u64); // argc
    i += 1;
    for s in args.argv.iter() {
        put_word(i, put_string(s));
        i += 1;
    }
    put_word(i, 0); // argv[argc] == NULL
    i += 1;
    for s in args.envp.iter() {
        put_word(i, put_string(s));
        i += 1;
    }
    put_word(i, 0); // envp terminator
    i += 1;
    put_word(i, 0); // auxv: AT_NULL ...
    put_word(i + 1, 0); // ... and its (unused) value

    STACK_ADDR + rsp_offset as u64
}

/// Read the fixed-size ELF header out of `image`.
///
/// `read_unaligned`, not a reference cast: an image loaded by `sys_exec`
/// is a heap `Vec<u8>` assembled from `fs` reads, with no alignment
/// guarantee at all, and a misaligned `&Elf64Header` would be undefined
/// behavior even on x86, where the load itself happens to work.
fn header_of(image: &[u8]) -> Result<Elf64Header, ElfError> {
    if image.len() < core::mem::size_of::<Elf64Header>() {
        return Err(ElfError::TooSmall);
    }
    let header = unsafe { (image.as_ptr() as *const Elf64Header).read_unaligned() };
    if &header.e_ident[0..4] != b"\x7fELF" {
        return Err(ElfError::NotElf);
    }
    if header.e_ident[4] != 2 || header.e_ident[5] != 1 {
        return Err(ElfError::NotElf64);
    }
    Ok(header)
}

/// Read program header `i`, bounds-checking it against the image first
/// (same `read_unaligned` reasoning as `header_of`).
fn program_header(
    image: &[u8],
    header: &Elf64Header,
    i: usize,
) -> Result<Elf64ProgramHeader, ElfError> {
    let entry_size = core::mem::size_of::<Elf64ProgramHeader>();
    if (header.e_phentsize as usize) < entry_size {
        return Err(ElfError::BadProgramHeaders);
    }
    let offset = (header.e_phoff as usize)
        .checked_add(i * header.e_phentsize as usize)
        .ok_or(ElfError::BadProgramHeaders)?;
    let end = offset.checked_add(entry_size).ok_or(ElfError::BadProgramHeaders)?;
    if end > image.len() {
        return Err(ElfError::BadProgramHeaders);
    }
    Ok(unsafe { (image.as_ptr().add(offset) as *const Elf64ProgramHeader).read_unaligned() })
}

/// Check every `PT_LOAD` segment before a single frame is allocated, so
/// a rejected image leaves nothing behind to clean up (this port has no
/// frame deallocator -- see `crate::proc::set_address_space`). See
/// `ElfError` for what each check is defending against.
///
/// `base_pml4` is needed because the most important check here is not
/// about the address at all, but about which PML4 slot it falls in: a
/// new address space copies only the top-level table, so a segment in a
/// slot the base already uses would be written into the *base's* own
/// lower-level tables (`memory::pml4_slots_unused`). Everything else --
/// bounds, overlaps, size -- exists so that `load_segment` cannot fail
/// once it starts.
fn validate(image: &[u8], header: &Elf64Header, base_pml4: PhysFrame) -> Result<(), ElfError> {
    if header.e_entry >= USER_SPACE_END {
        return Err(ElfError::SegmentOutsideUserSpace);
    }
    if header.e_phnum as usize > MAX_PROGRAM_HEADERS {
        return Err(ElfError::TooManySegments);
    }

    // Page ranges claimed so far, as inclusive `(first, last)` page
    // numbers, starting with the stack page this loader always maps --
    // so a segment colliding with the stack and a segment colliding with
    // another segment are the same check, made once.
    let stack_page = STACK_ADDR / PAGE_SIZE;
    let mut claimed: [(u64, u64); MAX_LOAD_SEGMENTS + 1] = [(0, 0); MAX_LOAD_SEGMENTS + 1];
    claimed[0] = (stack_page, stack_page);
    let mut claimed_len = 1;
    let mut total_pages: u64 = 1; // the stack page

    if !memory::pml4_slots_unused(
        base_pml4,
        VirtAddr::new(STACK_ADDR),
        VirtAddr::new(STACK_ADDR + PAGE_SIZE - 1),
    ) {
        return Err(ElfError::SegmentInSharedSlot);
    }

    for i in 0..header.e_phnum as usize {
        let ph = program_header(image, header, i)?;
        if ph.p_type != PT_LOAD || ph.p_memsz == 0 {
            continue;
        }
        if claimed_len > MAX_LOAD_SEGMENTS {
            return Err(ElfError::TooManySegments);
        }

        if ph.p_filesz > ph.p_memsz {
            return Err(ElfError::SegmentOutOfFile);
        }
        let file_end = ph.p_offset.checked_add(ph.p_filesz).ok_or(ElfError::SegmentOutOfFile)?;
        if file_end > image.len() as u64 {
            return Err(ElfError::SegmentOutOfFile);
        }

        let mem_end =
            ph.p_vaddr.checked_add(ph.p_memsz).ok_or(ElfError::SegmentOutsideUserSpace)?;
        if mem_end > USER_SPACE_END {
            return Err(ElfError::SegmentOutsideUserSpace);
        }

        // The real gate: would mapping this reach into the base address
        // space's own page tables?
        if !memory::pml4_slots_unused(
            base_pml4,
            VirtAddr::new(ph.p_vaddr),
            VirtAddr::new(mem_end - 1),
        ) {
            return Err(ElfError::SegmentInSharedSlot);
        }

        let first_page = ph.p_vaddr / PAGE_SIZE;
        let last_page = (mem_end - 1) / PAGE_SIZE;
        total_pages = total_pages
            .checked_add(last_page - first_page + 1)
            .ok_or(ElfError::ImageTooLarge)?;
        if total_pages > MAX_IMAGE_PAGES {
            return Err(ElfError::ImageTooLarge);
        }

        // Page-granular, not byte-granular: two segments sharing one page
        // without their bytes overlapping still means mapping that page
        // twice, which is exactly what `load_segment` cannot do.
        for &(other_first, other_last) in claimed[..claimed_len].iter() {
            if first_page <= other_last && last_page >= other_first {
                return Err(if other_first == stack_page && other_last == stack_page {
                    ElfError::SegmentOverlapsStack
                } else {
                    ElfError::SegmentsOverlap
                });
            }
        }
        claimed[claimed_len] = (first_page, last_page);
        claimed_len += 1;
    }

    if claimed_len == 1 {
        return Err(ElfError::NoLoadableSegments);
    }
    Ok(())
}

/// Parse `image` and map each `PT_LOAD` segment into a brand-new address
/// space derived from `base_pml4` (`memory::new_address_space_from`),
/// plus one stack page, and return where to start it.
///
/// `base_pml4` is explicit rather than "whatever is in `CR3`" because
/// `sys_exec` runs inside a ring-3 caller's own trap, where the active
/// address space is the one being discarded -- see
/// `memory::new_address_space_from`'s doc comment for why copying that
/// one would quietly hand the new image its predecessor's pages.
pub fn load_image(base_pml4: PhysFrame, image: &[u8]) -> Result<LoadedImage, ElfError> {
    load_image_with_args(base_pml4, image, &StartArgs::EMPTY)
}

/// `load_image`, additionally handing the new image an argument and
/// environment vector (`StartArgs`) on its stack -- what `exec` needs,
/// and the only thing that distinguishes it from loading a program at
/// boot. `args` is checked alongside the image itself, before anything
/// is allocated, so an oversized vector costs nothing but the error.
pub fn load_image_with_args(
    base_pml4: PhysFrame,
    image: &[u8],
    args: &StartArgs,
) -> Result<LoadedImage, ElfError> {
    let physical_memory_offset = memory::physical_memory_offset();
    let header = header_of(image)?;
    match args.block_size() {
        Some(size) if size <= MAX_START_ARGS_BYTES => {}
        _ => return Err(ElfError::ArgsTooLarge),
    }
    validate(image, &header, base_pml4)?;

    let (pml4_frame, mut mapper) =
        memory::new_address_space_from(base_pml4, physical_memory_offset)
            .ok_or(ElfError::MappingFailed)?;
    let mut frame_allocator = GlobalFrameAllocator;

    let mut map = MemMap::EMPTY;
    for i in 0..header.e_phnum as usize {
        let ph = program_header(image, &header, i)?;
        if ph.p_type != PT_LOAD || ph.p_memsz == 0 {
            continue;
        }
        load_segment(&mut mapper, &mut frame_allocator, image, &ph, physical_memory_offset)?;
        // Page-granular, matching what `load_segment` actually mapped
        // (and what `validate` counted): a segment's first and last
        // pages usually extend past its own byte range.
        let first_page = ph.p_vaddr / PAGE_SIZE;
        let last_page = (ph.p_vaddr + ph.p_memsz - 1) / PAGE_SIZE;
        let pages = (last_page - first_page + 1) as usize;
        if !map.push(VirtAddr::new(first_page * PAGE_SIZE), pages) {
            // Unreachable given the compile-time assertion above, since
            // `validate` has already bounded the segment count -- an
            // error rather than a panic for the same reason
            // `MappingFailed` is one (the caller may be ring 3).
            return Err(ElfError::TooManySegments);
        }
    }

    let stack_flags = PageTableFlags::PRESENT
        | PageTableFlags::WRITABLE
        | PageTableFlags::USER_ACCESSIBLE
        | PageTableFlags::NO_EXECUTE;
    let stack_page = Page::containing_address(VirtAddr::new(STACK_ADDR));
    let stack_frame = frame_allocator.allocate_frame().ok_or(ElfError::MappingFailed)?;
    unsafe {
        mapper
            .map_to(stack_page, stack_frame, stack_flags, &mut frame_allocator)
            .map_err(|_| ElfError::MappingFailed)?
            .ignore();
    }
    let stack_page_ptr: *mut u8 =
        (physical_memory_offset + stack_frame.start_address().as_u64()).as_mut_ptr();
    let stack_pointer = write_initial_stack(stack_page_ptr, args);

    if !map.push(VirtAddr::new(STACK_ADDR), 1) {
        return Err(ElfError::TooManySegments);
    }

    Ok(LoadedImage {
        pml4: pml4_frame,
        entry: header.e_entry,
        stack_pointer,
        map,
    })
}

/// `load_image` for a task that hasn't been spawned yet: records
/// `proc_nr`'s entry point/stack top in `ELF_TASK_PARAMS` for
/// `task_entry` to find once it's spawned under that same `proc_nr`, and
/// returns the new address space (page table plus memory map) for
/// `crate::proc::spawn` to record -- same shape as
/// `usermode::create_address_space`.
///
/// Both callers run in the kernel's own address space (`kernel_main`,
/// and `spawn_from_fs` below from a kernel task), so deriving from the
/// active `CR3` is the same thing as deriving from the kernel's; only
/// `sys_exec`, running inside a ring-3 caller's trap, has to say which
/// it means. The images loaded this way are the ones built into the
/// kernel binary, so a malformed one really is a build bug and panicking
/// is the right answer -- unlike `sys_exec`'s, which come from `fs`.
pub fn load(image: &[u8], proc_nr: i32) -> proc::AddressSpace {
    let (active_pml4, _) = Cr3::read();
    let loaded = load_image(active_pml4, image).expect("failed to load a built-in ELF image");
    record_params(proc_nr, &loaded);
    proc::AddressSpace { pml4: loaded.pml4, map: loaded.map }
}

/// Remember where `task_entry` should start `proc_nr` once it's spawned.
fn record_params(proc_nr: i32, loaded: &LoadedImage) {
    ELF_TASK_PARAMS.lock()[com::slot(proc_nr)] = (loaded.entry, loaded.stack_pointer);
}

/// Largest program `read_file` will pull out of `fs`. `MAX_IMAGE_PAGES`
/// cannot serve here: it bounds what a *validated* image may map, which
/// is only checked after the whole file is already sitting in the heap
/// -- and at 4 MiB it is larger than the heap itself
/// (`crate::allocator::HEAP_SIZE`, 1 MiB), so an oversized file would
/// exhaust the allocator before anything got to reject it. `fs` puts no
/// limit on how much a ring-3 task can write into a file
/// (`SYS_FS_WRITE` bounds one call at 256 bytes, not the file), so the
/// limit has to live here. Generous next to the ~9.5 KiB images this
/// port actually loads.
const MAX_IMAGE_BYTES: usize = 256 * 1024;

/// "File too large" -- `read_file`'s answer to a file bigger than
/// `MAX_IMAGE_BYTES`, sharing the numbering of the `fs` error codes it
/// is returned alongside (POSIX `EFBIG`).
pub const EFBIG: i64 = -27;

/// "Exec format error" -- what `spawn_from_fs` reports when the file it
/// read is not a program this loader can run. Shares the numbering of
/// the `fs` error codes it's returned alongside (`crate::fs`), since
/// both end up in the same `Result<(), i64>`, and matches POSIX's own
/// `ENOEXEC` for exactly this condition.
pub const ENOEXEC: i64 = -8;

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
) -> Result<(), ElfError> {
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
        // Both of these are `validate`'s job to have made impossible --
        // an image small enough to fit `MAX_IMAGE_PAGES`, in PML4 slots
        // nothing else uses, with no two segments sharing a page. They
        // return an error rather than panicking anyway: the caller may be
        // ring 3 (`crate::calls::sys_exec`), and a mistake in the
        // reasoning above should cost that caller its `exec`, not cost
        // the machine its kernel.
        let frame = frame_allocator.allocate_frame().ok_or(ElfError::MappingFailed)?;
        unsafe {
            mapper
                .map_to(page, frame, flags, frame_allocator)
                .map_err(|_| ElfError::MappingFailed)?
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
    Ok(())
}

/// Assemble a syntactically well-formed ELF64 image whose `PT_LOAD`
/// segments are `(p_vaddr, p_memsz)` pairs with no file contents, for
/// `validator_self_test` below. Deliberately hand-built rather than
/// produced by `as`/`ld`: the point is to express images a linker would
/// never emit.
fn synthetic_image(entry: u64, segments: &[(u64, u64)]) -> Vec<u8> {
    let mut image: Vec<u8> = Vec::new();
    image.extend_from_slice(b"\x7fELF\x02\x01\x01\x00");
    image.extend_from_slice(&[0u8; 8]); // e_ident padding
    image.extend_from_slice(&2u16.to_le_bytes()); // e_type = ET_EXEC
    image.extend_from_slice(&0x3eu16.to_le_bytes()); // e_machine = x86-64
    image.extend_from_slice(&1u32.to_le_bytes()); // e_version
    image.extend_from_slice(&entry.to_le_bytes());
    image.extend_from_slice(&64u64.to_le_bytes()); // e_phoff
    image.extend_from_slice(&0u64.to_le_bytes()); // e_shoff
    image.extend_from_slice(&0u32.to_le_bytes()); // e_flags
    image.extend_from_slice(&64u16.to_le_bytes()); // e_ehsize
    image.extend_from_slice(&56u16.to_le_bytes()); // e_phentsize
    image.extend_from_slice(&(segments.len() as u16).to_le_bytes()); // e_phnum
    image.extend_from_slice(&0u16.to_le_bytes()); // e_shentsize
    image.extend_from_slice(&0u16.to_le_bytes()); // e_shnum
    image.extend_from_slice(&0u16.to_le_bytes()); // e_shstrndx
    for &(vaddr, memsz) in segments {
        image.extend_from_slice(&PT_LOAD.to_le_bytes()); // p_type
        image.extend_from_slice(&(PF_W | 4).to_le_bytes()); // p_flags = RW
        image.extend_from_slice(&0u64.to_le_bytes()); // p_offset
        image.extend_from_slice(&vaddr.to_le_bytes()); // p_vaddr
        image.extend_from_slice(&vaddr.to_le_bytes()); // p_paddr
        image.extend_from_slice(&0u64.to_le_bytes()); // p_filesz -- pure BSS
        image.extend_from_slice(&memsz.to_le_bytes()); // p_memsz
        image.extend_from_slice(&PAGE_SIZE.to_le_bytes()); // p_align
    }
    image
}

/// Feed `load_image` the images a hostile `exec` would use and require
/// each one to come back as the right `ElfError`.
///
/// This exists because the first version of this loader's validation was
/// wrong in a way that testing the happy path could never have shown:
/// it bounded segments against `USER_SPACE_END` and called that "can't
/// map over the kernel", when in this port every kernel mapping is
/// *below* that line, and a new address space shares the base's
/// lower-level page tables for any PML4 slot the base already uses. An
/// image asking to be loaded at `crate::allocator::HEAP_START` was
/// accepted, and `map_to` then edited the kernel's own page tables.
/// Checking that legitimate programs still load says nothing about any
/// of that; only the rejections are evidence.
///
/// Every case below is rejected, so nothing here allocates a frame or
/// builds an address space -- safe to run from `kernel_main`, before the
/// scheduler exists.
pub fn validator_self_test() {
    let (base, _) = Cr3::read();
    let user_slot = 0x_5555_5559_0000u64; // a PML4 slot the kernel doesn't use

    let mut truncated = synthetic_image(user_slot, &[(user_slot, PAGE_SIZE)]);
    truncated.truncate(70); // header plus a fragment of one program header

    // Long enough to reach the magic check -- a short file is caught by
    // the length guard in `header_of` first, which would make a
    // "not an ELF" case silently test something else. (That is exactly
    // what the first version of this table did: its shell script was 46
    // bytes, so it asserted `TooSmall` under a `NotElf` label, and
    // deleting the magic check would not have failed a single case.)
    let mut script = Vec::from(&b"#!/bin/sh\necho this is a script, not a program\n"[..]);
    script.resize(200, b' ');

    let mut wrong_class = synthetic_image(user_slot, &[(user_slot, PAGE_SIZE)]);
    wrong_class[4] = 1; // ELFCLASS32

    let cases: [(&str, Vec<u8>, ElfError); 11] = [
        (
            "a segment on top of the kernel heap",
            synthetic_image(user_slot, &[(crate::allocator::HEAP_START as u64, PAGE_SIZE)]),
            ElfError::SegmentInSharedSlot,
        ),
        (
            "a segment on top of the kernel image",
            synthetic_image(user_slot, &[(0x20_0000, PAGE_SIZE)]),
            ElfError::SegmentInSharedSlot,
        ),
        (
            "two segments sharing one page",
            synthetic_image(user_slot, &[(user_slot, 0x10), (user_slot + 0x100, 0x10)]),
            ElfError::SegmentsOverlap,
        ),
        (
            "a segment on top of the stack page",
            synthetic_image(user_slot, &[(STACK_ADDR, PAGE_SIZE)]),
            ElfError::SegmentOverlapsStack,
        ),
        (
            "4 GiB of BSS from a 120-byte file",
            synthetic_image(user_slot, &[(user_slot, 0x1_0000_0000)]),
            ElfError::ImageTooLarge,
        ),
        (
            "a segment past the canonical boundary",
            synthetic_image(user_slot, &[(USER_SPACE_END, PAGE_SIZE)]),
            ElfError::SegmentOutsideUserSpace,
        ),
        (
            "no loadable segments at all",
            synthetic_image(user_slot, &[]),
            ElfError::NoLoadableSegments,
        ),
        ("a program header table past the end of the file", truncated, ElfError::BadProgramHeaders),
        ("a shell script long enough to reach the magic check", script, ElfError::NotElf),
        ("a 32-bit ELF", wrong_class, ElfError::NotElf64),
        ("a file too short to hold an ELF header", Vec::from(&b"#!/bin/sh\n"[..]), ElfError::TooSmall),
    ];

    for (name, image, expected) in cases.iter() {
        match load_image(base, image) {
            Err(actual) if actual == *expected => {
                serial_println!("[elf] rejected {}: {:?}", name, actual);
            }
            Err(actual) => panic!("{}: expected {:?}, got {:?}", name, expected, actual),
            Ok(_) => panic!("{}: was ACCEPTED -- the loader would have mapped it", name),
        }
    }
    serial_println!("[elf] validator self-test: all {} hostile images rejected", cases.len());

    // An otherwise perfectly good image with more argument bytes than
    // the stack page will give up: refused as a whole, before any of it
    // is mapped. (`crate::syscall`'s copy-in stops most such vectors
    // earlier, but this is the check that holds whoever the caller is.)
    let good = synthetic_image(user_slot, &[(user_slot, PAGE_SIZE)]);
    let big = [0x41u8; 256];
    let many: [&[u8]; 8] = [&big; 8]; // 8 * 257 bytes of strings alone
    let args = StartArgs { argv: &many, envp: &[] };
    match load_image_with_args(base, &good, &args) {
        Err(ElfError::ArgsTooLarge) => {
            serial_println!("[elf] rejected an argv larger than MAX_START_ARGS_BYTES: ArgsTooLarge");
        }
        Err(other) => panic!("oversized argv: expected ArgsTooLarge, got {:?}", other),
        Ok(_) => panic!("oversized argv was ACCEPTED -- it would have left the program no stack"),
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
    let image = read_file(path)?;
    // Unlike `load` above, this image came out of `fs` rather than out
    // of the kernel binary, so a malformed one is a bad file, not a bad
    // build: report it like any other failure to start a service rather
    // than bringing the kernel down over it.
    let loaded = load_image(proc::kernel_cr3(), &image).map_err(|_| ENOEXEC)?;
    record_params(proc_nr, &loaded);
    let space = proc::AddressSpace { pml4: loaded.pml4, map: loaded.map };
    proc::spawn(proc_nr, name, task_entry, priority, quantum, true, Some(space));
    Ok(())
}

/// Read an entire file out of `fs` into a heap buffer, or return `fs`'s
/// own negative error code (`ENOENT`, `EISDIR`, ...) unchanged. Shared by
/// `spawn_from_fs` (start a *new* process running this program) and
/// `crate::calls::sys_exec` (make an *existing* process start running it
/// instead of what it was) -- the two halves of "get a program off the
/// filesystem", which differ only in what they do with the bytes.
///
/// Must be called from a real task: `fs::open`/`fs::read` block on IPC
/// round trips, which needs a task context to block in.
pub fn read_file(path: &str) -> Result<Vec<u8>, i64> {
    // `open_existing`, not `open`: `fs::open` creates the file when it
    // isn't there, so asking for a program that doesn't exist would
    // otherwise succeed, hand back zero bytes, get rejected as a
    // malformed image (`ElfError::TooSmall` -> `ERR_BAD_ELF`), and leave
    // a stray empty file behind at the caller's chosen path. A missing
    // program is `ENOENT`, and a failed `exec` changes nothing.
    let fd = fs::open_existing(path);
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
        // Checked before appending, not after: the point is never to
        // hold more than this in the heap, and there is no allocation
        // error handler to catch it if we do (an over-large `Vec` growth
        // aborts).
        if image.len() + n as usize > MAX_IMAGE_BYTES {
            return Err(EFBIG);
        }
        image.extend_from_slice(&chunk[..n as usize]);
    }
    Ok(image)
}

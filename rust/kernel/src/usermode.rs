//! Entering ring 3 (user mode) as a real, schedulable task with its own
//! address space.
//!
//! This is the third slice of the "user-mode processes and address-space
//! isolation" roadmap item in `rust/README.md`, building on the first two
//! (proving the CPU mechanism worked at all, as a one-shot,
//! not-scheduler-integrated demo; then making it a real, asynchronously-
//! preemptible task, but still sharing the kernel's own address space).
//! This time, the demo task's code and stack pages are mapped into a
//! *separate* page table (`crate::memory::new_address_space`) that the
//! kernel's own mapper never sees -- real isolation, not just a CPU
//! privilege level. `crate::proc` switches `CR3` to this task's address
//! space on every switch to it, the same way it already switches `RSP0`.
//!
//! There's still no MINIX C file to port here, for the same reason
//! `crate::memory` doesn't have one: this is x86_64-specific groundwork
//! (2005 i386 MINIX used segment-based protection, not paged address
//! spaces the way this does), not a ported feature.

use crate::gdt;
use crate::memory::{self, GlobalFrameAllocator, MemMap};
use crate::proc::AddressSpace;
use crate::syscall;
use x86_64::structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags};
use x86_64::VirtAddr;

/// Arbitrary, fixed addresses for the demo's one code page and one stack
/// page. They only need to not collide with whatever the *kernel's*
/// address space already uses (the heap, its own code/data, the
/// physical-memory window) -- not with any other task's, since each
/// gets its own separate page table now. Picked far from
/// `allocator::HEAP_START` for the same reason as before.
pub const USER_CODE_ADDR: u64 = 0x_5555_5555_0000;
pub const USER_STACK_ADDR: u64 = 0x_6666_6666_0000;
pub(crate) const PAGE_SIZE: u64 = 4096;

/// Three real syscalls -- `SYS_GET_UPTIME` (`crate::syscall`) three times,
/// then `SYS_BLOCK_FOREVER` once -- hand-assembled a straight-line
/// instruction at a time (`mov eax, imm32` is `B8` + the four
/// little-endian immediate bytes; `int 0x80` is `CD 80`) since there's no
/// user-mode-capable toolchain to compile this from source yet (see
/// `rust/README.md`'s libc-equivalent roadmap item; contrast
/// `crate::elf`'s demo, which *is* built with a real assembler, since its
/// job is proving a real ELF loader rather than staying hand-encodable).
/// No trailing jump needed: `SYS_BLOCK_FOREVER`'s handler
/// (`crate::syscall::dispatch`) blocks this task for good from inside the
/// trap itself, so control never returns here a fourth time.
#[rustfmt::skip]
pub const USER_CODE: [u8; 28] = [
    0xB8, syscall::SYS_GET_UPTIME as u8, 0x00, 0x00, 0x00, 0xCD, 0x80, // mov eax, SYS_GET_UPTIME; int 0x80
    0xB8, syscall::SYS_GET_UPTIME as u8, 0x00, 0x00, 0x00, 0xCD, 0x80, // mov eax, SYS_GET_UPTIME; int 0x80
    0xB8, syscall::SYS_GET_UPTIME as u8, 0x00, 0x00, 0x00, 0xCD, 0x80, // mov eax, SYS_GET_UPTIME; int 0x80
    0xB8, syscall::SYS_BLOCK_FOREVER as u8, 0x00, 0x00, 0x00, 0xCD, 0x80, // mov eax, SYS_BLOCK_FOREVER; int 0x80
];

/// Build a new address space (`crate::memory::new_address_space`) for a
/// ring-3 task and map one code page (containing `code`) and one stack
/// page into *that* table (with `USER_ACCESSIBLE`, without which the CPU
/// refuses to execute or touch them at CPL 3 at all -- a `#PF`, not a
/// `#GP`) -- never into the kernel's own mapper, which is the whole point
/// of this module. Returns the new address space -- its top-level page
/// table frame for `crate::proc::spawn` to record as this task's `CR3`,
/// and the memory map naming the two pages as this task's own, so that
/// `fork` can find them without anyone hardcoding them a second time
/// (`crate::memory::MemMap`). Shared by `create_address_space` (this
/// module's own demo) and `crate::rs`'s `flaky` (a second, independent
/// ring-3 task at different addresses) so the address-space-building
/// logic only needs to be correct once.
pub(crate) fn build_ring3_address_space(
    physical_memory_offset: VirtAddr,
    code_addr: u64,
    code: &[u8],
    stack_addr: u64,
) -> AddressSpace {
    let (pml4_frame, mut mapper) = memory::new_address_space(physical_memory_offset);
    let flags =
        PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE;
    let mut frame_allocator = GlobalFrameAllocator;

    let code_page = Page::containing_address(VirtAddr::new(code_addr));
    let code_frame = frame_allocator
        .allocate_frame()
        .expect("out of physical frames for a ring-3 task's code page");
    unsafe {
        // This address space isn't active yet (its frame isn't loaded
        // into CR3), so the mapping itself doesn't need a TLB flush
        // (`.ignore()` rather than `.flush()`); and writing the code
        // bytes has to go through the physical-memory window rather than
        // `code_addr` directly, since that virtual address isn't mapped
        // in the *currently active* (kernel's) table at all.
        mapper
            .map_to(code_page, code_frame, flags, &mut frame_allocator)
            .expect("failed to map a ring-3 task's code page")
            .ignore();
        let code_via_phys_offset =
            (physical_memory_offset + code_frame.start_address().as_u64()).as_mut_ptr::<u8>();
        core::ptr::copy_nonoverlapping(code.as_ptr(), code_via_phys_offset, code.len());
    }

    let stack_page = Page::containing_address(VirtAddr::new(stack_addr));
    let stack_frame = frame_allocator
        .allocate_frame()
        .expect("out of physical frames for a ring-3 task's stack page");
    unsafe {
        mapper
            .map_to(stack_page, stack_frame, flags, &mut frame_allocator)
            .expect("failed to map a ring-3 task's stack page")
            .ignore();
    }

    let mut map = MemMap::EMPTY;
    assert!(
        map.push(VirtAddr::new(code_addr), 1) && map.push(VirtAddr::new(stack_addr), 1),
        "a two-page memory map should always fit"
    );
    AddressSpace { pml4: pml4_frame, map }
}

/// Build this module's own demo address space (`USER_CODE` at
/// `USER_CODE_ADDR`). Called once from `kernel_main`, before any tasks
/// are spawned.
pub fn create_address_space(physical_memory_offset: VirtAddr) -> AddressSpace {
    build_ring3_address_space(physical_memory_offset, USER_CODE_ADDR, &USER_CODE, USER_STACK_ADDR)
}

/// A `crate::proc` task body: jump to ring 3 at `entry_addr` on a stack
/// topped at `stack_top`. Shared by `ring3_task_entry` (this module's own
/// demo) and `crate::rs`'s `flaky_task_entry`. Unlike the first slice's
/// one-shot `demo`, this never "returns" to Rust code at this call site
/// in the ordinary sense -- from here on, this task's kernel-mode moments
/// are each a fresh trap entry (`SYSCALL_VECTOR`, a CPU exception, or the
/// timer interrupting it mid-`iretq`-loop), not a resumption of this
/// function. That's fine: like every other task body in `main.rs`, a
/// caller of this is only ever reached once, via `proc::spawn`'s
/// trampoline.
pub(crate) fn jump_to_ring3(entry_addr: u64, stack_top: u64) -> ! {
    let (code_sel, data_sel) = gdt::user_selectors();
    unsafe { enter_ring3(entry_addr, stack_top, code_sel.0 as u64, data_sel.0 as u64) }
}

/// This module's own demo task body: jump to ring 3 and run `USER_CODE`
/// there.
pub fn ring3_task_entry() -> ! {
    jump_to_ring3(USER_CODE_ADDR, USER_STACK_ADDR + PAGE_SIZE)
}

/// Build a ring-3 `iretq` frame and jump to `entry` at CPL 3 on
/// `stack_top`, with interrupts enabled on the ring-3 side (`0x202`,
/// unlike the first slice's demo, which deliberately left them off): the
/// whole point of this slice is proving a task can safely be
/// asynchronously preempted while in ring 3, which needs the timer to
/// actually be able to fire during it. `pub(crate)` so `crate::elf` can
/// reuse this same trampoline for a real, loaded ELF binary's entry point
/// rather than duplicating it.
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn enter_ring3(entry: u64, stack_top: u64, code_sel: u64, data_sel: u64) -> ! {
    core::arch::naked_asm!(
        // Build the iretq frame, low address (popped first) to high:
        // RIP, CS, RFLAGS, RSP, SS -- so push in the reverse order.
        "push rcx", // SS   = data_sel
        "push rsi", // RSP  = stack_top
        "push 0x202", // RFLAGS, interrupts enabled on the ring-3 side
        "push rdx", // CS   = code_sel
        "push rdi", // RIP  = entry
        "iretq",
    )
}

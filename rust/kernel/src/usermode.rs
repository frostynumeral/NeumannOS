//! Entering ring 3 (user mode) as a real, schedulable task.
//!
//! This is the second slice of the "user-mode processes and address-space
//! isolation" roadmap item in `rust/README.md`, building on the first
//! (proving the CPU mechanism -- GDT user segments, a ring-3-callable
//! syscall gate, the `iretq` dance -- worked at all, as a one-shot,
//! not-scheduler-integrated demo). This time, entering ring 3 happens
//! from inside an ordinary `crate::proc` task body, the task can be
//! asynchronously preempted by the timer while in ring 3 exactly like any
//! other task, and its repeated `int 0x80` calls are handled by ordinary,
//! repeatable trap entry/exit rather than a one-shot save/resume trick.
//! What made that safe to add is per-task `RSP0`: see `crate::gdt`'s
//! `set_rsp0` and `crate::proc::reschedule`'s call to it.
//!
//! There's still no MINIX C file to port here, for the same reason
//! `crate::memory` doesn't have one: this is x86_64-specific groundwork
//! (2005 i386 MINIX used segment-based protection, not ring 3 the way
//! this does), not a ported feature.

use crate::gdt;
use crate::interrupts::SYSCALL_VECTOR;
use x86_64::structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, Size4KiB};
use x86_64::VirtAddr;

/// Arbitrary, fixed addresses for the demo's one code page and one stack
/// page. No per-process address space exists yet (that's the *next*
/// increment on top of this one -- see `rust/README.md`), so these just
/// live in the single shared address space everything else runs in too --
/// picked far from the heap (`allocator::HEAP_START`) and the kernel's own
/// mappings to avoid colliding with either.
pub const USER_CODE_ADDR: u64 = 0x_5555_5555_0000;
pub const USER_STACK_ADDR: u64 = 0x_6666_6666_0000;
const PAGE_SIZE: u64 = 4096;

/// `int 0x80` (`SYSCALL_VECTOR`) followed by a two-byte jump back to the
/// start -- loop forever, trapping into the kernel each time round. Hand-
/// assembled because there's no user-mode-capable toolchain to compile
/// this from source yet (see `rust/README.md`'s libc-equivalent roadmap
/// item) -- this stands in for an entire user-mode program.
/// `crate::interrupts::syscall_handler` decides when the loop actually
/// stops (by not resuming ring 3 past a fixed number of iterations),
/// keeping this hand-assembly trivial rather than needing a real loop
/// counter encoded by hand.
const USER_CODE: [u8; 4] = [0xCD, SYSCALL_VECTOR, 0xEB, 0xFC];

/// Map the demo's code and stack pages (with `USER_ACCESSIBLE`, without
/// which the CPU refuses to execute or touch them at CPL 3 at all -- a
/// `#PF`, not a `#GP`) and write `USER_CODE` into the code page. Called
/// once from `kernel_main`, before any tasks are spawned, using the same
/// mapper/frame allocator `kernel_main` sets up for the heap
/// (`crate::memory`, `crate::allocator`).
pub fn map_demo_pages(
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) {
    let flags =
        PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE;

    let code_page = Page::containing_address(VirtAddr::new(USER_CODE_ADDR));
    let code_frame = frame_allocator
        .allocate_frame()
        .expect("out of physical frames for the user-mode demo's code page");
    unsafe {
        mapper
            .map_to(code_page, code_frame, flags, frame_allocator)
            .expect("failed to map the user-mode demo's code page")
            .flush();
        core::ptr::copy_nonoverlapping(
            USER_CODE.as_ptr(),
            USER_CODE_ADDR as *mut u8,
            USER_CODE.len(),
        );
    }

    let stack_page = Page::containing_address(VirtAddr::new(USER_STACK_ADDR));
    let stack_frame = frame_allocator
        .allocate_frame()
        .expect("out of physical frames for the user-mode demo's stack page");
    unsafe {
        mapper
            .map_to(stack_page, stack_frame, flags, frame_allocator)
            .expect("failed to map the user-mode demo's stack page")
            .flush();
    }
}

/// A `crate::proc` task body: jump to ring 3 and run `USER_CODE` there.
/// Unlike the first slice's `demo`, this never "returns" to Rust code at
/// this call site in the ordinary sense -- from here on, this task's
/// kernel-mode moments are each a fresh trap entry (`SYSCALL_VECTOR`,
/// or the timer interrupting it mid-`iretq`-loop), not a resumption of
/// this function. That's fine: like every other task body in `main.rs`,
/// this one is `fn() -> !` and is only ever reached once, via
/// `proc::spawn`'s trampoline.
pub fn ring3_task_entry() -> ! {
    let (code_sel, data_sel) = gdt::user_selectors();
    let stack_top = USER_STACK_ADDR + PAGE_SIZE;
    unsafe { enter_ring3(USER_CODE_ADDR, stack_top, code_sel.0 as u64, data_sel.0 as u64) }
}

/// Build a ring-3 `iretq` frame and jump to `entry` at CPL 3 on
/// `stack_top`, with interrupts enabled on the ring-3 side (`0x202`,
/// unlike the first slice's demo, which deliberately left them off): the
/// whole point of this slice is proving a task can safely be
/// asynchronously preempted while in ring 3, which needs the timer to
/// actually be able to fire during it.
#[unsafe(naked)]
unsafe extern "C" fn enter_ring3(entry: u64, stack_top: u64, code_sel: u64, data_sel: u64) -> ! {
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

//! Entering ring 3 (user mode) for the first time.
//!
//! This is the first slice of the "user-mode processes and address-space
//! isolation" roadmap item in `rust/README.md` -- proving the CPU
//! mechanism works (GDT user segments, a ring-3-callable syscall gate, and
//! the `iretq` dance to get into ring 3 at all) before tackling the much
//! bigger remaining piece, per-process page tables, which is what would
//! turn this into a real, schedulable user-mode *process* rather than a
//! one-shot demonstration. There's no MINIX C file to port here for the
//! same reason `crate::memory` doesn't have one: this is x86_64-specific
//! groundwork, not a 2005 i386-MINIX feature (which used segment-based
//! protection, not ring 3 the way this does).
//!
//! `demo()` is deliberately *not* wired into `crate::proc`'s scheduler
//! yet. Doing that safely needs each schedulable task to have its own
//! dedicated ring-0 stack for the CPU to switch to automatically on a
//! ring-3-to-ring-0 transition (the TSS's `RSP0`, currently one shared
//! value set once in `crate::gdt` and never updated); with only one
//! shared `RSP0`, two tasks both spending time in ring 3 could clobber
//! each other's saved state the moment either one faults or is
//! preempted. So instead, `demo()` runs once, early in `kernel_main`,
//! before the scheduler or the timer interrupt even exist, and returns
//! control to ordinary kernel code when it's done -- see below for how.

use crate::gdt;
use crate::interrupts::SYSCALL_VECTOR;
use x86_64::structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, Size4KiB};
use x86_64::VirtAddr;

/// Arbitrary, fixed addresses for the demo's one code page and one stack
/// page. No per-process address space exists yet (see the module doc
/// comment), so these just live in the single shared address space
/// everything else runs in too -- picked far from the heap
/// (`allocator::HEAP_START`) and the kernel's own mappings to avoid
/// colliding with either.
const USER_CODE_ADDR: u64 = 0x_5555_5555_0000;
const USER_STACK_ADDR: u64 = 0x_6666_6666_0000;
const PAGE_SIZE: u64 = 4096;

/// `int 0x80` (`SYSCALL_VECTOR`) followed by an infinite loop. Hand-
/// assembled because there's no user-mode-capable toolchain to compile
/// this from source yet (see `rust/README.md`'s libc-equivalent roadmap
/// item) -- this stands in for an entire user-mode program. The loop
/// after the `int` is never actually reached in this demo:
/// `crate::interrupts::syscall_handler` abandons the ring-3 side entirely
/// on the first (and only) syscall rather than resuming it.
const USER_CODE: [u8; 4] = [0xCD, SYSCALL_VECTOR, 0xEB, 0xFE];

/// Where `enter_ring3` should resume once `syscall_handler` decides the
/// demo is over. Written once by `enter_ring3` immediately before it
/// leaves for ring 3, read once by `syscall_handler`. Safe as a bare
/// `static mut`: interrupts are off for this entire excursion (the ring-3
/// side is entered with `IF` clear, precisely so nothing -- including the
/// timer, which does not exist yet at this point in `kernel_main` -- can
/// interleave with it), so there is exactly one logical thread of
/// execution touching this the whole time.
static mut RESUME_RSP: u64 = 0;

/// Map the demo's code and stack pages (with `USER_ACCESSIBLE`, without
/// which the CPU refuses to execute or touch them at CPL 3 at all -- a
/// `#PF`, not a `#GP`), enter ring 3 to run `USER_CODE` there, and return
/// once `crate::interrupts::syscall_handler` has confirmed it was reached
/// from ring 3 and sent control back. An ordinary function call from
/// `kernel_main`'s point of view, despite the CPU privilege level
/// round-trip in between.
pub fn demo(
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

    let (code_sel, data_sel) = gdt::user_selectors();
    // Stacks grow down; start at the top of the page we just mapped.
    let stack_top = USER_STACK_ADDR + PAGE_SIZE;
    unsafe {
        enter_ring3(USER_CODE_ADDR, stack_top, code_sel.0 as u64, data_sel.0 as u64);
    }
}

/// Save the six callee-saved registers, `RFLAGS`, and the current `rsp`
/// into `RESUME_RSP` -- exactly what `proc::switch_to` saves before
/// switching tasks, and for the same reason: it's what lets a much later,
/// unrelated piece of code (`syscall_handler`, in this case, rather than
/// another task) resume execution here as if this were an ordinary
/// function return. Then build a ring-3 `iretq` frame and jump to `entry`
/// at CPL 3 on `stack_top`, with interrupts left off on the ring-3 side
/// (see `RESUME_RSP`'s doc comment for why).
#[unsafe(naked)]
unsafe extern "C" fn enter_ring3(entry: u64, stack_top: u64, code_sel: u64, data_sel: u64) {
    core::arch::naked_asm!(
        "pushfq",
        "push rbx",
        "push rbp",
        "push r12",
        "push r13",
        "push r14",
        "push r15",
        "mov [rip + {resume_rsp}], rsp",
        // Build the iretq frame, low address (popped first) to high:
        // RIP, CS, RFLAGS, RSP, SS -- so push in the reverse order.
        "push rcx",   // SS   = data_sel
        "push rsi",   // RSP  = stack_top
        "push 0x2",   // RFLAGS, interrupts left off on the ring-3 side
        "push rdx",   // CS   = code_sel
        "push rdi",   // RIP  = entry
        "iretq",
        resume_rsp = sym RESUME_RSP,
    )
}

/// Mirror image of `enter_ring3`'s save half: restore the six callee-saved
/// registers and `RFLAGS` from `RESUME_RSP` and `ret`, resuming
/// `enter_ring3` -- and, from there, `demo` -- exactly as if it had
/// returned normally all along. Called from
/// `crate::interrupts::syscall_handler` once it's confirmed (by the mere
/// fact that it was reached at all) that the ring-3 side did its job.
#[unsafe(naked)]
pub unsafe extern "C" fn resume_kernel() -> ! {
    core::arch::naked_asm!(
        "mov rsp, [rip + {resume_rsp}]",
        "pop r15",
        "pop r14",
        "pop r13",
        "pop r12",
        "pop rbp",
        "pop rbx",
        "popfq",
        "ret",
        resume_rsp = sym RESUME_RSP,
    )
}

//! Global Descriptor Table and Task State Segment.
//!
//! Rust equivalent of the segment/TSS setup in `kernel/protect.c`
//! (`prot_init()`). Segmentation is mostly vestigial in 64-bit mode, but a
//! GDT and TSS are still required: the GDT to switch code segments (now
//! including user-mode ones, for `crate::usermode`) and load the TSS, and
//! the TSS for two dedicated stacks:
//!
//! - The Interrupt Stack Table entry the double-fault handler runs on, so
//!   a stack overflow *while already handling a fault* (e.g. a kernel
//!   stack overflow) is reported instead of faulting again on the same
//!   broken stack and triple-faulting the CPU -- see `crate::interrupts`.
//! - `privilege_stack_table[0]` (RSP0): the stack the CPU switches to
//!   automatically on *any* transition from ring 3 to ring 0 (a syscall,
//!   an interrupt, a fault -- anything that raises the CPL), not just the
//!   ones explicitly tagged with an IST index. Needed for user mode to
//!   work at all: left at its default of zero, the CPU would try to use
//!   physical address 0 as a stack pointer the moment any ring-3 code
//!   caused a trap.

use lazy_static::lazy_static;
use x86_64::registers::segmentation::{Segment, CS};
use x86_64::structures::gdt::{Descriptor, GlobalDescriptorTable, SegmentSelector};
use x86_64::structures::tss::TaskStateSegment;
use x86_64::VirtAddr;

pub const DOUBLE_FAULT_IST_INDEX: u16 = 0;

const STACK_SIZE: usize = 4096 * 5;

lazy_static! {
    static ref TSS: TaskStateSegment = {
        let mut tss = TaskStateSegment::new();
        tss.interrupt_stack_table[DOUBLE_FAULT_IST_INDEX as usize] = {
            // No allocator yet (see rust/README.md roadmap), so the
            // double-fault stack is a static array rather than a heap
            // allocation.
            static mut STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];
            let stack_start = VirtAddr::from_ptr(&raw const STACK);
            stack_start + STACK_SIZE as u64
        };
        tss.privilege_stack_table[0] = {
            static mut STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];
            let stack_start = VirtAddr::from_ptr(&raw const STACK);
            stack_start + STACK_SIZE as u64
        };
        tss
    };
}

struct Selectors {
    code_selector: SegmentSelector,
    tss_selector: SegmentSelector,
    user_code_selector: SegmentSelector,
    user_data_selector: SegmentSelector,
}

lazy_static! {
    static ref GDT: (GlobalDescriptorTable, Selectors) = {
        let mut gdt = GlobalDescriptorTable::new();
        let code_selector = gdt.append(Descriptor::kernel_code_segment());
        let tss_selector = gdt.append(Descriptor::tss_segment(&TSS));
        // Order matters on x86_64: the SYSRET instruction (not used here,
        // but the convention is entrenched enough that some CPUs/OSes
        // assume it) expects the user code selector directly after the
        // user data selector. Following that convention costs nothing and
        // avoids a surprise if this ever grows a SYSCALL/SYSRET path.
        let user_data_selector = gdt.append(Descriptor::user_data_segment());
        let user_code_selector = gdt.append(Descriptor::user_code_segment());
        (
            gdt,
            Selectors { code_selector, tss_selector, user_code_selector, user_data_selector },
        )
    };
}

pub fn init() {
    GDT.0.load();
    unsafe {
        CS::set_reg(GDT.1.code_selector);
        x86_64::instructions::tables::load_tss(GDT.1.tss_selector);
    }
}

/// The user-mode code and data segment selectors, for `crate::usermode` to
/// build a ring-3 `iretq` frame with.
pub fn user_selectors() -> (SegmentSelector, SegmentSelector) {
    (GDT.1.user_code_selector, GDT.1.user_data_selector)
}

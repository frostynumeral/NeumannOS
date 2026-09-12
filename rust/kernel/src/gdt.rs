//! Global Descriptor Table and Task State Segment.
//!
//! Rust equivalent of the segment/TSS setup in `kernel/protect.c`
//! (`prot_init()`). Segmentation is mostly vestigial in 64-bit mode, but a
//! GDT and TSS are still required: the GDT to switch code segments and load
//! the TSS, and the TSS to hand the CPU a known-good stack (via the
//! Interrupt Stack Table) to run the double-fault handler on. Without a
//! dedicated IST stack, a stack overflow *while already handling a fault*
//! (e.g. a kernel stack overflow) would fault again on the same broken
//! stack and triple-fault the CPU instead of reporting the error — see
//! `crate::interrupts`.

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
        tss
    };
}

struct Selectors {
    code_selector: SegmentSelector,
    tss_selector: SegmentSelector,
}

lazy_static! {
    static ref GDT: (GlobalDescriptorTable, Selectors) = {
        let mut gdt = GlobalDescriptorTable::new();
        let code_selector = gdt.append(Descriptor::kernel_code_segment());
        let tss_selector = gdt.append(Descriptor::tss_segment(&TSS));
        (gdt, Selectors { code_selector, tss_selector })
    };
}

pub fn init() {
    GDT.0.load();
    unsafe {
        CS::set_reg(GDT.1.code_selector);
        x86_64::instructions::tables::load_tss(GDT.1.tss_selector);
    }
}

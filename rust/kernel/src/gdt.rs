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
//!   caused a trap. Unlike the double-fault IST entry (set once and never
//!   touched again), this one is rewritten on every task switch
//!   (`set_rsp0`, called from `crate::proc`) to point at whichever task is
//!   now current's own dedicated kernel stack -- otherwise every task
//!   spending time in ring 3 would share the same one, and two of them
//!   doing so could clobber each other's saved state.

use lazy_static::lazy_static;
use x86_64::registers::segmentation::{Segment, CS};
use x86_64::structures::gdt::{Descriptor, GlobalDescriptorTable, SegmentSelector};
use x86_64::structures::tss::TaskStateSegment;
use x86_64::VirtAddr;

pub const DOUBLE_FAULT_IST_INDEX: u16 = 0;

const STACK_SIZE: usize = 4096 * 5;

/// A bare `static mut` rather than `lazy_static!` (unlike everything else
/// in this file): `crate::proc::reschedule` needs to keep *writing*
/// `privilege_stack_table[0]` on every switch (see `set_rsp0`), which
/// `lazy_static!`'s `Deref`-only access wouldn't allow. Sound because the
/// only writes happen here (`init`, once) and from `set_rsp0`, which is
/// only ever called from `crate::proc` with interrupts already disabled
/// (see its call sites).
static mut TSS: TaskStateSegment = TaskStateSegment::new();

struct Selectors {
    code_selector: SegmentSelector,
    tss_selector: SegmentSelector,
    user_code_selector: SegmentSelector,
    user_data_selector: SegmentSelector,
}

lazy_static! {
    static ref GDT: (GlobalDescriptorTable, Selectors) = {
        // Safety: this only needs TSS's fixed address (to build the TSS
        // descriptor), not its contents, which `init` below always
        // populates before this is ever used for a real privilege
        // transition (before interrupts are enabled).
        let tss: &'static TaskStateSegment = unsafe { &*(&raw const TSS) };

        let mut gdt = GlobalDescriptorTable::new();
        let code_selector = gdt.append(Descriptor::kernel_code_segment());
        let tss_selector = gdt.append(Descriptor::tss_segment(tss));
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
    unsafe {
        TSS.interrupt_stack_table[DOUBLE_FAULT_IST_INDEX as usize] = {
            // No allocator yet at this point in boot (see
            // rust/README.md's roadmap), so the double-fault stack is a
            // static array rather than a heap allocation.
            static mut STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];
            let stack_start = VirtAddr::from_ptr(&raw const STACK);
            stack_start + STACK_SIZE as u64
        };
        // A placeholder until the first real task starts: `proc::start`
        // sets this to that task's own stack before switching to it, same
        // as `proc::reschedule` does on every later switch (`set_rsp0`).
        TSS.privilege_stack_table[0] = VirtAddr::zero();
    }

    GDT.0.load();
    unsafe {
        CS::set_reg(GDT.1.code_selector);
        x86_64::instructions::tables::load_tss(GDT.1.tss_selector);
    }
}

/// Point the TSS's `RSP0` -- the stack the CPU switches to automatically
/// on any ring-3-to-ring-0 transition -- at `rsp0`. Called by
/// `crate::proc::reschedule`/`start` on every switch, so that whichever
/// task is "current" always has *its own* dedicated kernel stack backing
/// any trap it takes while in ring 3, rather than every task sharing one
/// (which would let two tasks both spending time in ring 3 clobber each
/// other's saved state).
pub fn set_rsp0(rsp0: VirtAddr) {
    unsafe {
        TSS.privilege_stack_table[0] = rsp0;
    }
}

/// The user-mode code and data segment selectors, for `crate::usermode` to
/// build a ring-3 `iretq` frame with.
pub fn user_selectors() -> (SegmentSelector, SegmentSelector) {
    (GDT.1.user_code_selector, GDT.1.user_data_selector)
}

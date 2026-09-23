//! NeumannOS kernel entry point.
//!
//! This is the Rust port's `kernel/main.c` equivalent: it boots, sets up
//! the GDT/IDT so CPU faults are reported instead of triple-faulting
//! (`crate::gdt`, `crate::interrupts`), sets up paging and a heap
//! allocator (`crate::memory`, `crate::allocator`), builds the ring-3 demo
//! task's own address space and maps its pages into it
//! (`crate::usermode`, `crate::memory::new_address_space`), spawns the
//! kernel tasks (`crate::proc`) -- including that ring-3 task, which now
//! runs in genuine isolation from the kernel's own address space, and can
//! still be asynchronously preempted and take repeated traps like any
//! other task, since each task has its own dedicated `RSP0`
//! (`crate::gdt::set_rsp0`) -- programs the PIC/PIT and enables interrupts
//! so the timer starts driving real, asynchronously-preemptive
//! scheduling, prints the boot image (the process table MINIX would load
//! into memory at this point), and hands off to the scheduler -- just as
//! `kernel/main.c` ends by calling `restart()`. See `rust/README.md` for
//! what's implemented versus planned.
#![no_std]
#![no_main]
#![feature(abi_x86_interrupt)]

extern crate alloc;

mod allocator;
mod calls;
mod com;
mod console;
mod elf;
mod font;
mod fs;
mod gdt;
mod interrupts;
mod ipc;
mod keyboard;
mod memory;
mod pic;
mod pit;
mod proc;
mod rs;
mod serial;
mod syscall;
mod table;
mod usermode;
mod vga;

use alloc::boxed::Box;
use alloc::vec::Vec;
use bootloader::{entry_point, BootInfo};
use core::panic::PanicInfo;
use x86_64::VirtAddr;

entry_point!(kernel_main);

fn kernel_main(boot_info: &'static BootInfo) -> ! {
    serial::init();
    serial_println!("NeumannOS kernel booting (Rust port of MINIX 3.1.0)");
    serial_println!();

    gdt::init();
    interrupts::init_idt();
    serial_println!("GDT/IDT loaded");
    // Self-test: deliberately trip a breakpoint exception. The handler
    // prints and returns, proving the IDT is wired up and that a fault
    // doesn't crash the kernel.
    x86_64::instructions::interrupts::int3();
    serial_println!("survived breakpoint exception");

    let phys_mem_offset = VirtAddr::new(boot_info.physical_memory_offset);

    // Draw the LCARS-style demo panel as early as possible: it only
    // needs the physical-memory window the bootloader's own
    // `map_physical_memory` feature already set up (see vga.rs), not
    // paging/heap/scheduler setup below, so it stays visible on screen
    // even if something later in boot panics.
    vga::init_palette();
    vga::draw_demo_panel(vga::framebuffer(phys_mem_offset));
    serial_println!("vga: painted the LCARS demo panel (mode 13h, 320x200)");

    let mut mapper = unsafe { memory::init(phys_mem_offset) };
    unsafe { memory::init_frame_allocator(&boot_info.memory_map) };
    allocator::init_heap(&mut mapper).expect("heap initialization failed");
    serial_println!(
        "heap mapped: {} KiB at {:#x}",
        allocator::HEAP_SIZE / 1024,
        allocator::HEAP_START
    );
    // Self-test: exercise the global allocator through both a Box (single
    // fixed-size allocation) and a growing Vec (multiple reallocations),
    // proving the heap actually works rather than merely compiling.
    let boxed = Box::new(41u32 + 1);
    let mut vec: Vec<u32> = Vec::new();
    for i in 0..100u32 {
        vec.push(i);
    }
    serial_println!(
        "heap self-test: boxed={}, vec.len()={}, vec.sum()={}",
        boxed,
        vec.len(),
        vec.iter().sum::<u32>()
    );
    drop(boxed);
    drop(vec);
    // The on-screen console needs `memory::init`'s physical-memory
    // offset to reach the framebuffer, so its check runs here, not next
    // to the panel painting above.
    console::self_test();

    // Build the ring-3 demo task's own address space and map its
    // code/stack pages into it now; the task itself (spawned below) does
    // the actual jump to ring 3 once the scheduler runs it.
    let ring3_address_space = usermode::create_address_space(phys_mem_offset);
    // Same idea, but loading a real ELF binary's segments (crate::elf)
    // instead of hand-placing a fixed byte array.
    let elf_address_space = elf::load(elf::HELLO_ELF, com::TTY_PROC_NR);
    // And a third: the first half of the exec() demo (`user/shell.s`),
    // which replaces itself with a *different* program the moment it
    // runs (crate::calls::sys_exec). Loaded here exactly like any other
    // ELF task -- exec is what makes it interesting, not how it starts.
    let shell_address_space = elf::load(elf::SHELL_ELF, com::SHELL_PROC_NR);
    // And the interactive shell, with the argv/envp a login shell gets.
    let sh_address_space = elf::load_with_args(
        elf::sh_elf(),
        com::SH_PROC_NR,
        &elf::StartArgs { argv: &[b"sh"], envp: &[b"PATH=/bin", b"HOME=/"] },
    );
    // Self-test: the loader's *rejections*, which are the only part of
    // its validation that a working boot can't demonstrate. See
    // `elf::validator_self_test` for why this is here and not assumed.
    elf::validator_self_test();
    // Self-test: the frame allocator can now take frames back, and a
    // whole address space can be torn down without leaking. See
    // `frame_reclaim_self_test`.
    frame_reclaim_self_test();
    // Self-test: fork shares pages rather than copying them, and the
    // first write to a shared page -- including a *kernel* write, via a
    // real page fault -- gives the writer a private copy. See
    // `cow_self_test`.
    cow_self_test();
    // Self-test: the heap region `brk` grows into is private to every
    // address space, and pages map and unmap without leaking.
    heap_self_test();
    // Self-test: this is the actual proof of isolation, not just that
    // things still work. The demo pages were never mapped into *this*
    // (the kernel's own) page table -- only into `ring3_address_space` --
    // so translating that address here must fail.
    use x86_64::structures::paging::Translate;
    serial_println!(
        "isolation self-test: ring-3 code page at {:#x} in the kernel's own address space: {:?}",
        usermode::USER_CODE_ADDR,
        mapper.translate_addr(VirtAddr::new(usermode::USER_CODE_ADDR)),
    );
    serial_println!();

    serial_println!("boot image:");
    for entry in table::BOOT_IMAGE.iter() {
        serial_println!("  proc_nr={:>3}  name={}", entry.proc_nr, entry.name);
    }
    serial_println!();

    // Tasks must be spawned (and so enqueued as ready) before interrupts
    // are enabled: the timer can start firing as soon as the PIT is
    // programmed, and its handler calls `reschedule()` unconditionally
    // (see `proc.rs`'s module doc comment) -- with no task enqueued yet,
    // there would be nothing for it to pick.
    spawn_tasks(ring3_address_space, elf_address_space, shell_address_space, sh_address_space);
    serial_println!("tasks spawned");

    pic::init();
    pit::init();
    x86_64::instructions::interrupts::enable();
    serial_println!("PIC remapped, PIT programmed for {} Hz, interrupts enabled", pit::HZ);
    serial_println!("handing off to the scheduler");
    serial_println!();
    proc::start()
}

/// Proves physical memory is genuinely reclaimed, in the two shapes that
/// matter: one frame at a time, and a whole address space at once.
///
/// Until now this port only ever handed frames out -- `exec` leaked the
/// image it replaced, and a process killed by `rs` leaked everything it
/// owned, so `flaky`'s three restarts cost three address spaces. "No
/// leak" has no other observable signature in a kernel with no process
/// accounting, so `memory::frame_stats` exists to make it assertable and
/// this checks it directly: the counts have to come back to exactly
/// where they started, and freed frames have to be the ones handed out
/// again (a free list that silently dropped frames would keep the *first*
/// check happy while still leaking).
///
/// Runs from `kernel_main` before any task exists, so nothing else is
/// allocating concurrently and the numbers mean what they say.
fn frame_reclaim_self_test() {
    use x86_64::structures::paging::FrameAllocator;

    let (baseline_in_use, _) = memory::frame_stats();

    // One frame at a time, and reused in LIFO order.
    let mut allocator = memory::GlobalFrameAllocator;
    let first = allocator.allocate_frame().expect("out of frames in the self-test");
    let second = allocator.allocate_frame().expect("out of frames in the self-test");
    assert_eq!(memory::frame_stats().0, baseline_in_use + 2, "allocation isn't being counted");
    unsafe {
        memory::deallocate_frame(second);
        memory::deallocate_frame(first);
    }
    assert_eq!(memory::frame_stats().0, baseline_in_use, "freed frames still counted as in use");
    let reused = allocator.allocate_frame().expect("out of frames in the self-test");
    assert_eq!(reused, first, "a freed frame should be handed out again before the bump cursor moves");
    unsafe { memory::deallocate_frame(reused) };

    // A whole address space, repeatedly. Five build-and-tear-down cycles
    // of a real ELF image (the same one `shell` runs) have to leave the
    // allocator exactly where they found it -- this is the `exec` loop
    // that used to consume physical memory without bound.
    let (before, _) = memory::frame_stats();
    let kernel_pml4 = x86_64::registers::control::Cr3::read().0;
    for _ in 0..5 {
        let image = elf::load_image(kernel_pml4, elf::SHELL_ELF).expect("SHELL_ELF should load");
        // Safety: just built here, never loaded into CR3, referenced by
        // nothing else.
        unsafe { memory::free_address_space(image.pml4, kernel_pml4) };
    }
    let (after, free_listed) = memory::frame_stats();
    serial_println!(
        "[memory] frame reclaim self-test: {} frames in use before and {} after five address-space build/teardown cycles ({} on the free list)",
        before,
        after,
        free_listed
    );
    assert_eq!(after, before, "building and freeing an address space leaked {} frames", after - before);
}

/// Copy-on-write `fork`, checked end to end on a real ELF address space
/// (`HELLO_ELF`, `tty`'s image) before anything depends on it:
///
/// 1. `memory::fork_address_space` copies *no* pages: every page of the
///    child translates to the parent's own frame, so the fork costs only
///    the child's page-table frames.
///    The shared `.data` page is read-only and `COW` on both sides.
/// 2. A real ring-0 write fault resolves it: with `CR3` switched to the
///    child, a plain store to that `.data` page traps (only because
///    `CR0.WP` is on), `interrupts::page_fault_handler` gives the child a
///    private copy, and the store completes. Afterwards the two sides map
///    different frames, the child sees its write and the parent doesn't.
///    Without `CR0.WP` the store would have gone straight into the shared
///    frame, and the parent-side check here is what would catch that.
/// 3. The parent's page, still marked `COW` but no longer shared with
///    anyone, gets its write permission back *without* a copy when
///    `copy_between_address_spaces` writes into it (the other COW-breaking
///    path, since that copy writes through the physical-memory window
///    rather than through the page's mapping).
/// 4. Freeing the child leaves the parent's still-shared `.text` frame
///    alone (it's the parent's live code), and freeing both returns the
///    allocator to exactly where it started.
fn cow_self_test() {
    use x86_64::structures::paging::PageTableFlags;
    use x86_64::registers::control::Cr3;

    let kernel_pml4 = Cr3::read().0;
    let (baseline, _) = memory::frame_stats();
    let parent = elf::load_image(kernel_pml4, elf::HELLO_ELF).expect("HELLO_ELF should load");
    let data = VirtAddr::new(elf::COUNTER_ADDR);
    let text = VirtAddr::new(elf::HELLO_TEXT_ADDR);

    let write_u32 = |pml4, addr: VirtAddr, value: u32| {
        let bytes = value.to_le_bytes();
        memory::copy_between_address_spaces(
            kernel_pml4,
            VirtAddr::new(bytes.as_ptr() as u64),
            pml4,
            addr,
            4,
        )
        .expect("self-test write failed");
    };
    let read_u32 = |pml4, addr: VirtAddr| {
        let mut bytes = [0u8; 4];
        memory::copy_between_address_spaces(
            pml4,
            addr,
            kernel_pml4,
            VirtAddr::new(bytes.as_mut_ptr() as u64),
            4,
        )
        .expect("self-test read failed");
        u32::from_le_bytes(bytes)
    };
    write_u32(parent.pml4, data, 0x1111_1111);

    // 1. Shared, not copied.
    let (before_fork, _) = memory::frame_stats();
    let child = memory::fork_address_space(parent.pml4, kernel_pml4, &parent.map)
        .expect("fork_address_space failed in the self-test");
    let fork_cost = memory::frame_stats().0 - before_fork;
    let (p_data, p_flags) = memory::translate_page(parent.pml4, data).unwrap();
    let (c_data, c_flags) = memory::translate_page(child, data).unwrap();
    let (p_text, _) = memory::translate_page(parent.pml4, text).unwrap();
    let (c_text, _) = memory::translate_page(child, text).unwrap();
    serial_println!(
        "[cow] fork of a {}-page image cost {} frame(s) (page tables only); .data frame {:#x} / {:#x}, .text frame {:#x} / {:#x} (parent / child)",
        parent.map.total_pages(),
        fork_cost,
        p_data.start_address().as_u64(),
        c_data.start_address().as_u64(),
        p_text.start_address().as_u64(),
        c_text.start_address().as_u64()
    );
    assert_eq!(p_data, c_data, "fork copied a writable page instead of sharing it");
    assert_eq!(p_text, c_text, "fork copied a read-only page instead of sharing it");
    // Every page, not just the two sampled above: none may have been
    // copied. (The frame count alone can't show that -- page tables for
    // three separate regions cost more frames than this tiny image has
    // pages.)
    for page in parent.map.pages() {
        let addr = page.start_address();
        assert_eq!(
            memory::translate_page(parent.pml4, addr).map(|(f, _)| f),
            memory::translate_page(child, addr).map(|(f, _)| f),
            "fork copied the page at {:#x} instead of sharing it",
            addr.as_u64()
        );
    }
    for flags in [p_flags, c_flags] {
        assert!(flags.contains(memory::COW), "a shared writable page isn't marked COW");
        assert!(!flags.contains(PageTableFlags::WRITABLE), "a shared writable page is still writable");
    }

    // 2. A real ring-0 write fault, in the child. Interrupts off: nothing
    // else may run while `CR3` is an address space no task owns.
    x86_64::instructions::interrupts::without_interrupts(|| unsafe {
        let (saved, flags) = Cr3::read();
        Cr3::write(child, flags);
        core::ptr::write_volatile(data.as_mut_ptr::<u32>(), 0x2222_2222);
        Cr3::write(saved, flags);
    });
    let (c_data_after, c_flags_after) = memory::translate_page(child, data).unwrap();
    assert_ne!(c_data_after, p_data, "the write fault didn't give the child its own copy");
    assert!(c_flags_after.contains(PageTableFlags::WRITABLE) && !c_flags_after.contains(memory::COW));
    assert_eq!(read_u32(child, data), 0x2222_2222, "the child's write didn't land in its copy");
    assert_eq!(
        read_u32(parent.pml4, data),
        0x1111_1111,
        "a write in the forked child reached the parent's page -- CR0.WP off, or COW not enforced"
    );
    assert_eq!(memory::extra_sharers(p_data), 0, "the copied frame is still counted as shared");

    // 3. The parent's page: COW, but its only mapping now -- reclaimed
    // in place, no copy.
    write_u32(parent.pml4, data, 0x3333_3333);
    let (p_data_after, p_flags_after) = memory::translate_page(parent.pml4, data).unwrap();
    assert_eq!(p_data_after, p_data, "a no-longer-shared COW page was copied instead of reclaimed");
    assert!(p_flags_after.contains(PageTableFlags::WRITABLE) && !p_flags_after.contains(memory::COW));
    assert_eq!(read_u32(parent.pml4, data), 0x3333_3333);
    serial_println!(
        "[cow] ring-0 write fault gave the child its own .data ({:#x}); the parent kept {:#x} and got write access back without a copy",
        c_data_after.start_address().as_u64(),
        p_data.start_address().as_u64()
    );

    // 3b. The same reclaim, but in a *child* -- whose page tables fork
    // built, not the loader. Fork again, let the parent copy first, and
    // have the child (now the page's only owner) write it: a real ring-0
    // fault that must come back `Reclaimed` and then let the store
    // through. This is the case that killed the interactive shell's
    // fifth command before `share_pages_into` spelled out its table
    // flags; a leaf made writable under a read-only table faults again,
    // with nothing COW about it, and the store never completes.
    let child2 = memory::fork_address_space(parent.pml4, kernel_pml4, &parent.map)
        .expect("second fork failed in the self-test");
    write_u32(parent.pml4, data, 0x4444_4444); // parent copies away
    let (shared, _) = memory::translate_page(child2, data).unwrap();
    assert_eq!(memory::extra_sharers(shared), 0, "the parent's copy didn't leave the child sole owner");
    x86_64::instructions::interrupts::without_interrupts(|| unsafe {
        let (saved, flags) = Cr3::read();
        Cr3::write(child2, flags);
        core::ptr::write_volatile(data.as_mut_ptr::<u32>(), 0x5555_5555);
        Cr3::write(saved, flags);
    });
    let (reclaimed, reclaimed_flags) = memory::translate_page(child2, data).unwrap();
    assert_eq!(reclaimed, shared, "a sole-owner COW page in a child was copied instead of reclaimed");
    assert!(reclaimed_flags.contains(PageTableFlags::WRITABLE));
    assert_eq!(read_u32(child2, data), 0x5555_5555);
    assert_eq!(read_u32(parent.pml4, data), 0x4444_4444);
    unsafe { memory::free_address_space(child2, kernel_pml4) };
    serial_println!("[cow] a forked child that became a page's only owner reclaimed it in place and wrote through");

    // 4. Teardown respects the sharing.
    let text_word = read_u32(parent.pml4, text);
    unsafe { memory::free_address_space(child, kernel_pml4) };
    assert_eq!(memory::translate_page(parent.pml4, text).map(|(f, _)| f), Some(p_text));
    assert_eq!(read_u32(parent.pml4, text), text_word, "freeing the child freed the parent's shared .text");
    assert_eq!(memory::extra_sharers(p_text), 0);
    unsafe { memory::free_address_space(parent.pml4, kernel_pml4) };
    let (after, _) = memory::frame_stats();
    assert_eq!(after, baseline, "fork + COW + teardown leaked {} frame(s)", after as isize - baseline as isize);
    serial_println!("[cow] copy-on-write self-test passed; frames back to {}", after);
}

/// `proc::brk`'s building blocks, on a real ELF address space: the whole
/// possible heap (`proc::HEAP_BASE`, `proc::HEAP_MAX`) is in PML4 slots
/// the kernel's own address space leaves empty (so growing it can never
/// edit the kernel's page tables), `memory::map_zeroed` hands out pages
/// that read as zero and take writes, and `memory::unmap_release` gives
/// them back -- the frame count balancing once the address space goes too.
fn heap_self_test() {
    use x86_64::registers::control::Cr3;
    let kernel_pml4 = Cr3::read().0;
    let base = VirtAddr::new(proc::HEAP_BASE);
    assert!(
        memory::pml4_slots_unused(kernel_pml4, base, base + (proc::HEAP_MAX - 1)),
        "the heap region overlaps a PML4 slot the kernel uses"
    );
    let (before, _) = memory::frame_stats();
    let image = elf::load_image(kernel_pml4, elf::HELLO_ELF).expect("HELLO_ELF should load");
    assert!(memory::map_zeroed(image.pml4, base, 4), "map_zeroed failed");
    let mut probe = [0xffu8; 8];
    let last = base + 3 * 4096 + 4088;
    memory::copy_between_address_spaces(image.pml4, last, kernel_pml4, VirtAddr::new(probe.as_mut_ptr() as u64), 8)
        .expect("the new heap page isn't mapped");
    assert_eq!(probe, [0; 8], "a fresh heap page wasn't zeroed");
    let word = 0x1234_5678_9abc_def0u64.to_le_bytes();
    memory::copy_between_address_spaces(kernel_pml4, VirtAddr::new(word.as_ptr() as u64), image.pml4, last, 8)
        .expect("the new heap page isn't writable");
    memory::unmap_release(image.pml4, base, 4);
    assert!(memory::translate_page(image.pml4, base).is_none(), "unmap_release left a page mapped");
    unsafe { memory::free_address_space(image.pml4, kernel_pml4) };
    let (after, _) = memory::frame_stats();
    assert_eq!(after, before, "heap map/unmap leaked {} frames", after as isize - before as isize);
    serial_println!("[heap] self-test: region private, pages zeroed and writable, unmap balances ({} frames)", after);
}

/// Spawn the kernel tasks. `IDLE` and `CLOCK` are real kernel tasks, same
/// as in the boot image; `pm`/`memory`/`driver` don't exist as real
/// servers yet (see `rust/README.md`), so their process table slots run
/// small stand-in bodies instead: `pm` exercises blocking `send`/
/// `receive` (and now real IPC-served file I/O via `fs`), `memory` (as
/// `busy_task`) exercises asynchronous preemption, and `driver` (as
/// `usermode::ring3_task_entry`) runs in its own address space
/// (`ring3_address_space`, built in `kernel_main`) -- one of three tasks
/// here that isn't sharing the kernel's. `fs` is now a real, in-memory
/// file server (`fs::InMemoryFs::serve`). `tty` (as `elf::task_entry`)
/// is another task with its own address space (`elf_address_space`): a
/// real ELF binary (`elf::HELLO_ELF`), loaded and run in ring 3 rather
/// than a hand-assembled demo. `rs` is now a real reincarnation server
/// (`rs::task`), monitoring `flaky` (`rs::spawn_flaky`, the third and
/// last ring-3 task here) -- a service that crashes the instant it runs
/// (on purpose, via `ud2`), letting `rs` prove it can restart a dead
/// process rather than the whole machine going down with it. Priorities,
/// quantum sizes, and preemptibility match `kernel/table.c`'s image
/// entries (`IDL_F`/`TSK_F`/`SRV_F` flags) where a real counterpart
/// exists.
fn spawn_tasks(
    ring3_address_space: proc::AddressSpace,
    elf_address_space: proc::AddressSpace,
    shell_address_space: proc::AddressSpace,
    sh_address_space: proc::AddressSpace,
) {
    proc::spawn(com::IDLE, "IDLE", idle_task, proc::IDLE_Q, 8, true, None);
    proc::spawn(com::CLOCK, "CLOCK", clock_task, proc::TASK_Q, 64, false, None);
    proc::spawn(com::PM_PROC_NR, "pm (demo)", demo_pm_task, 3, 32, true, None);
    proc::spawn(com::FS_PROC_NR, "fs (demo)", demo_fs_task, 4, 32, true, None);
    // Priority 5, one step above the other demo tasks (6): `rs` needs to
    // already be blocked in `ipc::receive` before `flaky` gets a chance to
    // crash, since `proc::kill`'s notification to `RS` is fire-and-forget,
    // exactly like every other notification in this port (see "known
    // simplifications" in rust/README.md) -- silently dropped if `RS`
    // isn't receiving yet. A strictly higher priority guarantees `rs`
    // gets the CPU (and reaches that first blocking `receive`) before any
    // priority-6 task, including `flaky`, regardless of spawn order.
    proc::spawn(com::RS_PROC_NR, "rs", rs::task, 5, 16, true, None);
    proc::spawn(com::MEM_PROC_NR, "memory (demo)", busy_task, 6, 16, true, None);
    proc::spawn(
        com::DRVR_PROC_NR,
        "driver (ring3 demo)",
        usermode::ring3_task_entry,
        6,
        16,
        true,
        Some(ring3_address_space),
    );
    proc::spawn(
        com::TTY_PROC_NR,
        "tty (elf demo)",
        elf::task_entry,
        6,
        16,
        true,
        Some(elf_address_space),
    );
    // The fork/exec demo, at the same priority as the other ring-3
    // tasks: it starts out running `elf::SHELL_ELF`, forks, and its
    // child ends up running /bin/exectest -- the pair a real shell is built
    // out of, rather than a process replacing itself in place.
    proc::spawn(
        com::SHELL_PROC_NR,
        "shell (fork/exec demo)",
        elf::task_entry,
        6,
        16,
        true,
        Some(shell_address_space),
    );
    // The interactive shell: reads commands from the keyboard, runs
    // them out of /bin (fork, exec, wait), and never exits on its own.
    proc::spawn(com::SH_PROC_NR, "sh", elf::task_entry, 6, 16, true, Some(sh_address_space));
    rs::spawn_flaky();
    proc::spawn(com::CONSOLE_PROC_NR, "console", keyboard::console_task, 5, 16, true, None);
}

/// Real MINIX's idle task just halts, waking on the next interrupt; ported
/// as-is. It's still preemptible (matching `IDL_F`), though with nothing
/// else runnable that mostly just means its own ticks get charged to it.
///
/// `IDLE` only ever becomes current once *everything* else has blocked
/// (it's the lowest-priority queue, `proc::IDLE_Q`) -- `memory`'s
/// busy-loop alone keeps that from happening until around tick 30 (see
/// `busy_task`), comfortably after `tty`'s entire sequence (its counter
/// loop, its `SYS_VIRCOPY`, and its alarm-triggered
/// `SYS_FS_OPEN`/`SYS_FS_WRITE`, `user/hello.s`) has had time to run --
/// so this, unlike `CLOCK`'s own fixed-alarm wakeup, is a genuinely safe
/// "everything that's going to happen already has" checkpoint, not a
/// race against `tty`'s actual progress. `elf_counter_demo` used to run
/// from `clock_task` instead, racing `tty`'s counter loop against
/// `CLOCK`'s own independent 3-tick alarm -- "true in practice" (per its
/// old doc comment) until real timing variance proved otherwise (an
/// observed, reproducible boot panic, not a hypothetical one); it and
/// `vircopy_from_ring3_verify` both belong here instead, for the same
/// reason `fs_from_ring3_verify` already does.
fn idle_task() -> ! {
    elf_counter_demo();
    fs_from_ring3_verify();
    vircopy_from_ring3_verify();
    vircopy_error_from_ring3_verify();
    fork_child_verify();
    exec_verify();
    fs_dir_check();
    ptr_safety_check();
    threads_check();
    heap_check();
    port_check();
    proc_slot_pool_check();
    wait_without_children_check();
    runtime_reclaim_check();
    serial_println!("[idle] no other task is ready, halting (uptime: {} ticks)", proc::uptime_ticks());
    halt_loop()
}

/// Proves `tty`'s `SYS_FS_OPEN`/`SYS_FS_WRITE` calls (`user/hello.s`, via
/// `crate::syscall`) genuinely reached `fs` over IPC and landed in real
/// storage: opens the same path *fresh* (a distinct file descriptor, its
/// own cursor at `0`, reached the ordinary kernel-side way -- `crate::fs`'s
/// client stubs, not a syscall) and reads back exactly the bytes `tty`
/// wrote from ring 3. Not just "the syscall didn't crash" -- the actual
/// content, read back through a completely independent path.
fn fs_from_ring3_verify() {
    let expected = b"written from ring 3 via a real syscall, IPC, and fs!";
    let fd = fs::open("/from_ring3.txt");
    assert!(fd >= 0, "fs::open(\"/from_ring3.txt\") failed: {}", fd);
    let mut buf = [0u8; 64];
    let n = fs::read(fd, &mut buf);
    serial_println!(
        "[idle] read back {:?} from /from_ring3.txt (written by tty from ring 3 via SYS_FS_OPEN/SYS_FS_WRITE)",
        core::str::from_utf8(&buf[..n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(n, expected.len() as i64, "wrong length read back from /from_ring3.txt");
    assert_eq!(&buf[..n as usize], expected, "fs content doesn't match what tty wrote via a real syscall");
}

/// Proves `tty`'s ring-3 `SYS_VIRCOPY` call (`user/hello.s`, via
/// `crate::syscall`) genuinely copied `driver`'s private memory into
/// `tty`'s *own* address space, not just that the syscall returned
/// success: reads `tty`'s `vircopy_buf` (`elf::VIRCOPY_BUF_ADDR`) back out
/// with a second, independent `sys_vircopy` call (kernel-side, the same
/// mechanism `vircopy_demo` above already proved) and checks it matches
/// `usermode::USER_CODE` -- the exact bytes `driver`'s own code page
/// holds. Same "everything that's going to happen already has" checkpoint
/// as `fs_from_ring3_verify` above, for the same reason: `tty`'s
/// `SYS_VIRCOPY` runs well before its own `SYS_READ_LINE` blocks, which is
/// itself well before `IDLE` ever becomes current.
fn vircopy_from_ring3_verify() {
    let mut buf = [0u8; usermode::USER_CODE.len()];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::VIRCOPY_BUF_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading tty's vircopy_buf");
    serial_println!(
        "[idle] read back {:?} from tty's own vircopy_buf (copied there by tty itself via ring-3 SYS_VIRCOPY)",
        buf
    );
    assert_eq!(
        buf, usermode::USER_CODE,
        "tty's ring-3 SYS_VIRCOPY didn't actually copy driver's code page"
    );
}

/// Proves the syscall ABI's distinct error codes (`crate::syscall`'s
/// `ERR_BAD_LENGTH`/`ERR_BAD_UTF8`/`ERR_VIRCOPY_FAILED`/
/// `ERR_UNKNOWN_CALL`) genuinely reach a ring-3 caller's `rax`, not just
/// that `dispatch` computes the right value internally: `tty` makes a
/// second, deliberately-invalid `SYS_VIRCOPY` call (an oversized `len`,
/// `user/hello.s`) and stashes the raw return value in `err_result`
/// (`elf::ERR_RESULT_ADDR`); this reads it back the same way
/// `vircopy_from_ring3_verify` reads `vircopy_buf` and checks it's
/// exactly `syscall::ERR_BAD_LENGTH`, not merely "some nonzero failure
/// code."
fn vircopy_error_from_ring3_verify() {
    let mut buf = [0u8; 8];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::ERR_RESULT_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading tty's err_result");
    let result = u64::from_le_bytes(buf);
    serial_println!(
        "[idle] read back {:#x} from tty's own err_result (expected ERR_BAD_LENGTH {:#x})",
        result,
        syscall::ERR_BAD_LENGTH
    );
    assert_eq!(
        result, syscall::ERR_BAD_LENGTH,
        "tty's deliberately-invalid ring-3 SYS_VIRCOPY didn't come back ERR_BAD_LENGTH"
    );
}

/// Proves `tty`'s ring-3 `SYS_FORK` call (`user/hello.s`, via
/// `crate::syscall`) genuinely created an independent child process, not
/// just that the syscall returned a plausible-looking proc_nr: checks two
/// things a shared-page aliasing bug couldn't fake. First, the forked
/// child's own `vircopy_buf` reads back the canary (`0xcafebabe`) the
/// child's ring-3 code wrote into *its own* copy right after forking --
/// read via a kernel-side `sys_vircopy` targeting the child's own
/// process number specifically, not `tty`. Second, `tty`'s own
/// `vircopy_buf` (already checked by `vircopy_from_ring3_verify` above)
/// still holds `driver`'s code bytes, not the child's canary -- if
/// `memory::fork_address_space` had left the data page merely aliased
/// instead of genuinely deep-copied, the child's later write would have
/// clobbered the parent's copy too, and that earlier check would already
/// have failed. Finally, reads back the distinguishing file the child
/// wrote (`/from_fork_child.txt`, a real `SYS_FS_OPEN`/`SYS_FS_WRITE` from
/// the child's own, separately-scheduled ring-3 execution) the same way
/// `fs_from_ring3_verify` does for `tty`'s own file.
fn fork_child_verify() {
    // The child's process number isn't reserved for it in `crate::com`
    // any more (`proc::alloc_proc_nr` hands one out at the moment of the
    // fork), so it's found the way a real system would find it: by
    // asking the process table who `tty`'s child is.
    let child = proc::child_of(com::TTY_PROC_NR)
        .expect("tty's ring-3 SYS_FORK didn't leave a child in the process table");
    serial_println!("[idle] tty's forked child is proc_nr {}", child);

    let mut canary_buf = [0u8; 4];
    calls::sys_vircopy(
        child,
        x86_64::VirtAddr::new(elf::VIRCOPY_BUF_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(canary_buf.as_mut_ptr() as u64),
        canary_buf.len(),
    )
    .expect("sys_vircopy failed reading the forked child's vircopy_buf");
    let canary = u32::from_le_bytes(canary_buf);
    serial_println!(
        "[idle] read back {:#x} from the forked child's own vircopy_buf (expected canary 0xcafebabe)",
        canary
    );
    assert_eq!(
        canary, 0xcafebabe,
        "the forked child's vircopy_buf doesn't hold its own canary -- fork may have left it aliased with tty's"
    );

    let expected = b"hello from the forked child, running independently in ring 3!";
    let fd = fs::open("/from_fork_child.txt");
    assert!(fd >= 0, "fs::open(\"/from_fork_child.txt\") failed: {}", fd);
    let mut buf = [0u8; 96];
    let n = fs::read(fd, &mut buf);
    serial_println!(
        "[idle] read back {:?} from /from_fork_child.txt (written by the forked child from its own ring-3 execution)",
        core::str::from_utf8(&buf[..n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(n, expected.len() as i64, "wrong length read back from /from_fork_child.txt");
    assert_eq!(&buf[..n as usize], expected, "fs content doesn't match what the forked child wrote");
}

/// Proves the fork/exec pair `user/shell.s` performs -- a ring-3
/// `SYS_FORK` whose *child* then `SYS_EXEC`s `/bin/exectest` -- did what a
/// real shell's does: one process became two, and exactly one of them
/// (the child) stopped being `shell` and became a different program
/// entirely, at its own process number, without disturbing the parent.
/// Eight independent checks, in the order they run, each ruling out a
/// different way a fake (or half-done) fork/exec could still look real:
///
/// 1. `/from_exec.txt` holds what `user/echo.s` writes -- read back
///    through `fs`'s ordinary kernel-side path, so the exec'd image
///    demonstrably ran and did real work, not just that the syscall
///    logged something. (An exec that loaded the image but never
///    transferred control would fail here.)
/// 2. `/exec_error.bin` holds `syscall::ERR_BAD_ELF`, written by
///    `shell` *after* a deliberately-failed exec of an ordinary text
///    file it created itself (`user/shell.s`). That file existing at all
///    is the real result: a failed exec has to leave the caller running
///    its original image (POSIX requires it, and `sys_exec` discards the
///    old address space only a few lines after building the new one), so
///    `shell` had to still be `shell` -- own `.data`, own stack -- long
///    enough to write it.
/// 3. `/evil_exec_error.bin` holds `syscall::ERR_BAD_ELF`, written by
///    the exec'd image after it tried, from ring 3, to exec a
///    hand-built ELF whose only segment asks to be mapped at
///    `allocator::HEAP_START` -- the kernel's own heap (`user/echo.s`).
///    The first version of this port's validator accepted exactly that
///    image, and mapping it wrote into the kernel's own page tables,
///    because a new address space copies only the top-level table and
///    the check bounded addresses rather than PML4 slots.
///    `elf::validator_self_test` covers the same ground kernel-side;
///    this is the one that proves the path is shut to an actual
///    unprivileged process.
/// 4. The process table records a child of `com::SHELL_PROC_NR`
///    (`proc::child_of`, from `Proc::parent`) -- the fork produced a
///    second, real process, not just a syscall return value.
/// 5. `/shell_fork.bin` holds that same process number, written by
///    `shell` itself from whatever `SYS_FORK` returned in its `rax`.
///    Ring 3's view of who its child is and the kernel's have to agree;
///    a plausible-looking number that belonged to nothing would pass
///    every other check here.
/// 6. `user/echo.s`'s own `.data` counter (`elf::ECHO_COUNTER_ADDR`)
///    reads `1` *in the child's address space* -- the new program's
///    instructions ran, in the process that called exec. It is also
///    *unmapped* in `shell` itself, which is what distinguishes "the
///    child replaced its own image" from "the exec'd image got loaded
///    into whoever asked".
/// 7. `user/shell.s`'s `pre_exec_marker` (`elf::SHELL_MARKER_ADDR`) still
///    reads `0xfeedface` in `com::SHELL_PROC_NR`: the parent kept the
///    image it was running. An `exec` that reached the wrong process
///    table slot -- the caller's parent rather than the caller -- would
///    fail here, and nothing else in this port would notice.
/// 8. `/from_exited_child.txt`, `/shell_wait1.bin` and
///    `/shell_wait2.bin` cover the *other* end of a process's life.
///    `shell` forks two more children that don't exec at all: each
///    terminates with `SYS_EXIT` and a status of its own, and `shell`
///    collects both with `SYS_WAIT`, parking each `(proc_nr, status)`
///    pair in a file. Checked here: the statuses are exactly `42` and
///    `7`, and the process numbers are ones `proc::alloc_proc_nr`
///    handed out and has since taken back, since collecting a status is
///    what frees the slot (`user/shell.s` explains why the two children
///    are separated -- one is waited for immediately, the other left to
///    become a zombie first).
/// 9. That same marker page is *unmapped in the child*: reading it back
///    has to fail with `CopyError::SrcNotMapped`. The child demonstrably
///    had it a moment ago (it inherited a copy from the fork, and
///    `shell` had already written to it), so this is what proves exec
///    genuinely threw the old image away rather than merely adding the
///    new one's mappings alongside it -- the easy way to get checks 1
///    and 6 to pass.
///
/// Runs from `idle_task` for the same reason every other check there
/// does; see its doc comment. Check 1 additionally waits rather than
/// assuming: `shell` sleeps between exec attempts if `/bin/exectest` hasn't
/// been installed yet (`user/shell.s`), and `IDLE` becoming runnable
/// during one of those naps is exactly the sort of interleaving this
/// port has already been bitten by once.
fn exec_verify() {
    let expected = b"written by the exec'd image, not the one that called exec";
    let mut buf = [0u8; 96];
    let n = read_when_available("/from_exec.txt", &mut buf);
    serial_println!(
        "[idle] read back {:?} from /from_exec.txt (written by the image shell exec'd itself into)",
        core::str::from_utf8(&buf[..n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(n, expected.len() as i64, "wrong length read back from /from_exec.txt");
    assert_eq!(&buf[..n as usize], expected, "fs content doesn't match what the exec'd image wrote");

    let mut err_buf = [0u8; 8];
    let err_n = read_when_available("/exec_error.bin", &mut err_buf);
    assert_eq!(err_n, 8, "wrong length read back from /exec_error.bin");
    let exec_error = u64::from_le_bytes(err_buf);
    serial_println!(
        "[idle] read back {:#x} from /exec_error.bin -- shell's deliberately-failed exec of a non-ELF file (expected ERR_BAD_ELF {:#x}), written by shell itself afterward",
        exec_error,
        syscall::ERR_BAD_ELF
    );
    assert_eq!(
        exec_error,
        syscall::ERR_BAD_ELF,
        "exec'ing a non-ELF file should come back ERR_BAD_ELF to the ring-3 caller"
    );

    let mut evil_buf = [0u8; 8];
    let evil_n = read_when_available("/evil_exec_error.bin", &mut evil_buf);
    assert_eq!(evil_n, 8, "wrong length read back from /evil_exec_error.bin");
    let evil_result = u64::from_le_bytes(evil_buf);
    serial_println!(
        "[idle] read back {:#x} from /evil_exec_error.bin -- a ring-3 exec of an image asking to be mapped onto the kernel heap (expected ERR_BAD_ELF {:#x})",
        evil_result,
        syscall::ERR_BAD_ELF
    );
    assert_eq!(
        evil_result,
        syscall::ERR_BAD_ELF,
        "a ring-3 process was able to exec an image targeting the kernel's own address range"
    );

    let child = proc::child_of(com::SHELL_PROC_NR)
        .expect("shell's ring-3 SYS_FORK didn't leave a child in the process table");
    serial_println!("[idle] shell's forked child -- the process that exec'd -- is proc_nr {}", child);

    let mut fork_buf = [0u8; 8];
    let fork_n = read_when_available("/shell_fork.bin", &mut fork_buf);
    assert_eq!(fork_n, 8, "wrong length read back from /shell_fork.bin");
    let observed = u64::from_le_bytes(fork_buf);
    serial_println!(
        "[idle] read back {} from /shell_fork.bin -- the proc_nr SYS_FORK returned to shell in ring 3 (process table says {})",
        observed,
        child
    );
    assert_eq!(
        observed, child as u64,
        "the proc_nr ring 3 got back from SYS_FORK isn't the child the kernel actually created"
    );

    let mut counter_buf = [0u8; 4];
    calls::sys_vircopy(
        child,
        x86_64::VirtAddr::new(elf::ECHO_COUNTER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(counter_buf.as_mut_ptr() as u64),
        counter_buf.len(),
    )
    .expect("sys_vircopy failed reading the exec'd image's counter out of the child's address space");
    let counter = i32::from_le_bytes(counter_buf);
    serial_println!(
        "[idle] read back {} from /bin/exectest's own .data counter, in the child's process (expected 1)",
        counter
    );
    assert_eq!(counter, 1, "the exec'd image's own code should have incremented its counter once");

    let mut in_parent = [0u8; 4];
    let leaked = calls::sys_vircopy(
        com::SHELL_PROC_NR,
        x86_64::VirtAddr::new(elf::ECHO_COUNTER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(in_parent.as_mut_ptr() as u64),
        in_parent.len(),
    );
    serial_println!(
        "[idle] reading /bin/exectest's counter page at {:#x} out of *shell* instead: {:?} (expected SrcNotMapped -- only the child exec'd)",
        elf::ECHO_COUNTER_ADDR,
        leaked
    );
    assert!(
        matches!(leaked, Err(memory::CopyError::SrcNotMapped)),
        "the exec'd image is mapped in the parent too -- exec reached the wrong address space"
    );

    let mut marker_buf = [0u8; 4];
    calls::sys_vircopy(
        com::SHELL_PROC_NR,
        x86_64::VirtAddr::new(elf::SHELL_MARKER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(marker_buf.as_mut_ptr() as u64),
        marker_buf.len(),
    )
    .expect("sys_vircopy failed reading shell's own marker page -- the parent lost its image");
    let marker = u32::from_le_bytes(marker_buf);
    serial_println!(
        "[idle] read back {:#x} from shell's own pre-fork marker at {:#x} (expected 0xfeedface -- the parent still is shell)",
        marker,
        elf::SHELL_MARKER_ADDR
    );
    assert_eq!(
        marker, 0xfeedface,
        "shell's own .data marker is gone -- its child's exec reached the parent's address space"
    );

    let expected_child = b"written by a forked child that then exited with a real status";
    let mut child_buf = [0u8; 96];
    let child_n = read_when_available("/from_exited_child.txt", &mut child_buf);
    serial_println!(
        "[idle] read back {:?} from /from_exited_child.txt (written by a child that then exited with a status)",
        core::str::from_utf8(&child_buf[..child_n.max(0) as usize]).unwrap_or("<invalid utf8>")
    );
    assert_eq!(
        &child_buf[..child_n.max(0) as usize],
        expected_child,
        "fs content doesn't match what shell's exiting child wrote"
    );

    for (path, expected_status) in [("/shell_wait1.bin", 42), ("/shell_wait2.bin", 7)] {
        let (child, status) = read_wait_result(path);
        serial_println!(
            "[idle] read back (proc_nr {}, status {}) from {} -- collected by shell's own SYS_WAIT (expected status {})",
            child,
            status,
            path,
            expected_status
        );
        assert_eq!(status, expected_status, "{} holds the wrong exit status", path);
        assert!(
            child >= com::FIRST_DYNAMIC_PROC_NR,
            "{} names proc_nr {}, which isn't a dynamically allocated one",
            path,
            child
        );
        // Collecting a status is what releases the slot, so by now the
        // number is free again -- and demonstrably reusable, which
        // `proc_slot_pool_check` goes on to confirm for the pool as a
        // whole.
        assert_eq!(
            proc::mem_map_of(child).total_pages(),
            0,
            "proc_nr {} still has a memory map after being waited for",
            child
        );
    }

    argv_verify(child);

    let mut stale_buf = [0u8; 4];
    let stale = calls::sys_vircopy(
        child,
        x86_64::VirtAddr::new(elf::SHELL_MARKER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(stale_buf.as_mut_ptr() as u64),
        stale_buf.len(),
    );
    serial_println!(
        "[idle] reading shell's marker page at {:#x} out of the *child* instead: {:?} (expected SrcNotMapped -- the inherited image is gone)",
        elf::SHELL_MARKER_ADDR,
        stale
    );
    assert!(
        matches!(stale, Err(memory::CopyError::SrcNotMapped)),
        "the image the child inherited from shell is still mapped after exec -- the old image wasn't replaced, only added to"
    );
}

/// `wait`'s one non-blocking answer: a process with no children at all
/// has to be told so (POSIX `ECHILD`, `crate::syscall`'s
/// `ERR_NO_CHILDREN`) rather than blocking forever on something that can
/// never happen. `IDLE` has never forked, so it is the right process to
/// ask -- and if `wait_for_child` got this wrong, this call would hang
/// the whole system rather than fail quietly, which is precisely why the
/// case is worth pinning down.
fn wait_without_children_check() {
    let result = proc::wait_for_child();
    serial_println!("[idle] wait_for_child() with no children -> {:?} (expected None)", result);
    assert!(result.is_none(), "wait must report having no children instead of blocking");
}

/// Read one of `user/shell.s`'s `(proc_nr, status)` pairs back out of
/// `fs`: sixteen bytes, written straight out of that program's `.data`
/// by a single `SYS_FS_WRITE` over two adjacent 8-byte slots. The status
/// occupies only the first four of its own eight, since `SYS_WAIT`
/// writes an `i32` through the pointer it's given and `shell` zeroes the
/// rest at link time.
fn read_wait_result(path: &str) -> (i32, i32) {
    let mut buf = [0u8; 16];
    let n = read_when_available(path, &mut buf);
    assert_eq!(n, 16, "wrong length read back from {}", path);
    let child = u64::from_le_bytes(buf[..8].try_into().unwrap()) as i32;
    let status = i32::from_le_bytes(buf[8..12].try_into().unwrap());
    (child, status)
}

/// `exec_verify`'s argument-passing half: `shell` exec'd `/bin/exectest`
/// with `argv = {"/bin/exectest", "hello", "from", "argv"}` and
/// `envp = {"GREETING=neumann"}` (`user/shell.s`), and this checks the
/// new image received exactly that, from three independent angles:
///
/// 1. What the program *did* with them: `/echo_output.txt` is its
///    arguments joined with spaces and `/echo_env.txt` its first
///    environment string, both written by `user/echo.s` itself.
/// 2. What was actually *on its stack*: read straight out of `child`'s
///    memory with `sys_vircopy`, starting at the `rsp` it recorded on
///    entry -- `argc`, then each `argv[i]` pointer followed to its string,
///    then `argv`'s NULL, the `envp` entry, `envp`'s NULL and the empty
///    auxiliary vector. This is the check that pins down the layout; the
///    first one would pass for any layout `echo.s` happened to agree with.
///    It also requires `rsp % 16 == 0`, which the ABI promises `_start`.
/// 3. That the copy-in refuses what it should: `/argv_errors.bin` holds
///    the results of four execs `echo.s` made with bad vectors -- `argv`
///    on the kernel heap, an element on the kernel heap, too many entries,
///    too many bytes -- each of which had to fail and leave it running.
fn argv_verify(child: i32) {
    let mut out = [0u8; 64];
    let n = read_when_available("/echo_output.txt", &mut out);
    let output = &out[..n.max(0) as usize];
    serial_println!(
        "[idle] read back {:?} from /echo_output.txt (/bin/exectest's own rendering of its argv)",
        core::str::from_utf8(output).unwrap_or("<invalid utf8>")
    );
    assert_eq!(output, b"hello from argv", "/bin/exectest didn't echo the argv shell exec'd it with");

    let mut env = [0u8; 64];
    let n = read_when_available("/echo_env.txt", &mut env);
    let env = &env[..n.max(0) as usize];
    serial_println!(
        "[idle] read back {:?} from /echo_env.txt (/bin/exectest's envp[0])",
        core::str::from_utf8(env).unwrap_or("<invalid utf8>")
    );
    assert_eq!(env, b"GREETING=neumann", "/bin/exectest didn't receive the envp shell exec'd it with");

    let read_u64 = |addr: u64| -> u64 {
        let mut buf = [0u8; 8];
        calls::sys_vircopy(
            child,
            x86_64::VirtAddr::new(addr),
            com::IDLE,
            x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
            8,
        )
        .expect("sys_vircopy failed reading /bin/exectest's start-up block");
        u64::from_le_bytes(buf)
    };
    let read_str = |addr: u64, buf: &mut [u8]| -> usize {
        calls::sys_vircopy(
            child,
            x86_64::VirtAddr::new(addr),
            com::IDLE,
            x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
            buf.len(),
        )
        .expect("sys_vircopy failed reading an argv string");
        buf.iter().position(|&b| b == 0).expect("argv string not NUL-terminated where expected")
    };

    let rsp = read_u64(elf::ECHO_ENTRY_RSP_ADDR);
    let argc_seen = read_u64(elf::ECHO_ARGC_ADDR);
    serial_println!(
        "[idle] /bin/exectest started with rsp {:#x} and read argc {} there (expected 4)",
        rsp,
        argc_seen
    );
    assert_eq!(rsp % 16, 0, "exec'd image's initial rsp isn't 16-byte aligned");
    assert!(
        (elf::STACK_ADDR..elf::STACK_ADDR + elf::STACK_SIZE).contains(&rsp),
        "exec'd image's initial rsp isn't in its stack"
    );
    assert_eq!(argc_seen, 4, "/bin/exectest read the wrong argc off its stack");
    assert_eq!(read_u64(rsp), 4, "argc on the child's stack is wrong");

    let expected_argv: [&[u8]; 4] = [b"/bin/exectest", b"hello", b"from", b"argv"];
    for (i, want) in expected_argv.iter().enumerate() {
        let ptr = read_u64(rsp + 8 + 8 * i as u64);
        // Exact length plus its NUL: long enough to catch a missing
        // terminator, short enough to stay inside the stack page.
        let mut buf = [0xffu8; 16];
        let len = read_str(ptr, &mut buf[..want.len() + 1]);
        assert_eq!(&buf[..len], *want, "argv[{}] on the child's stack is wrong", i);
    }
    let envp = rsp + 8 + 8 * (expected_argv.len() as u64 + 1);
    assert_eq!(read_u64(envp - 8), 0, "argv isn't NULL-terminated on the child's stack");
    let mut buf = [0xffu8; 17];
    let len = read_str(read_u64(envp), &mut buf);
    assert_eq!(&buf[..len], b"GREETING=neumann", "envp[0] on the child's stack is wrong");
    assert_eq!(read_u64(envp + 8), 0, "envp isn't NULL-terminated on the child's stack");
    assert_eq!(read_u64(envp + 16), 0, "the auxiliary vector doesn't start with AT_NULL");
    serial_println!("[idle] /bin/exectest's stack holds argc, argv[4] + NULL, envp[1] + NULL, AT_NULL -- the System V start-up layout");

    let mut errs = [0u8; 32];
    let n = read_when_available("/argv_errors.bin", &mut errs);
    assert_eq!(n, 32, "wrong length read back from /argv_errors.bin");
    let expected = [
        ("argv on the kernel heap", syscall::ERR_BAD_ARG_PTR),
        ("argv[0] on the kernel heap", syscall::ERR_BAD_ARG_PTR),
        ("70 arguments", syscall::ERR_ARGS_TOO_BIG),
        ("30 arguments of 100 bytes", syscall::ERR_ARGS_TOO_BIG),
    ];
    for (i, (what, want)) in expected.iter().enumerate() {
        let got = u64::from_le_bytes(errs[i * 8..i * 8 + 8].try_into().unwrap());
        serial_println!("[idle] ring-3 exec with {}: {:#x} (expected {:#x})", what, got, want);
        assert_eq!(got, *want, "ring-3 exec with {} wasn't refused correctly", what);
    }
}

/// `fs`'s directory listing and descriptor release, checked against a
/// tree built here: `/dirtest` holding a subdirectory and two files,
/// which `readdir` must list subdirectory first, then files in creation
/// order, with the right kinds and sizes, and then report the end; a
/// missing directory is `ENOENT` and a file is `ENOTDIR`. Then `close`:
/// a closed descriptor is `EBADF` to read and to close again. And paths
/// have one spelling: `//`, a trailing `/` and relative paths are
/// `EINVAL` everywhere (`ls` and `mkdir` from the shell made those
/// reachable: `mkdir /bin/` used to create a directory with an empty
/// name).
fn fs_dir_check() {
    assert_eq!(fs::mkdir("/dirtest"), 0);
    assert_eq!(fs::mkdir("/dirtest/sub"), 0);
    for (path, body) in [("/dirtest/a.txt", &b"alpha"[..]), ("/dirtest/b.txt", &b"bravo!"[..])] {
        let fd = fs::open(path);
        assert!(fd >= 0);
        assert_eq!(fs::write(fd, body), body.len() as i64);
        assert_eq!(fs::close(fd), 0);
    }
    let expected: [(&[u8], i64, u64); 3] =
        [(b"sub", fs::KIND_DIR, 0), (b"a.txt", fs::KIND_FILE, 5), (b"b.txt", fs::KIND_FILE, 6)];
    let mut entry = fs::DirEntry::EMPTY;
    for (i, (name, kind, size)) in expected.iter().enumerate() {
        assert_eq!(fs::readdir("/dirtest", i, &mut entry), 1, "readdir /dirtest #{} found nothing", i);
        assert_eq!(entry.name(), *name, "readdir /dirtest #{} has the wrong name", i);
        assert_eq!(entry.kind as i64, *kind, "readdir /dirtest #{} has the wrong kind", i);
        assert_eq!(entry.size, *size, "readdir /dirtest #{} has the wrong size", i);
    }
    assert_eq!(fs::readdir("/dirtest", 3, &mut entry), 0, "readdir /dirtest didn't end after 3 entries");
    assert_eq!(fs::readdir("/dirtest/sub", 0, &mut entry), 0, "an empty directory listed something");
    assert_eq!(fs::readdir("/no_such_dir", 0, &mut entry), fs::ENOENT);
    assert_eq!(fs::readdir("/dirtest/a.txt", 0, &mut entry), fs::ENOTDIR);

    let fd = fs::open_existing("/dirtest/a.txt");
    assert!(fd >= 0);
    assert_eq!(fs::close(fd), 0);
    let mut buf = [0u8; 4];
    assert_eq!(fs::read(fd, &mut buf), fs::EBADF, "a closed descriptor still reads");
    assert_eq!(fs::close(fd), fs::EBADF, "a descriptor closed twice");
    // (Deliberately no "the freed slot is the next one handed out"
    // assertion: the table is shared by every process, and any of them
    // opening a file between the `close` and the `open` would take it.)

    // Paths are compared as whole strings, so only one spelling of each
    // may get in.
    for bad in ["//dirtest", "/dirtest/", "/dirtest//a.txt", "dirtest"] {
        assert_eq!(fs::readdir(bad, 0, &mut entry), fs::EINVAL, "readdir accepted {:?}", bad);
        assert_eq!(fs::mkdir(bad), fs::EINVAL, "mkdir accepted {:?}", bad);
        assert_eq!(fs::open_existing(bad), fs::EINVAL, "open accepted {:?}", bad);
    }
    serial_println!(
        "[idle] fs: readdir /dirtest -> sub/, a.txt (5), b.txt (6), end; missing -> ENOENT, file -> ENOTDIR; a closed fd is EBADF; //, trailing / and relative paths are EINVAL"
    );
}

/// Runs `/bin/ptrtest` (`rust/user/src/bin/ptrtest.rs`) and requires its
/// verdict to be `ok`: twenty-four system calls given deliberately bad
/// pointers -- the kernel heap (mapped in every address space), unmapped
/// memory, the program's own read-only text as a write target -- each of
/// which must come back as an error code. Before `crate::syscall` copied
/// user memory through the caller's page tables, most of them halted the
/// machine with a ring-0 page fault, so reaching this check at all is
/// half the result; the verdict file is the other half, since a call
/// that *succeeded* against the kernel heap wouldn't have faulted either.
///
/// Started straight out of `fs` into a free dynamic slot, with no parent
/// (so nothing waits for it and its slot is released as soon as it
/// exits) -- before `proc_slot_pool_check`, so the slot it borrowed is
/// back in the pool by the time that one counts.
fn ptr_safety_check() {
    let proc_nr = proc::alloc_proc_nr().expect("no free process slot for ptrtest");
    elf::spawn_from_fs("/bin/ptrtest", proc_nr, "ptrtest", 6, 16).expect("couldn't start /bin/ptrtest");
    let mut buf = [0u8; 8];
    let n = read_when_available("/ptrtest.out", &mut buf);
    let verdict = &buf[..n.max(0) as usize];
    serial_println!(
        "[idle] /bin/ptrtest verdict: {:?} (24 syscalls handed bad pointers, process numbers or breaks, each must be refused)",
        core::str::from_utf8(verdict).unwrap_or("<invalid utf8>")
    );
    assert_eq!(verdict, b"ok", "a system call accepted a bad pointer -- see ptrtest's FAIL lines above");
}

/// Runs `/bin/threads` (`rust/user/src/bin/threads.rs`) and requires its
/// verdict to be `ok`: four real kernel threads in one team, a shared
/// counter incremented under a semaphore that has to come out exact
/// despite a critical section built to be preempted in, each join
/// returning that thread's own status, and joining a non-thread refused.
/// Afterwards the program exits with a fifth thread still spinning, and
/// the whole team -- that thread included -- has to be gone, its slots
/// back in the pool and its address space freed exactly once: checked
/// here by requiring every dynamic slot the run used to be free again.
fn threads_check() {
    let before = memory::frame_stats().0;
    let proc_nr = proc::alloc_proc_nr().expect("no free process slot for threads");
    elf::spawn_from_fs("/bin/threads", proc_nr, "threads", 6, 16).expect("couldn't start /bin/threads");
    let mut buf = [0u8; 8];
    let n = read_when_available("/threads.out", &mut buf);
    let verdict = &buf[..n.max(0) as usize];
    serial_println!(
        "[idle] /bin/threads verdict: {:?} (4 threads, a semaphore-guarded counter, joins)",
        core::str::from_utf8(verdict).unwrap_or("<invalid utf8>")
    );
    assert_eq!(verdict, b"ok", "the threads test failed -- see its output above");
    // The team (main thread plus the one left spinning) has to be gone.
    let deadline = proc::uptime_ticks() + 120;
    while proc::is_valid_proc_nr(proc_nr) {
        assert!(proc::uptime_ticks() < deadline, "the threads team never finished exiting");
        proc::yield_now();
    }
    let after = memory::frame_stats().0;
    serial_println!(
        "[idle] the threads team exited with a thread still running; its address space went back ({} frames in use before, {} after)",
        before,
        after
    );
    assert!(after <= before, "the threads team leaked {} frames", after - before);
}

/// Runs `/bin/heaptest` (`rust/user/src/bin/heaptest.rs`) and requires its
/// verdict to be `ok`: `Vec`/`String`/`Box` through `neumann_rt`'s
/// allocator over `SYS_BRK`, the break moving within its range, a forked
/// child's heap writes staying in the child, and three threads sharing
/// the heap -- and, once it has exited, every frame its heap took back.
fn heap_check() {
    let before = memory::frame_stats().0;
    let proc_nr = proc::alloc_proc_nr().expect("no free process slot for heaptest");
    elf::spawn_from_fs("/bin/heaptest", proc_nr, "heaptest", 6, 16).expect("couldn't start /bin/heaptest");
    let mut buf = [0u8; 8];
    let n = read_when_available("/heaptest.out", &mut buf);
    let verdict = &buf[..n.max(0) as usize];
    serial_println!(
        "[idle] /bin/heaptest verdict: {:?} (a 1.6 MB Vec, String, Box, a forked child, 3 threads, all on SYS_BRK)",
        core::str::from_utf8(verdict).unwrap_or("<invalid utf8>")
    );
    assert_eq!(verdict, b"ok", "the heap test failed -- see its output above");
    let deadline = proc::uptime_ticks() + 120;
    while proc::is_valid_proc_nr(proc_nr) {
        assert!(proc::uptime_ticks() < deadline, "heaptest never finished exiting");
        proc::yield_now();
    }
    let after = memory::frame_stats().0;
    serial_println!("[idle] heaptest's heap went back with it ({} frames in use before, {} after)", before, after);
    assert!(after <= before, "heaptest leaked {} frames", after - before);
}

/// Runs `/bin/porttest` (`rust/user/src/bin/porttest.rs`) and requires its
/// verdict to be `ok`: Haiku's port API with Haiku's semantics and status
/// codes -- create/find by name, FIFO order, counts, sizes and
/// `port_info`, truncating reads, `B_WOULD_BLOCK`/`B_TIMED_OUT` on a full
/// or empty port, a producer and consumer thread both blocking on a
/// 2-deep port, a message from a forked process, close (drain then
/// `B_BAD_PORT_ID`), delete waking a blocked reader, and a dead team's
/// port deleted with it.
fn port_check() {
    let proc_nr = proc::alloc_proc_nr().expect("no free process slot for porttest");
    elf::spawn_from_fs("/bin/porttest", proc_nr, "porttest", 6, 16).expect("couldn't start /bin/porttest");
    let mut buf = [0u8; 8];
    let n = read_when_available("/porttest.out", &mut buf);
    let verdict = &buf[..n.max(0) as usize];
    serial_println!(
        "[idle] /bin/porttest verdict: {:?} (Haiku's port API: queues, timeouts, threads, processes, close/delete)",
        core::str::from_utf8(verdict).unwrap_or("<invalid utf8>")
    );
    assert_eq!(verdict, b"ok", "the port test failed -- see its output above");
    let deadline = proc::uptime_ticks() + 120;
    while proc::is_valid_proc_nr(proc_nr) {
        assert!(proc::uptime_ticks() < deadline, "porttest never finished exiting");
        proc::yield_now();
    }
}

/// Exercises the two ends of the dynamic process-number pool
/// (`proc::alloc_proc_nr`/`release_proc_nr`) that a working boot doesn't
/// reach: what happens when it runs out, and whether a number handed
/// back is really available again.
///
/// Neither is reachable from anything this port does on its own. Every
/// fork here succeeds, because `com::NR_DYNAMIC_PROCS` is larger than
/// the number of processes that fork (two at boot, a third if a service
/// is launched from the console), so `crate::syscall`'s
/// `ERR_NO_FREE_PROC` and the `release_proc_nr` call on the
/// address-space-build failure path are both dead code in practice --
/// exactly the sort of path that is wrong the first time it is ever
/// needed. So: claim every remaining number until the pool says no,
/// check it said no only once there was nothing left, give them all
/// back, and check the pool is willing to hand out the same count again.
///
/// Runs after `fork_child_verify`/`exec_verify`, which need the real
/// forked children's slots intact, and leaves the pool exactly as it
/// found it so a console-launched service can still fork afterwards.
fn proc_slot_pool_check() {
    let mut claimed = [0i32; com::NR_DYNAMIC_PROCS];
    let mut n = 0;
    while let Some(proc_nr) = proc::alloc_proc_nr() {
        assert!(n < com::NR_DYNAMIC_PROCS, "the pool handed out more numbers than it has");
        claimed[n] = proc_nr;
        n += 1;
    }
    serial_println!(
        "[idle] process-number pool: claimed the {} remaining dynamic slot(s) ({:?}), then it correctly refused",
        n,
        &claimed[..n]
    );
    for &proc_nr in claimed[..n].iter() {
        proc::release_proc_nr(proc_nr);
    }
    let again = proc::alloc_proc_nr();
    assert_eq!(
        again, claimed[..n].first().copied(),
        "a released process number wasn't the next one handed out again"
    );
    if let Some(proc_nr) = again {
        proc::release_proc_nr(proc_nr);
    }
}

/// The boot-time `frame_reclaim_self_test` runs in an empty system;
/// this runs the same build-and-tear-down cycle at the *end* of a real
/// workload -- after four `flaky` crash/kill/restart rounds, a real
/// `exec`, a `fork`, and every demo task's allocations -- and requires
/// the allocator to still balance. A reclaim bug that only appeared once
/// frames had actually been recycled (which is the interesting case:
/// `flaky`'s kills are what first put frames on the free list) would
/// show up here and not there.
///
/// Also reports the final accounting, which is the one number that says
/// whether this port still leaks: before this change every one of
/// `flaky`'s restarts cost an address space permanently.
fn runtime_reclaim_check() {
    let (before, free_before) = memory::frame_stats();
    let kernel_pml4 = x86_64::registers::control::Cr3::read().0;
    for _ in 0..3 {
        let image = elf::load_image(kernel_pml4, elf::SHELL_ELF).expect("SHELL_ELF should load");
        // Safety: built here, never loaded into CR3, referenced by nothing.
        unsafe { memory::free_address_space(image.pml4, kernel_pml4) };
    }
    let (after, free_after) = memory::frame_stats();
    serial_println!(
        "[idle] frames in use: {} (free list {}) -- unchanged after three more address-space cycles: {} (free list {})",
        before,
        free_before,
        after,
        free_after
    );
    assert_eq!(after, before, "address-space teardown leaks once frames are being recycled");
}

/// Read `path`, waiting until it both exists *and* has content. `fs`
/// starts empty every boot and this port has no "wait for a file"
/// primitive (no `select`, no inotify, nothing), so the wait is a plain
/// poll bounded by a real deadline (see the tick comment below).
///
/// `fs::open_existing`, not `fs::open` -- and that distinction is the
/// whole reason this function works. `fs::open` creates a missing file
/// (`crate::fs`'s `open_path`), so polling with it would succeed on the
/// first attempt no matter what, read zero bytes, and leave behind an
/// empty file at the very path it was waiting for: a "wait" that can
/// never wait, and that destroys the evidence it was checking for. The
/// zero-length retry covers the other half of the same race -- the
/// writer having opened the file but not yet written to it.
fn read_when_available(path: &str, buf: &mut [u8]) -> i64 {
    // Bounded in real ticks, not in attempts. An attempt count is the
    // wrong unit here: `yield_now` from `IDLE` with every other task
    // asleep re-picks `IDLE` and returns immediately, so five hundred
    // attempts can elapse inside a single timer tick -- while the task
    // being waited for is blocked on a one-tick alarm and hasn't had a
    // chance to run at all. (Observed, not hypothetical: with `pm`'s
    // seeding artificially delayed, this gave up before `shell` had
    // retried its exec even twice.) Spinning on `yield_now` until the
    // clock moves is what actually lets the other task make progress --
    // and it has to be a spin rather than a real `sys_setalarm` sleep,
    // because `IDLE` blocking would leave the scheduler with nothing
    // runnable at all.
    const MAX_TICKS: u64 = 300; // 5 seconds at pit::HZ
    let deadline = proc::uptime_ticks() + MAX_TICKS;
    loop {
        let fd = fs::open_existing(path);
        if fd >= 0 {
            // A file that exists but is still empty means the writer got
            // as far as `open` and no further -- the same race, one step
            // later.
            let n = fs::read(fd, buf);
            // Every poll opens a fresh descriptor; close it either way,
            // or waiting for a file costs one `fs` slot per retry.
            fs::close(fd);
            if n != 0 {
                return n;
            }
        }
        if proc::uptime_ticks() >= deadline {
            panic!("{} never appeared within {} ticks", path, MAX_TICKS);
        }
        proc::yield_now();
    }
}

/// Stand-in for `kernel/clock.c`'s clock task. Now much closer to the real
/// thing than earlier milestones' version: it calls `sys_setalarm`
/// (`crate::calls`) and blocks in `receive`, exactly like real MINIX's
/// `while (TRUE) { receive(HARDWARE, &m); ...; }` waiting on
/// `do_clocktick`'s `lock_notify`, instead of polling
/// `proc::uptime_ticks()` in a spin-yield loop. `crate::proc::clock_tick`
/// is what actually notices the deadline and delivers the `SYN_ALARM`
/// that wakes this up.
///
/// Also demonstrates `sys_vircopy` (see `vircopy_demo` below) before
/// settling down, since `CLOCK` is a convenient, deterministic place to
/// run it: the ring-3 task's code page is populated in `kernel_main`
/// before any task ever runs, so this works regardless of scheduling
/// order, without needing to coordinate with that task directly.
/// (`elf_counter_demo`/`vircopy_from_ring3_verify` below used to run from
/// here too, racing `CLOCK`'s own fixed 3-tick alarm against `tty`'s
/// actual progress through the ready queue -- "true in practice" until it
/// wasn't; see `idle_task` for where they live now and why that's
/// actually safe.) Then demonstrates the other real-servers prerequisite
/// this milestone adds:
/// dynamically spawning a brand new task (`log`) at runtime, with the
/// scheduler already running other tasks -- the primitive real `rs`
/// (starting services on demand, not just at boot) actually needs.
fn clock_task() -> ! {
    calls::sys_setalarm(3);
    let notif = ipc::receive(com::CLOCK);
    assert_eq!(notif.m_type, calls::SYN_ALARM, "expected a SYN_ALARM notification");
    serial_println!(
        "[clock] woken by a real SYN_ALARM notification (m_type {:#x}) after {} real PIT ticks",
        notif.m_type,
        proc::uptime_ticks()
    );

    vircopy_demo();

    serial_println!("[clock] dynamically spawning a new task (log) at runtime");
    proc::spawn(com::LOG_PROC_NR, "log (dynamic)", dynamic_log_task, 5, 24, true, None);

    ipc::receive(com::ANY); // nothing left to receive; parks CLOCK for good
    unreachable!("nothing sends to CLOCK in this demo");
}

/// Spawned at runtime by `clock_task`, well after `kernel_main` has handed
/// off to the scheduler and other tasks are already running -- proving
/// `proc::spawn` genuinely works as a dynamic "start a new process now"
/// primitive, not just as boot-time setup.
fn dynamic_log_task() -> ! {
    serial_println!(
        "[log] dynamically spawned at runtime (proc_nr {}, uptime {} ticks)",
        proc::current_proc_nr(),
        proc::uptime_ticks()
    );
    ipc::receive(com::ANY); // nothing left to receive; parks log for good
    unreachable!("nothing sends to log in this demo");
}

/// Proves `sys_vircopy` (`crate::calls`) genuinely translates through a
/// *different* process's page table rather than the currently-active one:
/// reads the ring-3 demo task's code page back out of its own, private
/// address space (built in `kernel_main`) into a local kernel buffer, and
/// checks the bytes match what `usermode::create_address_space` wrote
/// there. `CLOCK` runs entirely in the kernel's own address space, so this
/// is a genuine cross-address-space copy, not a same-space one in
/// disguise.
fn vircopy_demo() {
    let mut buf = [0u8; usermode::USER_CODE.len()];
    calls::sys_vircopy(
        com::DRVR_PROC_NR,
        x86_64::VirtAddr::new(usermode::USER_CODE_ADDR),
        com::CLOCK,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed");
    serial_println!(
        "[clock] sys_vircopy read back {:?} from the ring-3 task's own address space (expected {:?})",
        buf,
        usermode::USER_CODE
    );
    assert_eq!(buf, usermode::USER_CODE, "sys_vircopy returned the wrong bytes");
}

/// Proves `elf::HELLO_ELF`'s loaded code genuinely ran -- not just that
/// it made the syscalls `crate::syscall::dispatch` logged, but that its
/// own `incl counter(%rip)` instruction, executing out of pages this
/// port's own ELF loader mapped (not the kernel poking bytes in
/// directly, like `usermode`'s demo), actually wrote through to physical
/// memory. Reads `tty`'s copy of its own `.data` counter back into a
/// local buffer via `sys_vircopy` and checks it against the five
/// `SYS_GET_UPTIME` calls `user/hello.s` makes before moving on to
/// `SYS_WRITE_LINE`/`SYS_BLOCK_FOREVER`.
///
/// Runs from `idle_task` (see its doc comment for why that's a genuinely
/// safe checkpoint, not a race) rather than `clock_task`: an earlier
/// version ran this right after `CLOCK`'s own fixed 3-tick alarm fired,
/// racing that independent timer against `tty`'s actual progress through
/// the ready queue -- "true in practice" until a run of real boots under
/// heavier host load reproducibly proved it wasn't (an observed panic:
/// `counter` read back `0`, not `5`), the same class of "two independent
/// timers, assumed-safe interleaving" bug the scheduler's own
/// `reschedule`/`start` race (see "A real bug found and fixed by a
/// multi-agent review" above) already showed up once in this port.
fn elf_counter_demo() {
    let mut buf = [0u8; 4];
    calls::sys_vircopy(
        com::TTY_PROC_NR,
        x86_64::VirtAddr::new(elf::COUNTER_ADDR),
        com::IDLE,
        x86_64::VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .expect("sys_vircopy failed reading the ELF task's counter");
    let counter = i32::from_le_bytes(buf);
    serial_println!(
        "[idle] sys_vircopy read back the ELF-loaded task's own .data counter: {} (expected 5)",
        counter
    );
    assert_eq!(
        counter, 5,
        "the loaded ELF binary's own code should have incremented its counter 5 times"
    );
}

/// Temporary stand-in for the `pm` server (see `spawn_tasks`): proves
/// blocking `send`/`receive` by pinging `fs` a few times and printing each
/// reply, then demonstrates `sys_fork` (see `fork_demo` below) -- fitting,
/// since real `pm` is exactly who would call it.
fn demo_pm_task() -> ! {
    for i in 0..3 {
        serial_println!("[pm] sending ping {} to fs", i);
        ipc::send(com::FS_PROC_NR, ipc::Message { source: com::PM_PROC_NR, m_type: 100, args: [i, 0, 0, 0] });
        let reply = ipc::receive(com::FS_PROC_NR);
        serial_println!("[pm] got reply from fs: {:?}", reply);
    }

    seed_bin();
    missing_program_check();
    fork_demo();
    fs_rw_demo();

    serial_println!("[pm] demo finished, blocking for good");
    ipc::receive(com::ANY); // nothing left to receive; parks pm so idle can run
    unreachable!("nothing sends to pm once the demo is done");
}

/// Install the two runnable programs into `fs` -- the "the binaries are
/// on disk" step a real package manager (or an install CD) would
/// otherwise have done. `fs` is in-memory and starts empty every boot, so
/// nothing can be loaded *by path* until a task has put it there:
/// `/bin/hello` is what `crate::rs`'s `SERVICES` table launches on
/// demand (`elf::spawn_from_fs`), and `/bin/exectest` is what `shell` execs
/// itself into (`crate::calls::sys_exec`, via `user/shell.s`). Must run
/// from a task, not `kernel_main` directly: `fs::open`/`fs::write` block
/// on a real IPC round trip, which needs a task context to block in.
///
/// Runs as early in `pm`'s demo as it can rather than at the end, since
/// `shell` is waiting on `/bin/exectest` to appear before it can get on with
/// its own job. "As early as it can" is immediately after the
/// `pm`/`fs` ping-pong: `fs` only becomes a real file server once those
/// three messages are done with (`demo_fs_task`), and a `mkdir` sent
/// before that gets answered by the ping-pong stand-in instead, which
/// replies to anything at all with `ping + 100`. It doesn't have to be
/// early for correctness -- `user/shell.s` retries rather than assuming
/// an ordering, deliberately (see its header comment) -- but there's no
/// reason to make it sit through pointless retries either.
fn seed_bin() {
    let rc = fs::mkdir("/bin");
    assert_eq!(rc, 0, "fs::mkdir(\"/bin\") failed: {}", rc);
    let asm_programs = [("/bin/hello", elf::HELLO_ELF), ("/bin/exectest", elf::ECHO_ELF)];
    for &(path, image) in asm_programs.iter().chain(elf::RUST_PROGRAMS.iter()) {
        let fd = fs::open(path);
        assert!(fd >= 0, "fs::open({:?}) failed: {}", path, fd);
        let n = fs::write(fd, image);
        fs::close(fd);
        serial_println!("[pm] seeded {} with {} bytes", path, n);
        assert_eq!(n, image.len() as i64, "fs::write didn't accept the whole ELF image");
    }
}

/// Proves that looking for a program that isn't there reports `ENOENT`
/// and *changes nothing* -- the second half of "a failed exec is a
/// no-op", and the half that isn't about the caller's own image.
///
/// Worth checking explicitly because `fs::open` creates the file it's
/// asked for (`crate::fs`'s `open_path`), so the obvious implementation
/// of `elf::read_file` had exec of a missing path succeed at opening
/// nothing, read zero bytes, report the *wrong* error (`ERR_BAD_ELF`,
/// as if the program were malformed rather than absent), and leave a
/// stray empty file behind at whatever path ring 3 named -- which a
/// ring-3 caller could repeat to grow `fs` without bound.
fn missing_program_check() {
    const MISSING: &str = "/no_such_program";
    assert_eq!(fs::open_existing(MISSING), fs::ENOENT, "{} exists before the test?", MISSING);
    let result = elf::read_file(MISSING);
    assert!(
        matches!(result, Err(fs::ENOENT)),
        "reading a program that doesn't exist should be ENOENT, got {:?}",
        result.map(|image| image.len())
    );
    assert_eq!(
        fs::open_existing(MISSING),
        fs::ENOENT,
        "looking for a missing program created it -- a failed exec has to change nothing"
    );
    serial_println!("[pm] reading {:?} -> ENOENT, and no empty file left behind", MISSING);
}

/// Proves `fs` (see `spawn_tasks`, now `fs::InMemoryFs::serve` instead of
/// a ping-pong stand-in) genuinely serves open/write/read requests over
/// IPC, backed by real in-memory storage rather than just echoing back
/// whatever it was sent: opens a file, writes to it, reopens it fresh
/// (a distinct file descriptor, its own cursor starting at 0) and reads
/// the bytes back, checking they round-trip -- then reads once more past
/// end of file and checks that comes back empty rather than repeating
/// data or blocking forever. Then exercises the real directory hierarchy
/// (`fs_directory_demo`): a path can't be opened until its parent
/// directory actually exists.
fn fs_rw_demo() {
    let written = b"hello from pm, stored in fs";

    let write_fd = fs::open("/hello.txt");
    assert!(write_fd >= 0, "fs::open failed: {}", write_fd);
    let n = fs::write(write_fd, written);
    serial_println!("[pm] wrote {} bytes to /hello.txt via fs (fd {})", n, write_fd);
    assert_eq!(n, written.len() as i64, "fs::write didn't accept the whole buffer");

    let read_fd = fs::open("/hello.txt");
    assert!(read_fd >= 0, "fs::open (reopen) failed: {}", read_fd);
    assert_ne!(read_fd, write_fd, "reopening the same file should hand back a fresh descriptor");
    let mut buf = [0u8; 64];
    let n = fs::read(read_fd, &mut buf);
    assert_eq!(n, written.len() as i64, "fs::read returned the wrong length");
    let round_tripped = &buf[..n as usize];
    serial_println!(
        "[pm] read back {:?} from fs (expected {:?})",
        core::str::from_utf8(round_tripped).unwrap_or("<invalid utf8>"),
        core::str::from_utf8(written).unwrap_or("<invalid utf8>")
    );
    assert_eq!(round_tripped, written, "fs did not return the bytes that were written");

    let n = fs::read(read_fd, &mut buf);
    serial_println!("[pm] read past end of file returned {} bytes (expected 0)", n);
    assert_eq!(n, 0, "reading past end of file should return 0, not repeat data or error");

    fs_directory_demo();
}

/// Proves `fs`'s directory hierarchy (`fs::InMemoryFs::mkdir`/`open_path`)
/// enforces the usual POSIX parent-directory rules, not just a flat,
/// exact-match namespace: a path under a directory that doesn't exist yet
/// is rejected (`ENOENT`), creating that directory makes the same open
/// succeed, creating it again fails (`EEXIST`), and a path that treats an
/// ordinary file as if it were a directory is rejected (`ENOTDIR`).
fn fs_directory_demo() {
    let missing = fs::open("/logs/today.txt");
    serial_println!("[pm] fs::open(\"/logs/today.txt\") before mkdir -> {} (expected ENOENT)", missing);
    assert_eq!(missing, fs::ENOENT, "opening a path under a nonexistent directory should fail with ENOENT");

    let made = fs::mkdir("/logs");
    serial_println!("[pm] fs::mkdir(\"/logs\") -> {} (expected 0)", made);
    assert_eq!(made, 0, "mkdir on a fresh path under an existing parent (\"/\") should succeed");

    let again = fs::mkdir("/logs");
    serial_println!("[pm] fs::mkdir(\"/logs\") again -> {} (expected EEXIST)", again);
    assert_eq!(again, fs::EEXIST, "mkdir on an already-existing path should fail with EEXIST");

    let as_dir = fs::open("/logs");
    serial_println!("[pm] fs::open(\"/logs\") (a directory) -> {} (expected EISDIR)", as_dir);
    assert_eq!(as_dir, fs::EISDIR, "opening a directory as if it were a file should fail with EISDIR");

    let fd = fs::open("/logs/today.txt");
    serial_println!("[pm] fs::open(\"/logs/today.txt\") after mkdir -> fd {} (expected >= 0)", fd);
    assert!(fd >= 0, "opening a path under a directory that now exists should succeed");

    let under_file = fs::open("/hello.txt/nested.txt");
    serial_println!(
        "[pm] fs::open(\"/hello.txt/nested.txt\") (parent is a file, not a directory) -> {} (expected ENOTDIR)",
        under_file
    );
    assert_eq!(
        under_file,
        fs::ENOTDIR,
        "opening a path under an existing *file* should fail with ENOTDIR, not silently succeed"
    );
}

/// Proves `sys_fork` (`crate::calls`) gives the child a genuinely
/// independent *copy* of the pages it inherits, not just another alias of
/// the same physical memory: forks the ring-3 demo task's whole address
/// space into a new child (`init`), overwrites the *child's* copy of its
/// code page with a canary value, then reads back both copies. If fork
/// only cloned the top-level page table
/// (`memory::new_address_space` alone) rather than deep-copying each page
/// in the parent's memory map (`memory::fork_address_space`), the
/// original task's page would show the canary too, since both sides would
/// still be the same physical frame.
///
/// Which pages get copied is no longer stated here: it comes from
/// `driver`'s own memory map (`proc::mem_map_of`, recorded when
/// `usermode::build_ring3_address_space` mapped them), so this demo
/// exercises the same discovery path a ring-3 `SYS_FORK` does rather than
/// naming the one page it intends to check.
fn fork_demo() {
    let code_addr = x86_64::VirtAddr::new(usermode::USER_CODE_ADDR);

    serial_println!(
        "[pm] forking the ring-3 task's address space ({} pages, from its own memory map) into a new child (init)",
        proc::mem_map_of(com::DRVR_PROC_NR).total_pages()
    );
    assert!(
        calls::sys_fork(
            com::DRVR_PROC_NR,
            com::INIT_PROC_NR,
            "init (forked child)",
            forked_child_task,
            5,
            24,
            true,
        ),
        "sys_fork failed to build the child's address space"
    );

    let canary: [u8; 4] = [0xAA, 0xBB, 0xCC, 0xDD];
    calls::sys_vircopy(
        com::PM_PROC_NR,
        x86_64::VirtAddr::new(canary.as_ptr() as u64),
        com::INIT_PROC_NR,
        code_addr,
        canary.len(),
    )
    .expect("writing the canary into the forked child's page failed");

    let mut child_copy = [0u8; 4];
    calls::sys_vircopy(
        com::INIT_PROC_NR,
        code_addr,
        com::PM_PROC_NR,
        x86_64::VirtAddr::new(child_copy.as_mut_ptr() as u64),
        4,
    )
    .expect("reading back the child's page failed");

    let mut original_copy = [0u8; 4];
    calls::sys_vircopy(
        com::DRVR_PROC_NR,
        code_addr,
        com::PM_PROC_NR,
        x86_64::VirtAddr::new(original_copy.as_mut_ptr() as u64),
        4,
    )
    .expect("reading back the original task's page failed");

    serial_println!(
        "[pm] after writing a canary into the forked child's copy: child={:?}, original={:?} (unchanged: {:?})",
        child_copy,
        original_copy,
        usermode::USER_CODE
    );
    assert_eq!(child_copy, canary, "the forked child's page didn't receive the write");
    assert_eq!(
        original_copy,
        usermode::USER_CODE[..4],
        "fork did not give the child an independent copy -- the write leaked into the original task's page!"
    );
}

/// The child `sys_fork` creates in `fork_demo`. Its code page was
/// overwritten with a canary value for the divergence test, so it
/// deliberately doesn't try to jump to ring 3 and execute it (it's no
/// longer valid `int 0x80` machine code) -- it exists only to prove the
/// process-table/scheduler side of `sys_fork` works, alongside the memory
/// side `fork_demo` checks directly.
fn forked_child_task() -> ! {
    serial_println!("[init] forked child task running (proc_nr {})", proc::current_proc_nr());
    ipc::receive(com::ANY); // nothing left to receive; parks init for good
    unreachable!("nothing sends to init in this demo");
}

/// `fs` (see `spawn_tasks`): still starts with the same ping/pong
/// `demo_pm_task` opens with (proving plain blocking `send`/`receive`
/// still works), then becomes a real file server -- an
/// `fs::InMemoryFs` that genuinely stores and serves open/read/write
/// requests over IPC (`fs_rw_demo`), rather than a stand-in that only
/// ever echoes a fixed reply.
fn demo_fs_task() -> ! {
    for _ in 0..3 {
        let ping = ipc::receive(com::PM_PROC_NR);
        serial_println!("[fs] received ping from pm: {:?}", ping);
        ipc::send(
            com::PM_PROC_NR,
            ipc::Message { source: com::FS_PROC_NR, m_type: 200, args: [ping.args[0] + 100, 0, 0, 0] },
        );
    }
    serial_println!("[fs] ping/pong done, now serving real open/read/write requests");
    fs::InMemoryFs::new().serve()
}

/// Proof of asynchronous preemption: a tight, CPU-bound loop with no
/// `yield_now()` or IPC call anywhere in it, spun forever until its own
/// timeout. The only way this ever gets interrupted at all is the timer
/// forcibly reordering the ready queue and switching away once its
/// quantum is used up (`proc::clock_tick` + `proc::reschedule`, called
/// from `crate::interrupts::timer_interrupt_handler`) -- with the
/// previous milestone's purely-cooperative scheduler, this would simply
/// never yield the CPU to anything else. (An earlier version of this
/// demo paired two of these -- `rs` and `memory` -- sharing a priority
/// queue, so their forced trade-off was visible in the log; `rs` has
/// since become a real task (`rs::task`), so `memory` now demonstrates
/// this alone -- its counter still resumes from exactly where it left
/// off after every other runnable task, including `rs`'s restart
/// activity, gets its turn.)
fn busy_task() -> ! {
    busy_loop("memory")
}

fn busy_loop(tag: &str) -> ! {
    let start = proc::uptime_ticks();
    let mut n: u64 = 0;
    loop {
        n = n.wrapping_add(1);
        if n % 4_000_000 == 0 {
            let now = proc::uptime_ticks();
            serial_println!("[{}] still spinning, n={} uptime={}", tag, n, now);
            if now >= start + 30 {
                break;
            }
        }
    }
    serial_println!("[{}] demo finished, blocking for good", tag);
    ipc::receive(com::ANY); // nothing left to receive; parks this task so idle can run
    unreachable!("nothing sends to {} once the demo is done", tag);
}

pub fn halt_loop() -> ! {
    loop {
        x86_64::instructions::hlt();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    serial_println!("KERNEL PANIC: {}", info);
    halt_loop()
}

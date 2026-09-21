//! Process table and scheduler.
//!
//! Rust port of `kernel/proc.h` and the scheduling half of `kernel/proc.c`:
//! a fixed-size table of process control blocks, `NR_SCHED_QUEUES`
//! priority-ordered ready queues, and `enqueue`/`dequeue`/`sched`/
//! `pick_proc`, ported near-verbatim from the C (see the doc comment on
//! each for the C original). `crate::ipc` builds the rendezvous IPC
//! algorithm (`mini_send`/`mini_receive`/`mini_notify`) on top of the
//! primitives here.
//!
//! One thing *is* different from the C kernel: there, `enqueue`/`dequeue`
//! only ever update `next_ptr`, and the actual context switch happens
//! unconditionally whenever the kernel returns from a trap or interrupt
//! (the `restart()` assembly in `kernel/mpx386.s` compares `next_ptr`
//! against `proc_ptr` on every single trap return). This port has no
//! single, central "trap return" choke point to hook that check into, so
//! instead every place that can change `next_ptr` -- `mini_send`,
//! `mini_receive`, `mini_notify`, `yield_now`, and the timer interrupt
//! handler's `clock_tick` -- calls `reschedule()` itself right afterwards,
//! which performs the switch immediately if `next_ptr` now differs from
//! the running task, or does nothing otherwise. Calling it unconditionally
//! after every one of those (not just the ones where the *caller* blocks)
//! means a task that just woke a higher-priority peer is preempted right
//! away, same as real MINIX's next trap return would.
//!
//! Critically, `reschedule()` (via `switch_to`) is called both from
//! ordinary task code *and* from inside the timer interrupt handler --
//! true asynchronous preemption, not just cooperative handoffs. Making
//! that safe took one addition beyond what a purely cooperative scheduler
//! needs: `switch_to` saves and restores `RFLAGS` (in particular the
//! interrupt flag) per task, alongside the callee-saved registers. Without
//! that, a switch triggered from inside the interrupt handler (where
//! interrupts are necessarily off) would leave interrupts looking
//! permanently disabled to whichever task got switched to -- restoring the
//! flags each task actually had at its own last suspension point is what
//! keeps the two switching paths (voluntary and interrupt-driven)
//! consistent with each other. See `switch_to`'s doc comment.
//!
//! Because a hardware interrupt can now land in the middle of any of the
//! functions below, every one of them that touches `SCHEDULER` does so with
//! interrupts disabled (`with_scheduler`) -- otherwise the timer interrupt
//! firing while, say, `mini_send` already holds the lock would deadlock
//! `clock_tick` spinning for a lock its own interrupted context can't let
//! go of until the handler returns.

use core::ptr;
use spin::Mutex;
use x86_64::instructions::interrupts::{are_enabled, disable, enable, without_interrupts};
use x86_64::registers::control::{Cr3, Cr3Flags};
use x86_64::structures::paging::PhysFrame;
use x86_64::VirtAddr;

use crate::com;
use crate::gdt;
use crate::ipc::Message;
use crate::memory::MemMap;

pub const NR_SCHED_QUEUES: usize = 16;
pub const TASK_Q: u8 = 0;
pub const IDLE_Q: u8 = (NR_SCHED_QUEUES - 1) as u8;

const STACK_SIZE: usize = 4096 * 8;

/// One slot per schedulable kernel task, plus one reserved slot for the
/// bootstrap context (`kernel_main`'s own stack, abandoned once it hands
/// off to the first task via `start()`).
const NR_PROCS: usize = com::NR_PROC_SLOTS + 1;
const BOOTSTRAP: usize = NR_PROCS - 1;

/// Bits for `Proc::rts_flags`, mirroring `SLOT_FREE`/`SENDING`/`RECEIVING`
/// in `kernel/proc.h`. A process is runnable iff `rts_flags == 0`.
pub mod rts {
    pub const SLOT_FREE: u8 = 0x01;
    pub const SENDING: u8 = 0x04;
    pub const RECEIVING: u8 = 0x08;
    /// Set by `crate::proc::kill` (a fault in a ring-3 task -- see
    /// `crate::interrupts`) and never cleared: this slot is permanently
    /// off every ready queue until a fresh `spawn()` overwrites it
    /// wholesale. No real MINIX `rts_flags` bit is quite this either --
    /// there, a dead process's slot is simply freed back to `SLOT_FREE`
    /// once `RS`/`PM` finish tearing it down; this port has no teardown
    /// step (no memory to free -- see `crate::rs`), so `DEAD` just marks
    /// "never schedule this slot again" until it's respawned.
    pub const DEAD: u8 = 0x10;
    /// Exited (or was killed) and still holds an uncollected exit
    /// status: a zombie, `mp_flags & ZOMBIE` in `servers/pm/mproc.h`.
    /// The slot -- and so the process number -- stays allocated until a
    /// parent collects the status (`crate::proc::wait_for_child`), which
    /// is the whole reason zombies exist: the status has to outlive the
    /// process it describes.
    pub const ZOMBIE: u8 = 0x40;
    /// Blocked in `crate::proc::wait_for_child` until one of this
    /// process's children terminates. `mp_flags & WAITING` in
    /// `servers/pm/mproc.h`; a distinct state from `RECEIVING` because
    /// what wakes it is a child exiting, not a message arriving.
    pub const WAITING: u8 = 0x80;
    /// Claimed by `crate::proc::alloc_proc_nr` but not yet filled in by
    /// the `spawn`/`fork_current` that asked for it: not free (so a
    /// second allocation skips it) and not runnable (so the scheduler
    /// never picks a slot that holds nothing). No MINIX counterpart --
    /// there, `PM` owns process-slot allocation and the kernel only ever
    /// sees a slot that is already a real process.
    pub const RESERVED: u8 = 0x20;
}

/// Everything a process's own address space consists of: the top-level
/// page table to load into `CR3`, and the memory map
/// (`crate::memory::MemMap`) saying which pages in it are the process's
/// own rather than the kernel's. The two travel together everywhere --
/// whoever builds an address space (`crate::usermode`, `crate::elf`)
/// knows both, and `fork` needs both -- so they are one value rather
/// than two parallel arguments that could get out of step.
#[derive(Clone, Copy)]
pub struct AddressSpace {
    pub pml4: PhysFrame,
    pub map: MemMap,
}

pub struct Proc {
    pub proc_nr: i32,
    pub name: &'static str,
    rts_flags: u8,
    priority: u8,
    max_priority: u8,
    ticks_left: i32,
    quantum_size: i32,
    /// Whether `clock_tick` should count this task's quantum down at all.
    /// Simplified stand-in for the `PREEMPTIBLE` bit in `kernel/priv.h`'s
    /// `struct priv` (not ported yet -- see `rust/README.md`): real MINIX
    /// kernel tasks other than IDLE (`CLOCK`, `SYSTEM`) run to completion or
    /// block voluntarily and are never charged for elapsed ticks.
    preemptible: bool,
    /// Saved kernel stack pointer. Valid only while this process is *not*
    /// the one currently executing (`kernel/proc.h`'s `p_reg` plays the same
    /// role, but saves the full register set explicitly; here the registers
    /// live on the process's own stack between switches, and this is just
    /// where that stack currently ends).
    rsp: u64,
    /// `p_nextready`: next process in this priority's ready queue.
    next_ready: Option<usize>,
    /// `p_caller_q`: head of the list of processes blocked trying to `SEND`
    /// to this one.
    caller_q: Option<usize>,
    /// `p_q_link`: this process's link to the next process in some other
    /// process's `caller_q`.
    q_link: Option<usize>,
    /// `p_getfrom`: source this process wants to `RECEIVE` from, if blocked.
    get_from: i32,
    /// `p_sendto`: destination this process is blocked trying to `SEND` to.
    send_to: i32,
    /// `p_messbuf`: pointer to the caller's own message buffer. Sound
    /// because IPC always happens in kernel context (even for a ring-3
    /// task, `mini_send`/`mini_receive` only ever run from inside a trap
    /// handler), where every task's memory is visible regardless of whose
    /// `cr3` happens to be loaded (`new_address_space` clones the
    /// kernel's mappings into every task-private table -- see
    /// `crate::memory`) -- and a blocked process's stack frame (and thus
    /// this pointer's target) stays alive, untouched, for exactly as long
    /// as it remains blocked.
    messbuf: *mut Message,
    /// This task's entry point. Not part of `struct proc` in the C kernel
    /// (there, `p_reg.pc` -- the saved instruction pointer -- serves the
    /// same purpose once execution is underway); kept separately here so
    /// `trampoline` can find it the first time this task is switched to.
    entry: fn() -> !,
    /// This task's own address space, if it has one distinct from the
    /// kernel's (`crate::memory::new_address_space`) -- `None` for every
    /// task so far except the ring-3 demo (`crate::usermode`), which is
    /// the only one with anything private to isolate. `reschedule`/
    /// `start` load this (or the kernel's own, if `None`) into `CR3`
    /// whenever this task becomes current.
    cr3: Option<PhysFrame>,
    /// Which pages of `cr3`'s address space belong to this process
    /// rather than to the kernel -- `MemMap::EMPTY` for a task with no
    /// address space of its own. The Rust counterpart of `mp_seg[]` in
    /// `servers/pm/mproc.h`, kept in the process table here rather than
    /// in a `pm` of its own because `fork` is a kernel call in this port
    /// (`crate::calls::sys_fork_from_frame`) and the kernel is therefore
    /// who needs to read it.
    mem_map: MemMap,
    /// The process that forked this one, or `com::NONE` for every
    /// process in the fixed system image (`crate::table`), which nothing
    /// created. `mp_parent` in `servers/pm/mproc.h`; here it exists so
    /// that a process whose number was handed out at runtime
    /// (`alloc_proc_nr`) can still be found from outside by asking who
    /// its parent is (`child_of`), rather than needing its number
    /// hardcoded somewhere.
    parent: i32,
    /// The status this process terminated with, while nobody has
    /// collected it yet -- `mp_exitstatus` in `servers/pm/mproc.h`, and
    /// the payload a zombie slot (`rts::ZOMBIE`) exists to hold.
    exit_status: Option<i32>,
    /// A terminated child's `(proc_nr, status)`, handed to this process
    /// by the child itself (`Scheduler::terminate`) because this process
    /// was already blocked in `wait_for_child` when it happened. The
    /// counterpart of `messbuf` for `wait` rather than IPC: the child
    /// writes the answer where the waiter will look for it, then wakes
    /// it, so no zombie needs to exist in this (much more common) case.
    wait_result: Option<(i32, i32)>,
    /// The tick (`Scheduler::ticks`) at which this task's watchdog alarm
    /// (`sys_setalarm`, `crate::calls`) should fire, if one is pending.
    /// Analogous to `kernel/clock.c`'s per-process alarm timer, minus the
    /// sorted-queue optimization (see `clock_tick`'s doc comment).
    alarm: Option<u64>,
}

fn never_spawned() -> ! {
    panic!("attempted to run a process table slot that was never spawned")
}

// Safety: see the `messbuf` doc comment above -- there is exactly one
// logical thread of execution at a time (no preemption or SMP yet), so
// there is no data race despite the raw pointer.
unsafe impl Send for Proc {}

impl Proc {
    const fn empty() -> Self {
        Proc {
            proc_nr: com::NONE,
            name: "",
            rts_flags: rts::SLOT_FREE,
            priority: IDLE_Q,
            max_priority: IDLE_Q,
            ticks_left: 0,
            quantum_size: 0,
            preemptible: false,
            rsp: 0,
            next_ready: None,
            caller_q: None,
            q_link: None,
            get_from: com::NONE,
            send_to: com::NONE,
            messbuf: ptr::null_mut(),
            entry: never_spawned,
            cr3: None,
            mem_map: MemMap::EMPTY,
            parent: com::NONE,
            exit_status: None,
            wait_result: None,
            alarm: None,
        }
    }

    fn is_ready(&self) -> bool {
        self.rts_flags == 0
    }
}

/// Per-task kernel stacks. No allocator yet (see `rust/README.md`), so
/// these are static arrays rather than heap allocations -- analogous to
/// `t_stack[]` in `kernel/table.c`, just sized generously per task instead
/// of packed tightly.
#[repr(align(16))]
struct Stacks([[u8; STACK_SIZE]; NR_PROCS]);
static mut STACKS: Stacks = Stacks([[0; STACK_SIZE]; NR_PROCS]);

/// The fixed top of `idx`'s dedicated kernel stack. Used both as the base
/// a freshly spawned task's initial stack frame is built downward from
/// (`spawn`) and, unchanged for that task's whole lifetime, as the value
/// `crate::gdt::set_rsp0` needs whenever this task becomes current
/// (`reschedule`, `start`): the CPU should always find an empty stack here
/// to push a trap frame onto, which holds because a task that has entered
/// ring 3 (`crate::usermode`) never touches its own kernel stack again
/// until a trap brings it back.
fn stack_top(idx: usize) -> u64 {
    unsafe {
        let stack = ptr::addr_of_mut!(STACKS.0[idx]);
        (stack as *mut u8).add(STACK_SIZE) as u64
    }
}

struct Scheduler {
    procs: [Proc; NR_PROCS],
    rdy_head: [Option<usize>; NR_SCHED_QUEUES],
    rdy_tail: [Option<usize>; NR_SCHED_QUEUES],
    /// `proc_ptr`: the process whose stack is currently live.
    current: usize,
    /// `next_ptr`: who `pick_proc` decided should run next.
    next_ptr: Option<usize>,
    /// `sched()`'s static `prev_ptr`, used only to detect a process burning
    /// through consecutive quanta.
    prev_for_penalty: Option<usize>,
    /// `realtime`: ticks elapsed since the timer was programmed.
    ticks: u64,
    /// The kernel's own address space (whatever was active when this was
    /// first read, before any task ever gets its own -- see
    /// `Proc::cr3`), loaded whenever the current task doesn't have one.
    kernel_cr3: (PhysFrame, Cr3Flags),
}

impl Scheduler {
    /// `enqueue()`: add `idx` to the tail (or head, for a process that still
    /// had time left in its quantum) of its priority queue, then repick.
    fn enqueue(&mut self, idx: usize) {
        let (queue, front) = self.sched(idx);
        let q = queue as usize;
        self.procs[idx].next_ready = None;
        match self.rdy_head[q] {
            None => {
                self.rdy_head[q] = Some(idx);
                self.rdy_tail[q] = Some(idx);
            }
            Some(head) if front => {
                self.procs[idx].next_ready = Some(head);
                self.rdy_head[q] = Some(idx);
            }
            Some(_) => {
                let tail = self.rdy_tail[q].unwrap();
                self.procs[tail].next_ready = Some(idx);
                self.rdy_tail[q] = Some(idx);
            }
        }
        self.pick_proc();
    }

    /// `dequeue()`: remove `idx` from its priority queue (it has blocked, or
    /// is being reinserted elsewhere), then repick if it was running.
    fn dequeue(&mut self, idx: usize) {
        let q = self.procs[idx].priority as usize;
        let mut cursor = self.rdy_head[q];
        let mut prev: Option<usize> = None;
        while let Some(cur) = cursor {
            if cur == idx {
                let next = self.procs[cur].next_ready;
                match prev {
                    Some(p) => self.procs[p].next_ready = next,
                    None => self.rdy_head[q] = next,
                }
                if self.rdy_tail[q] == Some(idx) {
                    self.rdy_tail[q] = prev;
                }
                break;
            }
            prev = cursor;
            cursor = self.procs[cur].next_ready;
        }
        if self.current == idx || self.next_ptr == Some(idx) {
            self.pick_proc();
        }
    }

    /// `sched()`: decide which queue a process being (re-)enqueued belongs
    /// in, and whether it goes to the front (quantum not yet used up) or
    /// back (fresh quantum) of that queue. Kernel tasks' priority never
    /// changes -- only user processes (none exist in this port yet) get the
    /// "used several quanta in a row" penalty / "gave up early" bonus.
    fn sched(&mut self, idx: usize) -> (u8, bool) {
        let time_left = self.procs[idx].ticks_left > 0;
        let mut penalty: i8 = 0;
        if !time_left {
            self.procs[idx].ticks_left = self.procs[idx].quantum_size;
            if self.prev_for_penalty == Some(idx) {
                penalty += 1;
            } else {
                penalty -= 1;
            }
            self.prev_for_penalty = Some(idx);
        }
        if penalty != 0 && idx >= com::NR_TASKS {
            let p = &mut self.procs[idx];
            let new_priority = (p.priority as i8 + penalty).max(p.max_priority as i8);
            p.priority = new_priority.min(IDLE_Q as i8 - 1) as u8;
        }
        (self.procs[idx].priority, time_left)
    }

    /// `pick_proc()`: scan the queues from highest priority to lowest and
    /// select the head of the first non-empty one.
    fn pick_proc(&mut self) {
        self.next_ptr = self.rdy_head.iter().find_map(|h| *h);
    }

    /// The `(PhysFrame, Cr3Flags)` to load into `CR3` for `idx`: its own
    /// address space if it has one (`Proc::cr3`), or the kernel's default
    /// otherwise. Shared by `reschedule`/`start` so the selection rule only
    /// has one place to get right (and one place to update, if it ever
    /// needs to stop just copying `kernel_cr3`'s flags verbatim).
    fn cr3_for(&self, idx: usize) -> (PhysFrame, Cr3Flags) {
        self.procs[idx].cr3.map_or(self.kernel_cr3, |frame| (frame, self.kernel_cr3.1))
    }

    /// Hand a process-table slot back: it holds no process, and
    /// `alloc_proc_nr` may give its number to a new one. Wholesale
    /// rather than field-by-field, so a slot can never be half-freed --
    /// `spawn`/`fork_current` overwrite every field anyway when they
    /// fill one in.
    fn free_slot(&mut self, idx: usize) {
        self.procs[idx] = Proc::empty();
    }

    /// The bookkeeping shared by the two ways a process stops existing:
    /// `exit_now` (it asked to) and `kill` (it faulted). Ported in
    /// spirit from `servers/pm/forkexit.c`'s `do_exit`, which is also
    /// where the exit/crash distinction lives in real MINIX -- there PM
    /// reaches this path either from an `exit()` call or from the
    /// kernel's `SYS_SIG` notification about a fatal signal.
    ///
    /// Takes the slot off every ready queue, detaches its address space
    /// and memory map, and then decides what happens to the *status*,
    /// which is the part that actually needs a policy:
    ///
    /// - A parent already blocked in `wait_for_child` gets the status
    ///   handed straight to it and is woken; the slot is freed outright,
    ///   so no zombie is created at all. This is the common case, and
    ///   the reason `wait_result` exists.
    /// - A live parent that isn't waiting yet leaves the slot a zombie
    ///   (`rts::ZOMBIE`) holding the status until it is collected. The
    ///   process number stays allocated for exactly as long as that
    ///   takes, which is what a zombie *is*.
    /// - No live parent (a member of the fixed system image, or an
    ///   orphan) frees the slot immediately: nothing can ever collect a
    ///   status nobody is related to, and keeping the slot would leak a
    ///   process number for good.
    ///
    /// Returns the address space to reclaim, which the caller has to do
    /// outside the scheduler lock (and, if the dying process is the
    /// running one, after moving `CR3` off it).
    fn terminate(&mut self, idx: usize, status: i32, crashed: bool) -> Option<PhysFrame> {
        if self.procs[idx].rts_flags == 0 {
            self.dequeue(idx); // already repicks if idx was current/next_ptr
        }
        if crashed {
            self.procs[idx].rts_flags |= rts::DEAD;
        }
        if self.current == idx {
            self.pick_proc();
        }
        // Detached here, under the lock, so nothing can observe a
        // terminated slot still pointing at memory that is about to be
        // handed back.
        let address_space = self.procs[idx].cr3.take();
        self.procs[idx].mem_map = MemMap::EMPTY;

        let proc_nr = self.procs[idx].proc_nr;
        let live_parent = Some(self.procs[idx].parent)
            .filter(|&parent| parent != com::NONE)
            .map(com::slot)
            .filter(|&parent| self.procs[parent].rts_flags & rts::SLOT_FREE == 0);

        match live_parent {
            Some(parent) if self.procs[parent].rts_flags & rts::WAITING != 0 => {
                self.procs[parent].wait_result = Some((proc_nr, status));
                self.procs[parent].rts_flags &= !rts::WAITING;
                if self.procs[parent].rts_flags == 0 {
                    self.enqueue(parent);
                }
                self.free_slot(idx);
            }
            Some(_) => {
                self.procs[idx].exit_status = Some(status);
                self.procs[idx].rts_flags |= rts::ZOMBIE;
            }
            None => self.free_slot(idx),
        }
        address_space
    }

    /// Deliver a notification (`m_type`, appearing to come from
    /// `src_proc_nr`) to `dst_idx` if it's currently blocked in
    /// `mini_receive` waiting for one; returns whether it was delivered.
    /// The core of `mini_notify`, factored out so `clock_tick` -- which is
    /// already inside `with_scheduler` when it wants to deliver a
    /// `SYN_ALARM` -- can call it directly instead of going back through
    /// the public, re-locking `mini_notify` (which would deadlock on
    /// `SCHEDULER`, a non-reentrant lock).
    fn try_deliver_notification(&mut self, dst_idx: usize, src_proc_nr: i32, m_type: i32) -> bool {
        let dst_receiving =
            self.procs[dst_idx].rts_flags & (rts::RECEIVING | rts::SENDING) == rts::RECEIVING;
        let accepted = dst_receiving
            && (self.procs[dst_idx].get_from == com::ANY
                || self.procs[dst_idx].get_from == src_proc_nr);
        if accepted {
            let msg = Message { source: src_proc_nr, m_type, args: [0; 4] };
            unsafe { *self.procs[dst_idx].messbuf = msg };
            self.procs[dst_idx].rts_flags &= !rts::RECEIVING;
            if self.procs[dst_idx].rts_flags == 0 {
                self.enqueue(dst_idx);
            }
        }
        accepted
    }
}

lazy_static::lazy_static! {
    static ref SCHEDULER: Mutex<Scheduler> = Mutex::new(Scheduler {
        procs: core::array::from_fn(|_| Proc::empty()),
        rdy_head: [None; NR_SCHED_QUEUES],
        rdy_tail: [None; NR_SCHED_QUEUES],
        current: BOOTSTRAP,
        next_ptr: None,
        prev_for_penalty: None,
        ticks: 0,
        kernel_cr3: Cr3::read(),
    });
}

/// Run `f` with the scheduler locked and interrupts disabled, so
/// `clock_tick` (called from the timer interrupt) can't fire mid-critical-
/// section and deadlock spinning for a lock its own interrupted context is
/// still holding. See the module doc comment.
fn with_scheduler<R>(f: impl FnOnce(&mut Scheduler) -> R) -> R {
    without_interrupts(|| f(&mut SCHEDULER.lock()))
}

/// Prepare a never-yet-run task's stack so that switching to it for the
/// first time lands in `trampoline` (see below), and enqueue it as ready.
/// Standalone equivalent of an image-table entry in `kernel/table.c` plus
/// the register initialization `kernel/main.c` does for each boot-image
/// process before the first `restart()`. `address_space` is `None` for a
/// task that runs in the kernel's own address space (every task so far
/// except the ring-3 demo -- see `Proc::cr3`), or
/// `Some(`the PML4 frame `crate::memory::new_address_space` returned`)`
/// for one that needs its own.
///
/// Unlike `kernel/main.c`'s boot-image setup, which only ever runs once
/// before `restart()`, nothing here requires `idx` to be free *because*
/// the scheduler hasn't started yet -- `with_scheduler` makes this just as
/// safe to call from an already-running task as from `kernel_main`, which
/// is the actual point: real `rs` brings services up on demand at
/// runtime, not only at boot, and this is the primitive that needs.
/// `main.rs`'s `clock_task` demonstrates exactly that: it calls `spawn`
/// for a brand new process table slot after the scheduler is already
/// running other tasks.
pub fn spawn(
    proc_nr: i32,
    name: &'static str,
    entry: fn() -> !,
    priority: u8,
    quantum: i32,
    preemptible: bool,
    address_space: Option<AddressSpace>,
) {
    let idx = com::slot(proc_nr);
    // Build the initial stack frame that `switch_to`'s epilogue will pop:
    // six callee-saved registers (unused, so zeroed) and an initial RFLAGS
    // (interrupts enabled), followed by a return address, which `ret` will
    // jump to -- landing in `trampoline` on this task's own stack for the
    // very first time it runs.
    let rsp = unsafe {
        let frame = (stack_top(idx) as usize & !0xf) as *mut u64; // 16-byte align
        let frame = frame.sub(8);
        frame.add(0).write(0); // r15
        frame.add(1).write(0); // r14
        frame.add(2).write(0); // r13
        frame.add(3).write(0); // r12
        frame.add(4).write(0); // rbp
        frame.add(5).write(0); // rbx
        frame.add(6).write(0x202); // rflags: reserved bit 1 + IF (interrupts enabled)
        frame.add(7).write(trampoline as *const () as u64);
        frame as u64
    };

    with_scheduler(|sched| {
        sched.procs[idx] = Proc {
            proc_nr,
            name,
            rts_flags: 0,
            priority,
            max_priority: priority,
            ticks_left: quantum,
            quantum_size: quantum,
            preemptible,
            rsp,
            next_ready: None,
            caller_q: None,
            q_link: None,
            get_from: com::NONE,
            send_to: com::NONE,
            messbuf: ptr::null_mut(),
            entry,
            cr3: address_space.map(|space| space.pml4),
            mem_map: address_space.map_or(MemMap::EMPTY, |space| space.map),
            parent: com::NONE,
            exit_status: None,
            wait_result: None,
            alarm: None,
        };
        sched.enqueue(idx);
    });
}

/// Hand out a process number for a process that is being created *now*,
/// rather than one reserved in `crate::com` for a known member of the
/// system image: the first slot in the dynamic range
/// (`com::FIRST_DYNAMIC_PROC_NR`) that is free, marked `rts::RESERVED`
/// so the same number can't be handed out twice and so the scheduler
/// never picks a slot that has nothing in it yet.
///
/// Claiming the slot here rather than in the following `fork_current` is
/// what makes the pair safe against the timer: the two calls are several
/// frame-allocating steps apart (`crate::memory::fork_address_space`), a
/// tick can land in the gap and switch to a task that forks too, and an
/// allocator that only *looked* would then hand that task the same
/// number and have it overwrite a half-built process.
///
/// `None` when the dynamic range is full. Real MINIX's counterpart is
/// `servers/pm/forkexit.c`'s scan of `mproc[]` for a `!(mp_flags &
/// IN_USE)` slot, which reports the same condition as `EAGAIN`.
pub fn alloc_proc_nr() -> Option<i32> {
    with_scheduler(|sched| {
        let free = (0..com::NR_DYNAMIC_PROCS as i32)
            .map(|i| com::FIRST_DYNAMIC_PROC_NR + i)
            .find(|&nr| sched.procs[com::slot(nr)].rts_flags & rts::SLOT_FREE != 0)?;
        sched.procs[com::slot(free)].rts_flags = rts::RESERVED;
        Some(free)
    })
}

/// Give a number from `alloc_proc_nr` back unused -- for the caller that
/// claimed one and then failed to build the process (out of physical
/// frames, say). Without this a failed `fork` would cost a process
/// number permanently.
pub fn release_proc_nr(proc_nr: i32) {
    with_scheduler(|sched| {
        let idx = com::slot(proc_nr);
        debug_assert_eq!(
            sched.procs[idx].rts_flags,
            rts::RESERVED,
            "releasing a process number that isn't merely reserved"
        );
        sched.procs[idx].rts_flags = rts::SLOT_FREE;
    });
}

/// The memory map of `proc_nr`'s own address space (`Proc::mem_map`) --
/// empty for a task that runs in the kernel's. What `fork` reads to find
/// out which pages it has to copy, in place of the per-caller hardcoded
/// list `crate::syscall`'s `SYS_FORK` handler used to carry.
pub fn mem_map_of(proc_nr: i32) -> MemMap {
    with_scheduler(|sched| sched.procs[com::slot(proc_nr)].mem_map)
}

/// The scheduling parameters `proc_nr` is running with:
/// `(priority, quantum, preemptible)`. A forked child inherits its
/// parent's rather than being given fresh ones by whoever implements the
/// call -- which is both what real `fork()` does (`servers/pm`'s child
/// inherits the parent's scheduling state) and one less hardcoded
/// constant in `crate::syscall`'s `SYS_FORK` handler.
pub fn sched_params_of(proc_nr: i32) -> (u8, i32, bool) {
    with_scheduler(|sched| {
        let p = &sched.procs[com::slot(proc_nr)];
        (p.max_priority, p.quantum_size, p.preemptible)
    })
}

/// The process `parent` forked, if it still has one (`Proc::parent`).
/// Lets a runtime-created process be found by its relationship rather
/// than by a process number reserved for it in advance -- which is the
/// whole point of `alloc_proc_nr`, and what `crate::main`'s fork/exec
/// verification uses now that no such number exists.
///
/// Returns the lowest-numbered such slot; no current caller forks twice
/// from the same parent, and a real `wait()`-shaped API (which this is
/// not) would need to enumerate rather than pick.
pub fn child_of(parent: i32) -> Option<i32> {
    with_scheduler(|sched| {
        sched
            .procs
            .iter()
            .find(|p| p.parent == parent && p.rts_flags & rts::SLOT_FREE == 0)
            .map(|p| p.proc_nr)
    })
}

/// A saved trap frame: the 15 general-purpose registers `crate::syscall`'s
/// `entry` pushes (in that exact order -- see its doc comment) followed by
/// the hardware-pushed `iretq` frame (`rip`/`cs`/`rflags`/`rsp`/`ss`) sitting
/// immediately above them, untouched, on the same stack. `repr(C)` so this
/// struct's field layout (fields at strictly increasing offsets, in
/// declaration order) matches that memory layout exactly, letting
/// `crate::syscall::dispatch` hand `fork_current` a raw pointer into a
/// live trap and have it read as this type directly, and letting
/// `fork_current` write one back out as plain bytes for `fork_child_resume`
/// to later pop.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TrapFrame {
    pub r15: u64,
    pub r14: u64,
    pub r13: u64,
    pub r12: u64,
    pub r11: u64,
    pub r10: u64,
    pub r9: u64,
    pub r8: u64,
    pub rbp: u64,
    pub rdi: u64,
    pub rsi: u64,
    pub rdx: u64,
    pub rcx: u64,
    pub rbx: u64,
    pub rax: u64,
    pub rip: u64,
    pub cs: u64,
    pub rflags: u64,
    pub rsp: u64,
    pub ss: u64,
}

impl TrapFrame {
    /// `RFLAGS` a freshly started ring-3 image gets: reserved bit 1 plus
    /// `IF`, the same value `crate::usermode::enter_ring3` pushes when it
    /// jumps to ring 3 from scratch. Deliberately not inherited from the
    /// caller -- a new program shouldn't start out with whatever
    /// arithmetic flags (or, worse, direction flag) the program it
    /// replaced happened to leave set.
    const USER_RFLAGS: u64 = 0x202;

    /// The trap frame that turns this trap's return into the *start* of a
    /// different program: same ring (`cs`/`ss` carried over from the
    /// caller, which is already running with the user selectors), new
    /// entry point, new stack, and every general-purpose register zeroed
    /// -- the exec'd image inherits no register state from its
    /// predecessor, exactly as a fresh `crate::elf::task_entry` jump
    /// would leave it. `crate::syscall`'s `SYS_EXEC` handler writes the
    /// result back over the live frame, so the `iretq` at the end of
    /// `crate::syscall::entry` lands in the new image instead of
    /// returning to the old one -- the Rust counterpart of
    /// `kernel/system/do_exec.c` assigning `rp->p_reg.pc`/`sp`.
    pub fn exec_into(&self, entry: u64, stack_top: u64) -> TrapFrame {
        TrapFrame {
            r15: 0,
            r14: 0,
            r13: 0,
            r12: 0,
            r11: 0,
            r10: 0,
            r9: 0,
            r8: 0,
            rbp: 0,
            rdi: 0,
            rsi: 0,
            rdx: 0,
            rcx: 0,
            rbx: 0,
            rax: 0,
            rip: entry,
            cs: self.cs,
            rflags: Self::USER_RFLAGS,
            rsp: stack_top,
            ss: self.ss,
        }
    }
}

/// `SYS_FORK`'s real counterpart to `spawn`: instead of starting a brand
/// new task at a fixed `fn() -> !` entry point, this makes `child_proc_nr`
/// resume *inside a trap*, at the exact ring-3 instruction right after
/// whichever `int 0x80` called `SYS_FORK` -- genuine `fork()` semantics
/// (same code, same point, diverging only in what `rax` holds), not
/// `sys_fork`'s existing "child starts fresh at a given function" shape
/// (`crate::calls::sys_fork`, still used by `pm`'s own demo).
///
/// Builds *two* stacked frames on the child's own dedicated kernel stack,
/// lowest address first: an ordinary `switch_to`-compatible frame (see
/// `spawn`, six callee-saved registers + `RFLAGS` + a return address) whose
/// return address is `fork_child_resume` instead of `trampoline`, followed
/// immediately by a full copy of `frame` (with `rax` already zeroed by the
/// caller -- see `crate::syscall`'s `SYS_FORK` handler -- the fork()
/// convention distinguishing the child from its parent). The first time
/// this task is switched to, `switch_to`'s own generic `ret` lands in
/// `fork_child_resume`, which pops that inner frame and `iretq`s with it,
/// same as an ordinary trap return -- so from ring 3's perspective, this
/// task simply *is* the parent, one instruction further along, with `rax`
/// reading `0`.
#[allow(clippy::too_many_arguments)]
pub fn fork_current(
    parent_proc_nr: i32,
    child_proc_nr: i32,
    name: &'static str,
    priority: u8,
    quantum: i32,
    preemptible: bool,
    address_space: AddressSpace,
    frame: &TrapFrame,
) {
    let idx = com::slot(child_proc_nr);
    let rsp = unsafe {
        let base = (stack_top(idx) as usize & !0xf) as *mut u64;
        let base = base.sub(28); // 8 (switch_to frame) + 20 (TrapFrame, 20 u64 fields)
        base.add(0).write(0); // r15 (outer switch_to frame; unused)
        base.add(1).write(0); // r14
        base.add(2).write(0); // r13
        base.add(3).write(0); // r12
        base.add(4).write(0); // rbp
        base.add(5).write(0); // rbx
        base.add(6).write(0x202); // rflags (transient -- switch_to's popfq only)
        base.add(7).write(fork_child_resume as *const () as u64);
        let inner = base.add(8) as *mut TrapFrame;
        inner.write(*frame);
        base as u64
    };

    with_scheduler(|sched| {
        sched.procs[idx] = Proc {
            proc_nr: child_proc_nr,
            name,
            rts_flags: 0,
            priority,
            max_priority: priority,
            ticks_left: quantum,
            quantum_size: quantum,
            preemptible,
            rsp,
            next_ready: None,
            caller_q: None,
            q_link: None,
            get_from: com::NONE,
            send_to: com::NONE,
            messbuf: ptr::null_mut(),
            entry: never_spawned, // never used -- this task's first resume bypasses `trampoline`
            cr3: Some(address_space.pml4),
            mem_map: address_space.map,
            parent: parent_proc_nr,
            exit_status: None,
            wait_result: None,
            alarm: None,
        };
        sched.enqueue(idx);
    });
}

/// Where a forked child's kernel stack "starts" the very first time it's
/// switched to (see `fork_current`): unlike an ordinary task, which lands
/// in `trampoline` and calls a plain `fn() -> !`, a forked child needs to
/// resume *inside a trap*. `switch_to`'s `ret` jumps here instead, straight
/// into the same contract `crate::syscall::entry`'s own tail fulfills: pop
/// the `TrapFrame` `fork_current` laid out immediately below the
/// `switch_to` frame (same order `entry` itself pushes/pops its 15
/// registers in), then `iretq` into whatever `rip`/`cs`/`rflags`/`rsp`/`ss`
/// the parent's own trap had at the moment it called `SYS_FORK`.
/// Deliberately duplicates `entry`'s pop sequence rather than jumping into
/// it directly: a naked function has no addressable internal label `sym`
/// can reach from another function, and a second, small, self-contained
/// stub is simpler than threading one through. Keep this in sync with
/// `crate::syscall::entry`'s own tail if that one ever changes.
#[unsafe(naked)]
unsafe extern "C" fn fork_child_resume() -> ! {
    core::arch::naked_asm!(
        "pop r15",
        "pop r14",
        "pop r13",
        "pop r12",
        "pop r11",
        "pop r10",
        "pop r9",
        "pop r8",
        "pop rbp",
        "pop rdi",
        "pop rsi",
        "pop rdx",
        "pop rcx",
        "pop rbx",
        "pop rax",
        "iretq",
    )
}

/// Every task's stack is primed to start here (see `spawn`). `reschedule`/
/// `start` stash the target task's entry point in `NEXT_ENTRY` immediately
/// before switching to it; a task that has run before never actually lands
/// back here (`switch_to`'s `ret` resumes wherever it last blocked), so the
/// stashed value only ever matters the first time. Tasks never return
/// (`fn() -> !`), matching real kernel tasks, which run forever; if one
/// somehow did, that would itself be the kind of bug `kernel/proc.c`'s
/// stack-guard check in `dequeue()` is there to catch.
static mut NEXT_ENTRY: fn() -> ! = never_spawned;

extern "C" fn trampoline() -> ! {
    let entry = unsafe { NEXT_ENTRY };
    entry()
}

/// Low-level context switch: save the six callee-saved registers (the
/// caller-saved ones are, by the C calling convention, already dead by the
/// time a function call happens), RFLAGS, and the current `rsp` into
/// `*prev_rsp`, then load `next_rsp` and restore the next task's saved
/// registers and flags. Ported in spirit from `restart()`/`save()` in
/// `kernel/mpx386.s`, minus the trap-frame handling those deal with and we
/// don't need (no user mode yet -- see `rust/README.md`).
///
/// Saving RFLAGS is what makes it sound to call this (via `reschedule`)
/// from *inside* the timer interrupt handler, not just from ordinary task
/// code: entering an interrupt handler through an interrupt gate clears
/// the interrupt flag, and without saving/restoring it per task here, that
/// disabled state would leak into whichever task got switched to and stay
/// disabled system-wide -- interrupts, having disabled themselves, would
/// never fire again to re-enable anything.
#[unsafe(naked)]
unsafe extern "C" fn switch_to(prev_rsp: *mut u64, next_rsp: u64) {
    core::arch::naked_asm!(
        "pushfq",
        "push rbx",
        "push rbp",
        "push r12",
        "push r13",
        "push r14",
        "push r15",
        "mov [rdi], rsp",
        "mov rsp, rsi",
        "pop r15",
        "pop r14",
        "pop r13",
        "pop r12",
        "pop rbp",
        "pop rbx",
        "popfq",
        "ret",
    )
}

/// Switch away from the current task if a higher-priority (or, at equal
/// priority, differently-queued) one is now runnable. Called at the end of
/// every IPC operation and from the timer interrupt handler, standing in
/// for the unconditional "did `next_ptr` change?" check that real MINIX
/// makes on every single trap return (see the module doc comment). Safe to
/// call from either ordinary task code or from inside an interrupt
/// handler; see `switch_to`'s doc comment for why.
///
/// Interrupts are disabled for the *entire* decide-and-switch sequence
/// below, not just the scheduler-lock-guarded part inside `with_scheduler`.
/// This closes a real race a review found: with only the lock guarded,
/// `sched.current` was updated (and the lock released, re-enabling
/// interrupts for an ordinary task-level caller) *before* `switch_to` had
/// actually performed the low-level stack/CR3 swap. A timer tick landing
/// in that gap would run `clock_tick` against a `sched.current` that
/// didn't yet match who was physically executing, and if that expired a
/// quantum and changed `next_ptr` again, the nested `reschedule` call the
/// timer handler makes would call `switch_to` using the *not-yet-switched-
/// to* task's slot as the "outgoing" side -- overwriting its saved `rsp`
/// with the real outgoing task's live stack pointer and permanently
/// corrupting it. Keeping interrupts off across the whole sequence (and
/// restoring them explicitly on resume, below, rather than relying on
/// `switch_to`'s own saved `RFLAGS` -- those reflect whichever *other*
/// task is being switched *to*, not this call's caller) removes the gap
/// entirely.
pub fn reschedule() {
    let interrupts_were_enabled = are_enabled();
    disable();

    let switch: Option<(usize, *mut u64, u64, PhysFrame, Cr3Flags)> = with_scheduler(|sched| {
        let next = match sched.next_ptr {
            Some(n) => n,
            None => panic!("reschedule(): no runnable process (not even IDLE?)"),
        };
        if next == sched.current {
            return None;
        }
        let prev = sched.current;
        sched.current = next;
        unsafe { NEXT_ENTRY = sched.procs[next].entry };
        let prev_ptr: *mut u64 = &mut sched.procs[prev].rsp;
        let next_rsp = sched.procs[next].rsp;
        let (next_cr3, next_cr3_flags) = sched.cr3_for(next);
        Some((next, prev_ptr, next_rsp, next_cr3, next_cr3_flags))
    });

    if let Some((next, prev_ptr, next_rsp, next_cr3, next_cr3_flags)) = switch {
        // Before the switch, not after: if `next` is already in ring 3 (or
        // gets there right after resuming), the CPU needs to find *its*
        // RSP0 (and its own address space) in place the moment a trap
        // lands, not the outgoing task's.
        gdt::set_rsp0(VirtAddr::new(stack_top(next)));
        unsafe { Cr3::write(next_cr3, next_cr3_flags) };
        unsafe { switch_to(prev_ptr, next_rsp) };
        // Resumed -- possibly much later, on this exact stack, once
        // something switches back to whichever task this call belonged to.
        // Falls through to the restore below, same as the no-switch case.
    }

    if interrupts_were_enabled {
        enable();
    }
}

/// The process number of whichever task is currently running.
pub fn current_proc_nr() -> i32 {
    with_scheduler(|sched| sched.procs[sched.current].proc_nr)
}

/// Ticks elapsed since the timer was programmed (`crate::pit::init`).
/// Analogous to `kernel/clock.c`'s `get_uptime()`.
pub fn uptime_ticks() -> u64 {
    with_scheduler(|sched| sched.ticks)
}

/// The status `wait_for_child` reports for a process that didn't exit on
/// its own terms but was killed by a fault (`kill`). Real MINIX keeps
/// the exit status and the terminating signal in two separate `mproc`
/// fields, and POSIX packs both into one `int` behind
/// `WIFEXITED`/`WTERMSIG`; this port has no signal numbers to put in
/// there yet, so one reserved value stands in for "did not exit".
pub const STATUS_KILLED: i32 = -1;

/// `exit()`: the calling process terminates with `status`. Ported in
/// spirit from `servers/pm/forkexit.c`'s `do_exit` -- `Scheduler::
/// terminate` holds the part the two callers share, and this is the "it
/// asked to" half (`kill` is the other).
///
/// Never returns, and not because it loops: by the time `reschedule()`
/// is reached this slot is off every ready queue, so the switch away
/// from it is permanent. Its kernel stack (`STACKS`, static per slot) is
/// simply abandoned where it is, and rebuilt from the top if something
/// ever `spawn`s into the slot again.
///
/// Interrupts stay off from here to that final `reschedule()`, and that
/// is load-bearing rather than tidy. `terminate` can release this slot,
/// and `switch_to` has not yet saved this (dead) task's stack pointer
/// into it -- so a timer tick landing in between could switch to a task
/// that forks, hand it *this* number, and have `switch_to` then
/// overwrite the brand-new process's `rsp` with a dead stack. Nothing
/// re-enables interrupts on the way out: whichever task is switched to
/// restores its own `RFLAGS` (`switch_to`'s `popfq`).
pub fn exit_now(status: i32) -> ! {
    disable();
    let (proc_nr, name, address_space, kernel_cr3) = with_scheduler(|sched| {
        let idx = sched.current;
        let proc_nr = sched.procs[idx].proc_nr;
        let name = sched.procs[idx].name;
        (proc_nr, name, sched.terminate(idx, status, false), sched.kernel_cr3)
    });
    crate::serial_println!("[proc] {} (proc_nr {}) exited with status {}", name, proc_nr, status);
    reclaim(address_space, true, kernel_cr3, name, proc_nr);
    reschedule();
    unreachable!("a process that has exited was scheduled again")
}

/// `wait()`: block until one of the calling process's children
/// terminates, and collect it. Returns the child's `(proc_nr, status)`,
/// or `None` if the caller has no children at all (POSIX `ECHILD`).
/// Ported in spirit from `servers/pm/forkexit.c`'s `do_waitpid`, minus
/// the `waitpid` half of it -- there is no way to ask for one specific
/// child, or to poll without blocking (`WNOHANG`).
///
/// Collecting happens here rather than in the exiting process because
/// this is the side that knows the answer was wanted: a zombie
/// (`rts::ZOMBIE`, a child that terminated before anyone asked) has its
/// status read out and its slot -- and process number -- released, which
/// is the only thing that ever reclaims one in this port.
pub fn wait_for_child() -> Option<(i32, i32)> {
    enum Outcome {
        /// `(child, status, came_from_a_zombie_slot)` -- the flag only
        /// feeds the log line below, but which of the two paths a
        /// collection took is exactly what the timing of a real run is
        /// otherwise silent about.
        Collected(i32, i32, bool),
        NoChildren,
        Blocked,
    }
    loop {
        let outcome = with_scheduler(|sched| {
            let idx = sched.current;
            let me = sched.procs[idx].proc_nr;

            // Handed to us directly by a child that terminated while we
            // were already blocked here (`Scheduler::terminate`).
            if let Some((child, status)) = sched.procs[idx].wait_result.take() {
                return Outcome::Collected(child, status, false);
            }
            // Or a child that terminated before we asked, and has been
            // holding its status in a zombie slot since.
            let zombie = (0..NR_PROCS).find(|&i| {
                sched.procs[i].parent == me && sched.procs[i].exit_status.is_some()
            });
            if let Some(child_idx) = zombie {
                let child = sched.procs[child_idx].proc_nr;
                let status = sched.procs[child_idx].exit_status.expect("zombie without a status");
                sched.free_slot(child_idx);
                return Outcome::Collected(child, status, true);
            }
            // Nothing to collect. Is there anything that *could* be?
            let any_children = (0..NR_PROCS)
                .any(|i| sched.procs[i].parent == me && sched.procs[i].rts_flags & rts::SLOT_FREE == 0);
            if !any_children {
                return Outcome::NoChildren;
            }
            if sched.procs[idx].rts_flags == 0 {
                sched.dequeue(idx);
            }
            sched.procs[idx].rts_flags |= rts::WAITING;
            Outcome::Blocked
        });
        match outcome {
            // Logged out here rather than inside the closure above:
            // `with_scheduler` runs with interrupts disabled, and a
            // serial write is slow enough (milliseconds, at 115200 baud)
            // that doing one in there would start costing timer ticks.
            Outcome::Collected(child, status, from_zombie) => {
                crate::serial_println!(
                    "[proc] proc_nr {} collected child proc_nr {} (status {}) {}",
                    current_proc_nr(),
                    child,
                    status,
                    if from_zombie {
                        "out of a zombie slot, which is now free again"
                    } else {
                        "handed over directly -- it was still blocked here when the child exited"
                    }
                );
                return Some((child, status));
            }
            Outcome::NoChildren => return None,
            // Same shape as `mini_receive`: switch away now that we're
            // blocked, and come back around the loop once a terminating
            // child wakes us -- by which point `wait_result` is set.
            Outcome::Blocked => reschedule(),
        }
    }
}

/// Permanently stop scheduling `proc_nr` because it faulted: the `kill`
/// half of the pair `Scheduler::terminate` serves (`exit_now` is the
/// other). On top of the shared bookkeeping it marks the slot
/// `rts::DEAD` -- "terminated by a fault, not by choice", the one thing
/// a collected status can't say in this port -- and notifies `RS`
/// (`com::proc_died`) so it can decide whether to restart the service
/// (`crate::rs`). Called from `crate::interrupts` when a *ring-3* task
/// takes a CPU exception; the kernel's own stand-in for real MINIX
/// converting a user-process fault into a fatal signal
/// (`kernel/exception.c`) and `PM` reporting the exit to `RS`
/// (`servers/rs/manager.c`), collapsed into one direct call since this
/// port has neither signals nor `PM`'s exit path yet.
///
/// A parent blocked in `wait_for_child` is woken with `STATUS_KILLED`,
/// same as for a voluntary exit -- otherwise a crashing child would
/// leave its parent blocked forever, which is the one way this pair can
/// deadlock a process that did nothing wrong.
///
/// Idempotent: a process that has already terminated (or a slot that
/// never held one) is left alone rather than notifying `RS` twice.
pub fn kill(proc_nr: i32, reason: &str) {
    let idx = com::slot(proc_nr);
    let already_gone = rts::SLOT_FREE | rts::DEAD | rts::ZOMBIE;
    let dying = with_scheduler(|sched| {
        if sched.procs[idx].rts_flags & already_gone != 0 {
            return None;
        }
        let name = sched.procs[idx].name;
        let was_current = sched.current == idx;
        let address_space = sched.terminate(idx, STATUS_KILLED, true);
        sched.try_deliver_notification(com::slot(com::RS_PROC_NR), com::KERNEL, com::proc_died(proc_nr));
        Some((name, address_space, was_current, sched.kernel_cr3))
    });

    let (name, address_space, was_current, kernel_cr3) = match dying {
        Some(dying) => dying,
        None => return,
    };
    crate::serial_println!("[proc] {} (proc_nr {}) killed: {}", name, proc_nr, reason);
    reclaim(address_space, was_current, kernel_cr3, name, proc_nr);
}

/// Hand a terminated process's address space back, outside the
/// scheduler lock. Shared by `exit_now` and `kill`, which differ only in
/// how they got here.
///
/// If the process that just terminated is the one whose page tables are
/// currently loaded -- always, for `exit_now`, and the usual case for
/// `kill`, which runs from the CPU exception the process's own code
/// raised -- `CR3` has to move off them before they can be freed. The
/// kernel's own address space is always a safe place to stand: this
/// code, this stack and the physical-memory window are mapped there
/// identically.
fn reclaim(
    address_space: Option<PhysFrame>,
    was_current: bool,
    kernel_cr3: (PhysFrame, Cr3Flags),
    name: &str,
    proc_nr: i32,
) {
    let Some(address_space) = address_space else { return };
    if was_current {
        unsafe { Cr3::write(kernel_cr3.0, kernel_cr3.1) };
    }
    // Safety: detached from the process table by `Scheduler::terminate`
    // and no longer in `CR3`.
    let freed = unsafe { crate::memory::free_address_space(address_space, kernel_cr3.0) };
    crate::serial_println!("[proc] reclaimed {} frames from {} (proc_nr {})", freed, name, proc_nr);
}

/// The `(PhysFrame, Cr3Flags)` `proc_nr`'s address space is rooted at --
/// its own (`Proc::cr3`), or the kernel's default if it doesn't have one.
/// For `crate::calls::sys_vircopy` to translate a virtual address in some
/// *other* process's address space via `crate::memory::page_table_for`.
pub fn cr3_of(proc_nr: i32) -> (PhysFrame, Cr3Flags) {
    let idx = com::slot(proc_nr);
    with_scheduler(|sched| sched.cr3_for(idx))
}

/// The PML4 the kernel's own address space is rooted at -- what
/// `Scheduler::cr3_for` falls back to for any task without one of its
/// own. For `crate::calls::sys_exec`, which needs to build a new address
/// space derived from *the kernel's* rather than from whatever happens to
/// be in `CR3` at the time (see `crate::memory::new_address_space_from`).
pub fn kernel_cr3() -> PhysFrame {
    with_scheduler(|sched| sched.kernel_cr3.0)
}

/// Point `proc_nr` at a different address space from now on -- the
/// process-table half of `exec()` (`crate::calls::sys_exec`), where a
/// process keeps its identity (process number, priority, kernel stack,
/// place in the ready queue) and swaps out only the memory it runs in.
/// Nothing in `kernel/proc.c` corresponds directly, because MINIX's
/// `exec` is a `PM` operation over segment descriptors
/// (`servers/pm/exec.c` calling `sys_newmap`), not a page-table pointer
/// swap; this is that step's paging-era equivalent.
///
/// If `proc_nr` is the *currently running* task -- which it always is
/// when `sys_exec` calls this, since exec happens inside the caller's own
/// trap -- `CR3` is reloaded immediately rather than waiting for the next
/// `reschedule`: the trap is about to `iretq` straight into the new
/// image, which only exists in the new address space. Interrupts are off
/// across the update so a timer tick can't land between the table write
/// and the `CR3` load and switch away with the two disagreeing.
///
/// The address space being replaced *is* freed
/// (`memory::free_address_space`), which is only safe in this order:
/// the new `CR3` is loaded first, so by the time the old tables are
/// handed back nothing is running on them. Freeing is deliberately not
/// done while the scheduler lock is held -- the walk touches hundreds of
/// frames -- and the old space is unreachable from the process table by
/// then, so nothing can pick it up in between.
pub fn set_address_space(proc_nr: i32, address_space: AddressSpace) {
    let idx = com::slot(proc_nr);
    let interrupts_were_enabled = are_enabled();
    disable();
    let (previous, load_now, kernel_pml4) = with_scheduler(|sched| {
        let previous = sched.procs[idx].cr3;
        sched.procs[idx].cr3 = Some(address_space.pml4);
        // The map has to move with the page table: it describes what is
        // in *this* address space, so leaving the old image's behind
        // would have a later `fork` of this process copy pages that no
        // longer exist.
        sched.procs[idx].mem_map = address_space.map;
        (previous, (sched.current == idx).then_some(sched.kernel_cr3.1), sched.kernel_cr3.0)
    });
    if let Some(flags) = load_now {
        unsafe { Cr3::write(address_space.pml4, flags) };
    }
    if interrupts_were_enabled {
        enable();
    }

    if let Some(previous) = previous {
        // Safety: `previous` is no longer in `CR3` (replaced just above
        // if this process was current, never loaded otherwise) and no
        // longer reachable from the process table.
        let freed = unsafe { crate::memory::free_address_space(previous, kernel_pml4) };
        crate::serial_println!(
            "[proc] {} (proc_nr {}) replaced its address space, freed {} frames of the old one",
            with_scheduler(|sched| sched.procs[idx].name),
            proc_nr,
            freed
        );
    }
}

/// `sys_setalarm()`: ask to be sent a `SYN_ALARM` notification once
/// `delay_ticks` real timer ticks have elapsed. Ported from
/// `kernel/system/do_setalarm.c`; `clock_tick` is what actually notices
/// the deadline and delivers it (`kernel/clock.c`'s `do_clocktick`).
/// Setting a new alarm replaces any pending one for this task, same as
/// the C version's single-watchdog-per-process model.
pub fn set_alarm(delay_ticks: u64) {
    with_scheduler(|sched| {
        let idx = sched.current;
        sched.procs[idx].alarm = Some(sched.ticks + delay_ticks);
    });
}

/// Called from the timer interrupt handler (`crate::interrupts`) on every
/// PIT tick, immediately followed there by `reschedule()`. Ported from
/// `kernel/clock.c`'s `clock_handler` plus `do_clocktick`: advance the
/// uptime counter; if the running task is preemptible, charge it a tick
/// and, once its quantum is used up, reorder it to the back of its ready
/// queue (for `reschedule()` to then act on); and check every task's
/// watchdog alarm (`sys_setalarm`, `crate::calls`), delivering `SYN_ALARM`
/// to any that just expired.
///
/// Simplification: real MINIX keeps a sorted timer queue and only checks
/// `next_timeout <= realtime`; this scans every process table slot every
/// tick instead. Fine at `NR_PROCS` scale (a dozen-ish slots); would want
/// the sorted-queue approach if this port ever supports many more tasks.
pub fn clock_tick() {
    with_scheduler(|sched| {
        sched.ticks += 1;
        let now = sched.ticks;

        let idx = sched.current;
        if idx != BOOTSTRAP && sched.procs[idx].preemptible {
            sched.procs[idx].ticks_left -= 1;
            if sched.procs[idx].ticks_left <= 0 {
                sched.dequeue(idx);
                sched.enqueue(idx);
            }
        }

        for i in 0..NR_PROCS {
            if sched.procs[i].alarm.is_some_and(|target| now >= target) {
                sched.procs[i].alarm = None;
                sched.try_deliver_notification(i, com::CLOCK, com::SYN_ALARM);
            }
        }
    });
}

/// Voluntarily give up the rest of the current quantum and move to the
/// back of this task's ready queue. Kernel tasks that want to interleave
/// with their peers between blocking IPC calls use this; it complements
/// `clock_tick`'s hardware-driven quantum accounting rather than
/// replacing it (real MINIX has no equivalent -- see its doc comment
/// there).
pub fn yield_now() {
    with_scheduler(|sched| {
        let idx = sched.current;
        sched.procs[idx].ticks_left = 0;
        sched.dequeue(idx);
        sched.enqueue(idx);
    });
    reschedule();
}

/// `mini_send()`: send `m` from the running task to `dst`. Ported from
/// `kernel/proc.c`. If `dst` is already blocked in `mini_receive` waiting
/// for this message, it's delivered immediately; otherwise the caller
/// blocks (dequeuing itself and queuing onto `dst`'s `caller_q`) until
/// `dst` (or a third party, via `mini_receive`) has picked the message up.
/// Either way, `reschedule()` gets a chance to run at the end: if the
/// caller itself just blocked, this is what actually switches away; if
/// delivery was immediate and woke a *higher-priority* `dst`, this
/// preempts to it right away instead of leaving it queued until some
/// later, unrelated reschedule point.
///
/// Simplification: the C version detects a SEND/SEND cycle and returns
/// `ELOCKED` so the caller can recover. Nothing here has a way to report
/// that back yet (there's no error-propagating IPC API in front of this,
/// and no signals -- see `rust/README.md`), so a cyclic deadlock panics
/// instead of returning an error. None of the tasks spawned so far can
/// trigger this; revisit once real servers can.
pub fn mini_send(dst: i32, m: &Message) {
    let dst_idx = com::slot(dst);
    with_scheduler(|sched| {
        let caller = sched.current;

        let mut xp = dst_idx;
        while sched.procs[xp].rts_flags & rts::SENDING != 0 {
            xp = com::slot(sched.procs[xp].send_to);
            if xp == caller {
                panic!(
                    "mini_send: cyclic SEND deadlock between {} and {}",
                    sched.procs[caller].name, sched.procs[dst_idx].name
                );
            }
        }

        let dst_receiving =
            sched.procs[dst_idx].rts_flags & (rts::RECEIVING | rts::SENDING) == rts::RECEIVING;
        let caller_proc_nr = sched.procs[caller].proc_nr;
        let accepted = dst_receiving
            && (sched.procs[dst_idx].get_from == com::ANY
                || sched.procs[dst_idx].get_from == caller_proc_nr);

        if accepted {
            unsafe { *sched.procs[dst_idx].messbuf = *m };
            sched.procs[dst_idx].rts_flags &= !rts::RECEIVING;
            if sched.procs[dst_idx].rts_flags == 0 {
                sched.enqueue(dst_idx);
            }
            return;
        }

        // Destination isn't waiting for this. Block: dequeue the caller,
        // mark it SENDING, and append it to dst's caller_q.
        sched.procs[caller].messbuf = m as *const Message as *mut Message;
        if sched.procs[caller].rts_flags == 0 {
            sched.dequeue(caller);
        }
        sched.procs[caller].rts_flags |= rts::SENDING;
        sched.procs[caller].send_to = dst;
        sched.procs[caller].q_link = None;
        match sched.procs[dst_idx].caller_q {
            None => sched.procs[dst_idx].caller_q = Some(caller),
            Some(head) => {
                let mut cur = head;
                while let Some(next) = sched.procs[cur].q_link {
                    cur = next;
                }
                sched.procs[cur].q_link = Some(caller);
            }
        }
    });
    // If we just blocked, this is what switches away; if delivery was
    // immediate, this only switches if it woke a higher-priority task.
    reschedule();
}

/// `mini_receive()`: get a message addressed (or, for `ANY`, addressed to
/// anyone) to the running task. Ported from `kernel/proc.c`. If a sender is
/// already queued on this task's `caller_q`, the message is taken
/// immediately; otherwise the caller blocks until `mini_send` or
/// `mini_notify` delivers one.
///
/// Simplification: the C version also checks a pending-notification
/// bitmap here before falling back to `caller_q` (so a `mini_notify` that
/// arrived while this task wasn't receiving isn't lost). That bitmap isn't
/// implemented yet (see `rust/README.md`), so a notification sent to a
/// task that isn't blocked in `mini_receive` at that exact moment is
/// simply dropped instead of queued for later delivery.
pub fn mini_receive(src: i32) -> Message {
    let mut placeholder = Message::empty();
    enum Outcome {
        Ready(Message),
        Blocked,
    }
    let outcome = with_scheduler(|sched| {
        let caller = sched.current;

        let mut prev: Option<usize> = None;
        let mut cursor = sched.procs[caller].caller_q;
        while let Some(cur) = cursor {
            if src == com::ANY || src == sched.procs[cur].proc_nr {
                let msg = unsafe { *sched.procs[cur].messbuf };
                let next = sched.procs[cur].q_link;
                match prev {
                    Some(p) => sched.procs[p].q_link = next,
                    None => sched.procs[caller].caller_q = next,
                }
                sched.procs[cur].rts_flags &= !rts::SENDING;
                if sched.procs[cur].rts_flags == 0 {
                    sched.enqueue(cur);
                }
                return Outcome::Ready(msg);
            }
            prev = cursor;
            cursor = sched.procs[cur].q_link;
        }

        // No sender ready. Block until one arrives; `placeholder` lives on
        // this task's own suspended stack for as long as we're blocked,
        // and whoever delivers the message writes straight into it
        // through `messbuf` before waking us back up.
        sched.procs[caller].messbuf = &mut placeholder as *mut Message;
        sched.procs[caller].get_from = src;
        if sched.procs[caller].rts_flags == 0 {
            sched.dequeue(caller);
        }
        sched.procs[caller].rts_flags |= rts::RECEIVING;
        Outcome::Blocked
    });
    // Same reasoning as mini_send: switches away if we just blocked, or
    // preempts to a newly-woken higher-priority sender if delivery was
    // immediate, or does nothing.
    reschedule();
    match outcome {
        Outcome::Ready(msg) => msg,
        Outcome::Blocked => placeholder,
    }
}

/// `mini_notify()`: a lightweight, fire-and-forget send used for kernel
/// events (alarms, interrupts). Ported from `kernel/proc.c`; see
/// `mini_receive`'s doc comment for the one respect (no pending-bitmap)
/// in which this port is simpler than the original. The caller never
/// blocks here, but still calls `reschedule()` afterwards in case this
/// just woke a higher-priority task -- same reasoning as `mini_send`.
pub fn mini_notify(dst: i32, m_type: i32) {
    let dst_idx = com::slot(dst);
    with_scheduler(|sched| {
        let caller_proc_nr = sched.procs[sched.current].proc_nr;
        sched.try_deliver_notification(dst_idx, caller_proc_nr, m_type);
    });
    reschedule();
}

/// Hand off from the bootstrap context (`kernel_main`'s own stack, set up
/// by the bootloader) to the first ready task. Equivalent to `main()`
/// calling `restart()` at the end of `kernel/main.c` -- like there, this
/// never returns: `kernel_main`'s stack is simply abandoned.
///
/// Interrupts are already enabled by the time `main.rs` calls this, so it
/// needs the same protection `reschedule()` does (see its doc comment):
/// disabled here for the whole decide-and-switch sequence, with no restore
/// afterward needed since this call itself never resumes.
pub fn start() -> ! {
    disable();
    let (next, next_rsp, next_cr3, next_cr3_flags) = with_scheduler(|sched| {
        sched.pick_proc();
        let next = sched.next_ptr.expect("start(): no task was spawned");
        sched.current = next;
        unsafe { NEXT_ENTRY = sched.procs[next].entry };
        let (next_cr3, next_cr3_flags) = sched.cr3_for(next);
        (next, sched.procs[next].rsp, next_cr3, next_cr3_flags)
    });
    gdt::set_rsp0(VirtAddr::new(stack_top(next)));
    unsafe { Cr3::write(next_cr3, next_cr3_flags) };
    let mut discarded: u64 = 0;
    unsafe { switch_to(&mut discarded, next_rsp) };
    unreachable!("switch_to into the first task must not return to the bootstrap context");
}

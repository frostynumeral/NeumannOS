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

pub const NR_SCHED_QUEUES: usize = 16;
pub const TASK_Q: u8 = 0;
pub const IDLE_Q: u8 = (NR_SCHED_QUEUES - 1) as u8;

const STACK_SIZE: usize = 4096 * 8;

/// One slot per schedulable kernel task, plus one reserved slot for the
/// bootstrap context (`kernel_main`'s own stack, abandoned once it hands
/// off to the first task via `start()`).
const NR_PROCS: usize = com::NR_BOOT_PROCS + 1;
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
    address_space: Option<PhysFrame>,
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
            cr3: address_space,
            alarm: None,
        };
        sched.enqueue(idx);
    });
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

/// Permanently stop scheduling `proc_nr`: dequeue it (if it was ready) and
/// mark it `DEAD` so `spawn()` is the only thing that can ever bring the
/// slot back, then notify `RS` (`com::proc_died`) so it can decide
/// whether to restart it (`crate::rs`). Called from `crate::interrupts`
/// when a *ring-3* task takes a CPU exception -- the kernel's own stand-in
/// for real MINIX converting a user-process fault into a fatal signal
/// (`kernel/exception.c`) and `PM` reporting the exit to `RS`
/// (`servers/rs/manager.c`), collapsed into one direct call since this
/// port has neither signals nor `PM`'s exit path yet.
///
/// Idempotent: a process that's already `DEAD` (e.g. a second fault
/// landing before `RS` gets around to restarting it -- shouldn't happen
/// with how `crate::rs` is written, but costs nothing to guard against)
/// is left alone rather than notifying `RS` twice.
pub fn kill(proc_nr: i32, reason: &str) {
    let idx = com::slot(proc_nr);
    let name = with_scheduler(|sched| {
        if sched.procs[idx].rts_flags & rts::DEAD != 0 {
            return None;
        }
        if sched.procs[idx].rts_flags == 0 {
            sched.dequeue(idx); // already repicks if idx was current/next_ptr
        }
        sched.procs[idx].rts_flags |= rts::DEAD;
        if sched.current == idx {
            sched.pick_proc();
        }
        sched.try_deliver_notification(com::slot(com::RS_PROC_NR), com::KERNEL, com::proc_died(proc_nr));
        Some(sched.procs[idx].name)
    });
    if let Some(name) = name {
        crate::serial_println!("[proc] {} (proc_nr {}) killed: {}", name, proc_nr, reason);
    }
}

/// The `(PhysFrame, Cr3Flags)` `proc_nr`'s address space is rooted at --
/// its own (`Proc::cr3`), or the kernel's default if it doesn't have one.
/// For `crate::calls::sys_vircopy` to translate a virtual address in some
/// *other* process's address space via `crate::memory::page_table_for`.
pub fn cr3_of(proc_nr: i32) -> (PhysFrame, Cr3Flags) {
    let idx = com::slot(proc_nr);
    with_scheduler(|sched| sched.cr3_for(idx))
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

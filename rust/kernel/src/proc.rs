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
use alloc::vec::Vec;

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
    pub const SLOT_FREE: u16 = 0x01;
    pub const SENDING: u16 = 0x04;
    pub const RECEIVING: u16 = 0x08;
    /// Set by `crate::proc::kill` (a fault in a ring-3 task -- see
    /// `crate::interrupts`) and never cleared: this slot is permanently
    /// off every ready queue until a fresh `spawn()` overwrites it
    /// wholesale. No real MINIX `rts_flags` bit is quite this either --
    /// there, a dead process's slot is simply freed back to `SLOT_FREE`
    /// once `RS`/`PM` finish tearing it down; this port has no teardown
    /// step (no memory to free -- see `crate::rs`), so `DEAD` just marks
    /// "never schedule this slot again" until it's respawned.
    pub const DEAD: u16 = 0x10;
    /// Exited (or was killed) and still holds an uncollected exit
    /// status: a zombie, `mp_flags & ZOMBIE` in `servers/pm/mproc.h`.
    /// The slot -- and so the process number -- stays allocated until a
    /// parent collects the status (`crate::proc::wait_for_child`), which
    /// is the whole reason zombies exist: the status has to outlive the
    /// process it describes.
    pub const ZOMBIE: u16 = 0x40;
    /// Blocked in `crate::proc::wait_for_child` until one of this
    /// process's children terminates. `mp_flags & WAITING` in
    /// `servers/pm/mproc.h`; a distinct state from `RECEIVING` because
    /// what wakes it is a child exiting, not a message arriving.
    pub const WAITING: u16 = 0x80;
    /// Claimed by `crate::proc::alloc_proc_nr` but not yet filled in by
    /// the `spawn`/`fork_current` that asked for it: not free (so a
    /// second allocation skips it) and not runnable (so the scheduler
    /// never picks a slot that holds nothing). No MINIX counterpart --
    /// there, `PM` owns process-slot allocation and the kernel only ever
    /// sees a slot that is already a real process.
    pub const RESERVED: u16 = 0x20;
    /// Blocked in `crate::proc::thread_join` until the thread it names
    /// (`Proc::join_target`) exits.
    pub const JOINING: u16 = 0x100;
    /// Blocked in `crate::proc::sem_acquire` until a unit of the
    /// semaphore in `Proc::sem_wait` is released to it.
    pub const SEM_WAIT: u16 = 0x200;
    /// A team's leader that has ended while other members are still
    /// finishing calls they were blocked in (`Proc::doomed`): parked,
    /// holding its status (and so its process number, and its team's
    /// identity) until the last of them is gone, when it terminates for
    /// real and its parent (and `RS`) hear about it.
    pub const DRAINING: u16 = 0x400;
    /// Blocked in a port operation (`crate::proc::port_*`) until the port
    /// changes -- a message arrives or leaves, or it's closed or deleted --
    /// or `Proc::wait_deadline` passes.
    pub const PORT_WAIT: u16 = 0x800;
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
    rts_flags: u16,
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
    /// The process number of this slot's *team* leader -- itself, for
    /// every ordinary process; the process that created it, for a thread
    /// (`spawn_thread`). BeOS's word for a process is "team": the unit
    /// that owns an address space, descriptors and semaphores, with one
    /// or more threads running in it. Every thread of a team is its own
    /// slot here (own kernel stack, own registers, scheduled on its own)
    /// sharing the leader's `cr3`.
    team: i32,
    /// Set when this slot's team was told to terminate while it was
    /// blocked in IPC with a server that will still reply to it (see
    /// `Scheduler::must_linger`). It can't be freed then -- the reply
    /// would land in a freed or reused slot, and a server sending to a
    /// slot that never receives blocks forever -- so it runs until that
    /// call returns, and `die_if_doomed` ends it on the way back out.
    /// `(status, crashed)`: what the team ended with.
    doomed: Option<(i32, bool)>,
    /// With `rts::JOINING`: the thread this one is waiting for.
    join_target: i32,
    /// With `rts::SEM_WAIT`: which semaphore, and this waiter's place in
    /// line (lower goes first).
    sem_wait: Option<(usize, u64)>,
    /// Set by `sem_release` when it hands a unit of this semaphore
    /// straight to this waiter, until the waiter runs and takes it.
    sem_granted: Option<usize>,
    /// The team's program break -- the end of its heap (`brk`). Only the
    /// leader's is meaningful; the heap is the team's.
    brk: u64,
    /// With `rts::PORT_WAIT`: the port slot waited on.
    port_wait: Option<usize>,
    /// The tick at which a timed wait gives up (`clock_tick` wakes it and
    /// sets `timed_out`).
    wait_deadline: Option<u64>,
    timed_out: bool,
    /// A `write_port` message waiting for room, held here rather than on
    /// the writer's kernel stack: if the team ends while it waits, the
    /// slot is freed without that stack ever unwinding, and a message
    /// left there would leak its kernel-heap buffer for good. Freeing the
    /// slot drops this.
    port_pending: Option<(i32, Vec<u8>)>,
}

/// Where every program's heap starts (`brk`): PML4 slot 112, clear of the
/// kernel, the Rust programs (slot 96), the assembly ones (the 0x5555...,
/// 0x6666... and 0x7777... slots) and the kernel heap (slot 136).
pub const HEAP_BASE: u64 = 0x3800_0000_0000;
/// Largest a heap may grow: 16 MiB.
pub const HEAP_MAX: u64 = 16 * 1024 * 1024;

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
            team: com::NONE,
            doomed: None,
            join_target: com::NONE,
            sem_wait: None,
            sem_granted: None,
            brk: HEAP_BASE,
            port_wait: None,
            wait_deadline: None,
            timed_out: false,
            port_pending: None,
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
    /// How many processes each slot has held: bumped every time `spawn`
    /// or `fork_current` fills it, never reset (unlike `procs[idx]`,
    /// which goes back to `Proc::empty()` when a process is released).
    /// A process number alone doesn't name one process for ever -- the
    /// dynamic ones are reused as soon as a child is reaped -- so anything
    /// that has to remember *which* process it was dealing with keeps the
    /// pair (`generation_of`). `crate::fs` does, for descriptor ownership:
    /// owning by number alone handed a dead child's open files to the
    /// next process given its number.
    generations: [u32; NR_PROCS],
    /// The semaphore table (`sem_create` and friends).
    sems: [Sem; NR_SEMS],
    /// The port table (`port_create` and friends), and each slot's
    /// generation, so a stale `port_id` can't reach a slot's next port.
    ports: Vec<Option<Port>>,
    port_generations: [u32; NR_PORTS],
    /// Bytes queued in every port together (`PORT_QUEUE_BYTES` bounds it).
    port_bytes: usize,
    /// Next `sem_wait` ticket: waiters are served in arrival order.
    sem_seq: u64,
}

/// How many semaphores exist system-wide.
pub const NR_SEMS: usize = 32;

/// A counting semaphore, BeOS's basic synchronization primitive
/// (`create_sem`/`acquire_sem`/`release_sem`/`delete_sem` in the Be
/// kernel kit): `count` units are available, `acquire` takes one or
/// blocks until one is released. Owned by the team that created it,
/// which is what deletes it -- explicitly, or by terminating.
#[derive(Clone, Copy)]
struct Sem {
    in_use: bool,
    count: i32,
    owner_team: i32,
}

impl Sem {
    const FREE: Sem = Sem { in_use: false, count: 0, owner_team: com::NONE };
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
        let address_space = self.release_address_space(idx);

        let proc_nr = self.procs[idx].proc_nr;
        let live_parent = Some(self.procs[idx].parent)
            .filter(|&parent| parent != com::NONE)
            .map(com::slot)
            .filter(|&parent| self.procs[parent].rts_flags & rts::SLOT_FREE == 0);
        // Whichever thread of the parent's team is blocked in `wait`: a
        // child belongs to the team, not to the one thread that forked it.
        let waiter = live_parent.and_then(|parent| {
            let team = self.procs[parent].team;
            (0..NR_PROCS).find(|&i| self.is_member(i, team) && self.procs[i].rts_flags & rts::WAITING != 0)
        });

        match live_parent {
            Some(_) if waiter.is_some() => {
                let waiter = waiter.unwrap();
                self.procs[waiter].wait_result = Some((proc_nr, status));
                self.procs[waiter].rts_flags &= !rts::WAITING;
                if self.procs[waiter].rts_flags == 0 {
                    self.enqueue(waiter);
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

    /// Detach `idx`'s address space and memory map, returning the page
    /// table to free -- or `None` if some other live slot still runs in
    /// it (another thread of the same team), in which case the last one
    /// out frees it instead.
    fn release_address_space(&mut self, idx: usize) -> Option<PhysFrame> {
        let pml4 = self.procs[idx].cr3.take()?;
        self.procs[idx].mem_map = MemMap::EMPTY;
        let shared = (0..NR_PROCS).any(|i| {
            i != idx && self.procs[i].rts_flags & rts::SLOT_FREE == 0 && self.procs[i].cr3 == Some(pml4)
        });
        (!shared).then_some(pml4)
    }

    /// Whether slot `i` holds a live (not free, not merely reserved)
    /// member of `team`.
    fn is_member(&self, i: usize, team: i32) -> bool {
        let flags = self.procs[i].rts_flags;
        flags & rts::SLOT_FREE == 0 && flags != rts::RESERVED && self.procs[i].team == team && team != com::NONE
    }

    /// Whether `idx` is blocked where freeing it would be unsafe: queued
    /// to send (it sits in the destination's `caller_q`), or waiting for
    /// a reply from a particular server, which will still arrive -- into
    /// a freed or reused slot, or, since a send to a slot that isn't
    /// receiving blocks, not at all, hanging the server. Every other
    /// blocked state (`WAITING`, `JOINING`, `SEM_WAIT`, an alarm wait on
    /// `CLOCK`, `SYS_BLOCK_FOREVER`'s receive from anyone) is this
    /// module's own bookkeeping and can be dropped on the spot.
    ///
    /// Plus one state that looks runnable: a server that has *already*
    /// replied, before the caller got back to `receive` (`send_receive`
    /// sends, the higher-priority server runs and replies at once, and
    /// blocks queued on the caller's `caller_q`). Freeing the caller then
    /// would wipe that queue and leave the server sending to a dead number
    /// forever.
    fn must_linger(&self, idx: usize) -> bool {
        let p = &self.procs[idx];
        p.rts_flags & rts::SENDING != 0
            || (p.rts_flags & rts::RECEIVING != 0 && p.get_from != com::ANY && p.get_from != com::CLOCK)
            || p.caller_q.is_some()
    }

    /// Whether `team` has a live member other than its leader.
    fn has_other_members(&self, team: i32) -> bool {
        let leader = com::slot(team);
        (0..NR_PROCS).any(|i| i != leader && self.is_member(i, team))
    }

    /// Hand a unit `sem_release` granted to `idx` -- which is going away
    /// before it ran to take it -- on to the next waiter, or back to the
    /// count; otherwise the semaphore would be one short for good.
    fn forfeit_grant(&mut self, idx: usize) {
        if let Some(id) = self.procs[idx].sem_granted.take() {
            if self.sems[id].in_use {
                self.release_unit(id);
            }
        }
    }

    /// `sem_release`'s core: give one unit of `id` to the longest waiter,
    /// or back to the count.
    fn release_unit(&mut self, id: usize) {
        let next = (0..NR_PROCS)
            .filter_map(|i| self.procs[i].sem_wait.filter(|&(sem, _)| sem == id).map(|(_, t)| (t, i)))
            .min();
        match next {
            Some((_, waiter)) => {
                self.procs[waiter].sem_wait = None;
                self.procs[waiter].sem_granted = Some(id);
                self.procs[waiter].rts_flags &= !rts::SEM_WAIT;
                if self.procs[waiter].rts_flags == 0 {
                    self.enqueue(waiter);
                }
            }
            None => self.sems[id].count = self.sems[id].count.saturating_add(1),
        }
    }

    /// The leader's end: terminate it (status to its parent, `RS` told if
    /// it crashed) -- unless members are still finishing calls, in which
    /// case it's parked `DRAINING` with its status, keeping its number
    /// and the team's identity reserved until `finish_thread` sees the
    /// last of them go. Without that, a lingering thread could outlive a
    /// reaped leader and be taken for a thread of the *next* process
    /// given that number.
    fn finish_leader(&mut self, leader: usize, status: i32, crashed: bool) -> Option<PhysFrame> {
        let team = self.procs[leader].proc_nr;
        self.forfeit_grant(leader);
        // Out of any semaphore's line, too: the leader may be waiting on
        // *another* team's semaphore (which `end_team` doesn't delete),
        // and a zombie or draining slot left in line would be handed a
        // unit it can never take. (Threads don't need this: `discard`
        // frees the slot, which clears it.)
        self.procs[leader].sem_wait = None;
        self.procs[leader].rts_flags &= !rts::SEM_WAIT;
        if self.has_other_members(team) {
            if self.procs[leader].rts_flags == 0 {
                self.dequeue(leader);
            }
            if self.current == leader {
                self.pick_proc();
            }
            self.procs[leader].rts_flags |= rts::DRAINING;
            self.procs[leader].doomed = Some((status, crashed));
            return None;
        }
        let space = self.terminate(leader, status, crashed);
        if crashed {
            self.try_deliver_notification(com::slot(com::RS_PROC_NR), com::KERNEL, com::proc_died(team));
        }
        space
    }

    /// A non-leader member's end, with no status to keep: discard it, and
    /// if that was the last member a `DRAINING` leader was waiting for,
    /// finish the leader too.
    fn finish_thread(&mut self, idx: usize) -> Option<PhysFrame> {
        let team = self.procs[idx].team;
        self.forfeit_grant(idx);
        let mut space = self.discard(idx);
        let leader = com::slot(team);
        if self.is_member(leader, team)
            && self.procs[leader].rts_flags & rts::DRAINING != 0
            && !self.has_other_members(team)
        {
            let (status, crashed) = self.procs[leader].doomed.take().unwrap_or((STATUS_KILLED, true));
            self.procs[leader].rts_flags &= !rts::DRAINING;
            if let Some(s) = self.finish_leader(leader, status, crashed) {
                space = Some(s);
            }
        }
        space
    }

    /// Remove a thread (never a leader) outright: no status, nothing to
    /// collect. Returns the address space if it was the last one in it.
    fn discard(&mut self, idx: usize) -> Option<PhysFrame> {
        if self.procs[idx].rts_flags == 0 {
            self.dequeue(idx);
        }
        if self.current == idx {
            self.pick_proc();
        }
        let space = self.release_address_space(idx);
        self.free_slot(idx);
        space
    }

    /// End every member of `team`, the leader last, the way `exit()` or a
    /// fatal fault ends a whole process however many threads it has: the
    /// leader's status goes to its parent as usual (`terminate`), and
    /// every other thread simply stops. A member blocked in IPC with a
    /// server (`must_linger`) is marked `doomed` instead and finishes
    /// dying when that call returns (`die_if_doomed`); the address space
    /// outlives it, so the last member out frees it. The team's
    /// semaphores go too, waking any waiter from another team.
    ///
    /// Returns the address space to free, if nothing still runs in it.
    fn end_team(&mut self, team: i32, status: i32, crashed: bool) -> Option<PhysFrame> {
        let leader = com::slot(team);
        let mut freed = None;
        for i in 0..NR_PROCS {
            if i == leader || !self.is_member(i, team) || self.procs[i].doomed.is_some() {
                continue;
            }
            if self.must_linger(i) {
                self.procs[i].doomed = Some((status, crashed));
            } else if let Some(space) = self.finish_thread(i) {
                freed = Some(space);
            }
        }
        for id in 0..NR_SEMS {
            if self.sems[id].in_use && self.sems[id].owner_team == team {
                self.delete_sem(id);
            }
        }
        // Ports too, as Haiku deletes a team's ports when it dies.
        for slot in 0..NR_PORTS {
            if self.ports[slot].as_ref().is_some_and(|p| p.owner_team == team) {
                self.delete_port_slot(slot);
            }
        }
        if self.is_member(leader, team) && self.procs[leader].doomed.is_none() {
            if self.must_linger(leader) {
                self.procs[leader].doomed = Some((status, crashed));
            } else if let Some(space) = self.finish_leader(leader, status, crashed) {
                freed = Some(space);
            }
        }
        freed
    }

    /// End `idx`'s port wait, making it runnable if nothing else holds it.
    fn stop_port_wait(&mut self, idx: usize) {
        self.procs[idx].port_wait = None;
        self.procs[idx].wait_deadline = None;
        self.procs[idx].rts_flags &= !rts::PORT_WAIT;
        if self.procs[idx].rts_flags == 0 {
            self.enqueue(idx);
        }
    }

    /// Wake everyone waiting on port `slot` to look again: something
    /// about it changed.
    fn wake_port_waiters(&mut self, slot: usize) {
        for i in 0..NR_PROCS {
            if self.procs[i].port_wait == Some(slot) {
                self.stop_port_wait(i);
            }
        }
    }

    /// The live port slot a `port_id` names, if it names one.
    fn port_slot(&self, id: i32) -> Option<usize> {
        if id < 0 {
            return None;
        }
        let slot = (id as usize) % NR_PORTS;
        let generation = (id as u32) / NR_PORTS as u32;
        (self.ports[slot].is_some() && self.port_generations[slot] == generation).then_some(slot)
    }

    fn delete_port_slot(&mut self, slot: usize) {
        if let Some(port) = self.ports[slot].take() {
            self.port_bytes -= port.queue.iter().map(|(_, data)| data.len()).sum::<usize>();
        }
        // Everyone: this slot's waiters to find it gone, and writers on
        // other ports waiting for the bytes it just released.
        self.wake_all_port_waiters();
    }

    fn wake_all_port_waiters(&mut self) {
        for i in 0..NR_PROCS {
            if self.procs[i].rts_flags & rts::PORT_WAIT != 0 {
                self.stop_port_wait(i);
            }
        }
    }

    fn delete_sem(&mut self, id: usize) {
        self.sems[id] = Sem::FREE;
        for i in 0..NR_PROCS {
            if self.procs[i].sem_wait.is_some_and(|(sem, _)| sem == id) {
                self.procs[i].sem_wait = None;
                self.procs[i].rts_flags &= !rts::SEM_WAIT;
                if self.procs[i].rts_flags == 0 {
                    self.enqueue(i);
                }
            }
        }
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
        generations: [0; NR_PROCS],
        sems: [Sem::FREE; NR_SEMS],
        ports: (0..NR_PORTS).map(|_| None).collect(),
        port_generations: [0; NR_PORTS],
        port_bytes: 0,
        sem_seq: 0,
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

    let brk = HEAP_BASE;
    with_scheduler(|sched| {
        sched.generations[idx] = sched.generations[idx].wrapping_add(1);
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
            team: proc_nr,
            doomed: None,
            join_target: com::NONE,
            sem_wait: None,
            sem_granted: None,
            brk,
            port_wait: None,
            wait_deadline: None,
            timed_out: false,
            port_pending: None,
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
    // The team's: a thread's own copy doesn't follow the heap growing.
    with_scheduler(|sched| {
        let team = sched.procs[com::slot(proc_nr)].team;
        let owner = if team == com::NONE { com::slot(proc_nr) } else { com::slot(team) };
        sched.procs[owner].mem_map
    })
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
    pub fn exec_into(&self, entry: u64, stack_pointer: u64) -> TrapFrame {
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
            rsp: stack_pointer,
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
    install_trapped(parent_proc_nr, child_proc_nr, child_proc_nr, name, priority, quantum, preemptible, address_space, frame);
}

/// The part `fork_current` and `spawn_thread` share: fill `child_proc_nr`'s
/// slot with a task whose first run resumes `frame` in ring 3. `team` is
/// the child itself for a forked process, and the creating team's
/// leader for a thread.
#[allow(clippy::too_many_arguments)]
fn install_trapped(
    parent_proc_nr: i32,
    child_proc_nr: i32,
    team: i32,
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
        // A forked child inherits its parent's heap (the pages came with
        // the address space); a thread's own field is never read.
        let brk = if parent_proc_nr == com::NONE {
            HEAP_BASE
        } else {
            sched.procs[com::slot(sched.procs[com::slot(parent_proc_nr)].team)].brk
        };
        sched.generations[idx] = sched.generations[idx].wrapping_add(1);
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
            team,
            doomed: None,
            join_target: com::NONE,
            sem_wait: None,
            sem_granted: None,
            brk,
            port_wait: None,
            wait_deadline: None,
            timed_out: false,
            port_pending: None,
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
        // The whole team: `exit()` from any thread ends the process.
        let team = sched.procs[idx].team;
        (proc_nr, name, sched.end_team(team, status, false), sched.kernel_cr3)
    });
    crate::serial_println!("[proc] {} (proc_nr {}) exited with status {}", name, proc_nr, status);
    reclaim(address_space, kernel_cr3, name, proc_nr);
    reschedule();
    unreachable!("a process that has exited was scheduled again")
}

/// Finish off the calling slot if its team was told to terminate while it
/// was blocked in IPC (`Proc::doomed`). `crate::syscall`'s `dispatch`
/// calls this on the way back out of every system call, which is the
/// first moment such a slot runs again. Returns normally if it isn't
/// doomed.
pub fn die_if_doomed() {
    let doomed = with_scheduler(|sched| sched.procs[sched.current].doomed);
    let Some((status, crashed)) = doomed else { return };
    disable();
    let (proc_nr, name, address_space, kernel_cr3) = with_scheduler(|sched| {
        let idx = sched.current;
        sched.procs[idx].doomed = None;
        let proc_nr = sched.procs[idx].proc_nr;
        let name = sched.procs[idx].name;
        let space = if sched.procs[idx].team == proc_nr {
            sched.finish_leader(idx, status, crashed)
        } else {
            sched.finish_thread(idx)
        };
        (proc_nr, name, space, sched.kernel_cr3)
    });
    crate::serial_println!("[proc] {} (proc_nr {}) finished its call and ended with its team", name, proc_nr);
    reclaim(address_space, kernel_cr3, name, proc_nr);
    reschedule();
    unreachable!("a doomed slot was scheduled again")
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
            // Children belong to the team, whichever of its threads forked them.
            let me = sched.procs[idx].team;

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
        // A fault in any thread is fatal to its whole team, as a fatal
        // signal is to a whole POSIX process; `RS` hears about the team.
        let team = sched.procs[idx].team;
        // `RS` is told once the leader actually terminates
        // (`finish_leader`) -- not now, if members are still draining.
        let address_space = sched.end_team(team, STATUS_KILLED, true);
        Some((name, address_space, sched.kernel_cr3))
    });

    let (name, address_space, kernel_cr3) = match dying {
        Some(dying) => dying,
        None => return,
    };
    crate::serial_println!("[proc] {} (proc_nr {}) killed: {}", name, proc_nr, reason);
    reclaim(address_space, kernel_cr3, name, proc_nr);
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
fn reclaim(address_space: Option<PhysFrame>, kernel_cr3: (PhysFrame, Cr3Flags), name: &str, proc_nr: i32) {
    let Some(address_space) = address_space else { return };
    // Whoever is running may be standing in it -- the dying process
    // itself, or one of its threads.
    if Cr3::read().0 == address_space {
        unsafe { Cr3::write(kernel_cr3.0, kernel_cr3.1) };
    }
    // Safety: detached from the process table by `Scheduler::terminate`
    // and no longer in `CR3`.
    let freed = unsafe { crate::memory::free_address_space(address_space, kernel_cr3.0) };
    crate::serial_println!("[proc] reclaimed {} frames from {} (proc_nr {})", freed, name, proc_nr);
}

/// The `(PhysFrame, Cr3Flags)` `proc_nr`'s address space is rooted at --
/// its own (`Proc::cr3`), or the kernel's default if it doesn't have one.
/// The leader of `proc_nr`'s team (itself, for an ordinary process).
pub fn team_of(proc_nr: i32) -> i32 {
    with_scheduler(|sched| sched.procs[com::slot(proc_nr)].team)
}

/// `(team leader, leader's generation)` for `proc_nr` -- the identity
/// `crate::fs` records as a descriptor's owner, so every thread of a
/// team can use the team's descriptors, and a reused number can't.
pub fn team_identity(proc_nr: i32) -> Option<(i32, u32)> {
    if !is_valid_proc_nr(proc_nr) {
        return None;
    }
    let team = team_of(proc_nr);
    generation_of(team).map(|g| (team, g))
}

/// How many live slots `proc_nr`'s team has (threads plus the leader).
pub fn team_size(proc_nr: i32) -> usize {
    with_scheduler(|sched| {
        let team = sched.procs[com::slot(proc_nr)].team;
        // A thread that exited and hasn't been joined is a zombie, not a
        // running thread: it doesn't stand in the way of `exec`.
        (0..NR_PROCS)
            .filter(|&i| sched.is_member(i, team) && sched.procs[i].rts_flags & rts::ZOMBIE == 0)
            .count()
    })
}

/// Start a new thread in the calling process's team: `child` (from
/// `alloc_proc_nr`) resumes `frame` in ring 3, in the same address space
/// as the caller. `frame` is the caller's trap frame with the new
/// thread's `rip`/`rsp`/argument already filled in (`crate::syscall`'s
/// `SYS_THREAD_SPAWN`). There is no C original to port: MINIX 3.1 has one
/// thread per process. This is BeOS's `spawn_thread` -- a thread is a
/// full kernel-scheduled entity, not a user-level coroutine.
pub fn spawn_thread(caller: i32, child: i32, frame: &TrapFrame) {
    let (team, priority, quantum, preemptible, space) = with_scheduler(|sched| {
        let p = &sched.procs[com::slot(caller)];
        let space = AddressSpace { pml4: p.cr3.expect("a ring-3 caller has an address space"), map: p.mem_map };
        (p.team, p.max_priority, p.quantum_size, p.preemptible, space)
    });
    install_trapped(com::NONE, child, team, "thread", priority, quantum, preemptible, space, frame);
}

/// End the calling thread (not the whole team) with `status`, for
/// `thread_join` to collect: handed straight to a thread already
/// blocked joining it, or kept in a zombie slot until one does. `Err`
/// if the caller is a team's leader -- its end is the team's
/// (`exit_now`).
pub fn thread_exit(status: i32) -> Result<core::convert::Infallible, ()> {
    let leader = with_scheduler(|sched| sched.procs[sched.current].team == sched.procs[sched.current].proc_nr);
    if leader {
        return Err(());
    }
    disable();
    let (proc_nr, name, address_space, kernel_cr3) = with_scheduler(|sched| {
        let idx = sched.current;
        let me = sched.procs[idx].proc_nr;
        let team = sched.procs[idx].team;
        // Read before the slot can be freed below.
        let name = sched.procs[idx].name;
        let joiners: Vec<usize> = (0..NR_PROCS)
            .filter(|&i| {
                sched.is_member(i, team) && sched.procs[i].rts_flags & rts::JOINING != 0 && sched.procs[i].join_target == me
            })
            .collect();
        let space = if let Some((&first, rest)) = joiners.split_first() {
            // The first joiner gets the status; any others are woken to
            // find the thread gone (`thread_join` reports that).
            sched.procs[first].wait_result = Some((me, status));
            for &j in core::iter::once(&first).chain(rest) {
                sched.procs[j].rts_flags &= !rts::JOINING;
                if sched.procs[j].rts_flags == 0 {
                    sched.enqueue(j);
                }
            }
            sched.finish_thread(idx)
        } else {
            sched.dequeue(idx);
            sched.pick_proc();
            let space = sched.release_address_space(idx);
            sched.procs[idx].exit_status = Some(status);
            sched.procs[idx].rts_flags |= rts::ZOMBIE;
            space
        };
        (me, name, space, sched.kernel_cr3)
    });
    crate::serial_println!("[proc] {} (proc_nr {}) exited with status {}", name, proc_nr, status);
    reclaim(address_space, kernel_cr3, name, proc_nr);
    reschedule();
    unreachable!("an exited thread was scheduled again")
}

/// Why `thread_join` couldn't join.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinError {
    /// Not a thread of the caller's team (or the caller itself, or the
    /// leader), or it went away -- joined by someone else first.
    NotAThread,
}

/// Block until thread `tid` of the caller's team exits, and collect its
/// status (BeOS's `wait_for_thread`).
pub fn thread_join(tid: i32) -> Result<i32, JoinError> {
    enum Outcome {
        Done(i32),
        Invalid,
        Blocked,
    }
    loop {
        let outcome = with_scheduler(|sched| {
            let idx = sched.current;
            let me = sched.procs[idx].proc_nr;
            if let Some((child, status)) = sched.procs[idx].wait_result.take() {
                if child == tid {
                    return Outcome::Done(status);
                }
            }
            let team = sched.procs[idx].team;
            let valid = tid >= -(com::NR_TASKS as i32) && com::slot(tid) < com::NR_PROC_SLOTS;
            if !valid || tid == me || tid == team || !sched.is_member(com::slot(tid), team) {
                return Outcome::Invalid;
            }
            let t = com::slot(tid);
            if let Some(status) = sched.procs[t].exit_status {
                sched.free_slot(t);
                return Outcome::Done(status);
            }
            if sched.procs[idx].rts_flags == 0 {
                sched.dequeue(idx);
            }
            sched.procs[idx].rts_flags |= rts::JOINING;
            sched.procs[idx].join_target = tid;
            Outcome::Blocked
        });
        match outcome {
            Outcome::Done(status) => return Ok(status),
            Outcome::Invalid => return Err(JoinError::NotAThread),
            Outcome::Blocked => reschedule(),
        }
    }
}

/// `brk()`: move the calling team's program break to `new_end`, mapping
/// fresh zeroed pages as it grows and giving pages back as it shrinks;
/// `0` just asks where it is. Returns the break afterwards, or `Err` if
/// the request is outside `HEAP_BASE..=HEAP_BASE + HEAP_MAX` or memory
/// ran out (the break is then unchanged). The heap is the team's: any
/// thread may move it, and it lives in the team's memory map, so `fork`
/// shares it copy-on-write like any other page. MINIX has the same call
/// (`servers/pm/break.c`'s `do_brk`), growing a data segment rather than
/// mapping pages.
pub fn brk(new_end: u64) -> Result<u64, ()> {
    let (leader, current, pml4) = with_scheduler(|sched| {
        let me = sched.current;
        let team = sched.procs[me].team;
        let leader = if team == com::NONE { me } else { com::slot(team) };
        (leader, sched.procs[leader].brk, sched.procs[me].cr3)
    });
    if new_end == 0 {
        return Ok(current);
    }
    let pml4 = pml4.ok_or(())?;
    if new_end < HEAP_BASE || new_end > HEAP_BASE + HEAP_MAX {
        return Err(());
    }
    let pages = |end: u64| ((end - HEAP_BASE) + 4095) / 4096;
    let (old_pages, new_pages) = (pages(current), pages(new_end));
    let base = VirtAddr::new(HEAP_BASE);
    // Room in the memory map is checked before anything is mapped: pages
    // mapped but missing from the map would be skipped by a later fork,
    // leaving the child faulting on its own heap.
    let has_room = with_scheduler(|sched| sched.procs[leader].mem_map.can_set_segment(base));
    if !has_room {
        return Err(());
    }
    if new_pages > old_pages {
        let from = base + old_pages * 4096;
        if !crate::memory::map_zeroed(pml4, from, (new_pages - old_pages) as usize) {
            return Err(());
        }
    } else if new_pages < old_pages {
        let from = base + new_pages * 4096;
        crate::memory::unmap_release(pml4, from, (old_pages - new_pages) as usize);
    }
    with_scheduler(|sched| {
        sched.procs[leader].brk = new_end;
        // Can't fail: `can_set_segment` said so above, and nothing
        // between (this call runs with interrupts off) changed the map.
        sched.procs[leader].mem_map.set_segment(base, new_pages as usize)
    });
    Ok(new_end)
}

/// Give `child` the same program break as `parent`'s team -- for
/// `crate::calls::sys_fork`, whose child is built with `spawn` (which
/// starts every process with an empty heap) but whose address space
/// carries a copy of the parent's heap pages and heap segment.
pub fn inherit_brk(child: i32, parent: i32) {
    with_scheduler(|sched| {
        let team = sched.procs[com::slot(parent)].team;
        let brk = sched.procs[com::slot(team)].brk;
        sched.procs[com::slot(child)].brk = brk;
    });
}

/// Create a semaphore with `count` units, owned by the caller's team.
/// `None` if the table is full.
pub fn sem_create(count: i32) -> Option<usize> {
    with_scheduler(|sched| {
        let team = sched.procs[sched.current].team;
        let id = (0..NR_SEMS).find(|&i| !sched.sems[i].in_use)?;
        sched.sems[id] = Sem { in_use: true, count: count.max(0), owner_team: team };
        Some(id)
    })
}

/// Why a semaphore operation failed: no such semaphore (or, for
/// `sem_delete`, not the caller's team's), or it was deleted while the
/// caller was waiting on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadSem;

/// Take one unit of semaphore `id`, blocking until one is released if
/// none is free. Waiters are served first come, first served.
pub fn sem_acquire(id: usize) -> Result<(), BadSem> {
    enum Outcome {
        Got,
        Bad,
        Blocked,
    }
    loop {
        let outcome = with_scheduler(|sched| {
            let idx = sched.current;
            if sched.procs[idx].sem_granted.take().is_some() {
                return Outcome::Got;
            }
            if sched.procs[idx].sem_wait.is_some() {
                // Woken with our place in line still held: spurious.
                return Outcome::Blocked;
            }
            if id >= NR_SEMS || !sched.sems[id].in_use {
                return Outcome::Bad;
            }
            if sched.sems[id].count > 0 {
                sched.sems[id].count -= 1;
                return Outcome::Got;
            }
            let ticket = sched.sem_seq;
            sched.sem_seq += 1;
            if sched.procs[idx].rts_flags == 0 {
                sched.dequeue(idx);
            }
            sched.procs[idx].rts_flags |= rts::SEM_WAIT;
            sched.procs[idx].sem_wait = Some((id, ticket));
            Outcome::Blocked
        });
        match outcome {
            Outcome::Got => return Ok(()),
            Outcome::Bad => return Err(BadSem),
            Outcome::Blocked => {
                reschedule();
                // Woken either with a unit (`sem_granted`) or because the
                // semaphore was deleted (`sem_wait` cleared, not granted).
                let deleted = with_scheduler(|sched| {
                    let p = &sched.procs[sched.current];
                    p.sem_wait.is_none() && p.sem_granted.is_none()
                });
                if deleted {
                    return Err(BadSem);
                }
            }
        }
    }
}

/// Release one unit of semaphore `id`: straight to the longest-waiting
/// acquirer if there is one, otherwise back into the count.
pub fn sem_release(id: usize) -> Result<(), BadSem> {
    with_scheduler(|sched| {
        if id >= NR_SEMS || !sched.sems[id].in_use {
            return Err(BadSem);
        }
        sched.release_unit(id);
        Ok(())
    })?;
    reschedule(); // a woken waiter may outrank the caller
    Ok(())
}

/// Delete semaphore `id`, waking every waiter with an error. Only the
/// owning team may.
pub fn sem_delete(id: usize) -> Result<(), BadSem> {
    with_scheduler(|sched| {
        let team = sched.procs[sched.current].team;
        if id >= NR_SEMS || !sched.sems[id].in_use || sched.sems[id].owner_team != team {
            return Err(BadSem);
        }
        sched.delete_sem(id);
        Ok(())
    })
}

// ---------------------------------------------------------------------
// Ports: Haiku's message queues (`create_port`/`write_port`/`read_port`
// and the rest of the kernel kit's port API, `headers/os/kernel/OS.h`),
// with Haiku's semantics and error values. A port is a named, bounded
// queue of `(int32 code, bytes)` messages any team can write to and
// read from; `BMessage`/`BLooper` messaging is built on them in Haiku.
// ---------------------------------------------------------------------

/// How many ports exist system-wide (Haiku's default is 4096; the
/// kernel heap here is 1 MiB).
pub const NR_PORTS: usize = 64;
/// Longest message a port takes. Haiku allows 256 KiB
/// (`PORT_MAX_MESSAGE_SIZE`); this port's kernel heap is 1 MiB, so less.
pub const PORT_MAX_MESSAGE: usize = 4096;
/// Most messages one port may queue (Haiku's `PORT_MAX_QUEUE`).
pub const PORT_MAX_CAPACITY: i32 = 4096;
/// Bytes all ports together may hold queued, so writers can't exhaust
/// the kernel heap: a write that would exceed it waits for room, as one
/// on a full port does.
pub const PORT_QUEUE_BYTES: usize = 256 * 1024;
/// Haiku's `B_OS_NAME_LENGTH`: a port's name, NUL included.
pub const B_OS_NAME_LENGTH: usize = 32;

/// Haiku's status codes, exactly as `headers/os/support/Errors.h`
/// defines them (`B_GENERAL_ERROR_BASE` is `INT_MIN`).
pub mod haiku {
    pub const B_OK: i32 = 0;
    const GENERAL: i32 = i32::MIN;
    const OS: i32 = GENERAL + 0x1000;
    pub const B_NO_MEMORY: i32 = GENERAL;
    pub const B_BAD_VALUE: i32 = GENERAL + 5;
    pub const B_NAME_NOT_FOUND: i32 = GENERAL + 7;
    pub const B_TIMED_OUT: i32 = GENERAL + 9;
    pub const B_WOULD_BLOCK: i32 = GENERAL + 11;
    pub const B_BAD_TEAM_ID: i32 = OS + 0x103;
    pub const B_BAD_PORT_ID: i32 = OS + 0x200;
    pub const B_NO_MORE_PORTS: i32 = OS + 0x201;
    pub const B_BAD_ADDRESS: i32 = OS + 0x301;
    /// `OS.h`'s timeout flags.
    pub const B_RELATIVE_TIMEOUT: u32 = 0x8;
    pub const B_ABSOLUTE_TIMEOUT: u32 = 0x10;
    /// `OS.h`'s `B_INFINITE_TIMEOUT`.
    pub const B_INFINITE_TIMEOUT: i64 = i64::MAX;
}

pub struct Port {
    name: [u8; B_OS_NAME_LENGTH],
    capacity: i32,
    owner_team: i32,
    closed: bool,
    queue: alloc::collections::VecDeque<(i32, Vec<u8>)>,
    total_read: i32,
}

/// `port_info` (`OS.h`), field for field.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PortInfo {
    pub port: i32,
    pub team: i32,
    pub name: [u8; B_OS_NAME_LENGTH],
    pub capacity: i32,
    pub queue_count: i32,
    pub total_count: i32,
}

/// A timeout as Haiku's `_etc` calls take one: `flags` (`B_RELATIVE_TIMEOUT`
/// or `B_ABSOLUTE_TIMEOUT`, else wait forever) and microseconds. Turned
/// into a deadline tick, or `Err(B_WOULD_BLOCK)` for a relative timeout
/// of zero (Haiku's "don't wait at all").
fn deadline_for(flags: u32, timeout_us: i64, now: u64) -> Result<Option<u64>, i32> {
    // `B_INFINITE_TIMEOUT` (`i64::MAX`, Haiku's usual way to say "wait
    // forever" even with a timeout flag set), and anything too far off to
    // count in ticks, is no deadline at all. The arithmetic saturates: an
    // overflow here used to panic the kernel (a debug build checks) on
    // exactly that idiom.
    let ticks = |us: i64| (us.max(0) as u64).checked_mul(crate::pit::HZ as u64).map(|t| t.div_ceil(1_000_000));
    if timeout_us == haiku::B_INFINITE_TIMEOUT {
        return Ok(None);
    }
    if flags & haiku::B_RELATIVE_TIMEOUT != 0 {
        if timeout_us <= 0 {
            return Err(haiku::B_WOULD_BLOCK);
        }
        Ok(ticks(timeout_us).and_then(|t| now.checked_add(t)))
    } else if flags & haiku::B_ABSOLUTE_TIMEOUT != 0 {
        Ok(ticks(timeout_us))
    } else {
        Ok(None)
    }
}

/// Microseconds since boot, at the timer's resolution -- Haiku's
/// `system_time()`, the clock `B_ABSOLUTE_TIMEOUT` is measured against.
pub fn system_time_us() -> i64 {
    (uptime_ticks() as i64) * 1_000_000 / crate::pit::HZ as i64
}

/// `create_port`: a new port holding up to `capacity` messages, owned by
/// the caller's team. `name` is cut to 31 bytes.
pub fn port_create(capacity: i32, name: &[u8]) -> i32 {
    if capacity <= 0 || capacity > PORT_MAX_CAPACITY {
        return haiku::B_BAD_VALUE;
    }
    with_scheduler(|sched| {
        let team = sched.procs[sched.current].team;
        let Some(slot) = (0..NR_PORTS).find(|&i| sched.ports[i].is_none()) else {
            return haiku::B_NO_MORE_PORTS;
        };
        let mut stored = [0u8; B_OS_NAME_LENGTH];
        let n = name.len().min(B_OS_NAME_LENGTH - 1);
        stored[..n].copy_from_slice(&name[..n]);
        sched.ports[slot] = Some(Port {
            name: stored,
            capacity,
            owner_team: team,
            closed: false,
            queue: alloc::collections::VecDeque::new(),
            total_read: 0,
        });
        // Generations keep ids positive and unique per slot reuse.
        sched.port_generations[slot] = (sched.port_generations[slot] + 1) % (i32::MAX as u32 / NR_PORTS as u32);
        (sched.port_generations[slot] * NR_PORTS as u32 + slot as u32) as i32
    })
}

/// `find_port`: the id of the port named `name` (compared as `create_port`
/// stored it), or `B_NAME_NOT_FOUND`.
pub fn port_find(name: &[u8]) -> i32 {
    let n = name.len().min(B_OS_NAME_LENGTH - 1);
    with_scheduler(|sched| {
        (0..NR_PORTS)
            .find(|&i| {
                sched.ports[i].as_ref().is_some_and(|p| {
                    let len = p.name.iter().position(|&b| b == 0).unwrap_or(B_OS_NAME_LENGTH);
                    p.name[..len] == name[..n]
                })
            })
            .map_or(haiku::B_NAME_NOT_FOUND, |slot| {
                (sched.port_generations[slot] * NR_PORTS as u32 + slot as u32) as i32
            })
    })
}

impl Scheduler {
    /// Mark the caller waiting on port `slot` until it changes or
    /// `deadline` passes -- called under the same lock hold that found it
    /// had to wait, so nothing can change the port in between and leave
    /// it waiting for something that already happened.
    fn begin_port_wait(&mut self, slot: usize, deadline: Option<u64>) {
        let idx = self.current;
        if self.procs[idx].rts_flags == 0 {
            self.dequeue(idx);
        }
        self.procs[idx].rts_flags |= rts::PORT_WAIT;
        self.procs[idx].port_wait = Some(slot);
        self.procs[idx].wait_deadline = deadline;
        self.procs[idx].timed_out = false;
    }
}

/// Switch away after `begin_port_wait`; `true` if the wait timed out.
fn finish_port_wait() -> bool {
    reschedule();
    with_scheduler(|sched| core::mem::take(&mut sched.procs[sched.current].timed_out))
}

/// `write_port_etc`: queue `(code, data)` on port `id`, waiting while it's
/// full (subject to the timeout). `B_OK`, or `B_BAD_PORT_ID` (no such
/// port, closed, or deleted while waiting), `B_BAD_VALUE` (too big),
/// `B_WOULD_BLOCK`/`B_TIMED_OUT`, `B_NO_MEMORY` (all ports' queues full).
pub fn port_write(id: i32, code: i32, data: Vec<u8>, flags: u32, timeout_us: i64) -> i32 {
    if data.len() > PORT_MAX_MESSAGE {
        return haiku::B_BAD_VALUE;
    }
    let deadline = deadline_for(flags, timeout_us, uptime_ticks());
    // The message waits in the process table, not on this stack (see
    // `Proc::port_pending`).
    with_scheduler(|sched| {
        let cur = sched.current;
        sched.procs[cur].port_pending = Some((code, data));
    });
    loop {
        enum Step {
            Done(i32),
            Wait,
        }
        let step = with_scheduler(|sched| {
            let cur = sched.current;
            let Some(slot) = sched.port_slot(id) else {
                sched.procs[cur].port_pending = None;
                return Step::Done(haiku::B_BAD_PORT_ID);
            };
            let len = sched.procs[cur].port_pending.as_ref().map_or(0, |(_, d)| d.len());
            let bytes_ok = sched.port_bytes + len <= PORT_QUEUE_BYTES;
            let port = sched.ports[slot].as_mut().unwrap();
            if port.closed {
                sched.procs[cur].port_pending = None;
                return Step::Done(haiku::B_BAD_PORT_ID);
            }
            // Room in this port *and* under the global byte bound: queue
            // it. Short of either, wait -- as Haiku's writers wait for
            // room -- rather than fail.
            if port.queue.len() < port.capacity as usize && bytes_ok {
                let message = sched.procs[cur].port_pending.take().unwrap();
                sched.ports[slot].as_mut().unwrap().queue.push_back(message);
                sched.port_bytes += len;
                sched.wake_port_waiters(slot);
                return Step::Done(haiku::B_OK);
            }
            match deadline {
                Err(status) => {
                    sched.procs[cur].port_pending = None;
                    Step::Done(status)
                }
                Ok(d) if d.is_some_and(|d| sched.ticks >= d) => {
                    sched.procs[cur].port_pending = None;
                    Step::Done(haiku::B_TIMED_OUT)
                }
                Ok(d) => {
                    sched.begin_port_wait(slot, d);
                    Step::Wait
                }
            }
        });
        match step {
            Step::Done(status) => {
                reschedule(); // a woken reader may outrank the caller
                return status;
            }
            Step::Wait => {
                if finish_port_wait() {
                    with_scheduler(|sched| {
                        let cur = sched.current;
                        sched.procs[cur].port_pending = None;
                    });
                    return haiku::B_TIMED_OUT;
                }
            }
        }
    }
}

/// What a read wants from the queue's head.
enum Take {
    /// Remove it: `read_port`.
    Message,
    /// Just its size: `port_buffer_size`.
    Size,
}

/// The common body of `read_port_etc` and `port_buffer_size_etc`: wait
/// (subject to the timeout) for a message, then take it or report its
/// size. `Ok((code, data))` -- `data` empty and `code` the size for
/// `Take::Size` -- or a Haiku status.
fn port_next(id: i32, flags: u32, timeout_us: i64, take: Take) -> Result<(i32, Vec<u8>), i32> {
    let deadline = deadline_for(flags, timeout_us, uptime_ticks());
    loop {
        enum Step {
            Got(i32, Vec<u8>),
            Fail(i32),
            Wait,
        }
        let step = with_scheduler(|sched| {
            let Some(slot) = sched.port_slot(id) else { return Step::Fail(haiku::B_BAD_PORT_ID) };
            let port = sched.ports[slot].as_mut().unwrap();
            match (port.queue.is_empty(), &take) {
                (false, Take::Size) => {
                    let size = port.queue.front().unwrap().1.len() as i32;
                    Step::Got(size, Vec::new())
                }
                (false, Take::Message) => {
                    let (code, data) = port.queue.pop_front().unwrap();
                    port.total_read = port.total_read.saturating_add(1);
                    sched.port_bytes -= data.len();
                    // Room for a writer now -- on this port, and, since the
                    // byte bound is global, possibly on any other.
                    sched.wake_all_port_waiters();
                    Step::Got(code, data)
                }
                // A closed port reads until it's empty, then is gone.
                (true, _) if port.closed => Step::Fail(haiku::B_BAD_PORT_ID),
                (true, _) => match deadline {
                    // A zero relative timeout: don't wait at all.
                    Err(status) => Step::Fail(status),
                    Ok(d) if d.is_some_and(|d| sched.ticks >= d) => Step::Fail(haiku::B_TIMED_OUT),
                    Ok(d) => {
                        sched.begin_port_wait(slot, d);
                        Step::Wait
                    }
                },
            }
        });
        match step {
            Step::Got(code, data) => return Ok((code, data)),
            Step::Fail(status) => return Err(status),
            Step::Wait => {
                if finish_port_wait() {
                    return Err(haiku::B_TIMED_OUT);
                }
            }
        }
    }
}

/// `read_port_etc`: take the next message, waiting for one.
pub fn port_read(id: i32, flags: u32, timeout_us: i64) -> Result<(i32, Vec<u8>), i32> {
    port_next(id, flags, timeout_us, Take::Message)
}

/// `port_buffer_size_etc`: the size of the next message, waiting for one.
pub fn port_buffer_size(id: i32, flags: u32, timeout_us: i64) -> i32 {
    match port_next(id, flags, timeout_us, Take::Size) {
        Ok((size, _)) => size,
        Err(status) => status,
    }
}

/// `port_count`: messages queued, or `B_BAD_PORT_ID`.
pub fn port_count(id: i32) -> i32 {
    with_scheduler(|sched| match sched.port_slot(id) {
        Some(slot) => sched.ports[slot].as_ref().unwrap().queue.len() as i32,
        None => haiku::B_BAD_PORT_ID,
    })
}

/// `close_port`: no more writes; readers drain what's queued, then get
/// `B_BAD_PORT_ID`. Writers waiting are woken to find it closed.
pub fn port_close(id: i32) -> i32 {
    with_scheduler(|sched| match sched.port_slot(id) {
        Some(slot) if !sched.ports[slot].as_ref().unwrap().closed => {
            sched.ports[slot].as_mut().unwrap().closed = true;
            sched.wake_port_waiters(slot);
            haiku::B_OK
        }
        _ => haiku::B_BAD_PORT_ID,
    })
}

/// `delete_port`: gone, queue and all; every waiter wakes to
/// `B_BAD_PORT_ID`.
pub fn port_delete(id: i32) -> i32 {
    with_scheduler(|sched| match sched.port_slot(id) {
        Some(slot) => {
            sched.delete_port_slot(slot);
            haiku::B_OK
        }
        None => haiku::B_BAD_PORT_ID,
    })
}

/// `set_port_owner`: hand port `id` to `team` (which must be a live team's
/// leader), so it's deleted when *that* team dies.
pub fn port_set_owner(id: i32, team: i32) -> i32 {
    // A live team: its leader's slot in use and not ended (a zombie or
    // draining leader's team has already had its ports deleted, and one
    // handed a port now would never delete it).
    let ended = rts::ZOMBIE | rts::DRAINING | rts::DEAD;
    let team_ok = is_valid_proc_nr(team)
        && team_of(team) == team
        && with_scheduler(|sched| {
            let f = sched.procs[com::slot(team)].rts_flags;
            f & ended == 0 && f != rts::RESERVED
        });
    with_scheduler(|sched| match sched.port_slot(id) {
        None => haiku::B_BAD_PORT_ID,
        Some(_) if !team_ok => haiku::B_BAD_TEAM_ID,
        Some(slot) => {
            sched.ports[slot].as_mut().unwrap().owner_team = team;
            haiku::B_OK
        }
    })
}

/// `get_port_info`.
pub fn port_info(id: i32) -> Result<PortInfo, i32> {
    with_scheduler(|sched| {
        let slot = sched.port_slot(id).ok_or(haiku::B_BAD_PORT_ID)?;
        let p = sched.ports[slot].as_ref().unwrap();
        Ok(PortInfo {
            port: id,
            team: p.owner_team,
            name: p.name,
            capacity: p.capacity,
            queue_count: p.queue.len() as i32,
            total_count: p.total_read,
        })
    })
}

/// Whether `proc_nr` names a process-table slot that is in use -- the
/// check anything taking a process number from ring 3 must make first,
/// since `com::slot` of an arbitrary number indexes the table out of
/// bounds.
pub fn is_valid_proc_nr(proc_nr: i32) -> bool {
    if proc_nr < -(com::NR_TASKS as i32) {
        return false;
    }
    let idx = com::slot(proc_nr);
    idx < com::NR_PROC_SLOTS
        && with_scheduler(|sched| sched.procs[idx].rts_flags & rts::SLOT_FREE == 0)
}

/// The generation of the process currently in `proc_nr`'s slot (see
/// `Scheduler::generations`), or `None` if the number names no live
/// process. Two calls returning the same `Some` mean the same process.
pub fn generation_of(proc_nr: i32) -> Option<u32> {
    if proc_nr < -(com::NR_TASKS as i32) || com::slot(proc_nr) >= com::NR_PROC_SLOTS {
        return None;
    }
    let idx = com::slot(proc_nr);
    // One lock for both reads, so the answer describes one moment.
    with_scheduler(|sched| {
        (sched.procs[idx].rts_flags & rts::SLOT_FREE == 0).then_some(sched.generations[idx])
    })
}

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
        // A new image starts with an empty heap.
        sched.procs[idx].brk = HEAP_BASE;
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
            // A timed port wait whose deadline has passed.
            if sched.procs[i].rts_flags & rts::PORT_WAIT != 0
                && sched.procs[i].wait_deadline.is_some_and(|d| now >= d)
            {
                sched.procs[i].timed_out = true;
                sched.stop_port_wait(i);
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

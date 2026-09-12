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
//! (the `restart()` assembly in `kernel/mpx386.s` compares `next_ptr` against
//! `proc_ptr` on every single trap return). This port has no interrupt-driven
//! preemption yet (see `rust/README.md`), so there is no such central
//! trap-return hook; instead, `reschedule()` is called explicitly at the end
//! of every blocking IPC operation and by the idle task, and performs the
//! switch immediately if `next_ptr` differs from the running process.

use core::ptr;
use spin::Mutex;

use crate::com;
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
}

pub struct Proc {
    pub proc_nr: i32,
    pub name: &'static str,
    rts_flags: u8,
    priority: u8,
    max_priority: u8,
    ticks_left: i32,
    quantum_size: i32,
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
    /// because everything in this port shares one address space and a
    /// blocked process's stack frame (and thus this pointer's target)
    /// stays alive, untouched, for exactly as long as it remains blocked.
    messbuf: *mut Message,
    /// This task's entry point. Not part of `struct proc` in the C kernel
    /// (there, `p_reg.pc` -- the saved instruction pointer -- serves the
    /// same purpose once execution is underway); kept separately here so
    /// `trampoline` can find it the first time this task is switched to.
    entry: fn() -> !,
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
            rsp: 0,
            next_ready: None,
            caller_q: None,
            q_link: None,
            get_from: com::NONE,
            send_to: com::NONE,
            messbuf: ptr::null_mut(),
            entry: never_spawned,
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
}

lazy_static::lazy_static! {
    static ref SCHEDULER: Mutex<Scheduler> = Mutex::new(Scheduler {
        procs: core::array::from_fn(|_| Proc::empty()),
        rdy_head: [None; NR_SCHED_QUEUES],
        rdy_tail: [None; NR_SCHED_QUEUES],
        current: BOOTSTRAP,
        next_ptr: None,
        prev_for_penalty: None,
    });
}

/// Prepare a never-yet-run task's stack so that switching to it for the
/// first time lands in `trampoline` (see below), and enqueue it as ready.
/// Standalone equivalent of an image-table entry in `kernel/table.c` plus
/// the register initialization `kernel/main.c` does for each boot-image
/// process before the first `restart()`.
pub fn spawn(proc_nr: i32, name: &'static str, entry: fn() -> !, priority: u8, quantum: i32) {
    let idx = com::slot(proc_nr);
    // Build the initial stack frame that `switch_to`'s epilogue will pop:
    // six callee-saved registers (unused, so zeroed) followed by a return
    // address, which `ret` will jump to -- landing in `trampoline` on this
    // task's own stack for the very first time it runs.
    let rsp = unsafe {
        let stack = ptr::addr_of_mut!(STACKS.0[idx]);
        let top = (stack as *mut u8).add(STACK_SIZE);
        let frame = (top as usize & !0xf) as *mut u64; // 16-byte align
        let frame = frame.sub(7);
        frame.add(0).write(0); // r15
        frame.add(1).write(0); // r14
        frame.add(2).write(0); // r13
        frame.add(3).write(0); // r12
        frame.add(4).write(0); // rbp
        frame.add(5).write(0); // rbx
        frame.add(6).write(trampoline as *const () as u64);
        frame as u64
    };

    let mut sched = SCHEDULER.lock();
    sched.procs[idx] = Proc {
        proc_nr,
        name,
        rts_flags: 0,
        priority,
        max_priority: priority,
        ticks_left: quantum,
        quantum_size: quantum,
        rsp,
        next_ready: None,
        caller_q: None,
        q_link: None,
        get_from: com::NONE,
        send_to: com::NONE,
        messbuf: ptr::null_mut(),
        entry,
    };
    sched.enqueue(idx);
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
/// time a function call happens) and the current `rsp` into `*prev_rsp`,
/// then load `next_rsp` and pop the next task's saved registers. Ported in
/// spirit from `restart()`/`save()` in `kernel/mpx386.s`, minus the
/// trap-frame handling those deal with and we don't need yet (no user mode,
/// no interrupts landing mid-task).
#[unsafe(naked)]
unsafe extern "C" fn switch_to(prev_rsp: *mut u64, next_rsp: u64) {
    core::arch::naked_asm!(
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
        "ret",
    )
}

/// Switch away from the current task if a higher-priority (or, at equal
/// priority, differently-queued) one is now runnable. Called at the end of
/// every blocking IPC operation in `crate::ipc`, standing in for the
/// unconditional "did `next_ptr` change?" check that real MINIX makes on
/// every single trap return (see the module doc comment).
pub fn reschedule() {
    let (prev_ptr, next_rsp): (*mut u64, u64) = {
        let mut sched = SCHEDULER.lock();
        let next = match sched.next_ptr {
            Some(n) => n,
            None => panic!("reschedule(): no runnable process (not even IDLE?)"),
        };
        if next == sched.current {
            return;
        }
        let prev = sched.current;
        sched.current = next;
        unsafe { NEXT_ENTRY = sched.procs[next].entry };
        let prev_ptr: *mut u64 = &mut sched.procs[prev].rsp;
        let next_rsp = sched.procs[next].rsp;
        (prev_ptr, next_rsp)
    };
    unsafe { switch_to(prev_ptr, next_rsp) };
}

/// The process number of whichever task is currently running.
pub fn current_proc_nr() -> i32 {
    let sched = SCHEDULER.lock();
    sched.procs[sched.current].proc_nr
}

/// Voluntarily give up the rest of the current quantum and move to the
/// back of this task's ready queue. Real MINIX never needs this: the
/// clock task forces the equivalent of it via `dequeue`+`enqueue` when a
/// running process's `p_ticks_left` hits zero on a clock interrupt. This
/// port has no timer interrupt yet (see `rust/README.md`), so the kernel
/// tasks that want round-robin behavior call this explicitly between
/// iterations of their main loop instead.
pub fn yield_now() {
    let mut sched = SCHEDULER.lock();
    let idx = sched.current;
    sched.procs[idx].ticks_left = 0;
    sched.dequeue(idx);
    sched.enqueue(idx);
    drop(sched);
    reschedule();
}

/// `mini_send()`: send `m` from the running task to `dst`. Ported from
/// `kernel/proc.c`. If `dst` is already blocked in `mini_receive` waiting
/// for this message, it's delivered immediately and this returns without
/// switching away. Otherwise the caller blocks (dequeuing itself and
/// queuing onto `dst`'s `caller_q`) and does not return until `dst` (or a
/// third party, via `mini_receive`) has picked the message up.
///
/// Simplification: the C version detects a SEND/SEND cycle and returns
/// `ELOCKED` so the caller can recover. Nothing here has a way to report
/// that back yet (there's no error-propagating IPC API in front of this,
/// and no signals -- see `rust/README.md`), so a cyclic deadlock panics
/// instead of returning an error. None of the tasks spawned so far can
/// trigger this; revisit once real servers can.
pub fn mini_send(dst: i32, m: &Message) {
    let dst_idx = com::slot(dst);
    let mut sched = SCHEDULER.lock();
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

    // Destination isn't waiting for this. Block: dequeue the caller, mark
    // it SENDING, and append it to dst's caller_q.
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
    drop(sched);
    // When this returns, some later mini_receive/mini_notify has already
    // copied `*m` out and cleared our SENDING flag -- see below.
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
    let mut sched = SCHEDULER.lock();
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
            return msg;
        }
        prev = cursor;
        cursor = sched.procs[cur].q_link;
    }

    // No sender ready. Block until one arrives; `placeholder` lives on
    // this task's own suspended stack for as long as we're blocked, and
    // whoever delivers the message writes straight into it through
    // `messbuf` before waking us back up.
    let mut placeholder = Message::empty();
    sched.procs[caller].messbuf = &mut placeholder as *mut Message;
    sched.procs[caller].get_from = src;
    if sched.procs[caller].rts_flags == 0 {
        sched.dequeue(caller);
    }
    sched.procs[caller].rts_flags |= rts::RECEIVING;
    drop(sched);
    reschedule();
    placeholder
}

/// `mini_notify()`: a lightweight, fire-and-forget send used for kernel
/// events (alarms, interrupts). Ported from `kernel/proc.c`; see
/// `mini_receive`'s doc comment for the one respect (no pending-bitmap)
/// in which this port is simpler than the original.
pub fn mini_notify(dst: i32, m_type: i32) {
    let dst_idx = com::slot(dst);
    let mut sched = SCHEDULER.lock();
    let caller_proc_nr = sched.procs[sched.current].proc_nr;

    let dst_receiving =
        sched.procs[dst_idx].rts_flags & (rts::RECEIVING | rts::SENDING) == rts::RECEIVING;
    let accepted = dst_receiving
        && (sched.procs[dst_idx].get_from == com::ANY
            || sched.procs[dst_idx].get_from == caller_proc_nr);
    if accepted {
        let msg = Message { source: caller_proc_nr, m_type, args: [0; 4] };
        unsafe { *sched.procs[dst_idx].messbuf = msg };
        sched.procs[dst_idx].rts_flags &= !rts::RECEIVING;
        if sched.procs[dst_idx].rts_flags == 0 {
            sched.enqueue(dst_idx);
        }
    }
}

/// Hand off from the bootstrap context (`kernel_main`'s own stack, set up
/// by the bootloader) to the first ready task. Equivalent to `main()`
/// calling `restart()` at the end of `kernel/main.c` -- like there, this
/// never returns: `kernel_main`'s stack is simply abandoned.
pub fn start() -> ! {
    let next_rsp = {
        let mut sched = SCHEDULER.lock();
        sched.pick_proc();
        let next = sched.next_ptr.expect("start(): no task was spawned");
        sched.current = next;
        unsafe { NEXT_ENTRY = sched.procs[next].entry };
        sched.procs[next].rsp
    };
    let mut discarded: u64 = 0;
    unsafe { switch_to(&mut discarded, next_rsp) };
    unreachable!("switch_to into the first task must not return to the bootstrap context");
}

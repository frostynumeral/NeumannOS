//! `rs` (reincarnation server): notices a crashed process and restarts
//! it. Ported in spirit from `servers/rs/manager.c`'s crash-handling
//! path -- real MINIX gets there via `PM` noticing a process's
//! unexpected exit and telling `RS`, which looks up that service's
//! startup parameters in its own table and re-execs it; this port has
//! neither signals nor `PM`'s exit path yet (see `rust/README.md`), so
//! `crate::proc::kill` -- called directly from a ring-3 task's own
//! exception handler (`crate::interrupts`) -- delivers a `com::proc_died`
//! notification straight to `RS`, and `task`'s restart parameters are
//! just hardcoded here rather than looked up in a real service table.
//!
//! `flaky` is this milestone's demo service: a real ring-3 task (its own
//! address space, built the same way as `crate::usermode`'s demo) whose
//! only instruction is `ud2` -- x86's guaranteed-`#UD` opcode -- so it
//! crashes the instant it runs, on purpose and deterministically, giving
//! the crash-isolation/restart pipeline something real to prove itself
//! against without depending on a coincidental bug.

use crate::com;
use crate::ipc;
use crate::memory;
use crate::proc;
use crate::serial_println;
use crate::usermode;

/// Distinct from `crate::usermode`'s own demo addresses purely for
/// clarity when reading a memory dump -- each ring-3 task gets its own
/// separate page table, so there's no real collision risk either way.
pub const FLAKY_CODE_ADDR: u64 = 0x_5555_5557_0000;
pub const FLAKY_STACK_ADDR: u64 = 0x_6666_6668_0000;

/// `ud2`: the x86 opcode the architecture itself guarantees will always
/// raise `#UD` (invalid opcode), regardless of what CPU it runs on or
/// what state the machine is in. Picked specifically *because* it's
/// deterministic -- a demo built around "eventually some real bug
/// crashes this" would be much harder to reason about or reproduce.
pub const FLAKY_CODE: [u8; 2] = [0x0F, 0x0B];

/// How many times `rs` will restart `flaky` before giving up and leaving
/// it dead. Real MINIX's `RS` has a similar backoff/give-up policy
/// (`servers/rs/manager.c`'s restart limits) for exactly the same
/// reason: a service that crashes instantly, every time, should
/// eventually stop being restarted rather than spinning forever.
const MAX_RESTARTS: u32 = 3;

/// Build `flaky`'s address space and spawn it. Used both for its initial
/// boot-time spawn (`main.rs`'s `spawn_tasks`) and for every restart
/// (`task`, below) -- restarting *is* just spawning again, since
/// `proc::spawn` already overwrites a process-table slot wholesale
/// regardless of whether anything was there before.
pub fn spawn_flaky() {
    let physical_memory_offset = memory::physical_memory_offset();
    let address_space = usermode::build_ring3_address_space(
        physical_memory_offset,
        FLAKY_CODE_ADDR,
        &FLAKY_CODE,
        FLAKY_STACK_ADDR,
    );
    proc::spawn(
        com::FLAKY_PROC_NR,
        "flaky (crash demo)",
        flaky_task_entry,
        6,
        16,
        true,
        Some(address_space),
    );
}

/// `flaky`'s only job: jump to ring 3 and immediately execute `ud2`,
/// raising `#UD` at CPL 3. `crate::interrupts::recover_or_halt` sees the
/// faulting `CS` selector's RPL is 3, so it calls `proc::kill` instead of
/// halting the machine -- proof this is genuine per-process fault
/// isolation, not just "nothing happens to crash yet."
pub fn flaky_task_entry() -> ! {
    usermode::jump_to_ring3(FLAKY_CODE_ADDR, FLAKY_STACK_ADDR + usermode::PAGE_SIZE)
}

/// `rs`'s real task body, replacing the `busy_task_a` stand-in: block for
/// a `com::proc_died` notification, decide whether to restart whoever
/// died, and loop. Exactly real MINIX's
/// `while (TRUE) { receive(ANY, &m); ...; }` shape in
/// `servers/rs/main.c`, minus the parts of that loop (a real service
/// table, `up`/`down`/`refresh` commands from the `service` utility) this
/// port has no equivalent for yet.
pub fn task() -> ! {
    let mut flaky_restarts: u32 = 0;
    loop {
        let notif = ipc::receive(com::ANY);
        let Some(slot) = com::proc_died_slot(notif.m_type) else {
            serial_println!(
                "[rs] received an unexpected notification (m_type {:#x}), ignoring",
                notif.m_type
            );
            continue;
        };
        let proc_nr = com::proc_nr_of_slot(slot);
        if proc_nr != com::FLAKY_PROC_NR {
            serial_println!(
                "[rs] proc_nr {} died -- no restart policy for it yet, leaving it dead",
                proc_nr
            );
            continue;
        }
        if flaky_restarts >= MAX_RESTARTS {
            serial_println!(
                "[rs] flaky (proc_nr {}) died again -- already restarted it {} times, giving up",
                proc_nr,
                MAX_RESTARTS
            );
            continue;
        }
        flaky_restarts += 1;
        serial_println!(
            "[rs] flaky (proc_nr {}) died -- restarting it (attempt {}/{})",
            proc_nr,
            flaky_restarts,
            MAX_RESTARTS
        );
        spawn_flaky();
    }
}

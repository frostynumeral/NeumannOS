//! `rs` (reincarnation server): notices a crashed process and restarts
//! it, and now also launches a named service on request. Ported in spirit
//! from `servers/rs/manager.c`'s crash-handling path -- real MINIX gets
//! there via `PM` noticing a process's unexpected exit and telling `RS`,
//! which looks up that service's startup parameters in its own table and
//! re-execs it; this port has neither signals nor `PM`'s exit path yet
//! (see `rust/README.md`), so `crate::proc::kill` -- called directly from
//! a ring-3 task's own exception handler (`crate::interrupts`) --
//! delivers a `com::proc_died` notification straight to `RS`.
//!
//! `flaky` is this port's crash-recovery demo: a real ring-3 task (its own
//! address space, built the same way as `crate::usermode`'s demo) whose
//! only instruction is `ud2` -- x86's guaranteed-`#UD` opcode -- so it
//! crashes the instant it runs, on purpose and deterministically, giving
//! the crash-isolation/restart pipeline something real to prove itself
//! against without depending on a coincidental bug. It stays hardcoded
//! (`spawn_flaky`, below) rather than folded into `SERVICES`, since it's a
//! hand-assembled two-byte payload, not an ELF binary loaded from `fs`.
//!
//! `SERVICES` is this port's first real service table: unlike `flaky`,
//! each entry is a real ELF file loaded from `fs` (`crate::elf::
//! spawn_from_fs`), launchable on demand via a `com::RS_LAUNCH_REQUEST`
//! (`crate::keyboard`'s console line discipline is the only current
//! sender -- a typed `run <name>` command), and restarted the same way
//! `flaky` is if it dies.

use crate::com;
use crate::elf;
use crate::ipc;
use crate::memory;
use crate::proc;
use crate::serial_println;
use crate::usermode;
use spin::Mutex;

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

/// One entry in `rs`'s real (if tiny) service table: a named, `fs`-backed
/// ELF binary this port knows how to start and restart. Mirrors the
/// startup parameters a real `servers/rs/manager.c` service table entry
/// carries (path, scheduling parameters, restart policy), minus anything
/// this port has no equivalent for yet (dependencies, privileges).
struct ServiceEntry {
    name: &'static str,
    path: &'static str,
    proc_nr: i32,
    priority: u8,
    quantum: i32,
    max_restarts: u32,
}

/// This port's one runtime-launchable app: `elf::HELLO_ELF`'s bytes,
/// seeded into `fs` at boot (`main.rs`) under this same path, loadable a
/// second time -- as a genuinely separate task under its own process
/// number -- independently of `tty`'s boot-time run of the very same
/// image. A real service table would have many entries; this has exactly
/// the one needed to prove the mechanism.
const SERVICES: [ServiceEntry; 1] = [ServiceEntry {
    name: "hello",
    path: "/bin/hello",
    proc_nr: com::APP1_PROC_NR,
    priority: 6,
    quantum: 16,
    max_restarts: 3,
}];

#[derive(Clone, Copy)]
struct ServiceState {
    running: bool,
    restarts: u32,
}

/// Per-`SERVICES`-entry runtime state, indexed the same way `SERVICES`
/// is. Separate from the const table itself since this part actually
/// changes across a service's lifetime (launched, crashed, restarted).
static SERVICE_STATE: Mutex<[ServiceState; SERVICES.len()]> =
    Mutex::new([ServiceState { running: false, restarts: 0 }; SERVICES.len()]);

/// Reply codes for a `com::RS_LAUNCH_REQUEST`, alongside `crate::elf::
/// spawn_from_fs`'s own negative `fs` error codes (forwarded as-is on a
/// load failure) -- `>= 0` always means success, matching the convention
/// `crate::fs`'s replies already use.
pub const RS_LAUNCH_OK: i64 = 0;
pub const RS_UNKNOWN_SERVICE: i64 = -1;
pub const RS_ALREADY_RUNNING: i64 = -2;

/// Look `name` up in `SERVICES` and, if it isn't already running, load and
/// spawn it (`elf::spawn_from_fs`). The one place that actually starts a
/// `SERVICES` entry for the first time; `task`'s death-notification branch
/// calls `elf::spawn_from_fs` directly instead (a restart, not a fresh
/// launch), since it already knows the entry and doesn't need the
/// name/`running` lookup this does.
fn launch_service(name: &str) -> i64 {
    let Some(idx) = SERVICES.iter().position(|s| s.name == name) else {
        return RS_UNKNOWN_SERVICE;
    };
    if SERVICE_STATE.lock()[idx].running {
        return RS_ALREADY_RUNNING;
    }
    let entry = &SERVICES[idx];
    match elf::spawn_from_fs(entry.path, entry.proc_nr, entry.name, entry.priority, entry.quantum) {
        Ok(()) => {
            SERVICE_STATE.lock()[idx].running = true;
            RS_LAUNCH_OK
        }
        Err(err) => err,
    }
}

/// Handle a `com::RS_LAUNCH_REQUEST`: `notif.args[0]`/`args[1]` are a
/// pointer/length pair naming the service to launch. Safe to dereference
/// directly (no kernel-stack-buffer copy, unlike `crate::syscall`'s
/// ring-3-facing calls): both `console_task` and `rs` run in the kernel's
/// own address space, the same reasoning `crate::fs`'s `FS_OPEN` relies
/// on for its own callers today.
fn handle_launch_request(notif: &ipc::Message) {
    let name_ptr = notif.args[0] as *const u8;
    let name_len = notif.args[1] as usize;
    let name =
        unsafe { core::str::from_utf8(core::slice::from_raw_parts(name_ptr, name_len)).unwrap_or("") };
    let result = launch_service(name);
    serial_println!("[rs] launch request for {:?} -> {}", name, result);
    ipc::send(
        notif.source,
        ipc::Message { source: com::RS_PROC_NR, m_type: com::RS_LAUNCH_REQUEST, args: [result, 0, 0, 0] },
    );
}

/// `rs`'s real task body: block for either a `com::proc_died` notification
/// or a `com::RS_LAUNCH_REQUEST`, act on whichever arrived, and loop.
/// Exactly real MINIX's `while (TRUE) { receive(ANY, &m); ...; }` shape in
/// `servers/rs/main.c`, minus the parts of that loop (`up`/`down`/
/// `refresh` commands from the `service` utility) this port has no
/// equivalent for yet.
pub fn task() -> ! {
    let mut flaky_restarts: u32 = 0;
    loop {
        let notif = ipc::receive(com::ANY);

        if notif.m_type == com::RS_LAUNCH_REQUEST {
            handle_launch_request(&notif);
            continue;
        }

        let Some(slot) = com::proc_died_slot(notif.m_type) else {
            serial_println!(
                "[rs] received an unexpected notification (m_type {:#x}), ignoring",
                notif.m_type
            );
            continue;
        };
        let proc_nr = com::proc_nr_of_slot(slot);

        if proc_nr == com::FLAKY_PROC_NR {
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
            continue;
        }

        let Some(idx) = SERVICES.iter().position(|s| s.proc_nr == proc_nr) else {
            serial_println!(
                "[rs] proc_nr {} died -- no restart policy for it yet, leaving it dead",
                proc_nr
            );
            continue;
        };
        SERVICE_STATE.lock()[idx].running = false;
        let entry = &SERVICES[idx];
        let restarts = SERVICE_STATE.lock()[idx].restarts;
        if restarts >= entry.max_restarts {
            serial_println!(
                "[rs] {} (proc_nr {}) died again -- already restarted it {} times, giving up",
                entry.name,
                proc_nr,
                entry.max_restarts
            );
            continue;
        }
        SERVICE_STATE.lock()[idx].restarts += 1;
        serial_println!(
            "[rs] {} (proc_nr {}) died -- restarting it (attempt {}/{})",
            entry.name,
            proc_nr,
            restarts + 1,
            entry.max_restarts
        );
        match elf::spawn_from_fs(entry.path, entry.proc_nr, entry.name, entry.priority, entry.quantum) {
            Ok(()) => SERVICE_STATE.lock()[idx].running = true,
            Err(err) => {
                serial_println!("[rs] failed to restart {} (proc_nr {}): {}", entry.name, proc_nr, err)
            }
        }
    }
}

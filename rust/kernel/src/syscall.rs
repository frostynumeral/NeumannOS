//! The real `int 0x80` gate: call-number/register dispatch, replacing
//! `crate::interrupts`'s old fixed-action, count-and-cut-off handler.
//!
//! Ported in spirit from `kernel/system.c`'s kernel-call dispatch table
//! (`map(SYS_xxx, do_xxx)`) and the trap gate that reaches it
//! (`kernel/mpx386.s`'s `s_call`/`sys_call`) -- real MINIX actually
//! dispatches kernel calls through the same message-passing rendezvous
//! every other IPC uses (a `SENDREC` to `SYSTEM` whose message type is
//! the call number), not a raw register convention; this port uses a
//! simpler, more conventional "call number in a register" ABI (closer to
//! Linux's `int 0x80` than MINIX's own) since there's no in-kernel
//! message-passing entry point from ring 3 yet, and building the real
//! thing needs this register-level plumbing first regardless.
//!
//! Convention: call number in `rax`, up to three arguments in `rdi`,
//! `rsi`, `rdx` (the first three System V integer-argument registers),
//! return value in `rax`. `entry` is the actual `SYSCALL_VECTOR` IDT handler (installed
//! via `Entry::set_handler_addr` in `crate::interrupts`, not
//! `set_handler_fn` -- this needs full control over the trap frame that
//! the `x86-interrupt` calling convention doesn't expose, namely the
//! caller's original register values, not just the hardware-pushed
//! `InterruptStackFrame`). It saves every general-purpose register the
//! CPU didn't already save, calls `dispatch` with the caller's original
//! `rax`/`rdi`/`rsi`, writes `dispatch`'s return value into the saved
//! `rax` slot, restores everything else unchanged, and `iretq`s -- so a
//! caller sees only its requested register (`rax`) change, exactly like
//! a real syscall.
//!
//! Notably *not* implemented here: any cross-address-space copy for
//! pointer arguments (`SYS_WRITE_LINE`'s `arg1`). That's safe to skip
//! because of a property already established for every ring-3 task with
//! its own address space (`crate::usermode`, `crate::elf`): entering this
//! handler via `int 0x80` does *not* switch `CR3` (only entering an
//! interrupt/trap gate itself is a hardware CR3-preserving operation,
//! unlike a `crate::proc::switch_to` task switch, which explicitly
//! reloads it) -- so `dispatch` runs with the *caller's own* address
//! space still active, and a pointer the caller passed is already
//! directly dereferenceable, the same way it would be for the caller
//! itself. A real `sys_vircopy`-style copy is only needed to reach a
//! *different* process's memory (see `crate::calls`/`crate::memory`),
//! not the currently-running one's own.
//!
//! `SYS_SET_ALARM`/`SYS_WAIT_ALARM` are the first of `crate::calls`' own
//! kernel calls reachable from ring 3 through this ABI: `SYS_SET_ALARM`
//! calls the same `calls::sys_setalarm` `CLOCK` already uses, and
//! `SYS_WAIT_ALARM` blocks the caller in `ipc::receive` *inside the trap
//! itself* until the real `SYN_ALARM` notification arrives -- proving a
//! ring-3 task can genuinely block on a kernel call and later resume
//! executing ring-3 code afterward (via this same trap's `iretq`), not
//! just make one-shot, always-returns-immediately calls.
//!
//! `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ` go one step further: a real
//! IPC round trip to `fs` (`crate::fs`), not just another kernel-internal
//! call. Their pointer arguments get one more layer of care than
//! `SYS_WRITE_LINE`'s: `dispatch` copies a caller's buffer into a local
//! (kernel-stack) buffer *before* calling into `crate::fs`'s client stubs,
//! rather than handing the caller's own pointer to them directly. That
//! extra copy matters here in a way it didn't for `SYS_WRITE_LINE`: `fs`
//! is a *different task*, and by the time it actually dereferences a
//! pointer in the request message, `CR3` may no longer be the caller's
//! (`crate::fs`'s own module doc comment covers the rest of this
//! caveat). A kernel-stack buffer is safe to hand across that boundary
//! because it lives in memory the kernel maps identically into every
//! address space (like any other kernel-static data), unlike a ring-3
//! task's own private pages.

use crate::{calls, com, fs, ipc, proc, serial_println};

pub const SYS_GET_UPTIME: u64 = 1;
pub const SYS_WRITE_LINE: u64 = 2;
pub const SYS_BLOCK_FOREVER: u64 = 3;
pub const SYS_SET_ALARM: u64 = 4;
pub const SYS_WAIT_ALARM: u64 = 5;
pub const SYS_FS_OPEN: u64 = 6;
pub const SYS_FS_WRITE: u64 = 7;
pub const SYS_FS_READ: u64 = 8;

/// Longest path/buffer `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ` will
/// copy through a local kernel-stack buffer in either direction.
const MAX_FS_BUF: usize = 256;

/// Longest string `SYS_WRITE_LINE` will read, purely as a sanity bound on
/// `arg2` (an untrusted length from ring 3) -- not a real buffer, since
/// the caller's bytes are read directly out of its own, still-active
/// address space (see the module doc comment).
const MAX_LINE_LEN: u64 = 256;

/// Sentinel error return: `SYS_WRITE_LINE` got a bad length or invalid
/// UTF-8, or the call number wasn't recognized at all. Every real
/// `SYS_*` return value here fits in far fewer bits, so this is
/// unambiguous -- a rough stand-in for a real syscall ABI's negative
/// `errno` convention (`crate::calls`' kernel calls already use actual
/// negative-`i64` returns for this; this one stays unsigned since
/// `SYS_GET_UPTIME`'s tick count has no natural sign to spare).
pub const ERROR: u64 = u64::MAX;

/// The actual dispatch, called by `entry` (via `core::arch::naked_asm!`'s
/// `sym` operand) with the caller's original `rax` (as `call_num`),
/// `rdi` (`arg1`), `rsi` (`arg2`), and `rdx` (`arg3`) -- `entry`'s doc
/// comment has the full register-to-argument mapping.
extern "C" fn dispatch(call_num: u64, arg1: u64, arg2: u64, arg3: u64) -> u64 {
    let caller = proc::current_proc_nr();
    match call_num {
        SYS_GET_UPTIME => {
            let ticks = proc::uptime_ticks();
            serial_println!("[syscall] proc {}: SYS_GET_UPTIME -> {}", caller, ticks);
            ticks
        }
        SYS_WRITE_LINE => {
            if arg2 > MAX_LINE_LEN {
                return ERROR;
            }
            // Safety: see the module doc comment -- `dispatch` runs with
            // the caller's own address space still active (entering this
            // trap gate never switched `CR3`), so a pointer the caller
            // just gave us in its own `rdi` is valid to dereference
            // directly, the same as it would be for the caller itself.
            let bytes = unsafe { core::slice::from_raw_parts(arg1 as *const u8, arg2 as usize) };
            match core::str::from_utf8(bytes) {
                Ok(s) => {
                    serial_println!("[syscall] proc {}: SYS_WRITE_LINE: {:?}", caller, s);
                    arg2
                }
                Err(_) => ERROR,
            }
        }
        SYS_BLOCK_FOREVER => {
            serial_println!("[syscall] proc {}: SYS_BLOCK_FOREVER, blocking for good", caller);
            // Never returns: nothing sends to this proc again. Runs
            // straight from kernel context mid-trap, exactly like any
            // kernel task calling `ipc::receive` directly -- there's
            // nothing ring-3-specific about blocking itself, only about
            // how we got here.
            ipc::receive(com::ANY);
            unreachable!("nothing sends to a process that called SYS_BLOCK_FOREVER");
        }
        SYS_SET_ALARM => {
            // The first of crate::calls' own kernel calls reachable from
            // ring 3 through this ABI (see the module doc comment's note
            // that none were, until now): calls the very same
            // calls::sys_setalarm CLOCK itself uses, operating on
            // `proc::current_proc_nr()` -- the caller, since dispatch runs
            // in the trapped task's own context without a task switch.
            serial_println!("[syscall] proc {}: SYS_SET_ALARM({} ticks)", caller, arg1);
            calls::sys_setalarm(arg1);
            0
        }
        SYS_WAIT_ALARM => {
            // Blocks the caller *inside this very trap*, exactly the way
            // crate::proc's own CLOCK task already blocks in `ipc::receive`
            // after calling `sys_setalarm` -- there's nothing ring-3-
            // specific about blocking in kernel context mid-syscall; the
            // scheduler already handles resuming an arbitrary blocked call
            // stack, and this one just happens to `iretq` back to ring 3
            // once it does. Not paired with a check that a SYS_SET_ALARM
            // actually preceded it: a caller that waits with no alarm
            // pending simply blocks forever, the same as `ipc::receive`
            // always would with nothing to wake it.
            let notif = ipc::receive(com::CLOCK);
            debug_assert_eq!(
                notif.m_type,
                calls::SYN_ALARM,
                "SYS_WAIT_ALARM woken by something other than a real alarm notification"
            );
            let ticks = proc::uptime_ticks();
            serial_println!(
                "[syscall] proc {}: SYS_WAIT_ALARM woken by a real SYN_ALARM notification (uptime {})",
                caller,
                ticks
            );
            ticks
        }
        SYS_FS_OPEN => {
            if arg2 as usize > MAX_FS_BUF {
                return ERROR;
            }
            // Safety: same reasoning as SYS_WRITE_LINE -- CR3 is still
            // the caller's own here.
            let src = unsafe { core::slice::from_raw_parts(arg1 as *const u8, arg2 as usize) };
            let mut path_buf = [0u8; MAX_FS_BUF];
            path_buf[..src.len()].copy_from_slice(src);
            match core::str::from_utf8(&path_buf[..src.len()]) {
                Ok(path) => {
                    let result = fs::open(path);
                    serial_println!("[syscall] proc {}: SYS_FS_OPEN({:?}) -> {}", caller, path, result);
                    result as u64
                }
                Err(_) => ERROR,
            }
        }
        SYS_FS_WRITE => {
            let fd = arg1 as i64;
            let len = arg3 as usize;
            if len > MAX_FS_BUF {
                return ERROR;
            }
            // Copy the caller's buffer into a local, kernel-mapped-
            // everywhere buffer *before* calling into crate::fs -- see
            // the module doc comment for why this copy (unlike
            // SYS_WRITE_LINE's lack of one) is load-bearing here.
            let src = unsafe { core::slice::from_raw_parts(arg2 as *const u8, len) };
            let mut buf = [0u8; MAX_FS_BUF];
            buf[..len].copy_from_slice(src);
            let result = fs::write(fd, &buf[..len]);
            serial_println!("[syscall] proc {}: SYS_FS_WRITE(fd {}, {} bytes) -> {}", caller, fd, len, result);
            result as u64
        }
        SYS_FS_READ => {
            let fd = arg1 as i64;
            let len = core::cmp::min(arg3 as usize, MAX_FS_BUF);
            let mut buf = [0u8; MAX_FS_BUF];
            let result = fs::read(fd, &mut buf[..len]);
            // By the time fs::read returns, this task has been resumed
            // (its own CR3 is active again -- see the module doc
            // comment), so writing straight to the caller's own pointer
            // here is safe again, the same as SYS_WRITE_LINE's read was.
            if result > 0 {
                let dst = unsafe { core::slice::from_raw_parts_mut(arg2 as *mut u8, result as usize) };
                dst.copy_from_slice(&buf[..result as usize]);
            }
            serial_println!("[syscall] proc {}: SYS_FS_READ(fd {}) -> {}", caller, fd, result);
            result as u64
        }
        _ => {
            serial_println!("[syscall] proc {}: unknown call number {}", caller, call_num);
            ERROR
        }
    }
}

/// The `SYSCALL_VECTOR` trap gate itself. Must be installed with
/// `Entry::set_handler_addr` (an arbitrary code address), not
/// `set_handler_fn`: this is deliberately *not* an `extern "x86-interrupt"`
/// function, since that calling convention only exposes the
/// hardware-pushed `InterruptStackFrame` (`RIP`/`CS`/`RFLAGS`/`RSP`/`SS`),
/// not the general-purpose registers a caller actually passes arguments
/// in -- reading those safely needs to happen in hand-written assembly,
/// before any compiler-generated prologue could touch them.
///
/// Saves all 15 general-purpose registers the CPU didn't already save
/// (every one except `rsp`, which `iretq` restores from the hardware
/// frame), in a fixed order, then reads the caller's original `rax`
/// (offset `+112` from the post-push `rsp`: 14 registers were pushed
/// after it) as the call number and `rdi`/`rsi`/`rdx` (`+72`/`+80`/`+88`)
/// as the first three arguments, calls `dispatch`, writes its `u64`
/// return value back into the saved `rax` slot, and pops everything --
/// so the only register the caller sees changed across the trap is
/// `rax`, exactly like a real syscall's result.
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn entry() -> ! {
    core::arch::naked_asm!(
        "push rax",
        "push rbx",
        "push rcx",
        "push rdx",
        "push rsi",
        "push rdi",
        "push rbp",
        "push r8",
        "push r9",
        "push r10",
        "push r11",
        "push r12",
        "push r13",
        "push r14",
        "push r15",
        "mov rdi, [rsp + 112]", // call_num = caller's original rax
        "mov rsi, [rsp + 72]",  // arg1     = caller's original rdi
        "mov rdx, [rsp + 80]",  // arg2     = caller's original rsi
        "mov rcx, [rsp + 88]",  // arg3     = caller's original rdx
        "call {dispatch}",
        "mov [rsp + 112], rax", // overwrite the saved rax slot with the result
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
        dispatch = sym dispatch,
    )
}

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
//! Convention: call number in `rax`, up to two arguments in `rdi`, `rsi`
//! (the first two System V integer-argument registers), return value in
//! `rax`. `entry` is the actual `SYSCALL_VECTOR` IDT handler (installed
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

use crate::{proc, serial_println};

pub const SYS_GET_UPTIME: u64 = 1;
pub const SYS_WRITE_LINE: u64 = 2;
pub const SYS_BLOCK_FOREVER: u64 = 3;

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
/// `rdi` (`arg1`), and `rsi` (`arg2`) -- `entry`'s doc comment has the
/// full register-to-argument mapping.
extern "C" fn dispatch(call_num: u64, arg1: u64, arg2: u64) -> u64 {
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
            crate::ipc::receive(crate::com::ANY);
            unreachable!("nothing sends to a process that called SYS_BLOCK_FOREVER");
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
/// after it) as the call number and `rdi`/`rsi` (`+72`/`+80`) as the
/// first two arguments, calls `dispatch`, writes its `u64` return value
/// back into the saved `rax` slot, and pops everything -- so the only
/// register the caller sees changed across the trap is `rax`, exactly
/// like a real syscall's result.
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

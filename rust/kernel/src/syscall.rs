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
//! Convention: call number in `rax`, up to four arguments in `rdi`,
//! `rsi`, `rdx`, `rcx` (the first four System V integer-argument
//! registers), return value in `rax`. `entry` is the actual
//! `SYSCALL_VECTOR` IDT handler (installed via `Entry::set_handler_addr`
//! in `crate::interrupts`, not `set_handler_fn` -- this needs full
//! control over the trap frame that the `x86-interrupt` calling
//! convention doesn't expose, namely the caller's original register
//! values, not just the hardware-pushed `InterruptStackFrame`). It saves
//! every general-purpose register the CPU didn't already save, calls
//! `dispatch` with the caller's original `rax`/`rdi`/`rsi`/`rdx`/`rcx`,
//! writes `dispatch`'s return value into the saved `rax` slot, restores
//! everything else unchanged, and `iretq`s -- so a caller sees only its
//! requested register (`rax`) change, exactly like a real syscall.
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
//!
//! `SYS_READ_LINE` closes the loop the other direction: a real keypress
//! (`crate::keyboard`) reaching a ring-3 task, not just `fs`. It hands
//! `crate::keyboard::read_line` a pointer to a local kernel-stack buffer
//! (the same reasoning as `SYS_FS_READ`'s buffer -- see `crate::keyboard`'s
//! `deliver_line` for the other end of that), blocks *inside this trap*
//! until a full line has actually been typed (there's no bound on how
//! long that takes), and only then copies the result into the caller's
//! own buffer, since by then this task's own `CR3` is active again.
//!
//! `SYS_VIRCOPY` is the first call here to need a fourth argument, so the
//! convention grows to match: call number in `rax`, up to *four*
//! arguments in `rdi`/`rsi`/`rdx`/`rcx` (`entry`'s doc comment has the
//! updated register map). It exposes `crate::calls::sys_vircopy` --
//! already proven kernel-side (`crate::main`'s `vircopy_demo`) -- directly
//! to a ring-3 caller: read `arg3` (`len`) bytes from process `arg0`
//! (`src_proc`) at address `arg1` (`src_addr`) into *this task's own*
//! buffer at `arg2` (`local_ptr`). The destination is always the caller
//! itself (not an arbitrary fourth process) since that's the only
//! direction a ring-3 task can actually make use of the result; unlike
//! `SYS_WRITE_LINE`/`SYS_FS_WRITE`'s pointers, `local_ptr` is *not* read
//! or written directly by `dispatch` -- it's handed to
//! `memory::copy_between_address_spaces` (via `calls::sys_vircopy`),
//! which reaches it by walking `arg0`/the caller's page tables through the
//! physical-memory offset window, the same way it already does for a
//! kernel-task caller like `CLOCK`, regardless of which `CR3` happens to
//! be loaded right now. No privilege check gates `src_proc`: any ring-3
//! task can read any other process's memory this way, unlike real
//! MINIX's IPC-bitmask-gated kernel calls (see "known simplifications in
//! the kernel calls").

use crate::{calls, com, elf, fs, ipc, keyboard, proc, serial_println};
use x86_64::VirtAddr;

pub const SYS_GET_UPTIME: u64 = 1;
pub const SYS_WRITE_LINE: u64 = 2;
pub const SYS_BLOCK_FOREVER: u64 = 3;
pub const SYS_SET_ALARM: u64 = 4;
pub const SYS_WAIT_ALARM: u64 = 5;
pub const SYS_FS_OPEN: u64 = 6;
pub const SYS_FS_WRITE: u64 = 7;
pub const SYS_FS_READ: u64 = 8;
pub const SYS_READ_LINE: u64 = 9;
pub const SYS_VIRCOPY: u64 = 10;
pub const SYS_FORK: u64 = 11;
pub const SYS_EXEC: u64 = 12;

/// Longest `SYS_VIRCOPY` copy this port will perform in one call, purely
/// a sanity bound on an untrusted `len` from ring 3 -- matches the size
/// of `user/hello.s`'s own `vircopy_buf` destination with room to spare.
const MAX_VIRCOPY_LEN: usize = 256;

/// Longest path/buffer `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ` will
/// copy through a local kernel-stack buffer in either direction.
const MAX_FS_BUF: usize = 256;

/// Longest string `SYS_WRITE_LINE` will read, purely as a sanity bound on
/// `arg2` (an untrusted length from ring 3) -- not a real buffer, since
/// the caller's bytes are read directly out of its own, still-active
/// address space (see the module doc comment).
const MAX_LINE_LEN: u64 = 256;

/// Syscall-level failure codes: small negative `i64` values reinterpreted
/// as `u64` (two's complement, so still distinguishable from any real
/// success value -- every real `SYS_*` return here fits in far fewer
/// bits), mirroring the negative-`errno`-style convention
/// `crate::calls`/`crate::fs` already use for their own failures, rather
/// than a single undifferentiated sentinel. Still coarser than a real
/// `errno` set (one code per *kind* of mistake this dispatch layer itself
/// can detect, not one per underlying cause -- `SYS_FS_*` do at least
/// forward `fs`'s own real error codes through unchanged on top of these,
/// since those are a separate, already-real-`errno`-shaped failure mode).
pub const ERR_BAD_LENGTH: u64 = (-1i64) as u64;
pub const ERR_BAD_UTF8: u64 = (-2i64) as u64;
pub const ERR_VIRCOPY_FAILED: u64 = (-3i64) as u64;
pub const ERR_UNKNOWN_CALL: u64 = (-4i64) as u64;
/// `SYS_FORK` from a caller with no address space of its own -- a kernel
/// task, whose memory *is* the kernel's (`crate::proc::mem_map_of`
/// returns an empty map). There is nothing to copy and nothing the child
/// could safely run, so this is refused rather than given a meaning.
/// It used to mean something much narrower and more embarrassing: "a
/// caller this dispatch layer doesn't have a hardcoded private-pages
/// list for", which was everyone except one known ring-3 task.
pub const ERR_FORK_UNSUPPORTED_CALLER: u64 = (-5i64) as u64;
/// `SYS_EXEC` found the file but it isn't a program this loader can run
/// (`crate::elf::ElfError` -- bad magic, a segment outside user space,
/// truncated headers, ...). Distinct from the `fs` error codes `SYS_EXEC`
/// forwards unchanged when the *path* is the problem (`ENOENT` and
/// friends), which is the far more common case and the one
/// `user/shell.s`'s retry loop is waiting on.
pub const ERR_BAD_ELF: u64 = (-6i64) as u64;
/// `SYS_EXEC` from a caller that wasn't in ring 3. Nothing in this port
/// does that (kernel tasks call `crate::calls` directly rather than
/// trapping), but the consequence if one ever did is bad enough to check
/// for: `exec` would swap a *kernel* task's address space out from under
/// it and `iretq` it into a ring-3 entry point with a kernel `CS`.
pub const ERR_EXEC_NOT_RING3: u64 = (-7i64) as u64;
/// `SYS_FORK` with every dynamically allocatable process number already
/// taken (`crate::proc::alloc_proc_nr`, `com::NR_DYNAMIC_PROCS`). Real
/// MINIX reports the same condition -- `mproc[]` full -- as POSIX's
/// `EAGAIN`.
pub const ERR_NO_FREE_PROC: u64 = (-8i64) as u64;
/// `SYS_FORK` couldn't build the child's address space: out of physical
/// frames, or the parent's memory map named a page its page tables don't
/// actually have (`crate::memory::fork_address_space`). Unlike
/// `ERR_FORK_UNSUPPORTED_CALLER` this is a real resource failure, the
/// one POSIX calls `ENOMEM`; the caller is left exactly as it was and no
/// process is created either way.
pub const ERR_FORK_FAILED: u64 = (-9i64) as u64;

/// The actual dispatch, called by `entry` (via `core::arch::naked_asm!`'s
/// `sym` operand) with the caller's original `rax` (as `call_num`),
/// `rdi` (`arg1`), `rsi` (`arg2`), `rdx` (`arg3`), and `rcx` (`arg4`), plus
/// `frame_ptr`: not a caller-supplied argument at all, but the address of
/// the 15 registers `entry` itself just pushed (`lea r9, [rsp]`, taken
/// *before* the four `mov`s above start reusing those same registers for
/// the call) -- `SYS_FORK` is the one call needing the caller's *entire*
/// trap frame, not just its first four arguments (see `proc::TrapFrame`
/// and the `SYS_FORK` match arm below). `entry`'s doc comment has the full
/// register-to-argument mapping.
extern "C" fn dispatch(call_num: u64, arg1: u64, arg2: u64, arg3: u64, arg4: u64, frame_ptr: u64) -> u64 {
    let caller = proc::current_proc_nr();
    match call_num {
        SYS_GET_UPTIME => {
            let ticks = proc::uptime_ticks();
            serial_println!("[syscall] proc {}: SYS_GET_UPTIME -> {}", caller, ticks);
            ticks
        }
        SYS_WRITE_LINE => {
            if arg2 > MAX_LINE_LEN {
                return ERR_BAD_LENGTH;
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
                Err(_) => ERR_BAD_UTF8,
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
                return ERR_BAD_LENGTH;
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
                Err(_) => ERR_BAD_UTF8,
            }
        }
        SYS_FS_WRITE => {
            let fd = arg1 as i64;
            let len = arg3 as usize;
            if len > MAX_FS_BUF {
                return ERR_BAD_LENGTH;
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
        SYS_READ_LINE => {
            let max_len = core::cmp::min(arg2 as usize, MAX_FS_BUF);
            let mut buf = [0u8; MAX_FS_BUF];
            serial_println!("[syscall] proc {}: SYS_READ_LINE, blocking for a real keypress", caller);
            // Genuinely blocks -- possibly for a long time, however long
            // a human (or a QMP send-key script) takes to type a line --
            // inside this very trap, exactly like SYS_WAIT_ALARM. `buf`
            // lives on this call's own stack frame, part of the caller's
            // own per-task kernel stack, so it stays valid (and
            // dereferenceable from crate::keyboard's task, regardless of
            // which CR3 is active) for as long as this call is blocked.
            let n = keyboard::read_line(buf.as_mut_ptr() as u64, max_len);
            if n > 0 {
                let dst = unsafe { core::slice::from_raw_parts_mut(arg1 as *mut u8, n as usize) };
                dst.copy_from_slice(&buf[..n as usize]);
            }
            serial_println!("[syscall] proc {}: SYS_READ_LINE -> {} bytes", caller, n);
            n as u64
        }
        SYS_VIRCOPY => {
            let src_proc = arg1 as i32;
            let len = arg4 as usize;
            if len > MAX_VIRCOPY_LEN {
                return ERR_BAD_LENGTH;
            }
            // Safety: `local_ptr` (arg3) is never dereferenced here --
            // it's only handed to `calls::sys_vircopy`, which reaches it
            // through `src_proc`/the caller's own page tables via the
            // physical-memory offset window (see the module doc
            // comment), not a direct pointer read/write in this task's
            // (possibly different) currently-active address space.
            match calls::sys_vircopy(src_proc, VirtAddr::new(arg2), caller, VirtAddr::new(arg3), len) {
                Ok(()) => {
                    serial_println!(
                        "[syscall] proc {}: SYS_VIRCOPY({} bytes from proc {}) -> ok",
                        caller,
                        len,
                        src_proc
                    );
                    0
                }
                Err(_) => ERR_VIRCOPY_FAILED,
            }
        }
        SYS_FORK => {
            // Which pages the child needs its own copy of comes from the
            // caller's own memory map now (`crate::memory::MemMap`,
            // filled in by whoever built its address space), not from a
            // list this dispatch layer keeps per known caller. A caller
            // with no map has no address space of its own -- a kernel
            // task -- and there is nothing to fork.
            if proc::mem_map_of(caller).is_empty() {
                return ERR_FORK_UNSUPPORTED_CALLER;
            }
            // Claimed before anything is built, and given back below if
            // building fails: see `proc::alloc_proc_nr` for why the claim
            // can't wait until the child is ready.
            let child = match proc::alloc_proc_nr() {
                Some(child) => child,
                None => return ERR_NO_FREE_PROC,
            };
            // A forked child inherits its parent's scheduling parameters
            // rather than being handed fresh ones here, the same way a
            // real `fork()` does.
            let (priority, quantum, preemptible) = proc::sched_params_of(caller);
            // Safety: `frame_ptr` points at the 15 general-purpose
            // registers `entry` pushed for *this* trap, immediately
            // followed by the untouched hardware iretq frame -- see the
            // module doc comment and `proc::TrapFrame`'s. Still live and
            // valid: `entry` hasn't popped anything yet at this point.
            let frame = unsafe { &*(frame_ptr as *const proc::TrapFrame) };
            let mut child_frame = *frame;
            child_frame.rax = 0; // fork()'s own convention: the child sees 0
            match calls::sys_fork_from_frame(
                caller,
                child,
                com::dynamic_proc_name(child),
                priority,
                quantum,
                preemptible,
                &child_frame,
            ) {
                Some(child) => {
                    serial_println!(
                        "[syscall] proc {}: SYS_FORK -> child proc_nr {}",
                        caller,
                        child
                    );
                    child as u64
                }
                None => {
                    serial_println!(
                        "[syscall] proc {}: SYS_FORK -> failed to build the child's address space",
                        caller
                    );
                    proc::release_proc_nr(child);
                    ERR_FORK_FAILED
                }
            }
        }
        SYS_EXEC => {
            if arg2 as usize > MAX_FS_BUF {
                return ERR_BAD_LENGTH;
            }
            // Safety: same reasoning as SYS_FS_OPEN -- `CR3` is still the
            // caller's own here. Note this read has to happen *before*
            // `calls::sys_exec` switches address spaces, after which the
            // caller's pointer refers to nothing at all; copying into a
            // kernel-stack buffer (mapped identically in every address
            // space) is what makes the path outlive its own image.
            let src = unsafe { core::slice::from_raw_parts(arg1 as *const u8, arg2 as usize) };
            let mut path_buf = [0u8; MAX_FS_BUF];
            path_buf[..src.len()].copy_from_slice(src);
            let path = match core::str::from_utf8(&path_buf[..src.len()]) {
                Ok(path) => path,
                Err(_) => return ERR_BAD_UTF8,
            };

            // Safety: `frame_ptr` points at this trap's own saved
            // registers -- see the `SYS_FORK` arm above and
            // `proc::TrapFrame`. A plain read, not a borrow: the `&mut`
            // is deliberately left until after everything below that can
            // block, so no reference into a live trap frame is held
            // across a task switch.
            let caller_cs = unsafe { (*(frame_ptr as *const proc::TrapFrame)).cs };
            if caller_cs & 3 != 3 {
                serial_println!("[syscall] proc {}: SYS_EXEC from ring {}, refusing", caller, caller_cs & 3);
                return ERR_EXEC_NOT_RING3;
            }

            // Reading the program out of `fs` blocks on real IPC round
            // trips, so this task may be switched away from and back
            // several times before the image is complete -- all of it
            // still running on the *old* address space, which is exactly
            // what we want: nothing has been replaced yet if the file
            // turns out not to exist.
            let image = match elf::read_file(path) {
                Ok(image) => image,
                Err(err) => {
                    serial_println!("[syscall] proc {}: SYS_EXEC({:?}) -> fs error {}", caller, path, err);
                    return err as u64;
                }
            };
            let loaded = match calls::sys_exec(caller, &image) {
                Ok(loaded) => loaded,
                Err(err) => {
                    serial_println!("[syscall] proc {}: SYS_EXEC({:?}) -> bad image: {:?}", caller, path, err);
                    return ERR_BAD_ELF;
                }
            };

            // The point of no return, and the reason this call has no
            // meaningful success value: overwrite this trap's own saved
            // registers so `entry`'s `iretq` resumes the *new* image at
            // its entry point instead of returning to the instruction
            // after the caller's `int 0x80`. `entry` still writes this
            // function's return value into the saved `rax` slot
            // afterward, which is why `0` is the only sensible thing to
            // return: it's what the new program will find in `rax` on its
            // first instruction, and `exec_into` zeroes every register
            // anyway.
            //
            // Safety: as above, and nothing below this point blocks, so
            // this borrow lives and dies inside one uninterrupted stretch
            // of this task's own execution.
            let frame = unsafe { &mut *(frame_ptr as *mut proc::TrapFrame) };
            *frame = frame.exec_into(loaded.entry, loaded.stack_top);
            serial_println!(
                "[syscall] proc {}: SYS_EXEC({:?}) -> replaced its own image, entering at {:#x} on a fresh stack",
                caller,
                path,
                loaded.entry
            );
            0
        }
        _ => {
            serial_println!("[syscall] proc {}: unknown call number {}", caller, call_num);
            ERR_UNKNOWN_CALL
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
/// after it) as the call number and `rdi`/`rsi`/`rdx`/`rcx`
/// (`+72`/`+80`/`+88`/`+96`) as the first four arguments, calls
/// `dispatch`, writes its `u64` return value back into the saved `rax`
/// slot, and pops everything -- so the only register the caller sees
/// changed across the trap is `rax`, exactly like a real syscall's
/// result. The call to `dispatch` itself reuses these same four
/// registers in the System V order its own `extern "C"` parameters
/// expect -- unrelated to, and overwriting, whatever the caller's
/// original `rdi`/`rsi`/`rdx`/`rcx` were, which are already safely saved
/// on the stack by this point and restored by the `pop`s below. A fifth
/// register, `r9`, carries `frame_ptr`: the address of the 15 pushed
/// registers themselves (just `rsp` at that point, captured *before* the
/// four `mov`s start clobbering registers for the call) -- `SYS_FORK`'s
/// handler is the one caller that needs the whole trap frame, not just
/// `dispatch`'s four named arguments (see `dispatch`'s own doc comment).
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
        "lea r9,  [rsp]",       // frame_ptr = base of the 15 pushed GPRs just above (proc::TrapFrame)
        "mov rdi, [rsp + 112]", // call_num = caller's original rax
        "mov rsi, [rsp + 72]",  // arg1     = caller's original rdi
        "mov rdx, [rsp + 80]",  // arg2     = caller's original rsi
        "mov r8,  [rsp + 96]",  // arg4     = caller's original rcx (read before rcx below is clobbered)
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

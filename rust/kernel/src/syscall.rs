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

use crate::{calls, com, elf, fs, ipc, keyboard, memory, proc, serial_println};
use alloc::vec::Vec;
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
pub const SYS_EXIT: u64 = 13;
pub const SYS_WAIT: u64 = 14;

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
/// `SYS_WAIT` from a process with no children to wait for -- POSIX's
/// `ECHILD`, and the one answer `wait` can give immediately that isn't a
/// terminated child.
pub const ERR_NO_CHILDREN: u64 = (-10i64) as u64;
/// `SYS_EXIT`/`SYS_WAIT` from a caller that wasn't in ring 3, checked
/// for the same reason `ERR_EXEC_NOT_RING3` is: `exit` would tear down a
/// *kernel* task's slot from under it, mid-trap, and the kernel tasks
/// here are the ones the system is made of. Kernel-side code that really
/// wants these calls the `crate::proc` functions directly.
pub const ERR_NOT_RING3: u64 = (-11i64) as u64;
/// `SYS_EXEC`'s `argv`/`envp` named memory that isn't the caller's own:
/// an unmapped address, or one in a PML4 slot the kernel's address space
/// uses -- the kernel heap, image or physical-memory window, which every
/// address space can *reach* but no ring-3 program owns. Refusing the
/// latter is not pedantry: the strings end up on the new image's stack,
/// so accepting a kernel pointer would be a way to read kernel memory
/// back out of ring 3. POSIX's `EFAULT`.
pub const ERR_BAD_ARG_PTR: u64 = (-12i64) as u64;
/// `SYS_EXEC`'s `argv`/`envp` were larger than this port will copy: more
/// than `MAX_EXEC_VECTOR` entries in either vector, or more than
/// `elf::MAX_START_ARGS_BYTES` once laid out. POSIX's `E2BIG`.
pub const ERR_ARGS_TOO_BIG: u64 = (-13i64) as u64;

/// Most entries `SYS_EXEC` reads out of either `argv` or `envp` before
/// giving up with `ERR_ARGS_TOO_BIG`. Only a bound on how long the
/// copy-in loop runs on an untrusted, possibly unterminated vector; the
/// loader's own byte limit (`elf::MAX_START_ARGS_BYTES`) is the real one,
/// and is always the tighter of the two for vectors of non-empty strings.
const MAX_EXEC_VECTOR: usize = 64;

/// Copy `buf.len()` bytes out of `caller`'s memory at `addr`, refusing
/// anything that isn't memory the caller privately owns.
///
/// Deliberately *not* a direct dereference, unlike the other pointer
/// arguments in this file: those are bounded, single-range reads a
/// malformed pointer at worst faults on, but `SYS_EXEC` follows a chain
/// of pointers the caller controls (the vector, then each string), and
/// the bytes it gathers are handed straight back to ring 3 on the new
/// stack. So it walks the caller's page tables instead
/// (`memory::copy_between_address_spaces`, which reports an unmapped page
/// rather than faulting on it), and first requires the whole range to
/// sit in PML4 slots the kernel's own address space leaves empty -- the
/// same "private to this address space" test the ELF loader uses
/// (`memory::pml4_slots_unused`), and in this port the only meaningful
/// definition of a user address.
fn copy_from_caller(caller: i32, addr: u64, buf: &mut [u8]) -> Result<(), u64> {
    if buf.is_empty() {
        return Ok(());
    }
    let end = addr.checked_add(buf.len() as u64 - 1).ok_or(ERR_BAD_ARG_PTR)?;
    let (Ok(start_va), Ok(end_va)) = (VirtAddr::try_new(addr), VirtAddr::try_new(end)) else {
        return Err(ERR_BAD_ARG_PTR);
    };
    let kernel = proc::kernel_cr3();
    if !memory::pml4_slots_unused(kernel, start_va, end_va) {
        return Err(ERR_BAD_ARG_PTR);
    }
    let (caller_cr3, _) = proc::cr3_of(caller);
    // `buf` is kernel memory (this task's kernel stack or the heap), so
    // it is reachable through the kernel's own tables.
    memory::copy_between_address_spaces(
        caller_cr3,
        start_va,
        kernel,
        VirtAddr::new(buf.as_mut_ptr() as u64),
        buf.len(),
    )
    .map_err(|_| ERR_BAD_ARG_PTR)
}

/// Read one NULL-terminated vector of NUL-terminated strings (`argv` or
/// `envp`, C's `char *const []`) out of `caller`'s memory at `vec`, into
/// kernel-owned buffers. `vec == 0` is an empty vector, which is what
/// every `SYS_EXEC` caller that has nothing to pass says. `budget` is the
/// bytes left for strings across both vectors, spent as they're read so
/// an enormous string is refused while being read rather than after.
///
/// Strings are read a page-bounded chunk at a time: a string that ends
/// just short of an unmapped page is legitimate, and reading a fixed
/// chunk past its NUL would wrongly reject it.
fn copy_in_vector(caller: i32, vec: u64, budget: &mut usize) -> Result<Vec<Vec<u8>>, u64> {
    let mut out = Vec::new();
    if vec == 0 {
        return Ok(out);
    }
    loop {
        if out.len() == MAX_EXEC_VECTOR {
            return Err(ERR_ARGS_TOO_BIG);
        }
        let slot = vec.checked_add(out.len() as u64 * 8).ok_or(ERR_BAD_ARG_PTR)?;
        let mut word = [0u8; 8];
        copy_from_caller(caller, slot, &mut word)?;
        let mut ptr = u64::from_le_bytes(word);
        if ptr == 0 {
            return Ok(out);
        }

        let mut s = Vec::new();
        loop {
            const CHUNK: u64 = 64;
            let to_page_end = 4096 - (ptr % 4096);
            let mut chunk = [0u8; CHUNK as usize];
            let n = CHUNK.min(to_page_end) as usize;
            copy_from_caller(caller, ptr, &mut chunk[..n])?;
            let (bytes, done) = match chunk[..n].iter().position(|&b| b == 0) {
                Some(nul) => (&chunk[..nul], true),
                None => (&chunk[..n], false),
            };
            // `+ 1` for the NUL the loader will put back.
            let cost = bytes.len() + if done { 1 } else { 0 };
            if cost > *budget {
                return Err(ERR_ARGS_TOO_BIG);
            }
            *budget -= cost;
            s.extend_from_slice(bytes);
            if done {
                break;
            }
            ptr += n as u64;
        }
        out.push(s);
    }
}

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
        SYS_EXIT => {
            // Same reasoning as `SYS_EXEC`'s ring check below, and more
            // load-bearing: this one doesn't come back at all, so a
            // kernel task reaching it would lose its slot mid-trap.
            // Safety: see `SYS_FORK`'s use of `frame_ptr` above.
            let caller_cs = unsafe { (*(frame_ptr as *const proc::TrapFrame)).cs };
            if caller_cs & 3 != 3 {
                serial_println!("[syscall] proc {}: SYS_EXIT from ring {}, refusing", caller, caller_cs & 3);
                return ERR_NOT_RING3;
            }
            serial_println!("[syscall] proc {}: SYS_EXIT({})", caller, arg1 as i32);
            // Never returns -- this trap has no `iretq` to reach, since
            // the process it would return to no longer exists.
            proc::exit_now(arg1 as i32)
        }
        SYS_WAIT => {
            // Safety: see `SYS_FORK`'s use of `frame_ptr` above.
            let caller_cs = unsafe { (*(frame_ptr as *const proc::TrapFrame)).cs };
            if caller_cs & 3 != 3 {
                serial_println!("[syscall] proc {}: SYS_WAIT from ring {}, refusing", caller, caller_cs & 3);
                return ERR_NOT_RING3;
            }
            // Blocks for as long as it takes a child to terminate, which
            // is unbounded -- the same "block inside the trap and resume
            // in ring 3 afterwards" shape `SYS_WAIT_ALARM` and
            // `SYS_READ_LINE` already have.
            let Some((child, status)) = proc::wait_for_child() else {
                serial_println!("[syscall] proc {}: SYS_WAIT -> no children", caller);
                return ERR_NO_CHILDREN;
            };
            serial_println!(
                "[syscall] proc {}: SYS_WAIT -> child proc_nr {} terminated with status {}",
                caller,
                child,
                status
            );
            // `arg1` is where the caller wants the status written, or 0
            // for "don't bother" (POSIX lets `wait(NULL)` do that).
            // Safety: the caller's own address space is active again by
            // now -- this task is running, so `CR3` is its own -- so this
            // is the same direct write `SYS_WRITE_LINE`'s read relies on.
            // Unlike `SYS_READ_LINE`, no other task ever touches this
            // pointer: `wait_for_child` returns the status through the
            // process table, not through a buffer.
            if arg1 != 0 {
                unsafe { (arg1 as *mut i32).write_unaligned(status) };
            }
            child as u64
        }
        SYS_EXEC => {
            if arg2 as usize > MAX_FS_BUF {
                return ERR_BAD_LENGTH;
            }

            // `argv` (`arg3`, `rdx`) and `envp` (`arg4`, `rcx`) first,
            // while the caller's memory is still the caller's: once
            // `calls::sys_exec` swaps address spaces there is nothing left
            // to copy them from. Read into the kernel heap, which every
            // address space maps identically, so they outlive the image
            // they came from the same way `path_buf` below does. A bad
            // vector fails the whole call *before* the file is even
            // looked up -- an exec that fails leaves the caller as it
            // was, whichever part of it was wrong.
            let mut budget = elf::MAX_START_ARGS_BYTES;
            let argv = match copy_in_vector(caller, arg3, &mut budget) {
                Ok(v) => v,
                Err(err) => {
                    serial_println!("[syscall] proc {}: SYS_EXEC: bad argv ({:#x})", caller, err);
                    return err;
                }
            };
            let envp = match copy_in_vector(caller, arg4, &mut budget) {
                Ok(v) => v,
                Err(err) => {
                    serial_println!("[syscall] proc {}: SYS_EXEC: bad envp ({:#x})", caller, err);
                    return err;
                }
            };
            let argv_refs: Vec<&[u8]> = argv.iter().map(|s| s.as_slice()).collect();
            let envp_refs: Vec<&[u8]> = envp.iter().map(|s| s.as_slice()).collect();
            let args = elf::StartArgs { argv: &argv_refs, envp: &envp_refs };

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
            let loaded = match calls::sys_exec(caller, &image, &args) {
                Ok(loaded) => loaded,
                Err(elf::ElfError::ArgsTooLarge) => {
                    serial_println!("[syscall] proc {}: SYS_EXEC({:?}) -> argv/envp too large", caller, path);
                    return ERR_ARGS_TOO_BIG;
                }
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
            *frame = frame.exec_into(loaded.entry, loaded.stack_pointer);
            serial_println!(
                "[syscall] proc {}: SYS_EXEC({:?}, argc {}, envc {}) -> replaced its own image, entering at {:#x} on a fresh stack",
                caller,
                path,
                argv.len(),
                envp.len(),
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

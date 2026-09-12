//! Message format and public IPC entry points.
//!
//! Rust port of `include/minix/ipc.h`. The message format lives here, same
//! as in the C tree; the actual rendezvous algorithm (`mini_send`/
//! `mini_receive`/`mini_notify`) lives in `crate::proc` instead of
//! `kernel/proc.c`'s split, because it needs direct access to the process
//! table and ready queues there. `send`/`receive`/`notify` below are thin
//! wrappers, standing in for the `SEND`/`RECEIVE`/`NOTIFY` kernel-call
//! trap handlers in `kernel/proc.c`'s `sys_call()`.
//!
//! Unlike the previous version of this module, these now genuinely block:
//! `send`/`receive` don't return until the rendezvous completes, context-
//! switching to another runnable task in the meantime via `crate::proc`.

use crate::proc;

/// Fixed-size message body, mirroring the `mess_1`..`mess_8` union in
/// `include/minix/ipc.h`. MINIX packs several differently-typed layouts
/// into one union to keep the message a fixed size across all call sites;
/// we use a flat field set instead, since Rust unions offer no safety
/// benefit here and the original's economy of memory doesn't matter with
/// today's hardware.
#[derive(Clone, Copy, Debug, Default)]
pub struct Message {
    /// Who sent the message (filled in by the kernel, like `m_source`).
    pub source: i32,
    /// What kind of message this is (a call number or notification type).
    pub m_type: i32,
    pub args: [i64; 4],
}

impl Message {
    pub const fn empty() -> Self {
        Message { source: 0, m_type: 0, args: [0; 4] }
    }
}

/// Analogous to the `SEND` kernel call: block until `dst` has received
/// this message.
pub fn send(dst: i32, msg: Message) {
    proc::mini_send(dst, &msg);
}

/// Analogous to the `RECEIVE` kernel call: block until a message addressed
/// to us (from `src`, or from anyone if `src == com::ANY`) arrives.
pub fn receive(src: i32) -> Message {
    proc::mini_receive(src)
}

/// Analogous to `NOTIFY`: a lightweight, non-blocking send used by the
/// kernel itself to signal events (alarms, interrupts) to a server. Never
/// blocks the caller, and is silently dropped if `dst` isn't already
/// blocked in `receive` (see `proc::mini_notify`'s doc comment).
pub fn notify(dst: i32, m_type: i32) {
    proc::mini_notify(dst, m_type);
}

/// Analogous to `SENDREC`: `send` a request to `dst`, then `receive` its
/// reply, matching the "call a server, block for its answer" pattern
/// every server client (`crate::fs`'s `open`/`read`/`write` client stubs,
/// for one) uses. Real MINIX implements this as one kernel call so a
/// send/receive pair can't be interleaved with an unrelated message from
/// someone else; this port just does the two blocking steps back to
/// back, which is equivalent as long as `dst` only ever replies to the
/// most recent request it received -- true for every current server.
pub fn send_receive(dst: i32, msg: Message) -> Message {
    send(dst, msg);
    receive(dst)
}

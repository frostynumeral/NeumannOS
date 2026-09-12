//! Message-passing IPC primitives.
//!
//! Rust port of `include/minix/ipc.h` and the send/receive primitives in
//! `kernel/proc.c`. In original MINIX, `SENDREC`/`SEND`/`RECEIVE`/`NOTIFY`
//! trap into the kernel and block the calling process until a matching
//! partner is ready (or forever, if none ever is). This port does not yet
//! have user-mode processes or a scheduler (see `rust/README.md`), so the
//! primitives here operate on an in-kernel mailbox per process slot: enough
//! to exercise the message format and dispatch logic that the future
//! servers (pm, fs, rs, ...) will be built on.

use crate::com::NR_BOOT_PROCS;
use lazy_static::lazy_static;
use spin::Mutex;

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

/// One process's mailbox: at most one pending message, matching the
/// "rendezvous" semantics of MINIX IPC (a sender blocks until the receiver
/// takes the message; there is no queuing).
struct Mailbox {
    pending: Option<Message>,
}

lazy_static! {
    static ref MAILBOXES: Mutex<[Mailbox; NR_BOOT_PROCS]> =
        Mutex::new(core::array::from_fn(|_| Mailbox { pending: None }));
}

/// Map a process number (as defined in `crate::com`) to a mailbox slot.
/// Kernel tasks use negative numbers (`IDLE`..`KERNEL`); boot-image servers
/// use small non-negative numbers. Both ranges are folded into one dense
/// array here.
fn slot(p_nr: i32) -> usize {
    (p_nr + crate::com::NR_TASKS as i32) as usize
}

#[derive(Debug)]
pub enum IpcError {
    BadProcNr,
    MailboxFull,
    NoMessage,
}

/// Analogous to the `SEND` kernel call: deposit a message for `dst`.
/// Fails with `MailboxFull` instead of blocking, since there is no
/// scheduler yet to park the caller on.
pub fn send(dst: i32, msg: Message) -> Result<(), IpcError> {
    let idx = slot(dst);
    let mut boxes = MAILBOXES.lock();
    let mailbox = boxes.get_mut(idx).ok_or(IpcError::BadProcNr)?;
    if mailbox.pending.is_some() {
        return Err(IpcError::MailboxFull);
    }
    mailbox.pending = Some(msg);
    Ok(())
}

/// Analogous to the `RECEIVE` kernel call: take a pending message addressed
/// to `who`. Returns `NoMessage` instead of blocking (see `send`).
pub fn receive(who: i32) -> Result<Message, IpcError> {
    let idx = slot(who);
    let mut boxes = MAILBOXES.lock();
    let mailbox = boxes.get_mut(idx).ok_or(IpcError::BadProcNr)?;
    mailbox.pending.take().ok_or(IpcError::NoMessage)
}

/// Analogous to `NOTIFY`: a lightweight, non-blocking send used by the
/// kernel itself to signal events (alarms, interrupts) to a server.
pub fn notify(dst: i32, m_type: i32) -> Result<(), IpcError> {
    send(dst, Message { source: crate::com::KERNEL, m_type, args: [0; 4] })
}

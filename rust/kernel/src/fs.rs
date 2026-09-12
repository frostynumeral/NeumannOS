//! `fs` (file server): an in-memory filesystem serving open/read/write
//! requests over IPC.
//!
//! No single C file to port here -- real `servers/fs` is a whole
//! subsystem (`servers/fs/open.c`, `read.c`, `write.c`, `path.c`, its own
//! inode/block-cache layer over a real block device) that this port has
//! no device driver or on-disk layout to back yet. This is the minimal
//! slice of its *external behavior*: a request/reply protocol over
//! `crate::ipc` (real `fs` is likewise driven entirely by messages from
//! `pm`/user processes, never called as a local function), backing files
//! that live in heap-allocated `Vec<u8>`s instead of on a block device.
//! `InMemoryFs` is the server-side state and dispatch (run by the `fs`
//! task's own loop in `main.rs`); the free functions below are the
//! client-side stubs a caller like `pm` uses to talk to it, mirroring the
//! shape of `crate::calls`' kernel-call wrappers even though these go
//! over a real IPC round trip rather than a direct call.
//!
//! Known simplification, shared with `crate::calls`: `fs` and its callers
//! all still run in the kernel's own address space (see `rust/README.md`),
//! so request/reply args carry raw pointers valid in that one shared
//! space directly, rather than needing a `sys_vircopy`-style
//! cross-address-space copy the way a real, isolated `fs` server would.

use crate::com;
use crate::ipc::{self, Message};
use alloc::string::String;
use alloc::vec::Vec;

pub const FS_OPEN: i32 = 300;
pub const FS_READ: i32 = 301;
pub const FS_WRITE: i32 = 302;

/// Reply `args[0]`: a negative POSIX-style error code on failure, mirroring
/// how real MINIX servers reply with `-EFOO` in the message body rather
/// than a separate status channel. Success replies are always `>= 0`
/// (a byte count, or a file descriptor).
pub const EBADF: i64 = -9;

struct OpenFile {
    file_index: usize,
    position: usize,
}

/// The server-side state: a flat table of named files and a table of open
/// file descriptors pointing into it. Deliberately not `Mutex`-protected
/// like `crate::memory`'s `GlobalFrameAllocator` -- unlike a kernel call
/// any task can invoke directly, this is only ever touched by the single
/// task running `InMemoryFs::serve`, the same way real `fs`'s in-memory
/// tables are only touched by the `fs` process itself.
pub struct InMemoryFs {
    files: Vec<(String, Vec<u8>)>,
    open: Vec<Option<OpenFile>>,
}

impl InMemoryFs {
    pub fn new() -> Self {
        InMemoryFs { files: Vec::new(), open: Vec::new() }
    }

    fn find_or_create(&mut self, name: &str) -> usize {
        if let Some(i) = self.files.iter().position(|(existing, _)| existing == name) {
            return i;
        }
        self.files.push((String::from(name), Vec::new()));
        self.files.len() - 1
    }

    fn alloc_fd(&mut self, open_file: OpenFile) -> usize {
        for (fd, slot) in self.open.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(open_file);
                return fd;
            }
        }
        self.open.push(Some(open_file));
        self.open.len() - 1
    }

    /// Handle one request, matching on `m_type` the way real MINIX's
    /// `fs`/`pm` main loops dispatch on a call number. Every reply is
    /// addressed back to `req.source`, standing in for a call table
    /// (`kernel/system.c`'s `map(SYS_xxx, do_xxx)`) this port doesn't
    /// have.
    fn handle(&mut self, req: &Message) -> Message {
        let source = com::FS_PROC_NR;
        match req.m_type {
            FS_OPEN => {
                let name_ptr = req.args[0] as *const u8;
                let name_len = req.args[1] as usize;
                // Safety: `fs` and its callers all share the kernel's own
                // address space today (see the module doc comment), so a
                // pointer valid for the caller is valid here too.
                let name = unsafe {
                    core::str::from_utf8(core::slice::from_raw_parts(name_ptr, name_len)).unwrap_or("")
                };
                let file_index = self.find_or_create(name);
                let fd = self.alloc_fd(OpenFile { file_index, position: 0 });
                Message { source, m_type: FS_OPEN, args: [fd as i64, 0, 0, 0] }
            }
            FS_WRITE => {
                let fd = req.args[0] as usize;
                let buf_ptr = req.args[1] as *const u8;
                let len = req.args[2] as usize;
                let result = if let Some(Some(open_file)) = self.open.get(fd) {
                    let file_index = open_file.file_index;
                    let position = open_file.position;
                    let data = unsafe { core::slice::from_raw_parts(buf_ptr, len) };
                    let file_data = &mut self.files[file_index].1;
                    if position + len > file_data.len() {
                        file_data.resize(position + len, 0);
                    }
                    file_data[position..position + len].copy_from_slice(data);
                    self.open[fd].as_mut().unwrap().position = position + len;
                    len as i64
                } else {
                    EBADF
                };
                Message { source, m_type: FS_WRITE, args: [result, 0, 0, 0] }
            }
            FS_READ => {
                let fd = req.args[0] as usize;
                let buf_ptr = req.args[1] as *mut u8;
                let len = req.args[2] as usize;
                let result = if let Some(Some(open_file)) = self.open.get(fd) {
                    let file_index = open_file.file_index;
                    let position = open_file.position;
                    let file_data = &self.files[file_index].1;
                    let available = file_data.len().saturating_sub(position);
                    let n = core::cmp::min(available, len);
                    let dst = unsafe { core::slice::from_raw_parts_mut(buf_ptr, n) };
                    dst.copy_from_slice(&file_data[position..position + n]);
                    self.open[fd].as_mut().unwrap().position = position + n;
                    n as i64
                } else {
                    EBADF
                };
                Message { source, m_type: FS_READ, args: [result, 0, 0, 0] }
            }
            other => Message { source, m_type: other, args: [EBADF, 0, 0, 0] },
        }
    }

    /// The `fs` task's main loop: block for the next request from anyone,
    /// handle it, reply -- exactly real MINIX's
    /// `while (TRUE) { get_work(); reply(...); }` shape in
    /// `servers/fs/main.c`, minus the parts of that loop (signal handling,
    /// suspending on a slow device) this port has no equivalent for yet.
    pub fn serve(&mut self) -> ! {
        loop {
            let req = ipc::receive(com::ANY);
            let requester = req.source;
            let reply = self.handle(&req);
            ipc::send(requester, reply);
        }
    }
}

/// Client-side stub: open (creating if necessary) the file named `name`,
/// returning a file descriptor. Mirrors `crate::calls`' kernel-call
/// wrappers in shape, even though this is a real IPC round trip rather
/// than a direct function call -- `fs` is a separate task, reached only
/// through `crate::ipc`.
pub fn open(name: &str) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message {
            source: com::PM_PROC_NR,
            m_type: FS_OPEN,
            args: [name.as_ptr() as i64, name.len() as i64, 0, 0],
        },
    );
    reply.args[0]
}

/// Client-side stub: write `data` to `fd` at its current position,
/// advancing it. Returns the number of bytes written (or a negative
/// error).
pub fn write(fd: i64, data: &[u8]) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message {
            source: com::PM_PROC_NR,
            m_type: FS_WRITE,
            args: [fd, data.as_ptr() as i64, data.len() as i64, 0],
        },
    );
    reply.args[0]
}

/// Client-side stub: read up to `buf.len()` bytes from `fd` at its
/// current position into `buf`, advancing it. Returns the number of
/// bytes read (`0` at end of file, or a negative error).
pub fn read(fd: i64, buf: &mut [u8]) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message {
            source: com::PM_PROC_NR,
            m_type: FS_READ,
            args: [fd, buf.as_mut_ptr() as i64, buf.len() as i64, 0],
        },
    );
    reply.args[0]
}

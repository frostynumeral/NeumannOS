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
//! over a real IPC round trip rather than a direct call. Every stub sets
//! the request's `source` to `proc::current_proc_nr()` -- the *real*
//! caller, not a hardcoded one -- since `fs` addresses its reply using
//! that field (`serve`, below): a caller who lied about it (or, before
//! this was fixed, a fixed `PM_PROC_NR` regardless of who actually
//! called) would have its reply delivered to the wrong mailbox.
//!
//! Known simplification, shared with `crate::calls`: request/reply args
//! carry raw pointers, valid only in whichever address space happens to
//! be active when they're actually dereferenced -- fine when `fs` and its
//! caller share the kernel's own address space (true of every *kernel*
//! caller, like `pm`), but not when the caller has its own, separate
//! address space (a ring-3 task -- see `crate::syscall`'s
//! `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ`, which route around this by
//! copying through a kernel-stack buffer *before* handing anything to
//! these stubs, rather than `fs` needing a `sys_vircopy`-style
//! cross-address-space copy of its own).

use crate::com;
use crate::ipc::{self, Message};
use crate::proc;
use alloc::string::String;
use alloc::vec::Vec;

pub const FS_OPEN: i32 = 300;
pub const FS_READ: i32 = 301;
pub const FS_WRITE: i32 = 302;
pub const FS_MKDIR: i32 = 303;
pub const FS_CLOSE: i32 = 304;
pub const FS_READDIR: i32 = 305;

/// What `readdir` says an entry is.
pub const KIND_FILE: i64 = 0;
pub const KIND_DIR: i64 = 1;

/// Reply `args[0]`: a negative POSIX-style error code on failure, mirroring
/// how real MINIX servers reply with `-EFOO` in the message body rather
/// than a separate status channel (matching the real `errno` numbers,
/// unlike `crate::syscall`'s single `u64::MAX` sentinel). Success replies
/// are always `>= 0` (a byte count, or a file descriptor).
pub const ENOENT: i64 = -2;
pub const EBADF: i64 = -9;
pub const EEXIST: i64 = -17;
pub const ENOTDIR: i64 = -20;
pub const EISDIR: i64 = -21;
pub const EINVAL: i64 = -22;

struct OpenFile {
    file_index: usize,
    position: usize,
    /// The process that opened it, and the only one that may read,
    /// write or close it. The table is global, so without this any
    /// process could close any other's descriptor -- and once `close`
    /// made slots reusable, the next `open` anywhere would inherit it,
    /// silently redirecting the first process's writes (`console_task`'s
    /// long-lived `/console.log`, say) into someone else's file.
    ///
    /// A process number *and* its generation (`proc::generation_of`):
    /// dynamic numbers are reused, and by number alone a reaped child's
    /// still-open files went to the next process given that number. The
    /// generation also retires a dead owner's descriptors -- `alloc_fd`
    /// reuses any slot whose owner is gone, which is this server's whole
    /// "close everything on exit".
    owner: (i32, u32),
}

/// The server-side state: a flat table of named files, a list of known
/// directory paths, and a table of open file descriptors pointing into
/// the file table. Deliberately not `Mutex`-protected like
/// `crate::memory`'s `GlobalFrameAllocator` -- unlike a kernel call any
/// task can invoke directly, this is only ever touched by the single
/// task running `InMemoryFs::serve`, the same way real `fs`'s in-memory
/// tables are only touched by the `fs` process itself.
///
/// Directories are just a flat `Vec<String>` of full paths that are
/// known to be directories (the root, `"/"`, always is, implicitly, and
/// is never itself stored) -- not a real tree (`servers/fs`'s inodes plus
/// directory-entry blocks). Good enough to support real, multi-level
/// paths and the usual POSIX "parent must exist and be a directory"
/// checks (`path_lookup`'s job in real `servers/fs/path.c`) without
/// needing an actual on-disk (or in-memory-block) directory format.
pub struct InMemoryFs {
    files: Vec<(String, Vec<u8>)>,
    directories: Vec<String>,
    open: Vec<Option<OpenFile>>,
}

impl InMemoryFs {
    pub fn new() -> Self {
        InMemoryFs { files: Vec::new(), directories: Vec::new(), open: Vec::new() }
    }

    /// Whether `path` is in the one form this server stores and compares:
    /// absolute, no empty components (`//`), no trailing `/` except the
    /// root itself. Paths are compared as whole strings, so without this
    /// `/x` and `//x` were two different entries, `mkdir /bin/` created a
    /// directory with an empty name, and `ls /bin/` found nothing.
    fn valid_path(path: &str) -> bool {
        path == "/" || (path.starts_with('/') && !path.ends_with('/') && !path.contains("//"))
    }

    fn is_dir(&self, path: &str) -> bool {
        path == "/" || self.directories.iter().any(|d| d == path)
    }

    fn is_file(&self, path: &str) -> bool {
        self.files.iter().any(|(existing, _)| existing == path)
    }

    /// The directory a path's last component lives in: everything before
    /// the final `/`, or `"/"` itself for a top-level path like
    /// `"/hello.txt"`. Assumes `path` starts with `/` (checked by every
    /// caller before reaching here).
    fn parent_dir(path: &str) -> &str {
        match path.rfind('/') {
            Some(0) => "/",
            Some(idx) => &path[..idx],
            None => "/",
        }
    }

    /// Create directory `path`, mirroring `mkdir()`'s usual rules: the
    /// parent must already exist and be a directory, and `path` itself
    /// must not already exist as either a file or a directory.
    fn mkdir(&mut self, path: &str) -> i64 {
        if !Self::valid_path(path) || path == "/" {
            return EINVAL;
        }
        let parent = Self::parent_dir(path);
        if self.is_file(parent) {
            return ENOTDIR;
        }
        if !self.is_dir(parent) {
            return ENOENT;
        }
        if self.is_dir(path) || self.is_file(path) {
            return EEXIST;
        }
        self.directories.push(String::from(path));
        0
    }

    /// Resolve `path` to a file-table index for `open`, creating it if
    /// it doesn't exist yet and `create` is set -- but only after the
    /// same parent-directory checks `mkdir` makes, so a caller can't
    /// `open("/missing/x")` or `open("/a_file/x")` and have it silently
    /// succeed.
    ///
    /// `create` exists because "open always creates" (this server's
    /// original, `O_CREAT`-is-implied behavior) is wrong for a caller
    /// that wants to *find out whether a file exists*: it turns every
    /// failed lookup into a successful open of a brand-new empty file,
    /// and leaves that file behind. `crate::elf::read_file` is the
    /// caller that cares -- `exec`'ing a path that doesn't exist has to
    /// report `ENOENT` and change nothing, not invent an empty program
    /// and then reject it for being malformed.
    fn open_path(&mut self, path: &str, create: bool) -> Result<usize, i64> {
        if !Self::valid_path(path) {
            return Err(EINVAL);
        }
        if self.is_dir(path) {
            return Err(EISDIR);
        }
        let parent = Self::parent_dir(path);
        if self.is_file(parent) {
            return Err(ENOTDIR);
        }
        if !self.is_dir(parent) {
            return Err(ENOENT);
        }
        if let Some(i) = self.files.iter().position(|(existing, _)| existing == path) {
            return Ok(i);
        }
        if !create {
            return Err(ENOENT);
        }
        self.files.push((String::from(path), Vec::new()));
        Ok(self.files.len() - 1)
    }

    /// The `index`-th entry of directory `path` -- subdirectories first,
    /// then files, each in creation order -- as `(name, kind, size)`,
    /// where `name` is the last path component. `Ok(None)` past the last
    /// entry. Index-based rather than a handle with a cursor (real
    /// `readdir` reads a directory *file* sequentially through an open
    /// descriptor): this server's directories aren't files, and a
    /// stateless request needs nothing cleaned up if the caller stops
    /// halfway. The cost is an O(entries) scan per call.
    fn readdir(&self, path: &str, index: usize) -> Result<Option<(&str, i64, usize)>, i64> {
        if !Self::valid_path(path) {
            return Err(EINVAL);
        }
        if self.is_file(path) {
            return Err(ENOTDIR);
        }
        if !self.is_dir(path) {
            return Err(ENOENT);
        }
        let last = |full: &'_ str| -> usize { full.rfind('/').map_or(0, |i| i + 1) };
        let dirs = self
            .directories
            .iter()
            .filter(|d| Self::parent_dir(d) == path)
            .map(|d| (&d[last(d)..], KIND_DIR, 0usize));
        let files = self
            .files
            .iter()
            .filter(|(name, _)| Self::parent_dir(name) == path)
            .map(|(name, data)| (&name[last(name)..], KIND_FILE, data.len()));
        Ok(dirs.chain(files).nth(index))
    }

    fn alloc_fd(&mut self, open_file: OpenFile) -> usize {
        for (fd, slot) in self.open.iter_mut().enumerate() {
            let dead_owner = matches!(slot, Some(f) if proc::generation_of(f.owner.0) != Some(f.owner.1));
            if slot.is_none() || dead_owner {
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
                // `args[2]` is the no-create flag (see `open_existing`);
                // absent (`0`) for every caller that predates it, which
                // keeps the original create-on-open behavior the default.
                let create = req.args[2] == 0;
                let result = match self.open_path(name, create) {
                    Ok(file_index) => {
                        let owner = (req.source, proc::generation_of(req.source).unwrap_or(0));
                        let fd = self.alloc_fd(OpenFile { file_index, position: 0, owner });
                        fd as i64
                    }
                    Err(err) => err,
                };
                Message { source, m_type: FS_OPEN, args: [result, 0, 0, 0] }
            }
            FS_MKDIR => {
                let name_ptr = req.args[0] as *const u8;
                let name_len = req.args[1] as usize;
                // Safety: see FS_OPEN above.
                let name = unsafe {
                    core::str::from_utf8(core::slice::from_raw_parts(name_ptr, name_len)).unwrap_or("")
                };
                let result = self.mkdir(name);
                Message { source, m_type: FS_MKDIR, args: [result, 0, 0, 0] }
            }
            FS_WRITE => {
                let fd = req.args[0] as usize;
                let buf_ptr = req.args[1] as *const u8;
                let len = req.args[2] as usize;
                let result = if let Some(Some(open_file)) = self.open.get(fd).filter(|f| owned_by(f, req.source)) {
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
                let result = if let Some(Some(open_file)) = self.open.get(fd).filter(|f| owned_by(f, req.source)) {
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
            FS_CLOSE => {
                // The slot goes back to `alloc_fd`'s free list; before this
                // existed, every open cost a slot forever.
                let fd = req.args[0] as usize;
                let result = match self.open.get_mut(fd) {
                    Some(slot @ Some(_)) if owned_by(slot, req.source) => {
                        *slot = None;
                        0
                    }
                    _ => EBADF,
                };
                Message { source, m_type: FS_CLOSE, args: [result, 0, 0, 0] }
            }
            FS_READDIR => {
                let name_ptr = req.args[0] as *const u8;
                let name_len = req.args[1] as usize;
                let index = req.args[2] as usize;
                let out = req.args[3] as *mut DirEntry;
                // Safety: see FS_OPEN above; `out` is likewise the
                // caller's own (kernel-mapped) buffer.
                let name = unsafe {
                    core::str::from_utf8(core::slice::from_raw_parts(name_ptr, name_len)).unwrap_or("")
                };
                let result = match self.readdir(name, index) {
                    Ok(Some((entry, kind, size))) => {
                        let n = entry.len().min(DIRENT_NAME_MAX);
                        let mut record = DirEntry { size: size as u64, kind: kind as u8, name_len: n as u8, name: [0; DIRENT_NAME_MAX] };
                        record.name[..n].copy_from_slice(&entry.as_bytes()[..n]);
                        unsafe { out.write_unaligned(record) };
                        1
                    }
                    Ok(None) => 0,
                    Err(err) => err,
                };
                Message { source, m_type: FS_READDIR, args: [result, 0, 0, 0] }
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

/// Client-side stub: open the file named `name`, creating it if it
/// doesn't exist (see `open_existing` for the variant that doesn't),
/// returning a file descriptor, or a negative error (`ENOENT` if some
/// parent directory doesn't exist, `ENOTDIR` if one exists but isn't a
/// directory, `EISDIR` if `name` itself is a directory). Mirrors
/// `crate::calls`' kernel-call wrappers in shape, even though this is a
/// real IPC round trip rather than a direct function call -- `fs` is a
/// separate task, reached only through `crate::ipc`.
pub fn open(name: &str) -> i64 {
    open_with(name, true)
}

/// Client-side stub: open `name` only if it already exists, returning
/// `ENOENT` rather than creating it. The distinction matters to anyone
/// asking "is this file here yet?" -- `crate::elf::read_file` (so that a
/// failed `exec` reports `ENOENT` and leaves nothing behind) and
/// `crate::main`'s `read_when_available` (so that waiting for a file to
/// appear can't be satisfied by conjuring it).
pub fn open_existing(name: &str) -> i64 {
    open_with(name, false)
}

fn open_with(name: &str, create: bool) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message {
            source: proc::current_proc_nr(),
            m_type: FS_OPEN,
            args: [name.as_ptr() as i64, name.len() as i64, i64::from(!create), 0],
        },
    );
    reply.args[0]
}

/// Whether `slot` holds a descriptor `source` opened. Anyone else's is
/// `EBADF`, exactly as if it weren't open.
fn owned_by(slot: &Option<OpenFile>, source: i32) -> bool {
    matches!(slot, Some(f) if f.owner.0 == source && proc::generation_of(source) == Some(f.owner.1))
}

/// Longest entry name `readdir` reports; longer ones are cut short.
pub const DIRENT_NAME_MAX: usize = 118;

/// One directory entry, as `readdir` fills it in: 128 bytes, `repr(C)`,
/// the same layout `crate::syscall`'s `SYS_FS_READDIR` hands to ring 3
/// (`neumann_rt::sys::DirEntry` is its twin).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DirEntry {
    pub size: u64,
    pub kind: u8,
    pub name_len: u8,
    pub name: [u8; DIRENT_NAME_MAX],
}

impl DirEntry {
    pub const EMPTY: DirEntry = DirEntry { size: 0, kind: 0, name_len: 0, name: [0; DIRENT_NAME_MAX] };

    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }
}

const _: () = assert!(core::mem::size_of::<DirEntry>() == 128);

/// Client-side stub: close `fd`, returning its slot to `fs`. `0`, or
/// `EBADF` if it wasn't open.
pub fn close(fd: i64) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message { source: proc::current_proc_nr(), m_type: FS_CLOSE, args: [fd, 0, 0, 0] },
    );
    reply.args[0]
}

/// Client-side stub: fill `out` with the `index`-th entry of directory
/// `path` (subdirectories first, then files). `1` if it did, `0` past the
/// last entry, or a negative error (`ENOENT`, `ENOTDIR`).
pub fn readdir(path: &str, index: usize, out: &mut DirEntry) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message {
            source: proc::current_proc_nr(),
            m_type: FS_READDIR,
            args: [path.as_ptr() as i64, path.len() as i64, index as i64, out as *mut DirEntry as i64],
        },
    );
    reply.args[0]
}

/// Client-side stub: create directory `path`. Returns `0` on success, or
/// a negative error (`ENOENT`/`ENOTDIR` for the same parent-directory
/// reasons as `open`, `EEXIST` if `path` already exists as either a file
/// or a directory).
pub fn mkdir(path: &str) -> i64 {
    let reply = ipc::send_receive(
        com::FS_PROC_NR,
        Message {
            source: proc::current_proc_nr(),
            m_type: FS_MKDIR,
            args: [path.as_ptr() as i64, path.len() as i64, 0, 0],
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
            source: proc::current_proc_nr(),
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
            source: proc::current_proc_nr(),
            m_type: FS_READ,
            args: [fd, buf.as_mut_ptr() as i64, buf.len() as i64, 0],
        },
    );
    reply.args[0]
}

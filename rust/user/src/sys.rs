//! System calls: one wrapper per `crate::syscall` call number in the
//! kernel, which is the authoritative list (and documents what each one
//! does on the other side). Errors come back as the kernel's own negative
//! codes, as `Err(code)`; `strerror` names the ones a program is likely
//! to want to report.

use core::arch::asm;

pub const SYS_GET_UPTIME: u64 = 1;
pub const SYS_WRITE_LINE: u64 = 2;
const SYS_BLOCK_FOREVER: u64 = 3;
const SYS_SET_ALARM: u64 = 4;
const SYS_WAIT_ALARM: u64 = 5;
pub const SYS_FS_OPEN: u64 = 6;
pub const SYS_FS_WRITE: u64 = 7;
pub const SYS_FS_READ: u64 = 8;
pub const SYS_READ_LINE: u64 = 9;
pub const SYS_VIRCOPY: u64 = 10;
const SYS_FORK: u64 = 11;
pub const SYS_EXEC: u64 = 12;
const SYS_EXIT: u64 = 13;
pub const SYS_WAIT: u64 = 14;
pub const SYS_FS_OPEN_EXISTING: u64 = 15;
pub const SYS_CONSOLE_WRITE: u64 = 16;
pub const SYS_FS_CLOSE: u64 = 17;
pub const SYS_FS_READDIR: u64 = 18;
pub const SYS_FS_MKDIR: u64 = 19;

/// Most bytes the kernel moves in one `SYS_FS_*`/`SYS_CONSOLE_WRITE`
/// call (its `MAX_FS_BUF`/`MAX_LINE_LEN`); the wrappers below loop over
/// anything larger.
pub const MAX_IO: usize = 256;

// Error codes, as the kernel returns them: `crate::fs`'s POSIX-numbered
// ones, and `crate::syscall`'s own `ERR_*`. The two ranges overlap (the
// kernel numbered its dispatch-level errors from -1 without regard to
// errno), so a code only means one thing in the context of the call that
// returned it -- `strerror` names the reading most calls mean.
pub const ENOENT: i64 = -2;
pub const ERR_BAD_ELF: i64 = -6;
/// `SYS_FORK` with every process slot taken (the kernel's
/// `ERR_NO_FREE_PROC`). Numerically POSIX's `ENOEXEC`, which the kernel
/// never returns to ring 3 (its loader reports bad images as
/// `ERR_BAD_ELF`).
pub const ERR_NO_FREE_PROC: i64 = -8;
pub const ERR_FORK_FAILED: i64 = -9;
pub const ERR_NO_CHILDREN: i64 = -10;
pub const ERR_BAD_ARG_PTR: i64 = -12;
pub const ERR_ARGS_TOO_BIG: i64 = -13;
pub const EEXIST: i64 = -17;
pub const ENOTDIR: i64 = -20;
pub const EISDIR: i64 = -21;
pub const EINVAL: i64 = -22;

/// Raw trap: call number `n`, four arguments, result in `rax`. The kernel
/// preserves every register but `rax`. Public for programs that need to
/// pass something the typed wrappers below can't express -- `ptrtest`,
/// which hands the kernel deliberately bad pointers.
///
/// Safety: whatever the call does with its arguments.
#[inline(always)]
pub unsafe fn syscall4(n: u64, a1: u64, a2: u64, a3: u64, a4: u64) -> i64 {
    let ret: u64;
    asm!(
        "int 0x80",
        inlateout("rax") n => ret,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        in("rcx") a4,
        options(nostack),
    );
    ret as i64
}

fn result(ret: i64) -> Result<i64, i64> {
    if ret < 0 {
        Err(ret)
    } else {
        Ok(ret)
    }
}

/// Timer ticks since boot (`pit::HZ` per second).
pub fn uptime() -> u64 {
    unsafe { syscall4(SYS_GET_UPTIME, 0, 0, 0, 0) as u64 }
}

/// A line in the kernel's log, prefixed with this process's number --
/// diagnostics, not output (for output, `console_write`).
pub fn log(line: &[u8]) {
    for chunk in line.chunks(MAX_IO) {
        unsafe { syscall4(SYS_WRITE_LINE, chunk.as_ptr() as u64, chunk.len() as u64, 0, 0) };
    }
}

/// Standard output.
pub fn console_write(bytes: &[u8]) {
    for chunk in bytes.chunks(MAX_IO) {
        unsafe { syscall4(SYS_CONSOLE_WRITE, chunk.as_ptr() as u64, chunk.len() as u64, 0, 0) };
    }
}

/// Sleep for `ticks` real timer ticks.
pub fn sleep(ticks: u64) {
    unsafe {
        syscall4(SYS_SET_ALARM, ticks, 0, 0, 0);
        syscall4(SYS_WAIT_ALARM, 0, 0, 0, 0);
    }
}

/// Open `path`, creating it if it doesn't exist.
pub fn open(path: &[u8]) -> Result<i64, i64> {
    result(unsafe { syscall4(SYS_FS_OPEN, path.as_ptr() as u64, path.len() as u64, 0, 0) })
}

/// Open `path` only if it already exists (`ENOENT` otherwise).
pub fn open_existing(path: &[u8]) -> Result<i64, i64> {
    result(unsafe { syscall4(SYS_FS_OPEN_EXISTING, path.as_ptr() as u64, path.len() as u64, 0, 0) })
}

/// Give `fd` back. Every open costs a slot in `fs` until it's closed.
pub fn close(fd: i64) -> Result<(), i64> {
    result(unsafe { syscall4(SYS_FS_CLOSE, fd as u64, 0, 0, 0) }).map(|_| ())
}

/// Create directory `path` (its parent must exist).
pub fn mkdir(path: &[u8]) -> Result<(), i64> {
    result(unsafe { syscall4(SYS_FS_MKDIR, path.as_ptr() as u64, path.len() as u64, 0, 0) }).map(|_| ())
}

/// Longest entry name `readdir` reports (the kernel's `fs::DIRENT_NAME_MAX`).
pub const DIRENT_NAME_MAX: usize = 118;

/// One directory entry: the kernel's `fs::DirEntry`, byte for byte.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DirEntry {
    pub size: u64,
    pub kind: u8,
    pub name_len: u8,
    pub name: [u8; DIRENT_NAME_MAX],
}

impl DirEntry {
    pub const fn empty() -> DirEntry {
        DirEntry { size: 0, kind: 0, name_len: 0, name: [0; DIRENT_NAME_MAX] }
    }

    pub fn name(&self) -> &[u8] {
        &self.name[..(self.name_len as usize).min(DIRENT_NAME_MAX)]
    }

    pub fn is_dir(&self) -> bool {
        self.kind == 1
    }
}

/// The `index`-th entry of directory `path` (subdirectories first, then
/// files): `Ok(true)` if `entry` was filled in, `Ok(false)` past the end.
pub fn readdir(path: &[u8], index: usize, entry: &mut DirEntry) -> Result<bool, i64> {
    result(unsafe {
        syscall4(
            SYS_FS_READDIR,
            path.as_ptr() as u64,
            path.len() as u64,
            index as u64,
            entry as *mut DirEntry as u64,
        )
    })
    .map(|n| n == 1)
}

/// Read up to `buf.len()` bytes; `Ok(0)` at end of file.
pub fn read(fd: i64, buf: &mut [u8]) -> Result<usize, i64> {
    let len = buf.len().min(MAX_IO);
    result(unsafe { syscall4(SYS_FS_READ, fd as u64, buf.as_mut_ptr() as u64, len as u64, 0) })
        .map(|n| n as usize)
}

/// Write all of `buf`.
pub fn write(fd: i64, buf: &[u8]) -> Result<usize, i64> {
    for chunk in buf.chunks(MAX_IO) {
        result(unsafe {
            syscall4(SYS_FS_WRITE, fd as u64, chunk.as_ptr() as u64, chunk.len() as u64, 0)
        })?;
    }
    Ok(buf.len())
}

/// Block until a whole line has been typed at the keyboard; returns its
/// length (no newline). Lines go to waiting readers oldest-first.
pub fn read_line(buf: &mut [u8]) -> usize {
    let len = buf.len().min(MAX_IO);
    unsafe { syscall4(SYS_READ_LINE, buf.as_mut_ptr() as u64, len as u64, 0, 0) as usize }
}

/// `fork()`: `Ok(0)` in the child, `Ok(child's process number)` in the
/// parent.
pub fn fork() -> Result<i64, i64> {
    result(unsafe { syscall4(SYS_FORK, 0, 0, 0, 0) })
}

/// `execve()`: replace this program with the one at `path`. `argv` and
/// `envp` are NULL-terminated arrays of pointers to NUL-terminated
/// strings, as C has them. Only returns on failure.
///
/// Safety: every pointer in both arrays must be valid up to its NUL.
pub unsafe fn exec(path: &[u8], argv: *const *const u8, envp: *const *const u8) -> i64 {
    syscall4(SYS_EXEC, path.as_ptr() as u64, path.len() as u64, argv as u64, envp as u64)
}

pub fn exit(status: i32) -> ! {
    unsafe {
        syscall4(SYS_EXIT, status as u32 as u64, 0, 0, 0);
        // Unreachable: SYS_EXIT doesn't return. If it somehow did,
        // parking is the only safe thing left to do.
        syscall4(SYS_BLOCK_FOREVER, 0, 0, 0, 0);
    }
    loop {}
}

/// `wait()`: block until some child terminates; `(its process number,
/// its exit status)`.
pub fn wait() -> Result<(i64, i32), i64> {
    let mut status: i32 = 0;
    let child = result(unsafe { syscall4(SYS_WAIT, &mut status as *mut i32 as u64, 0, 0, 0) })?;
    Ok((child, status))
}

/// A short description of an error code, for messages like
/// `cat: foo: no such file or directory`.
pub fn strerror(code: i64) -> &'static str {
    match code {
        ENOENT => "no such file or directory",
        ENOTDIR => "not a directory",
        EEXIST => "already exists",
        EINVAL => "invalid path (absolute, no // or trailing /)",
        EISDIR => "is a directory",
        ERR_BAD_ELF => "not an executable program",
        ERR_NO_FREE_PROC => "no free process slots",
        ERR_FORK_FAILED => "out of memory",
        ERR_NO_CHILDREN => "no child processes",
        ERR_BAD_ARG_PTR => "bad address",
        ERR_ARGS_TOO_BIG => "argument list too long",
        _ => "error",
    }
}

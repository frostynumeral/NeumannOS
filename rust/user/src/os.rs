//! Haiku's kernel kit, as NeumannOS implements it -- the functions, types
//! and constants of Haiku's `headers/os/kernel/OS.h` and
//! `headers/os/support/Errors.h`, with Haiku's names, semantics and
//! numeric values, so code written against Haiku's C API maps onto this
//! one-to-one. Where C passes a pointer and a length, this takes a
//! slice; where C takes a NUL-terminated name, a `&str`. Status codes
//! are Haiku's own (`B_OK`, `B_BAD_PORT_ID`, ...), returned exactly as
//! Haiku returns them: an `ssize_t` that is either a byte count or a
//! negative status.
//!
//! So far: ports (`create_port` and the rest of `OS.h`'s port API) and
//! `system_time`.

#![allow(non_camel_case_types)]

use crate::sys::syscall4;

pub type status_t = i32;
pub type ssize_t = isize;
pub type port_id = i32;
pub type team_id = i32;
pub type bigtime_t = i64;

pub const B_OS_NAME_LENGTH: usize = 32;

// `Errors.h`: B_GENERAL_ERROR_BASE is INT_MIN, B_OS_ERROR_BASE +0x1000.
pub const B_OK: status_t = 0;
pub const B_ERROR: status_t = -1;
const B_GENERAL_ERROR_BASE: status_t = i32::MIN;
const B_OS_ERROR_BASE: status_t = B_GENERAL_ERROR_BASE + 0x1000;
pub const B_NO_MEMORY: status_t = B_GENERAL_ERROR_BASE;
pub const B_BAD_VALUE: status_t = B_GENERAL_ERROR_BASE + 5;
pub const B_NAME_NOT_FOUND: status_t = B_GENERAL_ERROR_BASE + 7;
pub const B_TIMED_OUT: status_t = B_GENERAL_ERROR_BASE + 9;
pub const B_WOULD_BLOCK: status_t = B_GENERAL_ERROR_BASE + 11;
pub const B_INFINITE_TIMEOUT: bigtime_t = i64::MAX;
pub const B_BAD_TEAM_ID: status_t = B_OS_ERROR_BASE + 0x103;
pub const B_BAD_PORT_ID: status_t = B_OS_ERROR_BASE + 0x200;
pub const B_NO_MORE_PORTS: status_t = B_OS_ERROR_BASE + 0x201;
pub const B_BAD_ADDRESS: status_t = B_OS_ERROR_BASE + 0x301;

// `OS.h` timeout flags.
pub const B_TIMEOUT: u32 = 0x8;
pub const B_RELATIVE_TIMEOUT: u32 = 0x8;
pub const B_ABSOLUTE_TIMEOUT: u32 = 0x10;

/// `port_info` (`OS.h`), field for field.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct port_info {
    pub port: port_id,
    pub team: team_id,
    pub name: [u8; B_OS_NAME_LENGTH],
    pub capacity: i32,
    pub queue_count: i32,
    pub total_count: i32,
}

impl port_info {
    pub const fn new() -> port_info {
        port_info { port: 0, team: 0, name: [0; B_OS_NAME_LENGTH], capacity: 0, queue_count: 0, total_count: 0 }
    }

    /// `name` up to its NUL.
    pub fn name(&self) -> &[u8] {
        let len = self.name.iter().position(|&b| b == 0).unwrap_or(B_OS_NAME_LENGTH);
        &self.name[..len]
    }
}

impl Default for port_info {
    fn default() -> port_info {
        port_info::new()
    }
}

const SYS_CREATE_PORT: u64 = 28;
const SYS_FIND_PORT: u64 = 29;
const SYS_WRITE_PORT_ETC: u64 = 30;
const SYS_READ_PORT_ETC: u64 = 31;
const SYS_PORT_BUFFER_SIZE_ETC: u64 = 32;
const SYS_PORT_COUNT: u64 = 33;
const SYS_CLOSE_PORT: u64 = 34;
const SYS_DELETE_PORT: u64 = 35;
const SYS_GET_PORT_INFO: u64 = 36;
const SYS_SET_PORT_OWNER: u64 = 37;
const SYS_SYSTEM_TIME: u64 = 38;

/// A six-argument system call: the fifth and sixth go in `r8`/`r9`.
#[inline(always)]
unsafe fn syscall6(n: u64, a1: u64, a2: u64, a3: u64, a4: u64, a5: u64, a6: u64) -> i64 {
    let ret: u64;
    core::arch::asm!(
        "int 0x80",
        inlateout("rax") n => ret,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        in("rcx") a4,
        in("r8") a5,
        in("r9") a6,
        options(nostack),
    );
    ret as i64
}

/// Microseconds since boot.
pub fn system_time() -> bigtime_t {
    unsafe { syscall4(SYS_SYSTEM_TIME, 0, 0, 0, 0) }
}

pub fn create_port(capacity: i32, name: &str) -> port_id {
    unsafe { syscall4(SYS_CREATE_PORT, capacity as u32 as u64, name.as_ptr() as u64, name.len() as u64, 0) as port_id }
}

pub fn find_port(name: &str) -> port_id {
    unsafe { syscall4(SYS_FIND_PORT, name.as_ptr() as u64, name.len() as u64, 0, 0) as port_id }
}

pub fn write_port(port: port_id, code: i32, buffer: &[u8]) -> status_t {
    write_port_etc(port, code, buffer, 0, 0)
}

pub fn write_port_etc(port: port_id, code: i32, buffer: &[u8], flags: u32, timeout: bigtime_t) -> status_t {
    unsafe {
        syscall6(
            SYS_WRITE_PORT_ETC,
            port as u32 as u64,
            code as u32 as u64,
            buffer.as_ptr() as u64,
            buffer.len() as u64,
            flags as u64,
            timeout as u64,
        ) as status_t
    }
}

pub fn read_port(port: port_id, code: &mut i32, buffer: &mut [u8]) -> ssize_t {
    read_port_etc(port, code, buffer, 0, 0)
}

pub fn read_port_etc(port: port_id, code: &mut i32, buffer: &mut [u8], flags: u32, timeout: bigtime_t) -> ssize_t {
    unsafe {
        syscall6(
            SYS_READ_PORT_ETC,
            port as u32 as u64,
            code as *mut i32 as u64,
            buffer.as_mut_ptr() as u64,
            buffer.len() as u64,
            flags as u64,
            timeout as u64,
        ) as ssize_t
    }
}

pub fn port_buffer_size(port: port_id) -> ssize_t {
    port_buffer_size_etc(port, 0, 0)
}

pub fn port_buffer_size_etc(port: port_id, flags: u32, timeout: bigtime_t) -> ssize_t {
    unsafe { syscall4(SYS_PORT_BUFFER_SIZE_ETC, port as u32 as u64, flags as u64, timeout as u64, 0) as ssize_t }
}

pub fn port_count(port: port_id) -> ssize_t {
    unsafe { syscall4(SYS_PORT_COUNT, port as u32 as u64, 0, 0, 0) as ssize_t }
}

pub fn close_port(port: port_id) -> status_t {
    unsafe { syscall4(SYS_CLOSE_PORT, port as u32 as u64, 0, 0, 0) as status_t }
}

pub fn delete_port(port: port_id) -> status_t {
    unsafe { syscall4(SYS_DELETE_PORT, port as u32 as u64, 0, 0, 0) as status_t }
}

pub fn set_port_owner(port: port_id, team: team_id) -> status_t {
    unsafe { syscall4(SYS_SET_PORT_OWNER, port as u32 as u64, team as u32 as u64, 0, 0) as status_t }
}

pub fn get_port_info(port: port_id, info: &mut port_info) -> status_t {
    unsafe { syscall4(SYS_GET_PORT_INFO, port as u32 as u64, info as *mut port_info as u64, 0, 0) as status_t }
}

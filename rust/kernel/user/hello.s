# A trivial, freestanding ELF64 user-mode demo program, exercising a
# real syscall ABI (crate::syscall, called through int 0x80): increment
# a .data counter and call SYS_GET_UPTIME five times, then SYS_SET_ALARM
# a real alarm and SYS_WAIT_ALARM for it -- blocking this task inside the
# kernel and, once the alarm genuinely fires, resuming right here in
# ring 3 -- then call SYS_WRITE_LINE with a pointer into this program's
# own .data (proving a real cross-ring argument -- a pointer -- gets read
# correctly). Then it goes one step further: SYS_FS_OPEN a real file and
# SYS_FS_WRITE a message to it -- a genuine ring 3 -> syscall -> IPC ->
# fs round trip, not just a kernel-internal call. Finally, SYS_READ_LINE:
# this blocks for real, for as long as it takes a human (or a QMP
# send-key script) to actually type a line and press Enter, then writes
# whatever line arrives to a second file via SYS_FS_OPEN/SYS_FS_WRITE,
# before SYS_BLOCK_FOREVER, which never returns. No libc, no
# _start-time setup: the kernel's elf.rs loader jumps straight to
# _start with nothing but a stack.
#
# Note this means a plain, non-interactive boot will show this task
# blocked at SYS_READ_LINE indefinitely (nothing else in this port types
# anything on its own) -- exactly like a real shell waiting at a prompt,
# not a bug. See rust/README.md's "Running" section for how to actually
# supply a line over QEMU's QMP interface.
#
# Built into hello.elf (checked in alongside this file, since the kernel
# build has no cross toolchain wired in yet to assemble this
# automatically -- see rust/README.md's roadmap) via:
#
#   as -o hello.o hello.s
#   ld -static -nostdlib \
#     --section-start=.text=0x555555550000 \
#     --section-start=.data=0x555555560000 \
#     -o hello.elf hello.o
#
# The explicit --section-start addresses just need to be page-aligned
# and clear of whatever the kernel maps into a fresh address space by
# default (see memory::new_address_space); they don't need to avoid
# crate::usermode's own demo addresses, since each ring-3 task gets its
# own separate address space.

.section .text
.global _start
_start:
    mov $5, %ecx
1:
    incl counter(%rip)
    mov $1, %eax        # SYS_GET_UPTIME (crate::syscall::SYS_GET_UPTIME)
    int $0x80
    loop 1b

    mov $3, %edi        # delay_ticks = 3
    mov $4, %eax        # SYS_SET_ALARM (crate::syscall::SYS_SET_ALARM)
    int $0x80

    mov $5, %eax        # SYS_WAIT_ALARM (crate::syscall::SYS_WAIT_ALARM)
    int $0x80

    lea message(%rip), %rdi
    mov $message_len, %esi
    mov $2, %eax        # SYS_WRITE_LINE (crate::syscall::SYS_WRITE_LINE)
    int $0x80

    # Open a real file via fs (rdi=path ptr, rsi=path len); SYS_FS_OPEN
    # returns the new file descriptor in rax.
    lea path(%rip), %rdi
    mov $path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN (crate::syscall::SYS_FS_OPEN)
    int $0x80
    mov %rax, %r8       # stash the fd -- rax is about to be overwritten

    # Write file_message to that fd (rdi=fd, rsi=buf ptr, rdx=len).
    mov %r8, %rdi
    lea file_message(%rip), %rsi
    mov $file_message_len, %edx
    mov $7, %eax        # SYS_FS_WRITE (crate::syscall::SYS_FS_WRITE)
    int $0x80

    # Block for a real line of console input (rdi=buf ptr, rsi=max len).
    # Returns however many bytes were actually typed.
    lea line_buf(%rip), %rdi
    mov $line_buf_cap, %esi
    mov $9, %eax        # SYS_READ_LINE (crate::syscall::SYS_READ_LINE)
    int $0x80
    mov %rax, %r9       # stash the length read

    # Open a second file and write the received line to it.
    lea path2(%rip), %rdi
    mov $path2_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8       # fd

    mov %r8, %rdi
    lea line_buf(%rip), %rsi
    mov %r9, %rdx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    mov $3, %eax        # SYS_BLOCK_FOREVER (crate::syscall::SYS_BLOCK_FOREVER)
    int $0x80
    # unreachable: SYS_BLOCK_FOREVER's handler never returns.

.section .data
.global counter
counter:
    .long 0
message:
    .ascii "hello from the ELF-loaded ring-3 task, after waiting for a real alarm!"
message_len = . - message
path:
    .ascii "/from_ring3.txt"
path_len = . - path
file_message:
    .ascii "written from ring 3 via a real syscall, IPC, and fs!"
file_message_len = . - file_message
path2:
    .ascii "/from_console.txt"
path2_len = . - path2
line_buf:
    .skip 64
line_buf_cap = 64

# A trivial, freestanding ELF64 user-mode demo program, exercising a
# real syscall ABI (crate::syscall, called through int 0x80): increment
# a .data counter and call SYS_GET_UPTIME five times, then SYS_SET_ALARM
# a real alarm and SYS_WAIT_ALARM for it -- blocking this task inside the
# kernel and, once the alarm genuinely fires, resuming right here in
# ring 3 -- then call SYS_WRITE_LINE with a pointer into this program's
# own .data (proving a real cross-ring argument -- a pointer -- gets read
# correctly), then SYS_BLOCK_FOREVER, which never returns. No libc, no
# _start-time setup: the kernel's elf.rs loader jumps straight to _start
# with nothing but a stack.
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

# A trivial, freestanding ELF64 user-mode demo program: increment a
# .data counter, trap into the kernel (int 0x80 -- crate::interrupts'
# SYSCALL_VECTOR), repeat. No libc, no _start-time setup: the kernel's
# elf.rs loader jumps straight to _start with nothing but a stack.
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
1:
    incl counter(%rip)
    int $0x80
    jmp 1b

.section .data
.global counter
counter:
    .long 0

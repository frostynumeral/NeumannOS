# The program `user/shell.s` replaces itself with, via a real exec()
# (crate::syscall's SYS_EXEC): a second, completely separate freestanding
# ELF64 binary that has nothing in common with its predecessor -- its own
# code, its own .data, its own link addresses -- so "the caller's image
# was genuinely replaced" is observable from outside rather than taken on
# faith.
#
# What it does, in order:
#
#   1. `call bump_counter` -- deliberately a *call*, not an inline
#      `incl`: a call pushes a return address and `ret` pops it, so this
#      only works if exec handed this image a real, writable, mapped
#      stack (crate::elf's STACK_ADDR page, freshly allocated for the new
#      address space). A bad RSP in the rewritten trap frame would fault
#      here rather than silently limping along.
#   2. SYS_WRITE_LINE, so a human reading COM1 sees the new program
#      announce itself.
#   3. SYS_FS_OPEN/SYS_FS_WRITE to /from_exec.txt -- the same
#      independent-verification trick user/hello.s uses: a kernel task
#      (crate::main's exec_verify) later reads that file back through
#      fs's ordinary kernel-side path, so the proof that this code ran
#      doesn't depend on trusting the syscall log.
#   4. One hostile exec, from ring 3, for real: it writes a 120-byte
#      hand-built ELF to /evil whose single PT_LOAD asks to be mapped at
#      0x444444440000 -- crate::allocator::HEAP_START, the kernel's own
#      heap -- and execs it. That image was ACCEPTED by the first version
#      of this port's validator, which bounded segments against
#      USER_SPACE_END and believed that meant "not the kernel"; since a
#      new address space copies only the top-level page table, mapping it
#      reached into the kernel's own tables. It must now come back
#      crate::syscall::ERR_BAD_ELF (the loader's SegmentInSharedSlot).
#      The returned code is written to /evil_exec_error.bin for
#      crate::main's exec_verify to check. There is a kernel-side version
#      of this test too (crate::elf::validator_self_test), but only this
#      one proves the path is closed to an actual unprivileged process.
#   5. SYS_BLOCK_FOREVER, which never returns.
#
# `counter` sits at the very start of .data (0x666666670000, this file's
# --section-start below) so crate::main's exec_verify can sys_vircopy it
# back out of the exec'd process's address space and check it reads 1 --
# the exec'd image's own instructions having run, in the *caller's*
# process slot.
#
# Built into echo.elf (checked in alongside this file, same reasoning as
# user/hello.elf -- no cross toolchain wired into the kernel build yet)
# via:
#
#   as -o echo.o echo.s
#   ld -static -nostdlib \
#     --section-start=.text=0x666666660000 \
#     --section-start=.data=0x666666670000 \
#     -o echo.elf echo.o
#
# The addresses only need to be page-aligned and in PML4 slots the
# kernel's own address space doesn't use -- crate::elf::validate now
# enforces that second part rather than leaving it to whoever picks the
# link addresses (see memory::pml4_slots_unused). What exec_verify's "the
# old image is really gone" check additionally needs is only that this
# image does not map shell's marker *page*; keeping the two programs in
# different PML4 slots is one easy way to guarantee that, not a
# requirement of exec itself.

.section .text
.global _start
_start:
    call bump_counter

    lea message(%rip), %rdi
    mov $message_len, %esi
    mov $2, %eax        # SYS_WRITE_LINE (crate::syscall::SYS_WRITE_LINE)
    int $0x80

    lea path(%rip), %rdi
    mov $path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN (crate::syscall::SYS_FS_OPEN)
    int $0x80
    mov %rax, %r8       # stash the fd -- rax is about to be overwritten

    mov %r8, %rdi
    lea file_message(%rip), %rsi
    mov $file_message_len, %edx
    mov $7, %eax        # SYS_FS_WRITE (crate::syscall::SYS_FS_WRITE)
    int $0x80

    # --- hostile exec attempt (see the header comment) ---
    lea evil_path(%rip), %rdi
    mov $evil_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8

    mov %r8, %rdi
    lea evil_elf(%rip), %rsi
    mov $evil_elf_len, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    lea evil_path(%rip), %rdi
    mov $evil_path_len, %esi
    mov $12, %eax       # SYS_EXEC -- must fail; if it succeeds we never come back
    int $0x80
    mov %rax, evil_result(%rip)

    lea evil_err_path(%rip), %rdi
    mov $evil_err_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8

    mov %r8, %rdi
    lea evil_result(%rip), %rsi
    mov $8, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    mov $3, %eax        # SYS_BLOCK_FOREVER (crate::syscall::SYS_BLOCK_FOREVER)
    int $0x80
    # unreachable: SYS_BLOCK_FOREVER's handler never returns.

# Exercises the freshly-mapped stack exec set up for this image (see the
# header comment): push a return address, come back through it.
bump_counter:
    incl counter(%rip)
    ret

.section .data
.global counter
counter:
    .long 0
message:
    .ascii "hello from /bin/echo -- a different program, running in the same process after exec()!"
message_len = . - message
path:
    .ascii "/from_exec.txt"
path_len = . - path
file_message:
    .ascii "written by the exec'd image, not the one that called exec"
file_message_len = . - file_message
evil_path:
    .ascii "/evil"
evil_path_len = . - evil_path
evil_err_path:
    .ascii "/evil_exec_error.bin"
evil_err_path_len = . - evil_err_path
    .align 8
evil_result:
    .quad 0

# A hand-built ELF64 image asking to be loaded straight onto the kernel
# heap. Assembled here as data rather than linked, because no linker
# would ever emit it -- which is the point.
    .align 8
evil_elf:
    .ascii "\177ELF"
    .byte 2, 1, 1, 0            # ELFCLASS64, ELFDATA2LSB, EV_CURRENT
    .byte 0, 0, 0, 0, 0, 0, 0, 0 # e_ident padding
    .short 2                    # e_type = ET_EXEC
    .short 0x3e                 # e_machine = x86-64
    .long 1                     # e_version
    .quad 0x444444440000        # e_entry (crate::allocator::HEAP_START)
    .quad 64                    # e_phoff
    .quad 0                     # e_shoff
    .long 0                     # e_flags
    .short 64                   # e_ehsize
    .short 56                   # e_phentsize
    .short 1                    # e_phnum
    .short 0                    # e_shentsize
    .short 0                    # e_shnum
    .short 0                    # e_shstrndx
    .long 1                     # p_type = PT_LOAD
    .long 6                     # p_flags = RW
    .quad 0                     # p_offset
    .quad 0x444444440000        # p_vaddr = the kernel's heap
    .quad 0x444444440000        # p_paddr
    .quad 0                     # p_filesz -- pure BSS, needs no file bytes
    .quad 0x1000                # p_memsz
    .quad 0x1000                # p_align
evil_elf_len = . - evil_elf

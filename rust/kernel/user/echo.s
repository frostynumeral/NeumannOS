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
#   5. It is an echo: it reads the argc/argv/envp exec laid out on its
#      stack (crate::elf's write_initial_stack -- argc at rsp, then
#      argv[], NULL, envp[], NULL), joins argv[1..] with spaces, prints
#      the result with SYS_WRITE_LINE, and writes it to
#      /echo_output.txt; envp[0] goes to /echo_env.txt. It also records
#      its entry rsp and argc in .data (entry_rsp/argc_seen), so
#      exec_verify can walk the start-up block in this process's own
#      memory rather than trusting what this program did with it.
#   6. Three more hostile execs, each of which must fail and leave this
#      image running, with the four results (the last test contributes
#      two) written to /argv_errors.bin:
#        - argv itself pointing at the kernel heap: ERR_BAD_ARG_PTR. Had
#          it been accepted, whatever the kernel keeps there would have
#          been copied onto a ring-3 stack.
#        - a well-placed argv whose one element points at the kernel
#          heap: ERR_BAD_ARG_PTR, the same check made per string.
#        - 70 arguments, past crate::syscall's MAX_EXEC_VECTOR:
#          ERR_ARGS_TOO_BIG.
#        - 30 arguments of 100 bytes each, past
#          crate::elf::MAX_START_ARGS_BYTES: ERR_ARGS_TOO_BIG.
#      Each of these targets /bin/echo -- this very program -- so an exec
#      that wrongly succeeded would visibly start over rather than
#      quietly doing nothing.
#   7. SYS_BLOCK_FOREVER, which never returns.
#
# `counter` sits at the very start of .data (0x666666670000, this file's
# --section-start below) so crate::main's exec_verify can sys_vircopy it
# back out of the exec'd process's address space and check it reads 1 --
# the exec'd image's own instructions having run, in the *caller's*
# process slot. entry_rsp and argc_seen follow it at +8 and +16
# (crate::elf's ECHO_ENTRY_RSP_ADDR/ECHO_ARGC_ADDR).
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
    # Before anything pushes: rsp is exactly where exec put argc.
    mov %rsp, entry_rsp(%rip)
    mov (%rsp), %rax
    mov %rax, argc_seen(%rip)

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
    xor %edx, %edx      # argv = NULL
    xor %ecx, %ecx      # envp = NULL
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

    # --- echo: join argv[1..argc-1] with spaces into out_buf ---
    mov entry_rsp(%rip), %rbx   # rbx -> argc; argv[i] is 8(%rbx,i,8)
    mov (%rbx), %r12            # argc
    lea out_buf(%rip), %rdi     # write cursor
    lea out_buf_end(%rip), %r14 # copy_str's limit
    mov $1, %r13                # i
1:  cmp %r12, %r13
    jae 3f
    cmp $1, %r13
    je 2f                       # no separator before the first one
    cmp %r14, %rdi
    jae 3f
    movb $' ', (%rdi)
    inc %rdi
2:  mov 8(%rbx,%r13,8), %rsi
    call copy_str
    inc %r13
    jmp 1b
3:  lea out_buf(%rip), %rsi
    sub %rsi, %rdi
    mov %rdi, %r15              # length of the joined line

    lea out_buf(%rip), %rdi
    mov %r15, %rsi
    mov $2, %eax        # SYS_WRITE_LINE
    int $0x80

    lea out_path(%rip), %rdi
    mov $out_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %rdi
    lea out_buf(%rip), %rsi
    mov %r15, %rdx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    # --- envp[0], which sits just past argv's NULL: 16(%rbx,argc,8) ---
    mov 16(%rbx,%r12,8), %rsi
    test %rsi, %rsi
    jz 4f
    lea env_buf(%rip), %rdi
    lea env_buf_end(%rip), %r14
    call copy_str
    lea env_buf(%rip), %rsi
    sub %rsi, %rdi
    mov %rdi, %r15

    lea env_path(%rip), %rdi
    mov $env_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %rdi
    lea env_buf(%rip), %rsi
    mov %r15, %rdx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80
4:

    # --- hostile argv/envp execs (see the header comment), all of
    # /bin/echo, all of which must fail ---
    mov $0x444444440000, %rdx   # argv itself on the kernel heap
    xor %ecx, %ecx
    call exec_self
    mov %rax, argv_errors(%rip)

    lea heap_elem_argv(%rip), %rdx  # argv fine, argv[0] on the kernel heap
    xor %ecx, %ecx
    call exec_self
    mov %rax, argv_errors+8(%rip)

    lea many_argv(%rip), %rdx   # too many entries
    xor %ecx, %ecx
    call exec_self
    mov %rax, argv_errors+16(%rip)

    lea long_argv(%rip), %rdx   # too many bytes
    xor %ecx, %ecx
    call exec_self
    mov %rax, argv_errors+24(%rip)

    lea argv_err_path(%rip), %rdi
    mov $argv_err_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %rdi
    lea argv_errors(%rip), %rsi
    mov $32, %edx
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

# Copy the NUL-terminated string at rsi to rdi, stopping at its NUL
# (not copied) or at r14, whichever comes first; leaves rdi one past the
# last byte written. Clobbers al and rsi.
copy_str:
0:  cmp %r14, %rdi
    jae 1f
    movb (%rsi), %al
    test %al, %al
    jz 1f
    movb %al, (%rdi)
    inc %rsi
    inc %rdi
    jmp 0b
1:  ret

# SYS_EXEC /bin/echo with the argv/envp already in rdx/rcx; returns the
# (expected: error) result in rax.
exec_self:
    lea self_path(%rip), %rdi
    mov $self_path_len, %esi
    mov $12, %eax       # SYS_EXEC
    int $0x80
    ret

.section .data
.global counter
counter:
    .long 0
    .align 8
.global entry_rsp
entry_rsp:              # +8: rsp on entry, i.e. the address of argc
    .quad 0
.global argc_seen
argc_seen:              # +16: argc as read from the stack
    .quad 0
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

out_path:
    .ascii "/echo_output.txt"
out_path_len = . - out_path
env_path:
    .ascii "/echo_env.txt"
env_path_len = . - env_path
argv_err_path:
    .ascii "/argv_errors.bin"
argv_err_path_len = . - argv_err_path
self_path:
    .ascii "/bin/echo"
self_path_len = . - self_path
x_arg:
    .asciz "x"
hundred_arg:            # 100 bytes before its NUL
    .rept 10
    .ascii "0123456789"
    .endr
    .byte 0
    .align 8
argv_errors:
    .quad 0, 0, 0, 0
heap_elem_argv:
    .quad 0x444444440000, 0
many_argv:
    .rept 70
    .quad x_arg
    .endr
    .quad 0
long_argv:
    .rept 30
    .quad hundred_arg
    .endr
    .quad 0
out_buf:
    .skip 256
out_buf_end:
env_buf:
    .skip 128
env_buf_end:

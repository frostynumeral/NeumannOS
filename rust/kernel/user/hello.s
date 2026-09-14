# A trivial, freestanding ELF64 user-mode demo program, exercising a
# real syscall ABI (crate::syscall, called through int 0x80): increment
# a .data counter and call SYS_GET_UPTIME five times, then SYS_SET_ALARM
# a real alarm and SYS_WAIT_ALARM for it -- blocking this task inside the
# kernel and, once the alarm genuinely fires, resuming right here in
# ring 3 -- then call SYS_WRITE_LINE with a pointer into this program's
# own .data (proving a real cross-ring argument -- a pointer -- gets read
# correctly). Then it goes one step further: SYS_FS_OPEN a real file and
# SYS_FS_WRITE a message to it -- a genuine ring 3 -> syscall -> IPC ->
# fs round trip, not just a kernel-internal call. Then SYS_VIRCOPY: reads
# a *different* process's private memory (driver's own code page) straight
# from ring 3, into this task's own vircopy_buf -- the same cross-address-
# space copy crate::main's vircopy_demo already does kernel-side, now
# reachable through the syscall ABI. Then a deliberately-invalid
# SYS_VIRCOPY (an oversized len), to exercise the syscall ABI's distinct
# error codes (crate::syscall's ERR_BAD_LENGTH/ERR_BAD_UTF8/
# ERR_VIRCOPY_FAILED/ERR_UNKNOWN_CALL) rather than just a single
# undifferentiated failure sentinel -- the returned code is stashed in
# err_result for a kernel task to read back and check afterward, the same
# way vircopy_buf's real copy is. Finally, SYS_READ_LINE:
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
# Right after the counter loop, it also calls SYS_FORK -- creating a
# genuine child process (crate::syscall::SYS_FORK) that resumes at this
# *exact* point too, diverging only in what SYS_FORK's own return value
# (rax) reads: the parent sees the new child's proc_nr (nonzero) and falls
# through to continue the sequence above unchanged; the child sees 0 and
# branches off to write a canary into vircopy_buf (proving its copy of
# that page is genuinely independent from the parent's -- the parent
# populates its own copy with driver's code bytes later, via the real
# SYS_VIRCOPY below) and a distinguishing message to its own file, before
# blocking for good -- never touching SET_ALARM/READ_LINE, so it can't
# collide with the parent's own use of those.
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

    mov $11, %eax       # SYS_FORK (crate::syscall::SYS_FORK)
    int $0x80
    test %rax, %rax
    jnz 2f              # parent: rax = child's proc_nr (nonzero) -- fall through unchanged below

    # --- child path (rax == 0) ---
    movl $0xcafebabe, vircopy_buf(%rip) # canary: proves this page is a
                                         # genuinely independent copy, not
                                         # aliased with the parent's
    lea fork_child_path(%rip), %rdi
    mov $fork_child_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8

    mov %r8, %rdi
    lea fork_child_message(%rip), %rsi
    mov $fork_child_message_len, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    mov $3, %eax        # SYS_BLOCK_FOREVER
    int $0x80
    # unreachable: the child's story ends here.
2:
    # --- parent continues exactly as before ---
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

    # SYS_VIRCOPY (rdi=src_proc, rsi=src_addr, rdx=local dst ptr,
    # rcx=len): copy driver's own code page (a different process's
    # private address space, com::DRVR_PROC_NR / usermode::USER_CODE_ADDR)
    # into this task's own vircopy_buf.
    mov $6, %edi              # src_proc = DRVR_PROC_NR
    mov $0x555555550000, %rsi # src_addr = usermode::USER_CODE_ADDR
    lea vircopy_buf(%rip), %rdx
    mov $28, %ecx             # len = usermode::USER_CODE.len()
    mov $10, %eax             # SYS_VIRCOPY (crate::syscall::SYS_VIRCOPY)
    int $0x80

    # Deliberately-invalid SYS_VIRCOPY: len (rcx) exceeds
    # crate::syscall::MAX_VIRCOPY_LEN (256), so this should come back
    # ERR_BAD_LENGTH rather than actually copying anything.
    mov $6, %edi              # src_proc = DRVR_PROC_NR
    mov $0x555555550000, %rsi # src_addr = usermode::USER_CODE_ADDR
    lea vircopy_buf(%rip), %rdx
    mov $9999, %ecx           # len -- deliberately too large
    mov $10, %eax             # SYS_VIRCOPY
    int $0x80
    mov %rax, err_result(%rip)

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
vircopy_buf:
    .skip 32
err_result:
    .quad 0
fork_child_path:
    .ascii "/from_fork_child.txt"
fork_child_path_len = . - fork_child_path
fork_child_message:
    .ascii "hello from the forked child, running independently in ring 3!"
fork_child_message_len = . - fork_child_message

# A freestanding ELF64 program whose whole purpose is to stop being
# itself: it calls exec() (crate::syscall's SYS_EXEC) on /bin/echo and is
# replaced, in place, by a completely different binary (user/echo.s) --
# same process number, same kernel stack, same scheduler slot, brand new
# address space and brand new code. fork() (user/hello.s) makes a second
# process that *is* the caller; exec() keeps the one process and throws
# away everything about what it was running. This file is the second half
# of that pair.
#
# Before the real exec it deliberately gets one *wrong*, too: it creates
# /not_a_program (an ordinary text file), tries to exec that, and carries
# on. A failed exec has to be a no-op -- POSIX is explicit that a process
# whose exec fails keeps running its original image, and it's the one
# guarantee that's easy to break here, since loading a new image and
# discarding the old one happen a few lines apart in
# crate::calls::sys_exec. Reaching the instruction after that call at all
# is most of the proof; the rest is that the error code (expected
# crate::syscall::ERR_BAD_ELF) gets written to /exec_error.bin, from
# *this* image, using this image's own .data and stack, for
# crate::main's exec_verify to check later. It goes through a file rather
# than staying in memory because the successful exec below is about to
# throw this program's memory away.
#
# Before exec'ing it writes 0xfeedface into pre_exec_marker, the first
# thing in its own .data (0x555555580000, this file's --section-start
# below). That marker is how crate::main's exec_verify proves the *old*
# image is genuinely gone afterward rather than merely no longer running:
# it sys_vircopy's that exact address out of this process once exec has
# happened and requires the read to *fail* (CopyError::SrcNotMapped) --
# the page isn't mapped in the new address space at all. A verification
# that only checked "the new program ran" would pass just as happily if
# exec had bolted a second image onto the side of the first.
#
# The exec call sits in a bounded retry loop rather than being a
# straight-line call, and that is deliberate. /bin/echo has to be in
# `fs` before it can be exec'd, and it gets there at runtime (crate::main's
# seed_bin, from `pm`) -- so a straight-line exec here would be a bet on
# `pm` reaching that point before this task is first scheduled. That bet
# happens to be safe today by a wide margin, and "true in practice" is
# exactly the reasoning that produced this port's one reproducible
# boot-time race (see crate::main's elf_counter_demo and rust/README.md).
# Retrying on failure removes the assumption instead of documenting it:
# SYS_SET_ALARM/SYS_WAIT_ALARM sleep a real tick between attempts, so
# this task is genuinely blocked (not spinning) while `pm` gets on with
# installing the binary. ATTEMPTS bounds it so a permanently missing
# binary ends in a clear complaint rather than an endless loop.
#
# Built into shell.elf (checked in alongside this file, same reasoning as
# user/hello.elf -- no cross toolchain wired into the kernel build yet)
# via:
#
#   as -o shell.o shell.s
#   ld -static -nostdlib \
#     --section-start=.text=0x555555570000 \
#     --section-start=.data=0x555555580000 \
#     -o shell.elf shell.o
#
# See user/echo.s's header for the constraint on both programs' link
# addresses (PML4 slots the kernel doesn't use), and for why keeping the
# two programs in separate slots is convenient rather than required.

.equ ATTEMPTS, 60       # ~1 second of retries at pit::HZ (60) ticks/sec

.section .text
.global _start
_start:
    movl $0xfeedface, pre_exec_marker(%rip)

    lea before(%rip), %rdi
    mov $before_len, %esi
    mov $2, %eax        # SYS_WRITE_LINE (crate::syscall::SYS_WRITE_LINE)
    int $0x80

    # Create an ordinary, definitely-not-an-ELF file to exec at.
    # Self-contained on purpose: this doesn't wait on any other task
    # having put something suitable in `fs` first. Its contents are
    # padded past 64 bytes so the loader has to reject it on its missing
    # ELF magic, not just for being shorter than a header.
    lea not_elf_path(%rip), %rdi
    mov $not_elf_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN (crate::syscall::SYS_FS_OPEN)
    int $0x80
    mov %rax, %r8       # stash the fd -- rax is about to be overwritten

    mov %r8, %rdi
    lea junk(%rip), %rsi
    mov $junk_len, %edx
    mov $7, %eax        # SYS_FS_WRITE (crate::syscall::SYS_FS_WRITE)
    int $0x80

    # Exec it. This must fail (ERR_BAD_ELF) *and* leave this image
    # running: every instruction from here on is running on the stack and
    # out of the .data of the program that called exec.
    lea not_elf_path(%rip), %rdi
    mov $not_elf_path_len, %esi
    mov $12, %eax       # SYS_EXEC (crate::syscall::SYS_EXEC)
    int $0x80
    mov %rax, exec_error(%rip)

    # Park the error code somewhere that outlives this image.
    lea err_path(%rip), %rdi
    mov $err_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8

    mov %r8, %rdi
    lea exec_error(%rip), %rsi
    mov $8, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    mov $ATTEMPTS, %r12d
1:
    # SYS_EXEC (rdi=path ptr, rsi=path len). On success this never
    # returns *here*: the same iretq that would have resumed the next
    # instruction below instead lands at the new image's own entry point,
    # on the new image's own stack. Only a failure comes back.
    lea prog(%rip), %rdi
    mov $prog_len, %esi
    mov $12, %eax       # SYS_EXEC (crate::syscall::SYS_EXEC)
    int $0x80

    # Failed (almost certainly ENOENT -- /bin/echo not installed yet).
    # Sleep one real tick and try again, up to ATTEMPTS times.
    mov $1, %edi        # delay_ticks = 1
    mov $4, %eax        # SYS_SET_ALARM (crate::syscall::SYS_SET_ALARM)
    int $0x80
    mov $5, %eax        # SYS_WAIT_ALARM (crate::syscall::SYS_WAIT_ALARM)
    int $0x80
    dec %r12
    jnz 1b

    lea gave_up(%rip), %rdi
    mov $gave_up_len, %esi
    mov $2, %eax        # SYS_WRITE_LINE
    int $0x80

    mov $3, %eax        # SYS_BLOCK_FOREVER (crate::syscall::SYS_BLOCK_FOREVER)
    int $0x80
    # unreachable: SYS_BLOCK_FOREVER's handler never returns.

.section .data
.global pre_exec_marker
pre_exec_marker:
    .long 0
    .align 8
exec_error:
    .quad 0
not_elf_path:
    .ascii "/not_a_program"
not_elf_path_len = . - not_elf_path
junk:
    .ascii "this file is not an ELF binary, and exec must say so -- deliberately padded past the 64 bytes of an ELF64 header, so the loader rejects it on its magic number rather than merely on its length"
junk_len = . - junk
err_path:
    .ascii "/exec_error.bin"
err_path_len = . - err_path
before:
    .ascii "shell: about to replace myself with /bin/echo via a real exec()"
before_len = . - before
gave_up:
    .ascii "shell: /bin/echo never showed up in fs -- giving up on exec()"
gave_up_len = . - gave_up
prog:
    .ascii "/bin/echo"
prog_len = . - prog

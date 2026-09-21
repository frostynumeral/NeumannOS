# A freestanding ELF64 program that does what a shell does: it forks
# (crate::syscall's SYS_FORK), and the child replaces itself with a
# completely different binary (user/echo.s) via exec()
# (crate::syscall's SYS_EXEC) while the parent carries on being itself.
# fork() makes a second process that *is* the caller; exec() keeps one
# process and throws away everything about what it was running. Running
# a program is the two of them composed, and that composition is what
# this file is for.
#
# It didn't used to fork: this program used to exec /bin/echo over
# itself, which proved exec but meant the process that asked for a
# program was also the one that stopped existing -- no shell can work
# that way, since it would have nothing left to return to a prompt.
# Forking first needs two things this port didn't have: a process number
# handed out at runtime rather than reserved per caller in crate::com
# (crate::proc::alloc_proc_nr), and a per-process memory map saying which
# pages a fork has to copy (crate::memory::MemMap) -- without the
# latter, SYS_FORK only worked for one hardcoded caller, and this program
# wasn't it.
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
# Before forking it writes 0xfeedface into pre_exec_marker, the first
# thing in its own .data (0x555555580000, this file's --section-start
# below). That marker does double duty in crate::main's exec_verify,
# because after the fork there are two address spaces to ask about it.
# Read out of the *child*, it has to be gone (CopyError::SrcNotMapped):
# the child inherited a copy of this image and exec threw it away, so a
# verification that only checked "the new program ran" -- which would
# pass just as happily if exec had bolted a second image onto the side of
# the first -- isn't what's being checked. Read out of the *parent*, it
# has to still be 0xfeedface: this process kept running its own image
# while its child stopped running that image entirely.
#
# The parent also writes the proc_nr SYS_FORK handed back to it into
# /shell_fork.bin, so exec_verify can check that ring 3's idea of which
# process its child is matches the kernel's own (crate::proc::child_of).
#
# After that it forks twice more, and those two children are what
# exercise the other end of a process's life: SYS_EXIT (crate::proc's
# exit_now) and SYS_WAIT (wait_for_child). Neither of them execs -- they
# run this same inherited image, write a file to prove they really got
# scheduled, and exit with a status this program then collects. The
# child that execs deliberately does *not* exit, because crate::main's
# exec_verify has to read its address space long afterwards to prove the
# exec landed in the right process; a reaped child has no address space
# to read.
#
# The two are separated on purpose, because there are two distinct paths
# through a terminating child and only the timing tells them apart:
#
#   child B: forked, then waited for immediately. The parent is already
#     blocked in SYS_WAIT when B exits, so B hands its status straight
#     over and no zombie is ever created (Scheduler::terminate's
#     wait_result path). Status 42.
#   child C: forked, then given three real ticks to finish (a genuine
#     SYS_SET_ALARM/SYS_WAIT_ALARM sleep) before the parent waits at all.
#     C is a zombie by then -- terminated, slot and process number still
#     allocated, holding its status -- and the wait collects it and frees
#     the slot. Status 7.
#
# If the timing ever went the other way round, C would simply take B's
# path and both waits would still return the right answers; the sleep
# makes the zombie path *likely*, not load-bearing. Each wait's
# (proc_nr, status) pair goes to its own file for exec_verify to check.
#
# The child's exec call sits in a bounded retry loop rather than being a
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

    # Fork. The parent gets the child's proc_nr in rax and stays this
    # program; the child gets 0 and goes on to become a different one.
    mov $11, %eax       # SYS_FORK (crate::syscall::SYS_FORK)
    int $0x80
    test %rax, %rax
    jz 2f               # child (rax == 0): go exec /bin/echo below

    # --- parent path: record which process the child is, and stay shell ---
    mov %rax, fork_child_nr(%rip)
    lea fork_path(%rip), %rdi
    mov $fork_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8

    mov %r8, %rdi
    lea fork_child_nr(%rip), %rsi
    mov $8, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    # --- child B: fork one that exits, and wait for it right away ---
    mov $11, %eax       # SYS_FORK
    int $0x80
    test %rax, %rax
    jz 4f               # child B

    # Parent: block in SYS_WAIT (rdi = where to write the status).
    # B hasn't run yet, so this genuinely blocks and B's exit is what
    # wakes it -- no zombie in between.
    lea wait_status(%rip), %rdi
    mov $14, %eax       # SYS_WAIT (crate::syscall::SYS_WAIT)
    int $0x80
    mov %rax, wait_child(%rip)
    lea wait1_path(%rip), %rdi
    mov $wait1_path_len, %esi
    call write_pair

    # --- child C: fork one that exits, but let it become a zombie ---
    mov $11, %eax       # SYS_FORK
    int $0x80
    test %rax, %rax
    jz 5f               # child C

    # Parent: sleep three real ticks first, so C has long since exited
    # and is sitting in a zombie slot by the time we ask for it.
    mov $3, %edi        # delay_ticks = 3
    mov $4, %eax        # SYS_SET_ALARM
    int $0x80
    mov $5, %eax        # SYS_WAIT_ALARM
    int $0x80

    lea wait_status(%rip), %rdi
    mov $14, %eax       # SYS_WAIT
    int $0x80
    mov %rax, wait_child(%rip)
    lea wait2_path(%rip), %rdi
    mov $wait2_path_len, %esi
    call write_pair

    lea still_shell(%rip), %rdi
    mov $still_shell_len, %esi
    mov $2, %eax        # SYS_WRITE_LINE
    int $0x80

    mov $3, %eax        # SYS_BLOCK_FOREVER
    int $0x80
    # unreachable: the parent waits here forever, still running this
    # image -- which is exactly what exec_verify checks it is.

4:  # --- child B (rax == 0): prove we ran, then exit with 42 ---
    lea childb_path(%rip), %rdi
    mov $childb_path_len, %esi
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %r8

    mov %r8, %rdi
    lea childb_message(%rip), %rsi
    mov $childb_message_len, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80

    mov $42, %edi
    mov $13, %eax       # SYS_EXIT (crate::syscall::SYS_EXIT)
    int $0x80
    # unreachable: SYS_EXIT's handler never returns -- this process is
    # gone by the time the trap would have.

5:  # --- child C (rax == 0): exit straight away with 7 ---
    mov $7, %edi
    mov $13, %eax       # SYS_EXIT
    int $0x80
    # unreachable, same as above.

# Write the (proc_nr, status) pair SYS_WAIT just produced to the path in
# rdi/rsi -- sixteen bytes, wait_child followed by wait_status, which sit
# next to each other in .data precisely so one write covers both.
write_pair:
    mov $6, %eax        # SYS_FS_OPEN
    int $0x80
    mov %rax, %rdi
    lea wait_child(%rip), %rsi
    mov $16, %edx
    mov $7, %eax        # SYS_FS_WRITE
    int $0x80
    ret
2:
    # --- child path: replace this inherited image with /bin/echo ---
    mov $ATTEMPTS, %r12d
3:
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
    jnz 3b

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
    .ascii "shell: about to fork, and have the child become /bin/echo via a real exec()"
before_len = . - before
still_shell:
    .ascii "shell: forked -- my child is becoming /bin/echo, and I am still shell"
still_shell_len = . - still_shell
gave_up:
    .ascii "shell: /bin/echo never showed up in fs -- giving up on exec()"
gave_up_len = . - gave_up
fork_path:
    .ascii "/shell_fork.bin"
fork_path_len = . - fork_path
wait1_path:
    .ascii "/shell_wait1.bin"
wait1_path_len = . - wait1_path
wait2_path:
    .ascii "/shell_wait2.bin"
wait2_path_len = . - wait2_path
childb_path:
    .ascii "/from_exited_child.txt"
childb_path_len = . - childb_path
childb_message:
    .ascii "written by a forked child that then exited with a real status"
childb_message_len = . - childb_message
    .align 8
fork_child_nr:
    .quad 0
# Adjacent on purpose: write_pair above writes both in one SYS_FS_WRITE.
wait_child:
    .quad 0
wait_status:
    .quad 0
prog:
    .ascii "/bin/echo"
prog_len = . - prog

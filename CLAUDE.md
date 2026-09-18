# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

This repository is the source code of **MINIX 3.1.0**, extracted from the CD-ROM accompanying the textbook *Operating Systems Design and Implementation, 3rd edition*. It is a microkernel OS: a tiny kernel plus user-space servers and drivers communicating over message passing. See `README.md` for the full history and provenance, including how to obtain the ISO if needed.

This is 2005-era MINIX C code (K&R-style function definitions, `PUBLIC`/`PRIVATE`/`FORWARD` macros instead of raw `static`/`extern`, `_PROTOTYPE` macros for portable prototypes). Match the existing style when editing — do not modernize syntax.

There is also an in-progress **Rust rewrite** under `rust/` (see `rust/README.md`) that ports this same architecture (boot image, IPC message passing) from scratch. It is a skeleton, not a finished port — see its README for what's implemented and the roadmap for the rest. When asked to "build it in Rust" or work on the Rust port, work in `rust/`; when asked to fix or extend MINIX itself, work in the C tree described below. The two are independent code bases that happen to share a design.

## Building and running

This source tree is meant to be built **inside a running MINIX 3.1.0 system** (under `/usr/src`), not on the host machine directly — it targets an i386 toolchain (ACK/`cc`) and MINIX-specific headers/libc that don't exist on a modern Linux host. See `README.md` for the full QEMU/Bochs setup (creating a disk image, installing from the ISO, enabling networking).

Once inside the MINIX guest, from `/usr/src/tools`:

```
make clean
make images      # builds kernel + servers + drivers into a boot image
make hdboot       # installs the image to the hard disk boot sectors
```

Other useful targets (run `make` with no args in the relevant directory to see the full usage message):
- Top-level `Makefile`: `make world` (includes + libraries + commands + install), `make includes`, `make libraries`, `make cmds`, `make depend`, `make clean`.
- `tools/Makefile`: `make services` (kernel + servers + drivers only, no boot image), `make image`/`image_small`, `make fresh` (clean + rebuild libraries/services), `make bootable`, `make fdboot`.
- `kernel/Makefile`, `servers/Makefile`, `drivers/Makefile`, `lib/Makefile`: each supports `all`/`build`/`install`/`clean`/`depend` and recurses into its subdirectories (see each file for the exact list of subdirs it drives).

Build order matters: includes → libraries → kernel/servers/drivers → boot image, because the kernel and servers link against `lib/libc.a`, `lib/libsys.a`, `lib/libsysutil.a`, `lib/libtimers.a`.

### Tests

`test/` is the MINIX POSIX compliance test suite (`test/test1.c` … `test/test40.c`, plus `t10a`/`t11a`/`t11b` and `testsh1.sh`/`testsh2.sh`). Build with `cd test && make` (see `test/Makefile` for which tests need extra memory/root privileges — `BIGOBJ`/`ROOTOBJ`). Run the whole suite with `./run` from inside `test/` (must be run as an unprivileged, non-root-owned copy, as the runner refuses to run as root); it reports a pass/fail count and lists failing test numbers. Individual tests are plain executables (e.g. `./test14`) that can be run directly. `test/select/` has its own suite for `select()`.

## Architecture

MINIX is a microkernel: almost everything that would be kernel code in a monolithic OS instead runs as an unprivileged (or minimally privileged) user-space process, and all inter-process communication goes through the kernel's message-passing IPC. Understanding a feature almost always means tracing a message across 2-3 of these layers rather than reading one file.

### Process layout (`kernel/table.c`, `include/minix/com.h`)

The system image is a fixed, ordered list of processes baked into the boot image (`kernel/table.c: image[]`), matching the process-number constants in `include/minix/com.h`:

- **Kernel tasks** (run in kernel address space): `IDLE`, `CLOCK`, `SYSTEM`, `HARDWARE` (pseudo-process for interrupts).
- **Servers**: `pm` (process manager, `servers/pm/`), `fs` (file system, `servers/fs/`), `rs` (reincarnation server, `servers/rs/`), `is` (information server / debug dumps, `servers/is/`), `sm` (`servers/sm/`), `init` (`servers/init/`), `inet` (TCP/IP stack, `servers/inet/`).
- **Drivers** (`drivers/`): `tty`, `memory`, `log`, plus boot-medium drivers (`at_wini`, `bios_wini`, `floppy`) and others (`dpeth`/`dp8390`/`fxp`/`rtl8139`/`lance` for network cards, `printer`, `random`, `sb16`, `cmos`), built on the shared `drivers/libdriver/` framework.

Each image entry defines the process's privileges: which kernel traps it may use (`SENDREC`/`SEND`/`RECEIVE`/`ECHO`/`NOTIFY`), which other processes it's allowed to send to (an IPC bitmask), and which kernel calls it may invoke (e.g. `SYS_DEVIO`, `SYS_VIRCOPY`). Adding a new privileged operation to a server/driver usually means updating both the process's mask in `kernel/table.c` and the corresponding kernel-call handler.

### IPC (`include/minix/ipc.h`, `include/minix/com.h`, `kernel/proc.c`)

All communication is via a fixed-size `message` struct (a union of typed layouts `m1`..`m8` for different call shapes, see `include/minix/ipc.h`). The kernel implements the primitives in `kernel/proc.c: sys_call()` (cases `SENDREC`, `SEND`, `RECEIVE`, `NOTIFY`, `ECHO`). `include/minix/com.h` defines the notification types (`SYN_ALARM`, `SYS_SIG`, `HARD_INT`, etc.), the generic device-driver protocol (`DEV_OPEN`/`DEV_READ`/`DEV_WRITE`/`DEV_IOCTL`/... in the `DEV_RQ_BASE`/`DEV_RS_BASE` ranges), and server-specific message ranges.

### Kernel calls (`kernel/system.c`, `kernel/system/do_*.c`)

Privileged operations that servers/drivers need but can't do themselves (copying between address spaces, setting alarms, remapping memory, I/O port access, etc.) go through `kernel/system.c`'s dispatch table (`map(SYS_xxx, do_xxx)`), each implemented as its own file in `kernel/system/do_*.c`. `lib/syslib/sys_*.c` provides the C wrappers user-space code calls (e.g. `sys_vircopy()`, `sys_setalarm()`) which construct a message and trap into the kernel.

### System calls / libc (`lib/syscall/`, `lib/posix/`, `lib/other/syscall.c`)

POSIX calls like `read()`/`open()`/`fork()` are implemented in `lib/posix/_*.c`: each builds a `message` and calls `_syscall(FS or PM, CALL_NR, &m)` (`lib/other/syscall.c`), which does a `SENDREC` to the file system (`fs`) or process manager (`pm`) and translates a negative reply into `errno`. Call numbers are defined in `include/minix/callnr.h`; each server has its own dispatch table mapping them to handlers (e.g. `servers/fs/table.c: call_vec[]`, indexed by call number).

### File system (`servers/fs/`) and device mapping

`servers/fs/dmap.c` maps major device numbers to driver processes (`init_dmap[]`), determining which process handles I/O for each device node under `/dev`. Device drivers implement the common protocol from `drivers/libdriver/driver.c` (`driver_task()`), which handles `DEV_OPEN`/`DEV_CLOSE`/`DEV_READ`/`DEV_WRITE`/`DEV_GATHER`/`DEV_SCATTER`/`DEV_IOCTL`/`CANCEL` uniformly; device-specific drivers plug in read/write/ioctl callbacks.

### Reincarnation server (`servers/rs/`)

`rs` starts, stops, and restarts system services, driven by the `service` utility (`servers/rs/service.c`) which sends requests to `rs` rather than doing the work itself.

### Coding conventions to preserve

- `PUBLIC` = nothing (extern, opposite of `PRIVATE`), `PRIVATE` = `static`, `FORWARD` = `static` (forward declarations), `EXTERN` = `extern` — defined in `include/minix/const.h`. Used throughout the kernel/servers/drivers instead of raw C keywords.
- Function prototypes use the `_PROTOTYPE(fn, (args))` macro for K&R/ANSI portability, and many function definitions still use old-style K&R parameter declarations.
- Each server/driver directory's `Makefile` follows a common recursive pattern (`build`/`image`/`install`/`clean`/`depend` targets cd-ing into subdirectories); check the sibling `Makefile` for the exact recipe when adding a new component.

## Rust port (`rust/`)

Unlike the C tree, this builds and boots directly on a modern Linux host — no MINIX guest needed. It's a bare-metal `no_std` kernel (`rust/kernel/`) using the `bootloader` 0.9 crate to produce a bootable BIOS disk image, run under QEMU. It has exception handling, a priority-queue scheduler with real hardware-timer-driven asynchronous preemption, blocking rendezvous IPC, a heap allocator, a real ring-3 (user-mode) task with its own dedicated `RSP0` *and* its own address space (a private page table, confirmed absent from the kernel's own via an isolation self-test) that can be asynchronously preempted and trap in and out of the kernel repeatedly, and its first real kernel calls: `sys_vircopy` (a genuine cross-address-space copy), `sys_setalarm` (waking a blocked task via a real notification), and `sys_fork` (creates a real child process with a *deep-copied*, independent address space — verified by writing a canary into the child's copy and confirming the parent's is untouched). `fs` is now a real in-memory file server too: genuine open/write/read request-reply handling over IPC (backed by heap-allocated file storage, not a fixed-reply stand-in), exercised by `pm` opening a file, writing to it, reopening it fresh, and reading the bytes back — plus a real (if shallow) directory hierarchy on top (`mkdir`, and `open` enforcing `ENOENT`/`ENOTDIR`/`EISDIR`/`EEXIST` against a path's parent, not just an exact-match flat namespace). There's also a minimal ELF64 loader (`rust/kernel/src/elf.rs`): a second ring-3 task (`tty`) now runs a real, statically linked ELF binary (its `PT_LOAD` segments parsed and mapped at their own addresses/permissions, not hand-placed bytes), verified by reading its `.data` counter back via `sys_vircopy` after it's done. Both ring-3 tasks now speak a real syscall ABI too (`rust/kernel/src/syscall.rs`): `int 0x80` is a genuine call-number/register-dispatched gate (call number in `rax`, arguments in `rdi`/`rsi`, a hand-written naked trap frame saving every other register) rather than a fixed action — `SYS_GET_UPTIME`, `SYS_WRITE_LINE` (a real cross-ring pointer argument, safe to read directly since entering a trap gate never switches `CR3`), `SYS_SET_ALARM`/`SYS_WAIT_ALARM` (the first of `crate::calls`' own kernel calls reachable from ring 3 — `tty` genuinely blocks waiting for a real alarm and resumes in ring 3 once it fires, with the rest of the system running normally the whole time it's blocked), `SYS_FS_OPEN`/`SYS_FS_WRITE`/`SYS_FS_READ` (a real IPC round trip to `fs` — `tty` opens and writes a real file from ring 3, verified by `IDLE` reading it back afterward through a completely independent, kernel-side path — copying buffers through a kernel-stack buffer around the IPC call, since `fs` may see a different `CR3` by the time it dereferences anything), and `SYS_BLOCK_FOREVER` (which both tasks use to end themselves, instead of a kernel-side iteration counter cutting them off). The Rust port also now has real graphics (`rust/kernel/src/vga.rs`): VGA mode 13h (320x200, 256-color), a palette programmed over the VGA DAC ports, and `fill_rect`/`fill_rounded_rect` drawing primitives painting a static, LCARS-style demo panel (flat-colored bars/buttons with genuinely rounded corners, no text) into the real linear framebuffer at boot — verified pixel-exact via a QEMU QMP `screendump` (see `rust/README.md`'s "Running" section for the exact command and the 2x pixel-doubling caveat). There's also a real PS/2 keyboard driver (`rust/kernel/src/keyboard.rs`): IRQ1 unmasked, scancodes read off port `0x60` and translated to ASCII, verified asynchronous and hardware-driven via QEMU's QMP `send-key` command (works even after `IDLE` has already halted — the keypress wakes the CPU out of `hlt`). The two are now connected: pressing digit keys `1`-`4` calls `vga::select_button`, which redraws the panel with a white highlight frame around the matching button — a real (if minimal) input-to-output loop, verified via `send-key` followed by a `screendump` (note the "Running" section's timing caveat — wait briefly between the two QMP commands, since `send-key` returns before the guest has necessarily processed the interrupt). Keyboard input now also drives a real line discipline: every translated character feeds a shared line buffer, and a newline notifies `console_task` — a real, separately scheduled process (`com::CONSOLE_PROC_NR`), not a direct function call — which writes the completed line to a real file via `fs`. Verified interactively well after boot (with `IDLE` already halted): typing `"hi"` then `"world"` (each terminated with Enter) over QMP `send-key` produces two distinct `[console] received line: ...` entries. The syscall ABI's `SYS_READ_LINE` closes the loop back to ring 3: `tty` now blocks *inside its own trap* for a real, human-timed line of keyboard input (`console_task` delivers it straight into a kernel-stack buffer once one arrives), then writes what it received to a second real file — verified by typing `"neumann"` over QMP well after boot and seeing `tty` write it to `/from_console.txt`. `rs` is now a real reincarnation server too (`rust/kernel/src/rs.rs`): CPU exception handlers (`rust/kernel/src/interrupts.rs`'s `recover_or_halt`) now distinguish a fault in a *ring-3* task (killed via `crate::proc::kill`, which notifies `RS`) from one in kernel-trusted code (still halts the whole machine, as before) — `flaky`, a ring-3 task whose only instruction is `ud2` (guaranteed `#UD`), crashes deterministically every time it runs, and `rs` restarts it up to 3 times before giving up, with the rest of the system (both ring-3 tasks' real syscalls, `fs`, preemption) completely unaffected by the repeated crashes. `fork` and `exec` are both real and both reachable from ring 3 now: `SYS_FORK` (`rust/kernel/src/proc.rs`'s `TrapFrame`/`fork_current`) gives the child the parent's exact trapped instruction with `rax` reading `0`, and `SYS_EXEC` (`rust/kernel/src/calls.rs`'s `sys_exec`) goes the other way -- `shell`, a third ring-3 task, names `/bin/echo` by path, and the very trap it made returns into a *different binary*: loaded out of `fs`, validated, mapped into a brand-new address space derived from the kernel's (not the caller's), and resumed at its own entry point on its own fresh stack, with the process's number, priority, kernel stack and open descriptors carrying through. Verified from outside the process: the new image's `.data` counter reads back through the *caller's* process slot, while the old image's `.data` page is no longer mapped there at all (`sys_vircopy` must fail), plus a deliberately-failed exec of a non-ELF file proving a failed exec leaves the caller running exactly as it was, and `crate::main`'s `missing_program_check` confirming a missing path reports `ENOENT` without creating anything. Note the load-bearing safety check in `crate::elf::validate` is `memory::pml4_slots_unused` (every segment must land in a PML4 slot the base address space doesn't already use), NOT an address bound -- this port has no user/kernel address boundary, since the kernel image, heap (`allocator::HEAP_START`) and physical-memory window all sit in the lower half; a new address space copies only the top-level table, so a segment in an already-used slot edits the kernel's own page tables. `crate::elf::validator_self_test` runs nine hostile images through the loader at boot and requires each to be rejected with the right error, and `user/echo.s` additionally attempts the kernel-heap exec *from ring 3* so the block is proven against a real unprivileged process, not just a kernel-side call. See `rust/README.md` for the full architecture mapping (which MINIX C files each Rust module ports) and the roadmap of what's still missing (`exec` without `argv`/`envp`, and no freeing of the image it replaces -- this port has no frame deallocator at all; `SYS_FORK` still hardcoded to one known caller's private-page list, for want of a per-process memory map; `fs` growing `readdir` and a real backing store of its own (still no cross-address-space copy inside `fs` itself — only the syscall layer works around that), a real windowing layer and a real `tty`/echo/editing on top of the new line discipline (only one pending `SYS_READ_LINE` reader tracked at a time), beyond one static panel, four selectable buttons, and one write-only console log).

Long-term direction beyond the current MINIX-fidelity roadmap: the user's stated goal is a BeOS/Haiku-flavored "perfect desktop OS" (pervasive multithreading, a modern non-POSIX-first API, snappy desktop UX, Star-Trek-LCARS-inspired operating panels) once the MINIX-parity phase is done — see the `neumannos-rust-goal` memory for the full framing. Don't let this override explicit sequencing requests turn to turn, but keep it in view when proposing what's next.

```
rustup toolchain install nightly
rustup component add rust-src llvm-tools-preview --toolchain nightly
cargo install bootimage
cd rust/kernel
rustup override set nightly     # once per checkout
cargo bootimage                 # builds + wraps the kernel in a bootable image
qemu-system-x86_64 -drive format=raw,file=../target/x86_64-unknown-none/debug/bootimage-neumannos-kernel.bin -serial stdio
```

Two non-obvious build requirements, both already set in `kernel/.cargo/config.toml`:
- Needs nightly + `-Z build-std` because it targets bare metal (`x86_64-unknown-none`), which has no prebuilt `core`/`alloc`.
- `rustflags = ["-C", "relocation-model=static"]` is required — the builtin target defaults to a position-independent executable, which the `bootloader` 0.9 crate's ELF loader doesn't relocate, so the kernel silently jumps into garbage on boot without this flag.

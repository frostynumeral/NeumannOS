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

Unlike the C tree, this builds and boots directly on a modern Linux host — no MINIX guest needed. It's a bare-metal `no_std` kernel (`rust/kernel/`) using the `bootloader` 0.9 crate to produce a bootable BIOS disk image, run under QEMU. It has exception handling, a priority-queue scheduler with real hardware-timer-driven asynchronous preemption, blocking rendezvous IPC, a heap allocator, a real ring-3 (user-mode) task with its own dedicated `RSP0` *and* its own address space (a private page table, confirmed absent from the kernel's own via an isolation self-test) that can be asynchronously preempted and trap in and out of the kernel repeatedly, and its first two real kernel calls (`sys_vircopy`, a genuine cross-address-space copy; `sys_setalarm`, waking a blocked task via a real notification) — see `rust/README.md` for the full architecture mapping (which MINIX C files each Rust module ports) and the roadmap of what's still missing (a real kernel-call dispatch mechanism, the real servers, extending per-process address spaces beyond the one demo task).

Long-term direction beyond the current MINIX-fidelity roadmap: the user's stated goal is a BeOS-flavored "perfect desktop OS" (pervasive multithreading, a modern non-POSIX-first API, snappy desktop UX) once the MINIX-parity phase is done — see the `neumannos-rust-goal` memory for the full framing. Don't let this override explicit sequencing requests turn to turn, but keep it in view when proposing what's next.

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

# NeumannOS (Rust port)

This directory holds a from-scratch Rust rewrite of the MINIX 3.1.0 sources
in the rest of this repository (see the top-level `README.md` for the
original C codebase's history). It is a **starting skeleton**, not a
finished port: a real microkernel, its servers (`pm`, `fs`, `rs`, ...), and
its drivers are a multi-month undertaking on their own. What's here boots
in QEMU, prints the boot image, and exercises message-passing IPC — enough
to build the rest of the system on top of.

## What's implemented

- `src/com.rs` — process numbers and notification types, ported from
  `include/minix/com.h`.
- `src/table.rs` — the boot image (fixed process list), ported from
  `kernel/table.c`'s `image[]`.
- `src/ipc.rs` — `Message` (ported from the `mess_*`/`message` union in
  `include/minix/ipc.h`) plus `send`/`receive`/`notify`, a non-blocking
  stand-in for MINIX's `SEND`/`RECEIVE`/`NOTIFY` kernel calls.
- `src/serial.rs` + `src/main.rs` — boot entry point (via the `bootloader`
  crate), which prints the boot image table over COM1 and runs a
  send/receive/notify self-test.

## What's not implemented yet (roadmap)

Roughly in the order the original kernel needs them:

1. **GDT/IDT and exception handling** — currently any CPU fault triple-faults
   silently. Needed before anything else can be trusted.
2. **A real scheduler and process table** (`kernel/proc.c`, `kernel/proto.h`)
   — actual context switching between processes, not just an in-kernel
   mailbox array.
3. **User-mode processes and address-space isolation** — right now
   everything (including "servers") runs in kernel context. Blocking
   `send`/`receive` (parking a process until a partner is ready) only makes
   sense once there are processes to park.
4. **Kernel calls** (`kernel/system.c`, `kernel/system/do_*.c`) — the
   privileged operations servers need (`sys_vircopy`, `sys_setalarm`, etc.).
5. **The servers themselves**: `pm` (process manager), `fs` (file system),
   `rs` (reincarnation server), `tty`, `memory`, in roughly that dependency
   order, matching `servers/` and `drivers/` in the C tree.
6. **A libc-equivalent** for whatever runs in user mode, mirroring `lib/`.

## Building

Requires a nightly Rust toolchain (for `-Z build-std`, needed because this
targets bare metal and has no prebuilt `core`/`alloc` for it) and the
`bootimage` cargo subcommand:

```
rustup toolchain install nightly
rustup component add rust-src llvm-tools-preview --toolchain nightly
cargo install bootimage
```

Then, from `rust/kernel/`:

```
rustup override set nightly    # only needed once per checkout
cargo build                    # compiles the kernel binary
cargo bootimage                # wraps it in a bootable BIOS disk image
```

This produces
`rust/target/x86_64-unknown-none/debug/bootimage-neumannos-kernel.bin`.

The target (`x86_64-unknown-none`) and build flags are pinned in
`kernel/.cargo/config.toml`. Notably, `rustflags` forces
`relocation-model=static`: the builtin target defaults to a
position-independent executable, which the `bootloader` 0.9 crate's simple
ELF loader does not relocate, causing the kernel to jump into garbage on
boot if left at the default.

## Running

```
qemu-system-x86_64 -drive format=raw,file=target/x86_64-unknown-none/debug/bootimage-neumannos-kernel.bin -serial stdio
```

(or `cargo run`, which invokes `bootimage runner` per `.cargo/config.toml`
and does the same thing). Expected output on COM1: the boot image table,
followed by the IPC self-test results, then a halt message.

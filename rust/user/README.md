# NeumannOS ring-3 programs (Rust)

The programs that run in user mode on the Rust kernel (`../kernel/`), and
`neumann_rt`, the small runtime they're written on -- this port's first
"libc-equivalent" (roadmap item 12 in `../README.md`).

- `src/lib.rs` -- `neumann_rt`: `_start` (hands `main` the `argc`/`argv`/
  `envp` block `exec` put on the stack, turns `main`'s return into
  `exit`), `Args`, `print!`/`println!` over `SYS_CONSOLE_WRITE`, a panic
  handler (status 101).
- `src/sys.rs` -- one wrapper per system call; the kernel's
  `src/syscall.rs` is the authoritative list.
- `src/bin/sh.rs` -- the shell: reads a line, `fork`s, the child `exec`s
  `/bin/<word>` with the words as `argv`, the parent `wait`s.
- `src/bin/echo.rs`, `src/bin/cat.rs` -- what it runs.

## Building

```
./install.sh
```

builds everything (`cargo +nightly build --release`) and copies the ELFs
to `../kernel/user/bin/`, where the kernel embeds them and `pm` installs
them in `/bin` at boot. The copies are checked in, like the hand-assembled
programs beside them, so the kernel builds without this step; rerun it
after changing anything here, then rebuild the kernel.

This crate is excluded from the kernel's workspace (`../Cargo.toml`)
because it needs a different build configuration (`.cargo/config.toml`):

- `relocation-model=static` and `link.ld`, which links every program at
  `0x3000_0000_0000` -- PML4 slot 96, one the kernel's own address space
  leaves empty, which is what the kernel's ELF loader requires. Each
  program has its own address space, so they can all share it.
- `code-model=large`: the default code model assumes everything lives in
  the low 2 GiB, which here is PML4 slot 0 -- the kernel's.

There's no heap: no allocator, because there's no `brk`/`mmap` to back
one yet.

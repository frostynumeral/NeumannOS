#!/bin/sh
# Build every ring-3 program in this crate and copy the ELFs to
# ../kernel/user/bin/, where the kernel `include_bytes!`s them (see
# crate::elf's RUST_PROGRAMS) and `pm` installs them into `fs` at /bin at
# boot. The copies are checked in, like the hand-assembled programs next
# to them, so building the kernel doesn't require building this first.
set -e
cd "$(dirname "$0")"
cargo +nightly build --release
mkdir -p ../kernel/user/bin
for prog in sh echo cat ptrtest ls mkdir threads heaptest; do
    cp "target/x86_64-unknown-none/release/$prog" "../kernel/user/bin/$prog"
done
ls -l ../kernel/user/bin

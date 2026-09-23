// The linker script is passed through `.cargo/config.toml`'s rustflags,
// which Cargo doesn't treat as an input -- without this, editing
// `link.ld` would silently leave the old layout in place.
fn main() {
    println!("cargo:rerun-if-changed=link.ld");
}

//! `sh`: the NeumannOS shell. Reads a line from the keyboard, splits it
//! into words, and runs it: `fork`, then the child `exec`s the program
//! with those words as its `argv`, and the parent `wait`s for it and
//! reports a non-zero exit status. A command with no `/` in it is looked
//! up in `/bin`, the one directory on this system's `PATH`.
//!
//! Built-ins, because they have to change the shell itself rather than
//! a child: `exit [status]`, and `help`.
//!
//! What it doesn't do yet, and a real shell does: quoting, pipes,
//! redirection, variables, job control, and anything that needs a
//! current directory (there's no `chdir` in this system).

#![no_std]
#![no_main]

use neumann_rt::{print, println, sys, Args, Bytes};

neumann_rt::main!(main);

/// Most words in one command line (`argv[0]` included).
const MAX_WORDS: usize = 16;

fn main(args: Args) -> i32 {
    // The environment every command runs with: this shell's own, passed
    // through unchanged, the way a real shell exports what it inherited.
    // Built as C wants it -- a NULL-terminated array of pointers to
    // NUL-terminated strings -- which `exec` reads straight out of this
    // process's memory. The start-up block the kernel gave *us* is
    // already in exactly that form, so its pointers are reused as they are.
    let mut envp = [core::ptr::null::<u8>(); MAX_WORDS + 1];
    for (slot, var) in envp.iter_mut().zip(args.env().take(MAX_WORDS)) {
        *slot = var.as_ptr();
    }

    // Two short lines: the on-screen console is 24 columns wide.
    println!("NeumannOS sh");
    println!("type `help` for help");
    let mut line = [0u8; sys::MAX_IO];
    loop {
        print!("$ ");
        // No echo here: the kernel's line discipline echoes each key as
        // it's typed (`crate::keyboard::on_char`), Enter included.
        let n = sys::read_line(&mut line);
        if let Some(status) = run_line(&line[..n], &envp) {
            return status;
        }
    }
}

/// Run one command line. `Some(status)` means the shell itself should
/// exit with that status (`exit`).
fn run_line(line: &[u8], envp: &[*const u8]) -> Option<i32> {
    // Split into words, NUL-terminating each one in a private copy so
    // `argv` can point straight into it.
    let mut storage = [0u8; sys::MAX_IO + MAX_WORDS];
    let mut words: [&[u8]; MAX_WORDS] = [&[]; MAX_WORDS];
    let mut argv = [core::ptr::null::<u8>(); MAX_WORDS + 1];
    let mut count = 0;
    let mut used = 0;
    for word in line.split(|&b| b == b' ' || b == b'\t').filter(|w| !w.is_empty()) {
        if count == MAX_WORDS {
            println!("sh: too many words (at most {})", MAX_WORDS);
            return None;
        }
        storage[used..used + word.len()].copy_from_slice(word);
        storage[used + word.len()] = 0;
        count += 1;
        used += word.len() + 1;
    }
    let mut at = 0;
    for i in 0..count {
        let len = storage[at..].iter().position(|&b| b == 0).unwrap_or(0);
        words[i] = &storage[at..at + len];
        argv[i] = storage[at..].as_ptr();
        at += len + 1;
    }
    if count == 0 {
        return None;
    }

    match words[0] {
        b"exit" => {
            let status = words.get(1).filter(|_| count > 1).and_then(|w| parse_i32(w)).unwrap_or(0);
            return Some(status);
        }
        b"help" => {
            // Kept to the on-screen console's 24 columns.
            println!("programs in /bin:");
            println!(" echo WORDS...");
            println!(" cat FILE...");
            println!(" ls [DIR...]");
            println!(" mkdir DIR...");
            println!("built in: exit, help");
            println!("try: ls /bin");
            return None;
        }
        _ => {}
    }

    // `PATH` is `/bin`, and only `/bin`. The kernel takes paths of at
    // most `MAX_IO` bytes; say so here rather than let it come back as a
    // bare length error.
    if words[0].len() + 5 > sys::MAX_IO {
        println!("sh: {}: name too long", Bytes(words[0]));
        return None;
    }
    let mut path_buf = [0u8; 5 + sys::MAX_IO];
    let path: &[u8] = if words[0].contains(&b'/') {
        words[0]
    } else {
        path_buf[..5].copy_from_slice(b"/bin/");
        path_buf[5..5 + words[0].len()].copy_from_slice(words[0]);
        &path_buf[..5 + words[0].len()]
    };

    match sys::fork() {
        Err(code) => {
            println!("sh: fork: {}", sys::strerror(code));
            None
        }
        Ok(0) => {
            // The child: become the command. `exec` only comes back if it
            // failed, and then the child has nothing left to do but say so
            // and exit -- 127 is the status shells use for "command not
            // found".
            let code = unsafe { sys::exec(path, argv.as_ptr(), envp.as_ptr()) };
            println!("sh: {}: {}", Bytes(words[0]), sys::strerror(code));
            sys::exit(127)
        }
        Ok(_child) => {
            match sys::wait() {
                Ok((_, 0)) => {}
                Ok((_, status)) => println!("[exit {}]", status),
                Err(code) => println!("sh: wait failed: {}", sys::strerror(code)),
            }
            None
        }
    }
}

fn parse_i32(word: &[u8]) -> Option<i32> {
    let (negative, digits) = match word.split_first() {
        Some((b'-', rest)) => (true, rest),
        _ => (false, word),
    };
    if digits.is_empty() {
        return None;
    }
    let mut value: i32 = 0;
    for &d in digits {
        if !d.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?.checked_add((d - b'0') as i32)?;
    }
    Some(if negative { -value } else { value })
}

//! The on-screen text console: a grid of character cells drawn with
//! `crate::font` into the LCARS panel's open area (right of the purple
//! descender, below the orange sweep, left of the buttons), so the shell
//! and the programs it runs can be *seen*, not just read off COM1.
//!
//! The standard terminal behaviors, and nothing more: printable
//! characters advance the cursor and wrap at the right edge, `\n` starts
//! a new line, backspace (`0x08`) steps back and erases, and running off
//! the bottom scrolls everything up a row. Writes redraw only the cells
//! they change; a scroll repaints the whole area. `crate::vga`'s
//! `draw_demo_panel` calls `redraw` too, so repainting the panel (a
//! button press does) doesn't wipe the text.
//!
//! Written to from two places: `crate::syscall`'s `SYS_CONSOLE_WRITE`
//! (a program's standard output) and `crate::keyboard`'s echo of each
//! key as it's typed. The grid lives behind a spin lock taken only with
//! interrupts off -- the keyboard IRQ is one of the writers.

use crate::{font, vga};
use spin::Mutex;
use x86_64::instructions::interrupts::without_interrupts;

pub const COLS: usize = 24;
pub const ROWS: usize = 18;
const CELL: usize = 8;
/// Top-left pixel of the console area, chosen to sit inside the panel's
/// empty region (see `vga::draw_demo_panel`'s layout).
const ORIGIN_X: usize = 48;
const ORIGIN_Y: usize = 50;
const FG: u8 = vga::palette::TAN;
const BG: u8 = vga::palette::BLACK;

struct Grid {
    cells: [[u8; COLS]; ROWS],
    row: usize,
    col: usize,
}

static GRID: Mutex<Grid> = Mutex::new(Grid { cells: [[b' '; COLS]; ROWS], row: 0, col: 0 });

fn framebuffer() -> *mut u8 {
    vga::framebuffer(crate::memory::physical_memory_offset())
}

fn draw_cell(fb: *mut u8, row: usize, col: usize, c: u8) {
    let bitmap = font::glyph(c);
    let x0 = ORIGIN_X + col * CELL;
    let y0 = ORIGIN_Y + row * CELL;
    for (dy, bits) in bitmap.iter().enumerate() {
        for dx in 0..CELL {
            let lit = bits & (1 << dx) != 0;
            vga::put_pixel(fb, x0 + dx, y0 + dy, if lit { FG } else { BG });
        }
    }
}

fn draw_all(fb: *mut u8, grid: &Grid) {
    for (r, line) in grid.cells.iter().enumerate() {
        for (c, &ch) in line.iter().enumerate() {
            draw_cell(fb, r, c, ch);
        }
    }
}

impl Grid {
    /// Move to the start of the next line, scrolling if that's past the
    /// bottom. Returns whether it scrolled (so the caller repaints all).
    fn newline(&mut self) -> bool {
        self.col = 0;
        if self.row + 1 < ROWS {
            self.row += 1;
            return false;
        }
        self.cells.copy_within(1.., 0);
        self.cells[ROWS - 1] = [b' '; COLS];
        true
    }

    /// Apply one byte; returns whether the whole grid needs repainting.
    fn put(&mut self, fb: *mut u8, byte: u8) -> bool {
        match byte {
            b'\n' => self.newline(),
            b'\r' => {
                self.col = 0;
                false
            }
            0x08 => {
                // Wrapping is deferred (the cursor waits at `COLS` until
                // the next character), so a line that crossed the right
                // edge continues at column 0 of the next row -- and
                // backspacing through it has to step back up onto the row
                // above, or the screen stops matching the line being typed.
                if self.col > 0 {
                    self.col -= 1;
                } else if self.row > 0 {
                    self.row -= 1;
                    self.col = COLS - 1;
                } else {
                    return false;
                }
                self.cells[self.row][self.col] = b' ';
                draw_cell(fb, self.row, self.col, b' ');
                false
            }
            _ => {
                let mut scrolled = false;
                if self.col == COLS {
                    scrolled = self.newline();
                }
                let c = if (font::FIRST..=font::LAST).contains(&byte) { byte } else { b'?' };
                self.cells[self.row][self.col] = c;
                if !scrolled {
                    draw_cell(fb, self.row, self.col, c);
                }
                self.col += 1;
                scrolled
            }
        }
    }
}

/// Write `bytes` to the screen.
pub fn write(bytes: &[u8]) {
    without_interrupts(|| {
        let fb = framebuffer();
        let mut grid = GRID.lock();
        let mut repaint = false;
        for &b in bytes {
            repaint |= grid.put(fb, b);
        }
        if repaint {
            draw_all(fb, &grid);
        }
    });
}

/// Repaint every cell -- for `vga::draw_demo_panel`, which clears the
/// screen first.
pub fn redraw(fb: *mut u8) {
    without_interrupts(|| draw_all(fb, &GRID.lock()));
}

/// Blank the grid and home the cursor (for `self_test`, which leaves the
/// console as it found it: empty).
pub fn reset() {
    without_interrupts(|| {
        let mut grid = GRID.lock();
        grid.cells = [[b' '; COLS]; ROWS];
        grid.row = 0;
        grid.col = 0;
        draw_all(framebuffer(), &grid);
    });
}

/// Boot-time check of the terminal behaviors, against the character grid
/// rather than pixels: backspace erases, a line longer than `COLS` wraps,
/// and running off the bottom scrolls (the first line gone, the last
/// line blank). Runs before anything else writes to the console, and
/// resets it afterwards.
pub fn self_test() {
    write(b"abx\x08c\n");
    let long = [b'x'; COLS + 3];
    write(&long);
    let grid = snapshot();
    assert_eq!(&grid[0][..4], b"abc ", "backspace didn't erase");
    assert_eq!(grid[1], [b'x'; COLS], "a long line didn't fill its row");
    assert_eq!(&grid[2][..4], b"xxx ", "a long line didn't wrap");
    // Backspacing through the wrap: three from row 2 back to its start,
    // then one more must land on the last cell of row 1.
    write(b"\x08\x08\x08\x08");
    let grid = snapshot();
    assert_eq!(&grid[2][..3], b"   ", "backspace didn't erase the wrapped part");
    assert_eq!(grid[1][COLS - 1], b' ', "backspace didn't step back across the wrap");
    assert_eq!(grid[1][COLS - 2], b'x', "backspace across the wrap erased too much");
    // Scrolling must *shift*, not wipe: label every line, write two more
    // than fit. Each line's own newline moves the cursor down, so the
    // last three newlines each scroll once, and the first surviving row
    // is the fourth line written ('d'), the last line written ('t') sits
    // just above a blank bottom row.
    reset();
    for i in 0..ROWS + 2 {
        write(&[b'a' + i as u8, b'\n']);
    }
    let grid = snapshot();
    assert_eq!(grid[0][0], b'd', "scrolling didn't move rows up");
    assert_eq!(grid[ROWS - 2][0], b'a' + (ROWS + 1) as u8, "the last line written isn't where it should be");
    assert_eq!(grid[ROWS - 1][0], b' ', "the new bottom row isn't blank");
    reset();
    crate::serial_println!(
        "[console] self-test: backspace (across a wrap too), wrap and scroll behave ({}x{} cells)",
        COLS,
        ROWS
    );
}

/// The characters currently on screen, row by row -- for a self-test to
/// compare against, without reading pixels back.
pub fn snapshot() -> [[u8; COLS]; ROWS] {
    without_interrupts(|| GRID.lock().cells)
}

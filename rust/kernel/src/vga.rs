//! VGA mode 13h (320x200, 256-color) graphics: palette programming and a
//! handful of flat-shaded drawing primitives, enough to paint a static,
//! LCARS-flavored panel -- rectangular color blocks with rounded corners,
//! no text/font rendering.
//!
//! No MINIX C equivalent (2005-era MINIX has no graphics stack at all --
//! see `rust/README.md`'s long-term-direction note on the BeOS/Haiku-
//! flavored desktop-OS goal this is a first step toward, not something
//! being ported). Mode 13h itself is set by the `bootloader` crate's
//! `vga_320x200` feature (a real-mode `int 0x10` call with `AL=0x13`,
//! done before the jump to long mode -- see that crate's
//! `src/video_mode/vga_320x200.s`), not by this module; this module only
//! programs the color palette and writes pixels into the resulting
//! linear framebuffer.

use spin::Mutex;
use x86_64::instructions::port::Port;
use x86_64::VirtAddr;

pub const WIDTH: usize = 320;
pub const HEIGHT: usize = 200;

/// Mode 13h's linear framebuffer is memory-mapped at physical `0xA0000`
/// (one byte per pixel: an index into the 256-entry palette
/// `set_palette_color` programs, row-major, `WIDTH` bytes per row) --
/// standard VGA, unrelated to the `bootloader` crate's own real-mode
/// setup. Reached the same way `crate::calls`/`crate::memory` reach any
/// other physical address: through the physical-memory offset window
/// `map_physical_memory` mapped for the *entire* physical address range
/// (MMIO holes included, not just RAM-typed regions), so no separate
/// mapping is needed here.
const FRAMEBUFFER_PHYS_ADDR: u64 = 0xA0000;

/// Returns a pointer to the mode 13h framebuffer, `WIDTH * HEIGHT` bytes,
/// one palette index per pixel. `physical_memory_offset` is the same
/// value `crate::memory::init`/`init_frame_allocator` were given from
/// `BootInfo`.
pub fn framebuffer(physical_memory_offset: VirtAddr) -> *mut u8 {
    (physical_memory_offset + FRAMEBUFFER_PHYS_ADDR).as_mut_ptr::<u8>()
}

/// Program palette entry `index`'s color via the VGA DAC registers
/// (`0x3C8` selects the index to write, `0x3C9` then takes three
/// sequential 6-bit -- not 8-bit -- R/G/B writes). Real hardware
/// (and QEMU's standard VGA emulation) both support this regardless of
/// mode; nothing about it is mode-13h-specific, but this port has no
/// other use for palette control yet.
pub fn set_palette_color(index: u8, r6: u8, g6: u8, b6: u8) {
    let mut dac_index: Port<u8> = Port::new(0x3C8);
    let mut dac_data: Port<u8> = Port::new(0x3C9);
    unsafe {
        dac_index.write(index);
        dac_data.write(r6);
        dac_data.write(g6);
        dac_data.write(b6);
    }
}

/// LCARS-style palette, chosen to evoke the familiar orange/purple/red-
/// alert/blue/tan panel scheme. VGA's DAC takes 6 bits per channel
/// (0-63), not the usual 8 (0-255); each constant here is `round(c *
/// 63 / 255)` of the 8-bit color named in its doc comment.
pub mod palette {
    pub const BLACK: u8 = 0;
    pub const ORANGE: u8 = 1;
    pub const PURPLE: u8 = 2;
    pub const RED_ALERT: u8 = 3;
    pub const BLUE: u8 = 4;
    pub const TAN: u8 = 5;
    /// Used only for the selection frame `draw_demo_panel` paints behind
    /// whichever button `select_button` last picked -- not one of the
    /// panel's own resting colors.
    pub const HIGHLIGHT: u8 = 6;

    /// `(index, r6, g6, b6, "approximate 8-bit RGB")` for `init`.
    pub const ENTRIES: [(u8, u8, u8, u8); 7] = [
        (BLACK, 0, 0, 0),        // (0, 0, 0)
        (ORANGE, 63, 38, 0),     // (255, 153, 0)
        (PURPLE, 38, 25, 50),    // (153, 102, 204)
        (RED_ALERT, 50, 25, 25), // (204, 102, 102)
        (BLUE, 25, 38, 50),      // (102, 153, 204)
        (TAN, 63, 50, 38),       // (255, 204, 153)
        (HIGHLIGHT, 63, 63, 63), // (255, 255, 255)
    ];
}

/// Program every entry in `palette::ENTRIES`. Must run once before
/// drawing anything meant to show one of those colors; the DAC's default
/// power-on palette (or whatever the bootloader's own `vga_320x200`
/// printer left behind) doesn't match them.
pub fn init_palette() {
    for &(index, r, g, b) in palette::ENTRIES.iter() {
        set_palette_color(index, r, g, b);
    }
}

fn put_pixel(fb: *mut u8, x: usize, y: usize, color: u8) {
    if x < WIDTH && y < HEIGHT {
        unsafe { fb.add(y * WIDTH + x).write_volatile(color) };
    }
}

/// Flat-fill an unrounded `w`x`h` rectangle at `(x, y)`. The building
/// block every other shape here is built from.
pub fn fill_rect(fb: *mut u8, x: usize, y: usize, w: usize, h: usize, color: u8) {
    for row in 0..h {
        for col in 0..w {
            put_pixel(fb, x + col, y + row, color);
        }
    }
}

/// Which of a rectangle's four corners `fill_rounded_rect` should round.
#[derive(Clone, Copy)]
pub struct Corners {
    pub top_left: bool,
    pub top_right: bool,
    pub bottom_left: bool,
    pub bottom_right: bool,
}

impl Corners {
    pub const NONE: Corners =
        Corners { top_left: false, top_right: false, bottom_left: false, bottom_right: false };
    pub const ALL: Corners =
        Corners { top_left: true, top_right: true, bottom_left: true, bottom_right: true };
}

/// Fill a `w`x`h` rectangle at `(x, y)`, rounding whichever corners
/// `corners` selects to radius `r` -- the LCARS-panel-defining shape:
/// every bar and button in `draw_demo_panel` is one of these, not a
/// plain rectangle. Standard rounded-rectangle construction: a pixel in
/// one of the four `r`x`r` corner boxes is only painted if it falls
/// within a quarter circle of radius `r` centered at that corner box's
/// inner point; every other pixel (including corner boxes for corners
/// `corners` doesn't select) is painted unconditionally.
pub fn fill_rounded_rect(
    fb: *mut u8,
    x: usize,
    y: usize,
    w: usize,
    h: usize,
    r: usize,
    corners: Corners,
    color: u8,
) {
    for row in 0..h {
        for col in 0..w {
            let left = col < r;
            let right = w >= r && col >= w - r;
            let top = row < r;
            let bottom = h >= r && row >= h - r;

            let (rounded, center_col, center_row) = if corners.top_left && left && top {
                (true, r, r)
            } else if corners.top_right && right && top {
                (true, w - r, r)
            } else if corners.bottom_left && left && bottom {
                (true, r, h - r)
            } else if corners.bottom_right && right && bottom {
                (true, w - r, h - r)
            } else {
                (false, 0, 0)
            };

            let visible = if rounded {
                let dx = col as i64 - center_col as i64;
                let dy = row as i64 - center_row as i64;
                dx * dx + dy * dy <= (r * r) as i64
            } else {
                true
            };

            if visible {
                put_pixel(fb, x + col, y + row, color);
            }
        }
    }
}

/// Paint the demo panel this port's milestone calls for: flat-colored
/// rectangular blocks with rounded-corner elements, no text. A stand-in
/// "LCARS" layout -- an orange sweep bar with a rounded left end, a
/// purple descender bar below it with a rounded bottom end (together
/// forming an L, the classic LCARS silhouette), and a column of smaller,
/// fully-rounded buttons along the right edge (see `BUTTON_COLORS` and
/// friends). Whichever button `select_button` last picked (if any) gets
/// a highlighted frame drawn behind it -- the panel's one piece of live
/// state, driven by `crate::keyboard`'s digit keys.
pub fn draw_demo_panel(fb: *mut u8) {
    fill_rect(fb, 0, 0, WIDTH, HEIGHT, palette::BLACK);

    // Top sweep bar: rounded only on its left (outer) end.
    fill_rounded_rect(
        fb,
        0,
        15,
        200,
        28,
        14,
        Corners { top_left: true, bottom_left: true, top_right: false, bottom_right: false },
        palette::ORANGE,
    );

    // Descender bar directly beneath it (flush against its bottom edge,
    // so the two together read as one L-shaped bar), rounded only on
    // its bottom (outer) end.
    fill_rounded_rect(
        fb,
        0,
        43,
        40,
        120,
        14,
        Corners { bottom_left: true, bottom_right: true, top_left: false, top_right: false },
        palette::PURPLE,
    );

    // A column of small, fully-rounded "buttons" along the right edge.
    let selected = *SELECTED_BUTTON.lock();
    for (i, &color) in BUTTON_COLORS.iter().enumerate() {
        let y = BUTTON_Y0 + i * BUTTON_SPACING;
        if selected == Some(i) {
            // A highlight frame, slightly larger than the button and
            // drawn first so only its border shows once the button is
            // painted on top -- proof of a real input-to-output loop
            // (`crate::keyboard` -> `select_button` -> a visibly
            // different framebuffer), not just that the keypress was
            // read.
            fill_rounded_rect(
                fb,
                BUTTON_X - HIGHLIGHT_MARGIN,
                y - HIGHLIGHT_MARGIN,
                BUTTON_W + 2 * HIGHLIGHT_MARGIN,
                BUTTON_H + 2 * HIGHLIGHT_MARGIN,
                BUTTON_RADIUS + HIGHLIGHT_MARGIN,
                Corners::ALL,
                palette::HIGHLIGHT,
            );
        }
        fill_rounded_rect(fb, BUTTON_X, y, BUTTON_W, BUTTON_H, BUTTON_RADIUS, Corners::ALL, color);
    }
}

pub const BUTTON_COUNT: usize = 4;
const BUTTON_X: usize = 250;
const BUTTON_Y0: usize = 15;
const BUTTON_SPACING: usize = 40;
const BUTTON_W: usize = 55;
const BUTTON_H: usize = 30;
const BUTTON_RADIUS: usize = 10;
const HIGHLIGHT_MARGIN: usize = 4;
const BUTTON_COLORS: [u8; BUTTON_COUNT] = [palette::RED_ALERT, palette::BLUE, palette::TAN, palette::ORANGE];

/// Which button (`0..BUTTON_COUNT`) `select_button` last picked, if any.
/// The panel's only piece of live state -- everything else `draw_demo_panel`
/// paints is fixed at compile time.
static SELECTED_BUTTON: Mutex<Option<usize>> = Mutex::new(None);

/// Select button `index` (silently ignored if out of range) and
/// immediately repaint the whole panel to show it -- the input-to-output
/// half of the loop `crate::keyboard`'s digit-key handling starts.
/// Reaches the framebuffer itself via `crate::memory::physical_memory_offset`
/// rather than requiring a caller (an interrupt handler, in practice) to
/// have one on hand.
pub fn select_button(index: usize) {
    if index >= BUTTON_COUNT {
        return;
    }
    *SELECTED_BUTTON.lock() = Some(index);
    draw_demo_panel(framebuffer(crate::memory::physical_memory_offset()));
}

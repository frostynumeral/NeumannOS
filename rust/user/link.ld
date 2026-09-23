/* Every NeumannOS ring-3 program built from Rust is linked here. The only
 * hard requirement (crate::elf::validate in the kernel) is that it sit in
 * PML4 slots the kernel's own address space leaves empty; slot 96
 * (0x3000_0000_0000) is clear of the kernel image (slot 0), the heap
 * (0x4444_4444_0000, slot 136) and the assembly programs' 0x5555...,
 * 0x6666... and 0x7777... ranges. Each program has its own address space,
 * so they can all share one link address. Sections are page-aligned so no
 * two PT_LOAD segments share a page, which the loader rejects. The
 * `.l*` variants are what the large code model (.cargo/config.toml)
 * emits.
 */
ENTRY(_start)

SECTIONS
{
    . = 0x300000000000;

    .text : ALIGN(4096) {
        *(.text._start)
        *(.text .text.* .ltext .ltext.*)
    }

    .rodata : ALIGN(4096) {
        *(.rodata .rodata.* .lrodata .lrodata.*)
    }

    .data : ALIGN(4096) {
        *(.data .data.* .ldata .ldata.*)
    }

    /* Aligned too: with no .data, .bss would otherwise start mid-page
     * right after .rodata, in a page the loader has already given the
     * read-only segment -- and it rejects two segments sharing a page. */
    .bss : ALIGN(4096) {
        *(.bss .bss.* .lbss .lbss.*)
        *(COMMON)
    }

    /DISCARD/ : {
        *(.eh_frame*)
        *(.comment)
        *(.note*)
    }
}

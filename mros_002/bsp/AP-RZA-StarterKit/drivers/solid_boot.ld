/*
 *  $Id$
 */

INCLUDE "memory_map.ld"

MEMORY {
solid_ram (RWX) : ORIGIN = _smm_SOLID_PhysicalAddress, LENGTH = _smm_SOLID_Size
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm","elf32-littlearm") 
OUTPUT_ARCH(arm)

ENTRY(_solid_boot_start)

SECTIONS {
	. = _smm_SOLID_PhysicalAddress;

	_ram_start = .;
	_solid_mon_start = .;
	.text ALIGN(4) : {
		__start_text = .;
		*(vector)
		*(.text .stub .text.* .gnu.linkonce.t.*)
		*(.glue_7t)
		*(.glue_7)
		*(.solid_mon)
		*(.smon_text)
		*(.smon_data)
		__end_text = .;
	} > solid_ram

	.rodata ALIGN(4) : {
		__start_rodata = .;
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		__end_rodata = .;
	} > solid_ram

	. = ALIGN(4);
	.data : {
		__start_data = .;
		*(.data .data.* .gnu.linkonce.d.*)
		__end_data = .;
	} > solid_ram

	.bss ALIGN(4) : {
		__start_bss = .;
		*(.sbss .sbss.* .bss .bss.* .gnu.linkonce.b.*)
		*(COMMON)
		__end_bss = .;
	} > solid_ram

	.stacks (NOLOAD) : {
	    _stack_init_svc = .; 
	    . += 4K;			/* stack fo boot */
	    _stack_start_svc = .;
	} > solid_ram
	_solid_mon_end = .;

	. = _smm_SOLID_PhysicalAddress + 64K;

	.solid_cs ALIGN(4K) : {
		__start_solid_cs = .;
		*(.solid_cs)
		__end_solid_cs = .;
		. += 256K;	/* slack for .bss/TTB/stack of solid_cs */
        } > solid_ram


	_end = .;
	.comment 0 : { *(.comment) }

	/*
	 *  DWARF debug sections.
	 *  Symbols in the DWARF debugging sections are relative to
	 *  the beginning of the section so we begin them at 0.
	 */

	/* DWARF 1 */
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }

	/* GNU DWARF 1 extensions */
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }

	/* DWARF 1.1 and DWARF 2 */
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames	0 : { *(.debug_pubnames) }

	/* DWARF 2 */
	.debug_info		0 : { *(.debug_info) }
	.debug_abbrev	0 : { *(.debug_abbrev) }
	.debug_line		0 : { *(.debug_line) }
	.debug_frame	0 : { *(.debug_frame) }
	.debug_str		0 : { *(.debug_str) }
	.debug_loc		0 : { *(.debug_loc) }
	.debug_macinfo	0 : { *(.debug_macinfo) }

	/* SGI/MIPS DWARF 2 extensions */
	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames  0 : { *(.debug_varnames) }
}

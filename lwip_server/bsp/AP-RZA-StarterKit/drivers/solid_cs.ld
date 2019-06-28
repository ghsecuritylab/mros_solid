/*
 *  $Id$
 */

INCLUDE "memory_map.ld"

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm","elf32-littlearm") 
OUTPUT_ARCH(arm)

ENTRY(solid_start)

SECTIONS {
    . = _smm_SOLID_VirtualAddress;

    _solid_start = .;

    /* space for solid_boot */
    . += 64K;

    .text ALIGN(4K) : {
	_solid_text_start   = .;
	*(.head)
	KEEP(*(.solid_vector))
	*(.text .stub .text.* .gnu.linkonce.t.*)
	*(.glue_7t)
	*(.glue_7)
	_solid_text_end = .;
    }
    _solid_text_offs = _solid_text_start - _solid_start; 
    _solid_text_size = _solid_text_end - _solid_text_start;

    .solid_mon ALIGN(4K) :{
	_solid_mon_start = .;
	*(.solid_mon)
	*(.smon_text)
	*(.smon_data)
	_solid_mon_end = .;
    }
    _solid_mon_offs = _solid_mon_start - _solid_start;
    _solid_mon_size = _solid_mon_end - _solid_mon_start;

    .rodata ALIGN(4K) : {
	_solid_rodata_start = .;
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)

        . = ALIGN(4);
        _ctor_list = . ;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        LONG (0);

        _dtor_list = . ;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        LONG (0);
    } 

    .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } 
     __exidx_start = .;
    .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } 
     __exidx_end = .;
    _solid_rodata_end = .;

    _solid_rodata_offs = _solid_rodata_start - _solid_start;
    _solid_rodata_size = _solid_rodata_end   - _solid_rodata_start;

    .data ALIGN(4K) : {
        _solid_data_start = .;
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
		
        /* Append the LLVM profiling sections */
        . = ALIGN(8);
        PROVIDE(__start___llvm_prf_cnts = .);
        KEEP(*(__llvm_prf_cnts))
        PROVIDE(__stop___llvm_prf_cnts = .);

        . = ALIGN(8);
        PROVIDE(__start___llvm_prf_data = .);
        KEEP(*(__llvm_prf_data))
        PROVIDE(__stop___llvm_prf_data = .);
		
        _solid_data_end = .;
    }
    _solid_data_offs = _solid_data_start - _solid_start;
    _solid_sata_size = _solid_data_end   - _solid_data_start;

    . += 4K; /* barrier */

    .args ALIGN(4K) (NOLOAD) : {
	_solid_bss_start = .;
	_solid_args_start = .;
	*(.args)
	_solid_args_end = .;
    }

    .bss : {
	*(.sbss .sbss.*)
	*(.bss .bss.*)
	*(.gnu.linkonce.b.*)
	*(COMMON)
	_heap = .;
	. += 1K;
	_heap_end = .;
	_solid_ldr_buf_start = .;
	. += 64K;
	_solid_ldr_buf_end = .;
	_solid_bss_end = .;
    }
    _solid_bss_offs  = _solid_bss_start  - _solid_start;
    _solid_bss_size  = _solid_bss_end    - _solid_bss_start;
    _solid_args_offs = _solid_args_start - _solid_start;
    _solid_args_size = _solid_args_end   - _solid_args_start;
    _heap_size = _heap_end - _heap;
    _solid_ldr_buf_size = _solid_ldr_buf_end - _solid_ldr_buf_start;

    . += 4K; /* barrier */

    . = ALIGN(16K);
    .mmu (NOLOAD) : {
	_solid_mmu_start = .;
	_solid_ttbtable = .;
	. += 16K;
	_solid_l2table   = .;
	. += 16K;
	_solid_mmu_end = .;
    }
    _solid_ttb_size  = _solid_l2table - _solid_mmu_start;
    _solid_l2tb_size = _solid_mmu_end - _solid_l2table;

    . += 4K; /* barrier */

    SVC_STACK_SIZE = 4K;
    IRQ_STACK_SIZE = 4K;
    FIQ_STACK_SIZE = 4K;
    UND_STACK_SIZE = 4K;
    ABT_STACK_SIZE = 4K;
    SYS_STACK_SIZE = 4K;

    . = ALIGN(4K);
    .stacks (NOLOAD) : {
	_solid_stack_start = .;
	_stack_top	= .;
	_stack_svc_start = .;
	. += SVC_STACK_SIZE;	/* SVC stack */
	_stack_svc_end   = .;
	. = ALIGN(4K);
	. += 4K;		/* barrier */
	_stack_irq_start = .;
	. += IRQ_STACK_SIZE;	/* IRQ stack */
	_stack_irq_end   = .;
	. = ALIGN(4K);
	. += 4K;		/* barrier */
	_stack_fiq_start = .;
	. += FIQ_STACK_SIZE;	/* FIQ stack */
	_stack_fiq_end   = .;
	. = ALIGN(4K);
	. += 4K;		/* barrier */
	_stack_und_start = .;
	. += UND_STACK_SIZE;	/* UND stack */
	_stack_und_end   = .;
	. = ALIGN(4K);
	. += 4K;		/* barrier */
	_stack_abt_start = .;
	. += ABT_STACK_SIZE;	/* ABT stack */
	_stack_abt_end   = .;
	. = ALIGN(4K);
	. += 4K;		/* barrier */
	_stack_sys_start = .;
	. += SYS_STACK_SIZE;	/* SYS stack */
	_stack_sys_end   = .;
	_solid_stack_end = .;
   }
   _solid_stack_svc_size = _stack_svc_end - _stack_svc_start;
   _solid_stack_irq_size = _stack_irq_end - _stack_irq_start;
   _solid_stack_fiq_size = _stack_fiq_end - _stack_fiq_start;
   _solid_stack_und_size = _stack_und_end - _stack_und_start;
   _solid_stack_abt_size = _stack_abt_end - _stack_abt_start;
   _solid_stack_sys_size = _stack_sys_end - _stack_sys_start;

   _solid_end = .;

   ASSERT( _solid_end < _smm_SOLID_VirtualAddress + _smm_SOLID_Size, "SOLID AREA FULL" )


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




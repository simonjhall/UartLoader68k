ENTRY(_start)

SECTIONS
{
  . = 0;
  .vectors :
  {
  	vector_table = .;
  	*(.vectors)
  }
  . = 0x800;
  .text : { *(.text*) }
  .rodata : { *(.rodata*) }
  
  /*.data : { *(.data) } */
  
  /* origins */
/*  . = 0x100000;*/

/* big daddy pro */
/*	. = 0x400000;*/

   /* dynamic daddy */
/*  . = 0x800000;*/

   /* fpga 68008 */
    . = 0x10000;
	
  .bss : { *(.bss) }
}

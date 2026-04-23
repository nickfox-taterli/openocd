ENTRY(stub_entry)

MEMORY
{
	ram (rwx) : ORIGIN = 0x20202000, LENGTH = 0x00004000
}

SECTIONS
{
	.text : {
		KEEP(*(.text.stub_entry))
		*(.text*)
		*(.rodata*)
	} > ram

	.data : {
		*(.data*)
	} > ram

	.bss (NOLOAD) : {
		*(.bss*)
		*(COMMON)
	} > ram

	/DISCARD/ : {
		*(.comment*)
		*(.note*)
		*(.eh_frame*)
		*(.ARM.exidx*)
		*(.ARM.extab*)
	}
}

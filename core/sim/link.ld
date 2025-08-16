OUTPUT_ARCH("riscv")

ENTRY(prog_entry)

MEMORY
{
  PROG_ENTRY_ROM (rx) : ORIGIN = 0x00000000, LENGTH = 0x00000010
  PROG_ROM (rx)       : ORIGIN = 0x00000010, LENGTH = 0x0000FFF0
  RAM (rw)            : ORIGIN = 0x20000000, LENGTH = 0x00002000
}

SECTIONS
{
  .prog_entry : {
    . = ALIGN(4);
    *(.prog_entry)
  } > PROG_ENTRY_ROM

  .text : {
    . = ALIGN(4);
    *(.irq_handler)
    *(.text)
    *(.rodata)
    . = ALIGN(4);
  } > PROG_ROM

  .data : {
    . = ALIGN(4);
    _data_start = .;
    *(.data)
    . = ALIGN(4);
    _data_end = .;
  } > RAM AT > PROG_ROM

  .bss (NOLOAD) : {
    . = ALIGN(4);
    _bss_start = .;
    *(.bss)
    . = ALIGN(4);
    _bss_end = .;
  } > RAM
}

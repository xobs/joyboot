/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * KL25Z128 memory setup.
 */

ENTRY(Reset_Handler)

MEMORY
{
    bl_flash  :  org = 0x00000000,  len = 8k,
    storage   :  org = 0x00002000,  len = 2k,
    app_flash :  org = 0x00002800,  len = 118k,
    ram       :  org = 0x1FFFF000,  len = 16k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    .text : ALIGN(4) SUBALIGN(4) {
        . = 0;
        KEEP(*(.vectors))
        *(.startup*)
        . = 0x400;
        KEEP(*(.flashconfig*))
    } > bl_flash = 0xFF
    _eflash = .;

    .apptext (NOLOAD) : {
        . = ORIGIN(app_flash);
        *(.appvectors)
    } > app_flash = 0xFF


    /* Combined data and text, after relocation */
    .dtext : AT (_eflash) {
        . = ALIGN(4);
        _sdtext = .;
        *(.text*)
	*(.ramtext*)
        *(.rodata*)
        *(.data*)
        . = ALIGN(4);
        _edtext = .;
    } > ram

    .noinit (NOLOAD) : {
        *(.noinit*)
    } > ram

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end = .;
    } > ram

    __main_stack_end__ = ORIGIN(ram) + LENGTH(ram) - 4;
    boot_token = __main_stack_end__;

    /DISCARD/ : {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.note.gnu.build-id)
        *(.note.gnu.build-id.*);
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        *(.interp)
    }

  /* ld.bfd will segfault without these */
  .rel.plt : {
    . = ALIGN(4);
  }
  .got.plt : {
    . = ALIGN(4);
  }
}

/* RAM region to be used for Main stack. This stack accommodates the processing
      of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram);

/* RAM region to be used for the process stack. This is the stack used by
      the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram);

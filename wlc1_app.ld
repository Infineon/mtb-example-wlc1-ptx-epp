/***************************************************************************//**
* \file dfu_cm0p_app1.ld
* \version 4.10
*
* Linker file for the GNU C compiler.
* Used for the DFU SDK application 1 firmware project.
* Application 1 is uploaded through Bootloader Host Tool
* using the generated cyadc2 file.
*
* The main purpose of the linker script is to describe how the sections in the
* input files should be mapped into the output file, and to control the memory
* layout of the output file.
*
* \note The entry point location is fixed and starts at 0x00000000. The valid
* application image should be placed there.
*
* \note The linker files included with the PDL template projects must be generic
* and handle all common use cases. Your project may not use every section
* defined in the linker files. In that case you may see warnings during the
* build process. In your project, you can simply comment out or remove the
* relevant code in the linker file.
*
********************************************************************************
* \copyright
* Copyright 2019-2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

/*--------------------- Flash Configuration ----------------------------------
; <h> Flash Configuration
;   <o0> Flash Base Address <0x0-0xFFFFFFFF:8>
;   <o1> Flash Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
 -----------------------------------------------------------------------------*/
__FLASH_START = 0x00000000;
__FLASH_SIZE =  0x00020000;

/*--------------------- Embedded RAM Configuration ---------------------------
; <h> RAM Configuration
;   <o0> RAM Base Address    <0x0-0xFFFFFFFF:8>
;   <o1> RAM Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
 -----------------------------------------------------------------------------*/
__RAM_START = 0x20000000;
__RAM_SIZE =  0x00004000;

/*--------------------- Stack / Heap Configuration ---------------------------
; <h> Stack / Heap Configuration
;   <o0> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;   <o1> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
 -----------------------------------------------------------------------------*/
__STACK_SIZE = 0x00000400;
__HEAP_SIZE  = 0x000000C0;
__RAM_COMMON_SIZE = 0x00000040;

/* The MEMORY section below describes the location and size of blocks of memory in the target.
* Use this section to specify the memory regions available for allocation.
*/
MEMORY
{
 /* The ram and FLASH regions control RAM and FLASH memory allocation.
  *  You can change the memory allocation by editing the 'ram' and 'FLASH' regions.
  */
  
  /* Flash allocated for bootloader and application firmware region. This value is calculated by offseting config_table size */
  FLASH_boot (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00003000
  FLASH_fw (rx)  : ORIGIN = 0x00003800, LENGTH = 0x0001C4FC

  /* RAM allocated for bootloader and application firmware region */
  RAM_boot   (rwx) : ORIGIN = 0x20000000,   LENGTH = 0x00004000
  RAM_fw   (rwx) : ORIGIN = 0x20000000,   LENGTH = 0x00004000
  
  /* Flash allocated for cyacd2 file padding for all area except the blackbox row. */
  flash_padding (rx) : ORIGIN = 0x0001FCFC, LENGTH = 0x04

  /* Flash allocated metadata */
  flash_boot_meta   (rx)  : ORIGIN = 0x0001FF00, LENGTH = 0x100
  
  /* Flash allocated for config table. */
  cy_cfg_space (rx) : ORIGIN = 0x00003000, LENGTH = 0x800

}

/* The DFU SDK metadata limits */
__cy_boot_metadata_addr = ORIGIN(flash_boot_meta);
__cy_boot_metadata_length = __cy_memory_0_row_size;

/* The Product ID, used by CyMCUElfTool to generate a updating file */
__cy_product_id = 0x01020304;

/* The checksum type used by CyMCUElfTool to generate a updating file */
__cy_checksum_type = 0x00;

/* Used by the DFU SDK application to set the metadata */
__cy_boot_verify_start  = ORIGIN(FLASH_boot);
__cy_boot_verify_length = LENGTH(FLASH_boot) - __cy_boot_signature_size;
__cy_fw_verify_start  = ORIGIN(FLASH_fw);
__cy_fw_verify_length = LENGTH(FLASH_fw) + 0x4;

/*
* The size of the application signature.
* E.g. 4 for CRC-32,
*     32 for SHA256,
*    256 for RSA 2048.
*/
__cy_boot_signature_size = 4;

/*
* DFU SDK specific: aliases regions, so the rest of code does not use
* application specific memory region names
*/
REGION_ALIAS("FLASH", FLASH_fw);
REGION_ALIAS("RAM",     RAM_fw);


/* DFU SDK specific: sets an app Id */
__cy_app_id = 1;

/* DFU SDK specific */
/* CyMCUElfTool uses these ELF symbols to generate an application signature */
__cy_app_verify_start  = ORIGIN(FLASH);
__cy_app_verify_length = LENGTH(FLASH) + 0x4; 

__cy_config_fw_start = ORIGIN(cy_cfg_space);
__cy_config_fw_length = LENGTH(FLASH) + LENGTH(cy_cfg_space) +0x4; 



/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   __Vectors_End
 *   __Vectors_Size
 *
 * For the DFU SDK, these additional symbols are defined:
 *   __cy_app_id
 *   __cy_product_id
 *   __cy_checksum_type
 *   __cy_boot_metadata_addr
 *   __cy_boot_metadata_length
 */
ENTRY(Reset_Handler)

SECTIONS
{
    /* App0 uses it to initialize DFU SDK metadata, in the dfu_user.c file */
    .cy_metadata :
    {
    
    /* Meta data padding to differentiate blackbox rows */
        KEEP(*(.meta_padding))
        KEEP(*(.cy_metadata))
    } > flash_boot_meta

   .cy_app_header :
    {
        KEEP(*(.cy_app_header))
    } > FLASH
    
    /* Config Table region */
    .configSection :
    {
        KEEP(*(.configSection))
    } > cy_cfg_space

    .text :
    {
        . = ALIGN(4);
        KEEP(*(.vectors))
        . = ALIGN(4);
        __Vectors_End = .;
        __end__ = .;

		/* Reserve place for customer related info. */
		. = 0xC0;
		KEEP(*(.customer_region))
		/* Place the version and silicon ID information 0xE0 bytes from the start location. */
		. = 0xE0;
		KEEP(*(.base_version))
		KEEP(*(.app_version))
		KEEP(*(.dev_siliconid))
		KEEP(*(.fw_reserved))

        . = ALIGN(4);
        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        /* Read-only code (constants). */
        *(.rodata .rodata.* .constdata .constdata.* .conststring .conststring.*)

        KEEP(*(.eh_frame*))
    } > FLASH

   .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;

        /* Copy data section to RAM */
        LONG (__etext)                                      /* From */
        LONG (__data_start__)                               /* To   */
        LONG ((__data_end__ - __data_start__) / 4)          /* Size */

        __copy_table_end__ = .;
    } > FLASH

    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG ((__bss_end__ - __bss_start__) / 4)
        __zero_table_end__ = .;
    } > FLASH

   /* Location counter can end up 2byte aligned with narrow Thumb code but
    *  __etext is assumed by startup code to be the LMA of a section in RAM
    *  which must be 4byte aligned.
    */
    __etext = ALIGN (4);
    
    .ramVectors (NOLOAD) : ALIGN(8)
    {
        __ram_vectors_start__ = .;
        KEEP(*(.ram_vectors))
        __ram_vectors_end__   = .;
    } > RAM
    
    .srom_out_callback __ram_vectors_end__ (NOLOAD) : ALIGN(4)
    {
    	__srom_pointers_start__ = .;
        KEEP(*(.srom_out_cbk))
    } > RAM

    .srom_in_callback (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.srom_in_cbk))
    } > RAM

    .srom_status (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.srom_status_var))
    } > RAM

    .srom_hpi (NOLOAD) : ALIGN(4)
    {
        . += 0x3b8;
        KEEP(*(.hpi_globals))
        __srom_pointers_end__ = .;
    } > RAM

    .btldr_run __srom_pointers_end__ (NOLOAD) : ALIGN(8)
    {
        KEEP(*(.bootloaderruntype))
    } > RAM
    
    .ramCommon (NOLOAD) : ALIGN(4)
    {
        __ram_common_start__ = .;
        KEEP(*(.cy_boot_noinit.appId))
        KEEP(*(.cy_boot_noinit))
        __ram_common_end__ = __ram_common_start__ + __RAM_COMMON_SIZE;
    } > RAM

    .data __ram_common_end__ : AT (__etext)
    {
        __data_start__ = .;

        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;

    } > RAM

    /* Place variables in the section that should not be initialized during the
    *  device startup.
    */
    .noinit (NOLOAD) : ALIGN(8)
    {
      KEEP(*(.noinit))
    } > RAM


    /* Secondary data section, optional
    *
    * Remember to add each additional data section
    * to the .copy.table above to asure proper
    * initialization during startup.
    */
    /*
    __etext2 = ALIGN (4);

    .data2 : AT (__etext2)
    {
    . = ALIGN(4);
        __data2_start__ = .;
    *(.data2)
    *(.data2.*)
        . = ALIGN(4);
        __data2_end__ = .;

    } > RAM
    */

   /* The uninitialized global or static variables are placed in this section.
    *
    * The NOLOAD attribute tells linker that .bss section does not consume
    * any space in the image. The NOLOAD attribute changes the .bss type to
    * NOBITS, and that  makes linker to A) not allocate section in memory, and
    * A) put information to clear the section with all zeros during application
    * loading.
    *
    * Without the NOLOAD attribute, the .bss section might get PROGBITS type.
    * This  makes linker to A) allocate zeroed section in memory, and B) copy
    * this section to RAM during application loading.
    */
    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
    *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    /**
    * Secondary bss section, optional
    *
    * Remember to add each additional bss section
    * to the .zero.table above to asure proper
    * initialization during startup.
    */
    /*
    .bss2 :
    {
        . = ALIGN(4);
        __bss2_start__ = .;
    *(.bss2)
        *(.bss2.*)
        . = ALIGN(4);
        __bss2_end__ = .;
    } > RAM2
    */

    .heap :
    {
    . = ALIGN(4);
        __end__ = .;
        PROVIDE(end = .);
        . = . + __HEAP_SIZE;
    . = ALIGN(4);
        __HeapLimit = .;
    } > RAM
   /* .stack section doesn't contains any symbols. It is only used
    * for linker to calculate size of stack sections, and assign
    * values to stack symbols later.
    */
    .stack :
    {
       /* Set stack top to end of RAM, and stack limit move down by
        * size of stack_dummy section
        */
        . = ORIGIN(RAM) + LENGTH(RAM) - __STACK_SIZE;
        . = ALIGN(4);
        __StackLimit = .;
        . = . + __STACK_SIZE;
        . = ALIGN(4);
        __StackTop = .;
    } > RAM
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
    
    .flash_padding :
    {
        KEEP(*(.flash_padding))
    } > flash_padding

    /*
    * The DFU SDK section for an app verification signature.
    * Must be placed at the end of the application.
    * In this case, last N bytes of the last FLASH row inside the application.
    */
    .cy_app_signature :
    {
        KEEP(*(.cy_app_signature))
    } > flash_boot_meta

    /* These sections are used for additional metadata (silicon revision,
    * Silicon/JTAG ID, etc.) storage.
    */
    .cyflashprotect 0x90400000 : { KEEP(*(.cyflashprotect)) } :NONE
    .cymeta         0x90500000 : { KEEP(*(.cymeta)) } :NONE
    .cychipprotect  0x90600000 : { KEEP(*(.cychipprotect)) } :NONE
}


/* The following symbols used by the cymcuelftool. */
/* Flash */
__cy_memory_0_start    = 0x00000000;
__cy_memory_0_length   = 0x0001FD00;
__cy_memory_0_row_size = 0x100;

/* Meta region */
__cy_memory_1_start    = 0x0001FF00;
__cy_memory_1_length   = 0x00000100;
__cy_memory_1_row_size = 0x100;


/* EOF */

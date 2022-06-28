MEMORY {
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000 /* start from 0x0, fullsize flash, 20KB */
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000 /* full-size SRAM, 128K */
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000 /* full-size SRAM, 128K, redefinition because of basicsmac */
    BLFLASH (rx) : ORIGIN = 0x00000000, LENGTH = 20K /* from SemTech bootloader, but 20K instead of 12K */
    FWFLASH(rx) : ORIGIN = 0x00000000 + 20K, LENGTH = 256K - 20K /* from SemTech bootloader */
}

/* Shared RISC-V and ARM memory map */
_FlashLength_ =  __FlashLength;
MEMORY {
    ROM     (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00010000 /* 64kB - ROM */
    
    FLASH   (rx)  : ORIGIN = __FlashStart, LENGTH = _FlashLength_ /* 256kB - Flash - can be used by RV */

    SRAM0   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000 /* 32kB SRAM0 - ARM Data */

    SRAM1   (rwx) : ORIGIN = 0x20008000, LENGTH = 0x00008000 /* 32kB SRAM1 - ARM Stack */

    SRAM2   (rwx) : ORIGIN = 0x20010000, LENGTH = 0x00010000 /* 64kB SRAM2 - ARM */

    SRAM3   (rwx) : ORIGIN = 0x20020000, LENGTH = 0x00010000 /* 64kB SRAM3 - ARM */

    SRAM4   (rwx) : ORIGIN = 0x20030000, LENGTH = 0x00010000 /* 64kB SRAM4 - RV, ARM */

    SRAM5   (rwx) : ORIGIN = 0x20040000, LENGTH = 0x00010000 /* 64kB SRAM5 - RV, ARM */

    SRAM6   (rwx) : ORIGIN = 0x20050000, LENGTH = 0x0000C000 /* 48kB SRAM6 - RV, ARM */

    SRAM7   (rwx) : ORIGIN = 0x2005C000, LENGTH = 0x00004000 /* 16kB SRAM7 - RV, ICC1, or ARM */
}


/* Shared RISC-V and ARM memory map */
_FlashLength_ =  __FlashLength;

MEMORY {
    ROM     (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00010000 /* 64kB - ROM */
    
    FLASH   (rx)  : ORIGIN = __FlashStart, LENGTH = _FlashLength_ /* 256kB - Flash - can be used by RV */

    SRAM0   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000 /* 32 kB - ARM Data */

    SRAM1   (rwx) : ORIGIN = 0x20008000, LENGTH = 0x00008000 /* 32 kB - ARM Stack */
    
    SRAM2   (rwx) : ORIGIN = 0x20010000, LENGTH = 0x0000BF80 /* ~48kB SRAM2 - RV Data */

    SHARED  (rw)  : ORIGIN = 0x2001BF80, LENGTH = 0x00000080 /* 128B Shared - ARM, RV Data */
    
    SRAM3   (rwx) : ORIGIN = 0x2001C000, LENGTH = 0x00004000 /* 16kB SRAM3 - RV Code */
}

/* The address and length of the SHARED section defined above. */
__shared_origin = ORIGIN(SHARED);
__shared_length = LENGTH(SHARED);


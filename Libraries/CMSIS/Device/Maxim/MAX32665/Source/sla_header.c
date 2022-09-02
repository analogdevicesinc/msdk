#define SWAP(x) \
    (((x) >> 24) | (((x)&0x00FF0000) >> 8) | (((x)&0x0000FF00) << 8) | (((x)&0x000000FF) << 24))

typedef enum {
    ROM_A1_VERSION = 0x01000000,
} enum_rom_version_t;

//#ifdef __SLA_FWK__
/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

extern unsigned int _start_SWAP;
extern unsigned int _SLA_Size_SWAP;

typedef enum {
    /* ES17 Requires key id (offset 0, low 4 bits) to be set to 0x8 for ECDSA256 */
    MagicH = 0x48495357,
    /* Cannot use this one with ES17 */
    //	MagicH = 0x46495357,
    /* NOTE (from ME14): The 0xF nibble (of 0xF6) means something called stack_method=1.
 * If set to 0xF, the ROM fetches SP and PC immediately after the header at offset 0x20.
 * If set to 0x4, the ROM uses the PC in the header at offset 0x14.
 */
    /*	MagicH = 0xF6495357,*/
    MagicL = 0x45444744,
} enum_magic_t;

typedef struct {
    enum_magic_t MagicHigh; //> SLA Header magic
    enum_magic_t MagicLow;  //> SLA Header magic
} magic_t;

typedef struct {
    magic_t Magic;
    enum_rom_version_t RomVersion; //> ROM version
    unsigned int LoadAddr;         //> Relocation address.
    unsigned int SLA_CodeSize;     //> SLA code size in bytes
    unsigned int* JumpAddr;        //> Rom code will jump at this address
    unsigned int ArgSize;          //> Size of the Argument
    unsigned int AppVersionNumber; //> Version of this application
} flash_app_header_t;

__attribute__((section(".sb_sla_header"))) __attribute__((__used__))
const flash_app_header_t sb_header = {
    .Magic =
        {
            .MagicHigh = SWAP(MagicH),
            .MagicLow  = SWAP(MagicL),
        },

    .RomVersion       = SWAP(ROM_A1_VERSION),
    .LoadAddr         = SWAP(0x10000000),
    .SLA_CodeSize     = (unsigned int)&_SLA_Size_SWAP, // Trick to get constant defined at link time
    .JumpAddr         = &_start_SWAP,
    .ArgSize          = 0,
    .AppVersionNumber = SWAP(0x01000000), // 0xAABBCCCC for version AA.BB.CCCC
};

//__attribute__ ((section(".sb_sla_trailer"))) __attribute__ ((__used__))
//const unsigned int dummy_signature=0xCAFEFADE;

//#endif //__SLA_FWK__

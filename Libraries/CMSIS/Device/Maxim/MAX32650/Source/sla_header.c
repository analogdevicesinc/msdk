/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

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
    MagicH = 0x46495357,
    /* NOTE: The 0xF nibble means something called stack_method=1.
 * If set to 0xF, the ROM fetches SP and PC immediately after the header at offset 0x20.
 * If set to 0x4, the ROM uses the PC in the header at offset 0x14.
 */
    /*  MagicH = 0xF6495357,*/
    MagicL = 0x45444744,
} enum_magic_t;

typedef struct {
    enum_magic_t MagicHigh; //> SLA Header magic
    enum_magic_t MagicLow; //> SLA Header magic
} magic_t;

typedef struct {
    magic_t Magic;
    enum_rom_version_t RomVersion; //> ROM version
    unsigned int LoadAddr; //> Relocation address.
    unsigned int SLA_CodeSize; //> SLA code size in bytes
    unsigned int *JumpAddr; //> Rom code will jump at this address
    unsigned int ArgSize; //> Size of the Argument
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

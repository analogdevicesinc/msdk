/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************
*/

#define SWAP(x) \
    (((x) >> 24) | (((x)&0x00FF0000) >> 8) | (((x)&0x0000FF00) << 8) | (((x)&0x000000FF) << 24))

typedef enum {
    ROM_A1_VERSION = 0x01000000,
} enum_rom_version_t;

#include <stdio.h>
#include <stdint.h>

extern unsigned int _start_SWAP;
extern unsigned int _SLA_Size_SWAP;

typedef enum {
    MagicH = 0x48495357,
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

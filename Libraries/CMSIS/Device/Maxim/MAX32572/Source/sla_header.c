/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved.
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
 *
 ******************************************************************************/

#define SWAP(x) \
    (((x) >> 24) | (((x)&0x00FF0000) >> 8) | (((x)&0x0000FF00) << 8) | (((x)&0x000000FF) << 24))

typedef enum {
    ROM_A1_VERSION = 0x01000000,
    LESS_THAN_ROM_A1_VERSION = 0x00000001,
    GREATER_THAN_ROM_A1_VERSION = 0x01000001,
} enum_rom_version_t;

typedef enum {
    APP_VERSION = 0x01000000,
    LESS_THAN_APP_VERSION = 0x00AA0055,
    GREATER_THAN_APP_VERSION = 0x010000CC,
} enum_app_version_t;

//#ifdef __SLA_FWK__
/***** Includes *****/
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

extern uint32_t _FLASH;

__attribute__((section(".sb_sla_header"))) __attribute__((__used__))
const flash_app_header_t sb_header =
{
        .Magic =
        {
            .MagicHigh      = SWAP(MagicH),
            .MagicLow       = SWAP(MagicL),
        },

        .RomVersion         = SWAP(ROM_A1_VERSION),
//        .RomVersion            = SWAP(LESS_THAN_ROM_A1_VERSION),
//        .RomVersion            = SWAP(GREATER_THAN_ROM_A1_VERSION),
        .LoadAddr           = SWAP(0x08000000),
//        .LoadAddr             = SWAP(0x1007FC00),
        .SLA_CodeSize       = (unsigned int)&_SLA_Size_SWAP,  // Trick to get constant defined at link time
        .JumpAddr           = &_start_SWAP,
        .ArgSize            = 0,
        .AppVersionNumber   = SWAP(APP_VERSION), // 0xAABBCCCC for version AA.BB.CCCC
//        .AppVersionNumber   = SWAP(LESS_THAN_APP_VERSION),
//        .AppVersionNumber   = SWAP(GREATER_THAN_APP_VERSION),
};

//__attribute__ ((section(".sb_sla_trailer"))) __attribute__ ((__used__))
//const unsigned int dummy_signature=0xCAFEFADE;

//#endif //__SLA_FWK__

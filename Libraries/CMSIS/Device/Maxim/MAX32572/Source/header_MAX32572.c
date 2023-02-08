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

#ifdef __SLA_FWK__
/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

extern unsigned int _start_SWAP;
extern unsigned int _SLA_Size_SWAP;

typedef enum {
    MagicH = 0x44495357,
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

#endif //__SLA_FWK__

#ifdef __SCPA_FWK__
/** Global declarations */

#include <string.h>

typedef enum {
    MagicH = 0xDEADBEEF,
    MagicL = 0xCAFEFADE,
} enum_magic_t;

typedef struct {
    enum_magic_t MagicHigh; //> SLA Header magic
    enum_magic_t MagicLow; //> SLA Header magic
} magic_t;

typedef int (*__scpa_write_t)(unsigned int dest, unsigned int length, unsigned char *p_src);
typedef int (*__scpa_erase_t)(unsigned int dest, unsigned int length);

/** Generic Plugin Operations */
typedef struct {
    __scpa_write_t write; //> Write to memory
    __scpa_write_t compare; //> Compare memory data
    __scpa_erase_t erase; //> Erase memory
} scpa_ops_t;

typedef struct {
    magic_t Magic;
    enum_rom_version_t RomVersion; //> ROM version
    unsigned int mem_base_addr; //> Base address of memory targetted by applet
    unsigned int mem_size; //> Size of this memory
    scpa_ops_t ops; //> Operations of the SCP Applet
} scpa_header_t;

int start_scpa_write(unsigned int dest, unsigned int length, unsigned char *p_src);
int start_scpa_compare(unsigned int dest, unsigned int length, unsigned char *p_src);
int start_scpa_erase(unsigned int dest, unsigned int length);

int __attribute__((weak)) scpa_write(unsigned int dest, unsigned int length, unsigned char *p_src);
int __attribute__((weak))
scpa_compare(unsigned int dest, unsigned int length, unsigned char *p_src);
int __attribute__((weak)) scpa_erase(unsigned int dest, unsigned int length);

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int __bss_magic__;

#ifndef SCPA_MEM_BASE_ADDR
#define SCPA_MEM_BASE_ADDR 0xC0000000
#warning 'SCPA_MEM_BASE_ADDR not defined using default value 0xC0000000'
#endif

#ifndef SCPA_MEM_SIZE
#define SCPA_MEM_SIZE 1024
#warning 'SCPA_MEM_SIZE not defined using default value 1024'
#endif

unsigned int __attribute__((section(".scpa_init"))) Magic_bss = 0xABADCAFE;

__attribute__((section(".scpa_header"))) __attribute__((__used__))
const scpa_header_t scpa_header = {
    .Magic =
        {
            .MagicHigh = MagicH,
            .MagicLow  = MagicL,
        },
#ifdef MAX32572_A1
    .RomVersion = SWAP(ROM_A1_VERSION),
#else
#error "Please Select a chip ROM revision"
#endif
    .mem_base_addr = SCPA_MEM_BASE_ADDR,
    .mem_size      = SCPA_MEM_SIZE,
    .ops =
        {
            .write   = (__scpa_write_t)start_scpa_write,
            .compare = (__scpa_write_t)start_scpa_compare,
            .erase   = (__scpa_erase_t)start_scpa_erase,
        },
};

int __attribute__((section(".scpa_ops")))
start_scpa_write(unsigned int dest, unsigned int length, unsigned char *p_src)
{
    volatile unsigned int bss_size =
        (volatile unsigned int)&__bss_end__ - (volatile unsigned int)&__bss_start__;
    volatile unsigned char *p_bss = (volatile unsigned char *)&__bss_start__;
    volatile unsigned int *p_magic = (volatile unsigned int *)&__bss_magic__;

    // Automatic Code for bss init
    if (*p_magic == 0xABADCAFE) {
        memset((void *)p_bss, 0x00, bss_size);
        *p_magic = 0x0;
    }
    return scpa_write(dest, length, p_src);
}

int __attribute__((section(".scpa_ops")))
start_scpa_compare(unsigned int dest, unsigned int length, unsigned char *p_src)
{
    volatile unsigned int bss_size =
        (volatile unsigned int)&__bss_end__ - (volatile unsigned int)&__bss_start__;
    volatile unsigned char *p_bss = (volatile unsigned char *)&__bss_start__;
    volatile unsigned int *p_magic = (volatile unsigned int *)&__bss_magic__;

    // Automatic Code for bss init
    if (*p_magic == 0xABADCAFE) {
        memset((void *)p_bss, 0x00, bss_size);
        *p_magic = 0x0;
    }
    return scpa_compare(dest, length, p_src);
}

int __attribute__((section(".scpa_ops"))) start_scpa_erase(unsigned int dest, unsigned int length)
{
    volatile unsigned int bss_size =
        (volatile unsigned int)&__bss_end__ - (volatile unsigned int)&__bss_start__;
    volatile unsigned char *p_bss = (volatile unsigned char *)&__bss_start__;
    volatile unsigned int *p_magic = (volatile unsigned int *)&__bss_magic__;

    // Automatic Code for bss init
    if ((*p_magic == 0xABADCAFE)) {
        memset((void *)p_bss, 0x00, bss_size);
        *p_magic = 0x0;
    }
    return scpa_erase(dest, length);
}

int __attribute__((section(".scpa_ops")))
scpa_write(unsigned int dest, unsigned int length, unsigned char *p_src)
{
    (void)dest;
    (void)length;
    (void)p_src;
    return 0;
}

int __attribute__((section(".scpa_ops")))
scpa_compare(unsigned int dest, unsigned int length, unsigned char *p_src)
{
    (void)dest;
    (void)length;
    (void)p_src;
    return 0;
}

int __attribute__((section(".scpa_ops"))) scpa_erase(unsigned int dest, unsigned int length)
{
    (void)dest;
    (void)length;
    return 0;
}

void Reset_Handler(void) {}

#endif //__SCPA_FWK__

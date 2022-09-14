/**
 * @file hpb.c
 * @brief      This file contains the function implementations for the
 *             HyperBus (HPB) peripheral module.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "hpb.h"
#include "s26k.h"

/**
 * @defgroup S26K
 * @{
 */

/* **** Definitions **** */

#define S26K_STATUS_DEVICE_READY        0x80
#define S26K_STATUS_ERASE_STATUS        0x20
#define S26K_STATUS_PGM_STATUS          0x10
#define S26K_STATUS_SECTOR_ERASE_STATUS 0x01

#define S26K_UNLOCKADDR0 (0x555 << 1)
#define S26K_UNLOCKADDR1 (0x2AA << 1)
#define S26K_CFI_ADDR    (0x55 << 1)

#define S26K_UNLOCKDATA0 0xAA
#define S26K_UNLOCKDATA1 0x55

#define S26K_CMDERASE        0x80
#define S26K_CMDCHIP_ERASE   0x10
#define S26K_CMDSECTOR_ERASE 0x30
#define S26K_CMDREAD_STATUS  0x70
#define S26K_CMDCLEAR_STATUS 0x71
#define S26K_CMDBLANK_CHECK  0x33
#define S26K_CMDWRITE16      0xA0
#define S26K_CMDWRITE        0x25
#define S26K_CMDWRITE_BUFFER 0x29
#define S26K_CMDREAD_ID      0x90
#define S26K_CMDREAD_CFI     0x98
#define S26K_CMDEXIT_ASO     0xF0

#define S26K_CMDPPB_ENTRY     0xC0
#define S26K_CMDPPB_STATUS    0x60
#define S26K_CMDPPB_PROGRAM   0xA0
#define S26K_CMDPPB_ERASE0    0x80
#define S26K_CMDPPB_ERASE1    0x30
#define S26K_CMDPPBLOCK_ENTRY 0x50
#define S26K_CMDPPBLOCK_CLEAR 0xA0
#define S26K_CMDDYB_ENTRY     0xE0
#define S26K_CMDDYB_SET       0xA0
#define S26K_CMDASP_ENTRY     0x40
#define S26K_CMDABORT_RESET   0xF0

#define S26K_BUFFER_SIZE 0x200
#define S26K_SECTOR_SIZE 0x40000

#define S26K_WRITE16(a, d) *(volatile uint16_t*)(s26k_base_addr + a) = (uint16_t)d
#define S26K_READ16(a, d)  d = *((volatile uint16_t*)(s26k_base_addr + a));

/* **** Globals **** */
static uint32_t s26k_base_addr;
/* **** Functions **** */

/* ************************************************************************** */
void S26K_Init(unsigned int cs, uint32_t base)
{
    hpb_mem_config_t mem;
    sys_hpb_cfg_t sys_cfg;

    sys_cfg.en_cs0 = !cs;
    sys_cfg.en_cs1 = !!cs;

    mem.base_addr      = base;
    mem.merge_read     = 0;
    mem.asym_cache     = 0;
    mem.register_space = 0;
    mem.hyper_ram      = 0;
    mem.gpo_high       = 0;
    //mem.wrap_size       = HPB_WRAP_32;
    mem.read_cs_high   = HPB_CS_HIGH_1_5;
    mem.write_cs_high  = HPB_CS_HIGH_1_5;
    mem.read_cs_setup  = HPB_CS_SETUP_HOLD_2;
    mem.write_cs_setup = HPB_CS_SETUP_HOLD_2;
    mem.read_cs_hold   = HPB_CS_SETUP_HOLD_2;
    mem.write_cs_hold  = HPB_CS_SETUP_HOLD_2;

    if (sys_cfg.en_cs0) {
        HPB_Init(&mem, NULL, &sys_cfg);
    } else {
        HPB_Init(NULL, &mem, &sys_cfg);
    }

    s26k_base_addr = base;

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
uint16_t S26K_GetStatus(void)
{
    uint16_t status;

    // Issue status read sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_STATUS);
    S26K_READ16(0, status);

    return status;
}

/* ************************************************************************** */
void S26K_ClearStatus(void)
{
    // Issue clear status command
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDCLEAR_STATUS);
}

/* ************************************************************************** */
int S26K_ChipErase(void)
{
    uint16_t status;
    int result;

    // Issue chip erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDERASE);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDCHIP_ERASE);

    while (1) {
        // Issue status read sequence
        S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_STATUS);
        S26K_READ16(0, status);

        // Check status
        if ((status & S26K_STATUS_DEVICE_READY) == S26K_STATUS_DEVICE_READY) {
            if ((status & S26K_STATUS_ERASE_STATUS) == 0x0) {
                result = E_NO_ERROR;
            } else {
                result = E_UNKNOWN;
            }

            break;
        }
    }

    return result;
}

/* ************************************************************************** */
int S26K_BlankCheck(uint32_t addr)
{
    uint16_t status;
    int result;

    // Issue blank check sequence
    S26K_WRITE16((addr | S26K_UNLOCKADDR0), S26K_CMDBLANK_CHECK);

    while (1) {
        // Issue status read sequence
        S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_STATUS);
        S26K_READ16(0, status);

        // Check status
        if ((status & S26K_STATUS_DEVICE_READY) == S26K_STATUS_DEVICE_READY) {
            if ((status & S26K_STATUS_SECTOR_ERASE_STATUS) == 0x0) {
                result = E_NO_ERROR;
            } else {
                result = E_UNKNOWN;
            }

            break;
        }
    }

    return result;
}

/* ************************************************************************** */
int S26K_SectorErase(uint32_t addr)
{
    uint16_t status;
    int result;

    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDERASE);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(addr, S26K_CMDSECTOR_ERASE);

    while (1) {
        // Issue status read sequence
        S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_STATUS);
        S26K_READ16(0, status);

        // Check status
        if ((status & S26K_STATUS_DEVICE_READY) == S26K_STATUS_DEVICE_READY) {
            if ((status & (S26K_STATUS_SECTOR_ERASE_STATUS | S26K_STATUS_ERASE_STATUS))) {
                result = E_UNKNOWN;
            } else {
                result = E_NO_ERROR;
            }

            break;
        }
    }

    return result;
}

/* ************************************************************************** */
int S26K_Write16(uint32_t addr, uint16_t data)
{
    uint16_t status;
    int result;

    // Issue write16 sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDWRITE16);
    S26K_WRITE16(addr, data);

    while (1) {
        // Issue status read sequence
        S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_STATUS);
        S26K_READ16(0, status);

        // Check status
        if ((status & S26K_STATUS_DEVICE_READY) == S26K_STATUS_DEVICE_READY) {
            if ((status & S26K_STATUS_PGM_STATUS) == 0x0) {
                result = E_NO_ERROR;
            } else {
                result = E_UNKNOWN;
            }

            break;
        }
    }

    return result;
}

/* ************************************************************************** */
void S26K_WriteBufferAbortReset(void)
{
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDABORT_RESET);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
int S26K_Write(uint32_t addr, uint16_t* data, unsigned len)
{
    uint32_t sector_addr, offset;
    uint16_t status;
    unsigned buffer_len, i;

    // make sure that addr is on a buffer size boundary
    if (addr % S26K_BUFFER_SIZE != 0) {
        return E_BAD_PARAM;
    }

    while (len) {
        if (len > S26K_BUFFER_SIZE) {
            buffer_len = S26K_BUFFER_SIZE;
        } else {
            buffer_len = len;
        }

        len -= buffer_len;

        sector_addr = (addr / S26K_SECTOR_SIZE) * S26K_SECTOR_SIZE;
        offset      = addr % S26K_SECTOR_SIZE;
        addr += buffer_len;

        // Issue write to buffer sequence
        S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
        S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
        S26K_WRITE16(sector_addr, S26K_CMDWRITE);
        S26K_WRITE16(sector_addr, buffer_len - 1);

        // Write the data to the buffer
        for (i = 0; i < buffer_len; i++) {
            S26K_WRITE16(sector_addr + offset + i, data[i]);
        }

        // Issue write to buffer conform sequence
        S26K_WRITE16(sector_addr, S26K_CMDWRITE_BUFFER);

        while (1) {
            // Issue status read sequence
            S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_STATUS);
            S26K_READ16(0, status);

            // Check status
            if ((status & S26K_STATUS_DEVICE_READY) == S26K_STATUS_DEVICE_READY) {
                if ((status & S26K_STATUS_PGM_STATUS) != 0x0) {
                    return E_UNKNOWN;
                }

                break;
            }
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
void S26K_GetID(uint32_t offset, uint16_t* data, unsigned len)
{
    unsigned i;

    // Issue ID read sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDREAD_ID);

    // Issue ID/CFI read
    for (i = 0; i < len; i++) {
        S26K_READ16(offset + 2 * i, data[i]);
    }

    // Exit ASO
    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
void S26K_GetCFI(uint32_t offset, uint16_t* data, unsigned len)
{
    unsigned i;

    // Issue CFI read sequence
    S26K_WRITE16(S26K_CFI_ADDR, S26K_CMDREAD_CFI);

    // Issue ID/CFI read
    for (i = 0; i < len; i++) {
        S26K_READ16((offset + 2 * i), data[i]);
    }

    // Exit ASO
    S26K_WRITE16(s26k_base_addr, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
uint16_t S26K_GetSectorProtection(uint32_t addr)
{
    uint16_t status;

    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPB_ENTRY);

    S26K_WRITE16(0x0, S26K_CMDPPB_STATUS);
    S26K_READ16(addr, status);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);

    return status;
}

/* ************************************************************************** */
uint16_t S26K_GetSectorPPB(uint32_t addr)
{
    uint16_t status;

    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPB_ENTRY);

    S26K_READ16(addr, status);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);

    return status;
}

/* ************************************************************************** */
void S26K_SetSectorPPB(uint32_t addr)
{
    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPB_ENTRY);

    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPB_PROGRAM);
    S26K_WRITE16(addr, 0);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
void S26K_PPBErase(void)
{
    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPB_ENTRY);

    S26K_WRITE16(0x0, S26K_CMDPPB_ERASE0);
    S26K_WRITE16(0, S26K_CMDPPB_ERASE1);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
uint16_t S26K_GetPPBLockStatus(void)
{
    uint16_t status;

    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPBLOCK_ENTRY);

    S26K_READ16(0x0, status);
    // S26K_READ16(0, status);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);

    return status;
}

/* ************************************************************************** */
void S26K_ClearPBLock(void)
{
    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDPPBLOCK_ENTRY);

    // S26K_WRITE16(0x0, S26K_CMDPPBLOCK_CLEAR);
    S26K_WRITE16(0, S26K_CMDPPBLOCK_CLEAR);
    S26K_WRITE16(0, 0);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
uint16_t S26K_GetDYBStatus(uint32_t addr)
{
    uint16_t status;

    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDDYB_ENTRY);

    S26K_READ16(addr, status);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);

    return status;
}

/* ************************************************************************** */
void S26K_SetDYB(uint32_t addr)
{
    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDDYB_ENTRY);

    S26K_WRITE16(0x0, S26K_CMDDYB_SET);
    S26K_WRITE16(addr, 0);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
void S26K_ClearDYB(uint32_t addr)
{
    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDDYB_ENTRY);

    S26K_WRITE16(0x0, S26K_CMDDYB_SET);
    S26K_WRITE16(addr, 1);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);
}

/* ************************************************************************** */
uint16_t S26K_GetASPStatus(void)
{
    uint16_t status;

    // Issue sector erase sequence
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_UNLOCKDATA0);
    S26K_WRITE16(S26K_UNLOCKADDR1, S26K_UNLOCKDATA1);
    S26K_WRITE16(S26K_UNLOCKADDR0, S26K_CMDASP_ENTRY);

    S26K_READ16(0, status);

    S26K_WRITE16(0x0, S26K_CMDEXIT_ASO);

    return status;
}

/**@} end of group S26K */

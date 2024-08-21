/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

/**
 * @file mxc_sys.c
 * @brief      System layer driver.
 * @details    This driver is used to control the system layer of the device.
 */

/* **** Includes **** */
#include <stddef.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc.h"
#include "tpu.h"
#include "mxc_delay.h"
#include "gcr_regs.h"
#include "fcr_regs.h"
#include "mcr_regs.h"

/**
 * @ingroup mxc_sys
 * @{
 */

/* **** Definitions **** */
#define MXC_SYS_CLOCK_TIMEOUT MXC_DELAY_MSEC(5)
#define MXC_I2C0 MXC_I2C0_BUS0
#define MXC_I2C1 MXC_I2C1_BUS0
#define MXC_I2C2 MXC_I2C2_BUS0

// DAP Lock macros
#define INFOBLOCK_DAP_LOCK_OFFSET 0x30
#define DAP_LOCK_SEQUENCE 0x5A5AA5A5

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SYS_GetUSN(uint8_t *usn, uint8_t *checksum)
{
    uint32_t *infoblock = (uint32_t *)MXC_INFO0_MEM_BASE;

    /* Read the USN from the info block */
    MXC_FLC_UnlockInfoBlock(MXC_INFO0_MEM_BASE);

    memset(usn, 0, MXC_SYS_USN_CHECKSUM_LEN);

    usn[0] = (infoblock[0] & 0x007F8000) >> 15;
    usn[1] = (infoblock[0] & 0x7F800000) >> 23;
    usn[2] = (infoblock[1] & 0x0000007F) << 1;
    usn[2] |= (infoblock[0] & 0x80000000) >> 31;
    usn[3] = (infoblock[1] & 0x00007F80) >> 7;
    usn[4] = (infoblock[1] & 0x007F8000) >> 15;
    usn[5] = (infoblock[1] & 0x7F800000) >> 23;
    usn[6] = (infoblock[2] & 0x007F8000) >> 15;
    usn[7] = (infoblock[2] & 0x7F800000) >> 23;
    usn[8] = (infoblock[3] & 0x0000007F) << 1;
    usn[8] |= (infoblock[2] & 0x80000000) >> 31;
    usn[9] = (infoblock[3] & 0x00007F80) >> 7;
    usn[10] = (infoblock[3] & 0x007F8000) >> 15;

    // Compute the checksum
    if (checksum != NULL) {
        uint8_t check_csum[MXC_SYS_USN_CHECKSUM_LEN];
        uint8_t key[MXC_SYS_USN_CHECKSUM_LEN];

        /* Initialize the remainder of the USN and key */
        memset(key, 0, MXC_SYS_USN_CHECKSUM_LEN);
        memset(checksum, 0, MXC_SYS_USN_CSUM_FIELD_LEN);

        /* Read the checksum from the info block */
        checksum[1] = ((infoblock[3] & 0x7F800000) >> 23);
        checksum[0] = ((infoblock[4] & 0x007F8000) >> 15);

        MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES128);
        MXC_TPU_Cipher_AES_Encrypt((const char *)usn, NULL, (const char *)key,
                                   MXC_TPU_CIPHER_AES128, MXC_TPU_MODE_ECB, MXC_AES_DATA_LEN,
                                   (char *)check_csum);

        /* Verify the checksum */
        if ((checksum[0] != check_csum[0]) || (checksum[1] != check_csum[1])) {
            MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
            return E_UNKNOWN;
        }
    }

    /* Add the info block checksum to the USN */
    usn[11] = ((infoblock[3] & 0x7F800000) >> 23);
    usn[12] = ((infoblock[4] & 0x007F8000) >> 15);

    MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        return !(MXC_GCR->perckcn1 & (0x1 << clock));
    } else {
        return !(MXC_GCR->perckcn0 & (0x1 << clock));
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->perckcn1 |= (0x1 << clock);
    } else {
        MXC_GCR->perckcn0 |= (0x1 << clock);
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->perckcn1 &= ~(0x1 << clock);
    } else {
        MXC_GCR->perckcn0 &= ~(0x1 << clock);
    }
}
/* ************************************************************************** */
void MXC_SYS_RTCClockEnable()
{
    MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_X32K_EN;
}

/* ************************************************************************** */
int MXC_SYS_RTCClockDisable(void)
{
    /* Check that the RTC is not the system clock source */
    if ((MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CLKSEL) != MXC_S_GCR_CLKCN_CLKSEL_XTAL32K) {
        MXC_GCR->clkcn &= ~MXC_F_GCR_CLKCN_X32K_EN;
        return E_NO_ERROR;
    } else {
        return E_BAD_STATE;
    }
}

/******************************************************************************/
int MXC_SYS_ClockSourceEnable(mxc_sys_system_clock_t clock)
{
    switch (clock) {
    case MXC_SYS_CLOCK_HIRC96:
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC96M_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC96M_RDY);
        break;
    case MXC_SYS_CLOCK_HIRC8:
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC8M_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC8M_RDY);
        break;
    case MXC_SYS_CLOCK_HIRC:
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC_RDY);
        break;
    case MXC_SYS_CLOCK_LIRC8K:
        // The 8k clock is always enabled
        return E_NO_ERROR;
        break;
    case MXC_SYS_CLOCK_XTAL32M:
        MXC_GCR->btle_ldocr |= MXC_F_GCR_BTLE_LDOCR_LDORXEN | MXC_F_GCR_BTLE_LDOCR_LDOTXEN;
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_X32M_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_X32M_RDY);
        break;
    case MXC_SYS_CLOCK_XTAL32K:
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_X32K_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_X32K_RDY);
        break;
    default:
        return E_BAD_PARAM;
        break;
    }
}

/******************************************************************************/
int MXC_SYS_ClockSourceDisable(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;

    current_clock = MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CLKSEL;

    // Don't turn off the clock we're running on
    if (clock == current_clock) {
        return E_BAD_PARAM;
    }

    switch (clock) {
    case MXC_SYS_CLOCK_HIRC96:
        MXC_GCR->clkcn &= ~MXC_F_GCR_CLKCN_HIRC96M_EN;
        break;
    case MXC_SYS_CLOCK_HIRC8:
        MXC_GCR->clkcn &= ~MXC_F_GCR_CLKCN_HIRC8M_EN;
        break;
    case MXC_SYS_CLOCK_HIRC:
        MXC_GCR->clkcn &= ~MXC_F_GCR_CLKCN_HIRC_EN;
        break;
    case MXC_SYS_CLOCK_LIRC8K:
        // The 8k clock is always enabled
        break;
    case MXC_SYS_CLOCK_XTAL32M:
        MXC_GCR->clkcn &= ~MXC_F_GCR_CLKCN_X32M_EN;
        break;
    case MXC_SYS_CLOCK_XTAL32K:
        MXC_GCR->clkcn &= ~MXC_F_GCR_CLKCN_X32K_EN;
        break;
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_Clock_Timeout(uint32_t ready)
{
    // Start timeout, wait for ready
    MXC_DelayAsync(MXC_SYS_CLOCK_TIMEOUT, NULL);

    do {
        if (MXC_GCR->clkcn & ready) {
            MXC_DelayAbort();
            return E_NO_ERROR;
        }
    } while (MXC_DelayCheck() == E_BUSY);

    return E_TIME_OUT;
}

/* ************************************************************************** */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;

    // Save the current system clock
    current_clock = MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CLKSEL;

    switch (clock) {
    case MXC_SYS_CLOCK_HIRC96:
        // Enable HIRC96 clock
        if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC96M_EN)) {
            MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC96M_EN;

            // Check if HIRC96 clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC96M_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set HIRC96 clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_HIRC96);

        break;
    case MXC_SYS_CLOCK_HIRC8:
        // Enable HIRC8 clock
        if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC8M_EN)) {
            MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC8M_EN;

            // Check if HIRC8 clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC8M_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set HIRC8 clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_HIRC8);

        break;
    case MXC_SYS_CLOCK_HIRC:
        // Enable HIRC clock
        if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC_EN)) {
            MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;

            // Check if HIRC clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set HIRC clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_HIRC);

        break;
    case MXC_SYS_CLOCK_XTAL32M:
        // Enable XTAL32M clock
        if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_X32M_EN)) {
            MXC_GCR->btle_ldocr |= MXC_F_GCR_BTLE_LDOCR_LDORXEN | MXC_F_GCR_BTLE_LDOCR_LDOTXEN;
            MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_X32M_EN;

            // Check if XTAL32M clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_X32M_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set XTAL32M clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_XTAL32M);

        break;
    case MXC_SYS_CLOCK_LIRC8K:
        // Set LIRC8 clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_LIRC8);

        break;
    case MXC_SYS_CLOCK_XTAL32K:
        // Enable XTAL32K clock
        if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_X32K_EN)) {
            MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_X32K_EN;

            // Check if XTAL32K clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_X32K_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set XTAL32K clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_XTAL32K);

        break;
    default:
        return E_BAD_PARAM;
    }

    // Wait for system clock to be ready
    if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_CKRDY) != E_NO_ERROR) {
        // Restore the old system clock if timeout
        MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, current_clock);

        return E_TIME_OUT;
    }

    // Update the system core clock
    SystemCoreClockUpdate();

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SYS_Clock_Div(mxc_sys_system_div_t div)
{
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_PSC, div);

    // Update the system core clock
    SystemCoreClockUpdate();
}

/* ************************************************************************** */
void MXC_SYS_Reset_Periph(mxc_sys_reset_t reset)
{
    /* The mxc_sys_reset_t enum uses enum values that are the offset by 32 for the rstr1 register. */
    if (reset > 31) {
        reset -= 32;
        MXC_GCR->rstr1 = (0x1 << reset);
        while (MXC_GCR->rstr1 & (0x1 << reset)) {}
    } else {
        MXC_GCR->rstr0 = (0x1 << reset);
        while (MXC_GCR->rstr0 & (0x1 << reset)) {}
    }
}

/* ************************************************************************** */
uint8_t MXC_SYS_GetRev(void)
{
    uint8_t serialNumber[MXC_SYS_USN_CHECKSUM_LEN];
    MXC_SYS_GetUSN(serialNumber, NULL);

    if ((serialNumber[0] < 0x9F) | ((serialNumber[0] & 0x0F) > 0x09)) {
        // Fail back to the hardware register
        return MXC_GCR->revision & 0xFF;
    }

    return serialNumber[0];
}

/* ************************************************************************** */
int MXC_SYS_LockDAP_Permanent(void)
{
#ifdef DEBUG
    // Locking the DAP is not supported while in DEBUG.
    // To use this function, build for release ("make release")
    // or set DEBUG = 0
    // (see https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-tables)
    return E_NOT_SUPPORTED;
#else
    int err;
    uint32_t info_blk_addr;
    uint32_t lock_sequence[4];

    // Infoblock address to write lock sequence to
    info_blk_addr = MXC_INFO_MEM_BASE + INFOBLOCK_DAP_LOCK_OFFSET;

    // Set lock sequence
    lock_sequence[0] = DAP_LOCK_SEQUENCE;
    lock_sequence[1] = DAP_LOCK_SEQUENCE;
    lock_sequence[2] = DAP_LOCK_SEQUENCE;
    lock_sequence[3] = DAP_LOCK_SEQUENCE;

    // Initialize FLC
    MXC_FLC_Init();

    // Unlock infoblock
    MXC_FLC_UnlockInfoBlock(info_blk_addr);

    // Write DAP lock sequence to infoblock
    err = MXC_FLC_Write128(info_blk_addr, lock_sequence);

    // Re-lock infoblock
    MXC_FLC_LockInfoBlock(info_blk_addr);

    return err;
#endif
}

/**@} end of mxc_sys */

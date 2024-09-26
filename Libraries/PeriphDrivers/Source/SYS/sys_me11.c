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
#include "mxc_errors.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "flc.h"
#include "gcr_regs.h"
#include "fcr_regs.h"

/**
 * @ingroup mxc_sys
 * @{
 */

/* **** Definitions **** */
#define MXC_SYS_CLOCK_TIMEOUT MXC_DELAY_MSEC(1)

// DAP Lock macros
#define INFOBLOCK_DAP_LOCK_OFFSET 0x30
#define DAP_LOCK_SEQUENCE_01 0x5A5AA5A5
#define DAP_LOCK_SEQUENCE_23 0xFFFFFFFF

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SYS_GetUSN(uint8_t *usn, int len, int part)
{
    if (len != MXC_SYS_USN_LEN) {
        return E_BAD_PARAM;
    } else if (usn == NULL) {
        return E_NULL_PTR;
    }

    uint32_t infoblock[6];

    MXC_FLC_UnlockInfoBlock(MXC_INFO_MEM_BASE);
    infoblock[0] = *(uint32_t *)MXC_INFO_MEM_BASE;
    infoblock[1] = *(uint32_t *)(MXC_INFO_MEM_BASE + 4);
    infoblock[2] = *(uint32_t *)(MXC_INFO_MEM_BASE + 8);
    infoblock[3] = *(uint32_t *)(MXC_INFO_MEM_BASE + 12);
    infoblock[4] = *(uint32_t *)(MXC_INFO_MEM_BASE + 16);
    infoblock[5] = *(uint32_t *)(MXC_INFO_MEM_BASE + 20);
    MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);

    if (part == 0) {
        usn[0] = (infoblock[0] & 0x000000FF);
        usn[1] = (infoblock[0] & 0x0000FF00) >> 8;
        usn[2] = (infoblock[0] & 0x00FF0000) >> 16;
        usn[3] = (infoblock[0] & 0x3F000000) >> 24;
        usn[3] |= (infoblock[1] & 0x00000003) << 6;
        usn[4] = (infoblock[1] & 0x000003FC) >> 2;
        usn[5] = (infoblock[1] & 0x0003FC00) >> 10;
        usn[6] = (infoblock[1] & 0x03FC0000) >> 18;
        usn[7] = (infoblock[1] & 0x3C000000) >> 26;
    } else if (part == 1) {
        usn[0] = (infoblock[2] & 0x000000FF);
        usn[1] = (infoblock[2] & 0x0000FF00) >> 8;
        usn[2] = (infoblock[2] & 0x00FF0000) >> 16;
        usn[3] = (infoblock[2] & 0xFF000000) >> 24;
        usn[4] = (infoblock[3] & 0x000000FF);
        usn[5] = (infoblock[3] & 0x0000FF00) >> 8;
        usn[6] = (infoblock[3] & 0x00FF0000) >> 16;
        usn[7] = (infoblock[3] & 0xFF000000) >> 24;
    } else if (part == 2) {
        usn[0] = (infoblock[4] & 0x000000FF);
        usn[1] = (infoblock[4] & 0x0000FF00) >> 8;
        usn[2] = (infoblock[4] & 0x00FF0000) >> 16;
        usn[3] = (infoblock[4] & 0xFF000000) >> 24;
        usn[4] = (infoblock[5] & 0x000000FF);
        usn[5] = (infoblock[5] & 0x0000FF00) >> 8;
        usn[6] = (infoblock[5] & 0x00FF0000) >> 16;
        usn[7] = (infoblock[5] & 0xFF000000) >> 24;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the pclkdis1 register. */
    if (clock > 31) {
        clock -= 32;
        return !(MXC_GCR->pclk_dis1 & (0x1 << clock));
    } else {
        return !(MXC_GCR->pclk_dis0 & (0x1 << clock));
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the pclkdis1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclk_dis1 |= (0x1 << clock);
    } else {
        MXC_GCR->pclk_dis0 |= (0x1 << clock);
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the pclkdis1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclk_dis1 &= ~(0x1 << clock);
    } else {
        MXC_GCR->pclk_dis0 &= ~(0x1 << clock);
    }
}
/* ************************************************************************** */
void MXC_SYS_RTCClockEnable()
{
    MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_X32K_EN;
}

/* ************************************************************************** */
int MXC_SYS_RTCClockDisable(void)
{
    /* Check that the RTC is not the system clock source */
    if ((MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CLKSEL) != MXC_S_GCR_CLK_CTRL_CLKSEL_HFXIN) {
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_X32K_EN;
        return E_NO_ERROR;
    } else {
        return E_BAD_STATE;
    }
}

/******************************************************************************/
int MXC_SYS_ClockSourceEnable(mxc_sys_system_clock_t clock)
{
    switch (clock) {
    case MXC_SYS_CLOCK_HIRC:
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_HIRC_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_HIRC_RDY);
        break;

    case MXC_SYS_CLOCK_HFXIN:
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_X32K_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_X32K_RDY);
        break;

    case MXC_SYS_CLOCK_NANORING:
        // 80khz nanoring is always enabled
        return E_NO_ERROR;
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

    current_clock = MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CLKSEL;

    // Don't turn off the clock we're running on
    if (clock == current_clock) {
        return E_BAD_PARAM;
    }

    switch (clock) {
    case MXC_SYS_CLOCK_HIRC:
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_HIRC_EN;
        break;

    case MXC_SYS_CLOCK_HFXIN:
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_X32K_EN;
        break;

    case MXC_SYS_CLOCK_NANORING:
        // 80khz nanoring is always enabled
        return E_BAD_PARAM;

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
        if (MXC_GCR->clk_ctrl & ready) {
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
    current_clock = MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CLKSEL;

    switch (clock) {
    case MXC_SYS_CLOCK_HIRC:

        // Enable HIRC clock
        if (!(MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_HIRC_EN)) {
            MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_HIRC_EN;

            // Check if HIRC clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_HIRC_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set HIRC clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_CLKSEL, MXC_S_GCR_CLK_CTRL_CLKSEL_HIRC);

        break;

    case MXC_SYS_CLOCK_HFXIN:

        // Enable HFXtal clock
        if (!(MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_X32K_EN)) {
            MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_X32K_EN;

            // Check if HFXtal clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_X32K_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set HFXtal clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_CLKSEL, MXC_S_GCR_CLK_CTRL_CLKSEL_HFXIN);

        break;

    case MXC_SYS_CLOCK_NANORING:
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_LIRC8K_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }

        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_CLKSEL,
                     MXC_S_GCR_CLK_CTRL_CLKSEL_NANORING);

        break;

    default:
        return E_BAD_PARAM;
    }

    // Wait for system clock to be ready
    if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_CLKRDY) != E_NO_ERROR) {
        // Restore the old system clock if timeout
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_CLKSEL, current_clock);

        return E_TIME_OUT;
    }

    // Update the system core clock
    SystemCoreClockUpdate();

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SYS_SetClockDiv(mxc_sys_system_clock_div_t div)
{
    /* Return if this setting is already current */
    if (div == MXC_SYS_GetClockDiv()) {
        return;
    }

    MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_PSC, div);

    SystemCoreClockUpdate();
}

/* ************************************************************************** */
mxc_sys_system_clock_div_t MXC_SYS_GetClockDiv(void)
{
    return (MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_PSC);
}

/* ************************************************************************** */
void MXC_SYS_Reset_Periph(mxc_sys_reset_t reset)
{
    /* The mxc_sys_reset_t enum uses enum values that are the offset by 32 for the rstr1 register. */
    if (reset > 31) {
        reset -= 32;
        MXC_GCR->rst1 = (0x1 << reset);
        while (MXC_GCR->rst1 & (0x1 << reset)) {}
    } else {
        MXC_GCR->rst0 = (0x1 << reset);
        while (MXC_GCR->rst0 & (0x1 << reset)) {}
    }
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
    lock_sequence[0] = DAP_LOCK_SEQUENCE_01;
    lock_sequence[1] = DAP_LOCK_SEQUENCE_01;
    lock_sequence[2] = DAP_LOCK_SEQUENCE_23;
    lock_sequence[3] = DAP_LOCK_SEQUENCE_23;

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

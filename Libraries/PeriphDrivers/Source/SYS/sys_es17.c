/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2016-10-10 15:10:41 -0500 (Mon, 10 Oct 2016) $
 * $Revision: 24650 $
 *
 *************************************************************************** */

/**
 * @file mxc_sys.c
 * @brief      System layer driver.
 * @details    This driver is used to control the system layer of the device.
 */

/* **** Includes **** */
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc.h"
#include "mxc_delay.h"
#include "gcr_regs.h"
#include "fcr_regs.h"
#include "mcr_regs.h"

/**
 * @ingroup mxc_sys
 * @{
 */

/* **** Definitions **** */
#define MXC_SYS_CLOCK_TIMEOUT MSEC(1)

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the pclkdis1 register. */
    if (clock > 31) {
        clock -= 32;
        return !(MXC_GCR->pclkdis1 & (0x1 << clock));
    } else {
        return !(MXC_GCR->pclkdis0 & (0x1 << clock));
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the pclkdis1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclkdis1 |= (0x1 << clock);
    } else {
        MXC_GCR->pclkdis0 |= (0x1 << clock);
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 for the pclkdis1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclkdis1 &= ~(0x1 << clock);
    } else {
        MXC_GCR->pclkdis0 &= ~(0x1 << clock);
    }
}

/******************************************************************************/
int MXC_SYS_ClockSourceEnable(mxc_sys_system_clock_t clock)
{
    switch (clock) {
    case MXC_SYS_CLOCK_IPO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IPO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IPO_RDY);
        break;

    case MXC_SYS_CLOCK_IBRO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IBRO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IBRO_RDY);
        break;

    case MXC_SYS_CLOCK_INRO:
        // The nano ring clock is always enabled
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_INRO_RDY);
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

    current_clock = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL;

    // Don't turn off the clock we're running on
    if (clock == current_clock) {
        return E_BAD_PARAM;
    }

    switch (clock) {
    case MXC_SYS_CLOCK_IPO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_IPO_EN;
        break;

    case MXC_SYS_CLOCK_IBRO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_IBRO_EN;
        break;

    case MXC_SYS_CLOCK_INRO:
        // The 8k clock is always enabled
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
        if (MXC_GCR->clkctrl & ready) {
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
    current_clock = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL;

    switch (clock) {
    case MXC_SYS_CLOCK_IPO:

        // Enable IPO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IPO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IPO_EN;

            // Check if IPO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IPO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set IPO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPO);

        break;

    case MXC_SYS_CLOCK_IBRO:

        // Enable IBRO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IBRO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IBRO_EN;

            // Check if IBRO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IBRO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set IBRO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IBRO);

        break;

    case MXC_SYS_CLOCK_INRO:
        // Set INRO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_INRO);

        break;

    default:
        return E_BAD_PARAM;
    }

    // Wait for system clock to be ready
    if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_SYSCLK_RDY) != E_NO_ERROR) {
        // Restore the old system clock if timeout
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL, current_clock);

        return E_TIME_OUT;
    }

    // Update the system core clock
    SystemCoreClockUpdate();

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SYS_Reset_Periph(mxc_sys_reset_t reset)
{
    /* The mxc_sys_reset_t enum uses enum values that are the offset by 32 for the rst1 register. */
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
int MXC_SYS_GetUSN(uint8_t *serialNumber, int len)
{
    if (len != 13) {
        return E_BAD_PARAM;
    }

    uint32_t infoblock[6];

    MXC_FLC_UnlockInfoBlock(0x0000);
    infoblock[0] = *(uint32_t *)MXC_INFO_MEM_BASE;
    infoblock[1] = *(uint32_t *)(MXC_INFO_MEM_BASE + 4);
    infoblock[2] = *(uint32_t *)(MXC_INFO_MEM_BASE + 8);
    infoblock[3] = *(uint32_t *)(MXC_INFO_MEM_BASE + 12);
    infoblock[4] = *(uint32_t *)(MXC_INFO_MEM_BASE + 16);
    infoblock[5] = *(uint32_t *)(MXC_INFO_MEM_BASE + 20);
    MXC_FLC_LockInfoBlock(0x0000);

    serialNumber[0] = (infoblock[0] & 0x007F8000) >> 15;
    serialNumber[1] = (infoblock[0] & 0x7F800000) >> 23;
    serialNumber[2] = (infoblock[0] & 0x80000000) >> 31;
    serialNumber[2] |= (infoblock[1] & 0x0000007F) << 1;
    serialNumber[3] = (infoblock[1] & 0x00007F80) >> 7;
    serialNumber[4] = (infoblock[1] & 0x007F8000) >> 15;
    serialNumber[5] = (infoblock[1] & 0x7F800000) >> 23;
    serialNumber[6] = (infoblock[2] & 0x007F8000) >> 15;
    serialNumber[7] = (infoblock[2] & 0x7F800000) >> 23;
    serialNumber[8] = (infoblock[2] & 0x80000000) >> 31;
    serialNumber[8] |= (infoblock[3] & 0x0000007F) << 1;
    serialNumber[9] = (infoblock[3] & 0x00007F80) >> 7;
    serialNumber[10] = (infoblock[3] & 0x007F8000) >> 15;
    serialNumber[11] = (infoblock[3] & 0x7F800000) >> 23;
    serialNumber[12] = (infoblock[4] & 0x007F8000) >> 15;

    return E_NO_ERROR;
}

/* ************************************************************************** */
uint8_t MXC_SYS_GetRev(void)
{
    uint8_t serialNumber[13];
    MXC_SYS_GetUSN(serialNumber, 13);

    if ((serialNumber[0] < 0x9F) | ((serialNumber[0] & 0x0F) > 0x09)) {
        // Fail back to the hardware register
        return MXC_GCR->revision & 0xFF;
    }

    return serialNumber[0];
}
/**@} end of mxc_sys */

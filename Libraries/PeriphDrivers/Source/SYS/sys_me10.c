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

/* **** Includes **** */
#include <stddef.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "gcr_regs.h"
#include "tmr_regs.h"
#include "flc_regs.h"
#include "icc_regs.h"
#include "nbbfc_regs.h"
#include "mxc_delay.h"
#include "sdhc_regs.h"
#include "usbhs_regs.h"
#include "bbfc_regs.h"
#include "hpb_regs.h"
#include "rtc.h"
#include "flc.h"
#include "pwrseq_regs.h"

/* **** Definitions **** */
#define SYS_CLOCK_TIMEOUT MXC_DELAY_MSEC(1)

#define SYS_RTC_CLK 32768UL

// DAP Lock macros
#define INFOBLOCK_DAP_LOCK_OFFSET 0x30
#define DAP_LOCK_SEQUENCE_01 0x5A5AA5A5
#define DAP_LOCK_SEQUENCE_23 0xFFFFFFFF

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock)
{
    /* The sys_periph_clock_t enum uses enum values that are the offset by 32 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        return !(MXC_GCR->pclk_dis1 & (0x1 << clock));
    } else {
        return !(MXC_GCR->pclk_dis0 & (0x1 << clock));
    }
}

/* ************************************************************************** */
int MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock)
{
    /* The sys_periph_clock_t enum uses enum values that are the offset by 32 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclk_dis1 |= (0x1 << clock);
    } else {
        MXC_GCR->pclk_dis0 |= (0x1 << clock);
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock)
{
    /* The sys_periph_clock_t enum uses enum values that are the offset by 32 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclk_dis1 &= ~(0x1 << clock);
    } else {
        MXC_GCR->pclk_dis0 &= ~(0x1 << clock);
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_RTCClockEnable(void)
{
    // Enable 32k Oscillator
    MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_X32K_EN;

    // Check if 32k clock is ready
    if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_X32K_RDY) != E_NO_ERROR) {
        return E_TIME_OUT;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_RTCClockDisable(void)
{
    // Enable 32k Oscillator
    MXC_GCR->clk_ctrl &= (~MXC_F_GCR_CLK_CTRL_X32K_EN);
    return E_NO_ERROR;
}

/**
 * @brief Disable System Clock Source
 * @param      clock The clock to disable
 * @return     E_NO_ERROR if everything is successful
 */
int MXC_SYS_ClockSourceDisable(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;

    current_clock = MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_SYSOSC_SEL;

    // Don't turn off the clock we're running on
    if (clock == current_clock) {
        return E_BAD_PARAM;
    }

    switch (clock) {
    case MXC_SYS_CLOCK_CRYPTO:
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_CRYPTO_EN;
        break;
    case MXC_SYS_CLOCK_NANORING: //Always enabled
        break;
    case MXC_SYS_CLOCK_HIRC96:
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_HIRC96_EN;
        break;
    case MXC_SYS_CLOCK_HIRC8:
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_HIRC8_EN;
        break;
    case MXC_SYS_CLOCK_X32K:
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_X32K_EN;
        break;
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/**
 * @brief Enable System Clock Source without switching to it
 * @param      clock The clock to enable
 * @return     E_NO_ERROR if everything is successful
 */
int MXC_SYS_ClockSourceEnable(mxc_sys_system_clock_t clock)
{
    switch (clock) {
    case MXC_SYS_CLOCK_CRYPTO:
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_CRYPTO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_CRYPTO_RDY);
    case MXC_SYS_CLOCK_NANORING: //Always enabled
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_NANORING_RDY);
    case MXC_SYS_CLOCK_HIRC96:
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_HIRC96_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_HIRC96_RDY);
    case MXC_SYS_CLOCK_HIRC8:
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_HIRC8_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_HIRC8_RDY);
    case MXC_SYS_CLOCK_X32K:
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_X32K_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_X32K_RDY);
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;

    // Save the current system clock
    current_clock = MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_SYSOSC_SEL;

    switch (clock) {
    case MXC_SYS_CLOCK_CRYPTO:
        // Enable CRYPTO clock
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_CRYPTO_EN;

        // Check if CRYPTO clock is ready
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_CRYPTO_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }

        // Set CRYPTO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSOSC_SEL,
                     MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_CRYPTO);

        break;
    case MXC_SYS_CLOCK_NANORING:
        // Check if NANORING clock is ready
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_NANORING_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }

        // Set NANORING clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSOSC_SEL,
                     MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_NANORING);

        break;
    case MXC_SYS_CLOCK_HIRC96:
        // Enable HIRC96 clock
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_HIRC96_EN;

        // Check if HIRC96 clock is ready
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_HIRC96_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }

        // Set HIRC96 clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSOSC_SEL,
                     MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96);

        break;
    case MXC_SYS_CLOCK_HIRC8:
        // Enable HIRC8 clock
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_HIRC8_EN;

        // Check if HIRC8 clock is ready
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_HIRC8_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }

        // Set HIRC8 clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSOSC_SEL,
                     MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC8);

        break;
    case MXC_SYS_CLOCK_HFXIN:
    case MXC_SYS_CLOCK_X32K:
        // Enable 32k Oscillator
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_X32K_EN;

        // Check if 32k clock is ready
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_X32K_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }

        // Set 32k clock as System Clock
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSOSC_SEL, MXC_F_GCR_CLK_CTRL_X32K_RDY);

        break;
    default:
        return E_BAD_PARAM;
    }

    // Wait for system clock to be ready
    if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLK_CTRL_SYSOSC_RDY) != E_NO_ERROR) {
        // Restore the old system clock if timeout
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSOSC_SEL, current_clock);

        return E_TIME_OUT;
    }

    // Disable other clocks
    switch (clock) {
    case MXC_SYS_CLOCK_CRYPTO:
        MXC_GCR->clk_ctrl &= ~(MXC_F_GCR_CLK_CTRL_X32K_EN | MXC_F_GCR_CLK_CTRL_HIRC96_EN |
                               MXC_F_GCR_CLK_CTRL_HIRC8_EN);
        break;

    case MXC_SYS_CLOCK_NANORING:
        MXC_GCR->clk_ctrl &= ~(MXC_F_GCR_CLK_CTRL_X32K_EN | MXC_F_GCR_CLK_CTRL_CRYPTO_EN |
                               MXC_F_GCR_CLK_CTRL_HIRC96_EN | MXC_F_GCR_CLK_CTRL_HIRC8_EN);
        break;

    case MXC_SYS_CLOCK_HIRC96:
        MXC_GCR->clk_ctrl &= ~(MXC_F_GCR_CLK_CTRL_X32K_EN | MXC_F_GCR_CLK_CTRL_CRYPTO_EN |
                               MXC_F_GCR_CLK_CTRL_HIRC8_EN);
        break;

    case MXC_SYS_CLOCK_HIRC8:
        MXC_GCR->clk_ctrl &= ~(MXC_F_GCR_CLK_CTRL_X32K_EN | MXC_F_GCR_CLK_CTRL_CRYPTO_EN |
                               MXC_F_GCR_CLK_CTRL_HIRC96_EN);
        break;

    case MXC_SYS_CLOCK_HFXIN:
    case MXC_SYS_CLOCK_X32K:
        MXC_GCR->clk_ctrl &= ~(MXC_F_GCR_CLK_CTRL_CRYPTO_EN | MXC_F_GCR_CLK_CTRL_HIRC96_EN |
                               MXC_F_GCR_CLK_CTRL_HIRC8_EN);
        break;
    }

    // Update the system core clock
    SystemCoreClockUpdate();

    return E_NO_ERROR;
}

/**
 * @brief Select the system clock divider.
 * @param clock     Enumeration for desired system clock divider.
 */
void MXC_SYS_Clock_Div(mxc_sys_system_div_t div)
{
    MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE, div);

    // Update the system core clock
    SystemCoreClockUpdate();
}

/* ************************************************************************** */
int MXC_SYS_Clock_Timeout(uint32_t ready)
{
    // Start timeout, wait for ready
    MXC_DelayAsync(SYS_CLOCK_TIMEOUT, NULL);
    do {
        if (MXC_GCR->clk_ctrl & ready) {
            MXC_DelayAbort();
            return E_NO_ERROR;
        }
    } while (MXC_DelayCheck() == E_BUSY);

    return E_TIME_OUT;
}

/* ************************************************************************** */
int MXC_SYS_Reset_Periph(mxc_sys_reset_t reset)
{
    if (reset > 31) {
        reset -= 32;
        MXC_GCR->rst1 = (1 << reset);
        while (MXC_GCR->rst1 != 0x0) {}
    } else {
        MXC_GCR->rst0 = (1 << reset);
        while (MXC_GCR->rst0 != 0x0) {}
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
uint8_t MXC_SYS_GetRev(void)
{
    uint8_t serialNumber[MXC_SYS_USN_LEN];
    MXC_SYS_GetUSN(serialNumber, MXC_SYS_USN_LEN);

    if ((serialNumber[0] < 0x9F) | ((serialNumber[0] & 0x0F) > 0x09)) {
        // Fail back to the hardware register
        return MXC_GCR->rev & 0xFF;
    }
    return serialNumber[0];
}

/* ************************************************************************** */
int MXC_SYS_GetUSN(uint8_t *serialNumber, int len)
{
    if (len != MXC_SYS_USN_LEN) {
        return E_BAD_PARAM;
    } else if (serialNumber == NULL) {
        return E_NULL_PTR;
    }

    uint32_t infoblock[6];

    MXC_FLC_UnlockInfoBlock(MXC_INFO_MEM_BASE);
    infoblock[0] = *(uint32_t *)0x10800000;
    infoblock[1] = *(uint32_t *)0x10800004;
    infoblock[2] = *(uint32_t *)0x10800008;
    infoblock[3] = *(uint32_t *)0x1080000C;
    infoblock[4] = *(uint32_t *)0x10800010;
    infoblock[5] = *(uint32_t *)0x10800014;
    MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);

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
int MXC_SYS_SCACHE_Init(const mxc_sys_cfg_scache_t *sys_cfg)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_SCACHE_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SCACHE);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_USBHS_Init(const mxc_sys_cfg_usbhs_t *sys_cfg)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_USB);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_USBHS_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_USB);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_SysTick_Config(uint32_t ticks, int clk_src, mxc_tmr_regs_t *tmr)
{
    if (ticks == 0)
        return E_BAD_PARAM;

    /* If SystemClock, call default CMSIS config and return */
    if (clk_src) {
        return SysTick_Config(ticks);
    } else { /* External clock source requested. Enable RTC clock in run mode*/
        MXC_RTC_Init(0, 0);
        MXC_RTC_Start();

        /* Disable SysTick Timer */
        SysTick->CTRL = 0;
        /* Check reload value for valid */
        if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk) {
            /* Reload value impossible */
            return E_BAD_PARAM;
        }
        /* set reload register */
        SysTick->LOAD = ticks - 1;

        /* set Priority for Systick Interrupt */
        NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

        /* Load the SysTick Counter Value */
        SysTick->VAL = 0;

        /* Enable SysTick IRQ and SysTick Timer leaving clock source as external */
        SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Function successful */
        return E_NO_ERROR;
    }
}

/* ************************************************************************** */
int MXC_SYS_SysTick_Disable(void)
{
    SysTick->CTRL = 0;

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_SysTick_Delay(uint32_t ticks)
{
    uint32_t cur_ticks, num_full, num_remain, previous_ticks, num_subtract, i;
    uint32_t reload, value, ctrl; /* save/restore variables */

    if (ticks == 0)
        return E_BAD_PARAM;

    /* If SysTick is not enabled we can take it for our delay */
    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)) {
        /* Save current state in case it's disabled but already configured, restore at return.*/
        reload = SysTick->LOAD;
        value = SysTick->VAL;
        ctrl = SysTick->CTRL;

        /* get the number of ticks less than max RELOAD. */
        num_remain = ticks % SysTick_LOAD_RELOAD_Msk;

        /* if ticks is < Max SysTick Reload num_full will be 0, otherwise it will
           give us the number of max SysTicks cycles required */
        num_full = (ticks - 1) / SysTick_LOAD_RELOAD_Msk;

        /* Do the required full systick countdowns */
        if (num_full) {
            /* load the max count value into systick */
            SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
            /* load the starting value */
            SysTick->VAL = 0;
            /*enable SysTick counter with SystemClock source internal, immediately forces LOAD register into VAL register */
            SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            /* CountFlag will get set when VAL reaches zero */
            for (i = num_full; i > 0; i--) {
                do {
                    cur_ticks = SysTick->CTRL;
                } while (!(cur_ticks & SysTick_CTRL_COUNTFLAG_Msk));
            }
            /* Disable systick */
            SysTick->CTRL = 0;
        }
        /* Now handle the remainder of ticks */
        if (num_remain) {
            SysTick->LOAD = num_remain;
            SysTick->VAL = 0;
            SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            /* wait for countflag to get set */
            do {
                cur_ticks = SysTick->CTRL;
            } while (!(cur_ticks & SysTick_CTRL_COUNTFLAG_Msk));
            /* Disable systick */
            SysTick->CTRL = 0;
        }

        /* restore original state of SysTick and return */
        SysTick->LOAD = reload;
        SysTick->VAL = value;
        SysTick->CTRL = ctrl;

        return E_NO_ERROR;

    } else {
        /* SysTick is enabled
           When SysTick is enabled count flag can not be used
           and the reload can not be changed.
           Do not read the CTRL register -> clears count flag */

        /* Get the reload value for wrap/reload case */
        reload = SysTick->LOAD;

        /* Read the starting systick value */
        previous_ticks = SysTick->VAL;

        do {
            /* get current SysTick value */
            cur_ticks = SysTick->VAL;
            /* Check for wrap/reload of timer countval */
            if (cur_ticks > previous_ticks) {
                /* subtract count to 0 (previous_ticks) and wrap (reload value - cur_ticks) */
                num_subtract = (previous_ticks + (reload - cur_ticks));
            } else {
                /* standard case (no wrap) - subtract off the number of ticks since last pass */
                num_subtract = (previous_ticks - cur_ticks);
            }
            /* check to see if we are done. */
            if (num_subtract >= ticks)
                return E_NO_ERROR;
            else
                ticks -= num_subtract;
            /* cur_ticks becomes previous_ticks for next timer read. */
            previous_ticks = cur_ticks;
        } while (ticks > 0);
        /* Should not ever be reached */
        return E_NO_ERROR;
    }
}

/* ************************************************************************** */
uint32_t MXC_SYS_SysTick_GetFreq(void)
{
    /* Determine is using internal (SystemCoreClock) or external (32768) clock */
    if ((SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk) ||
        !(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)) {
        return SystemCoreClock;
    } else {
        return SYS_RTC_CLK;
    }
}

/* ************************************************************************** */
int MXC_SYS_SysTick_DelayUs(uint32_t us)
{
    MXC_SYS_SysTick_Delay((uint32_t)(((uint64_t)MXC_SYS_SysTick_GetFreq() * us) / 1000000));

    return E_NO_ERROR;
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

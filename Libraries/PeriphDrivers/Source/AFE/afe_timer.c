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
#include <stdint.h>
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "afe_timer.h"
#include "nvic_table.h"
#include "tmr_revb.h"

#define MAX32675_FIRST_LP_TIMER_INSTANCE_NUMBER 4
#define MAX32680_FIRST_LP_TIMER_INSTANCE_NUMBER 4

#define TIMER_16A_OFFSET 0
#define TIMER_16B_OFFSET 16

// Globals
afe_timeout_complete_t afe_irq_callback;
afe_timeout_complete_t hart_irq_callback;

uint32_t g_timer_prescaler = 0;
mxc_tmr_regs_t *g_afe_timer_inst;
uint32_t g_afe_spi_timer_complete = 0;
uint32_t g_afe_hart_timer_complete = 0;

static void AFE_TMR_Stop_16(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bitMode)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint32_t timerOffset;

    if (bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    tmr->ctrl0 &= ~(MXC_F_TMR_CTRL0_EN_A << timerOffset);
}

static void AFE_TMR_SetCompare_16(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bitMode, uint16_t cmp_cnt)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint32_t timerOffset;

    if (bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    tmr->cmp &= ~(0xFFFF << timerOffset);
    tmr->cmp |= (cmp_cnt << timerOffset);
}

static void AFE_TMR_SetCount_16(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bitMode, uint16_t cnt)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint32_t timerOffset;

    if (bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    // Note: must wait for Timer Write Done when setting TMR_CNT
    tmr->cnt &= ~(0xFFFF << timerOffset);
    while (!(tmr->intfl & (MXC_F_TMR_REVB_INTFL_WRDONE_A << timerOffset))) {}
    tmr->cnt |= (cnt << timerOffset);
    while (!(tmr->intfl & (MXC_F_TMR_REVB_INTFL_WRDONE_A << timerOffset))) {}
}

static void AFE_TMR_ClearFlags_16(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bitMode)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint32_t timerOffset;

    if (bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    tmr->intfl |= (MXC_F_TMR_INTFL_IRQ_A << timerOffset);
}

static void AFE_TMR_EnableInt_16(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bitMode)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint32_t timerOffset;

    if (bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    tmr->ctrl1 |= (MXC_F_TMR_CTRL1_IE_A << timerOffset);
}

static void AFE_TMR_Start_16(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bitMode)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint32_t timerOffset;

    if (bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    tmr->ctrl0 |= (MXC_F_TMR_CTRL0_EN_A << timerOffset);
    while (!(tmr->ctrl1 & (MXC_F_TMR_CTRL1_CLKEN_A << timerOffset))) {}
}

// Note, this only applies to NON-Lower Power Timers
static int AFE_TMR_GetClockSrc(mxc_tmr_clock_t clk, uint8_t *clockSource)
{
    switch (clk) {
    case MXC_TMR_APB_CLK:
        *clockSource = MXC_TMR_CLK0;
        break;

    case MXC_TMR_8M_CLK:
        *clockSource = MXC_TMR_CLK2;
        break;

    case MXC_TMR_32M_CLK:
        *clockSource = MXC_TMR_CLK3;
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

static int AFE_TMR_Config_16(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    uint32_t timerOffset;
    uint8_t clk_src = 0;
    int status = E_NO_ERROR;
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    if (cfg->bitMode == TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    // Convert Clock Type
    status = AFE_TMR_GetClockSrc(cfg->clock, &clk_src);
    if (status != E_NO_ERROR) {
        return status;
    }

    // Clear any existing config
    tmr->ctrl0 &= ~(0xFFFF << timerOffset);
    tmr->ctrl1 &= ~(0xFFFF << timerOffset);
    tmr->cmp &= ~(0xFFFF << timerOffset);
    tmr->cnt &= ~(0xFFFF << timerOffset);

    tmr->ctrl0 |= (MXC_F_TMR_CTRL0_CLKEN_A << timerOffset);
    while (!(tmr->ctrl1 & (MXC_F_TMR_CTRL1_CLKRDY_A << timerOffset))) {}

    tmr->ctrl0 |= (cfg->mode << timerOffset);
    tmr->ctrl0 |= ((cfg->pol << MXC_F_TMR_CTRL0_POL_A_POS) << timerOffset);
    tmr->ctrl0 |= ((cfg->pres << MXC_F_TMR_CTRL0_CLKDIV_A_POS) << timerOffset);
    tmr->ctrl1 |= ((clk_src << MXC_F_TMR_CTRL0_CLKDIV_A_POS) << timerOffset);
    tmr->cnt = (0x1 << timerOffset);
    tmr->cmp = (cfg->cmp_cnt << timerOffset);

    return E_NO_ERROR;
}

static void afe_timer_interrupt_handler(void)
{
    uint32_t flags = MXC_TMR_GetFlags(g_afe_timer_inst);

    if (flags & MXC_F_TMR_INTFL_IRQ_A) {
        // AFE Timer Interrupt
        AFE_TMR_ClearFlags_16(g_afe_timer_inst, AFE_SPI_TIMER);

        g_afe_spi_timer_complete = 1;

        if (afe_irq_callback) {
            afe_irq_callback(E_ABORT);
        }
    } else if (flags & MXC_F_TMR_INTFL_IRQ_B) {
        // HART Timer Interrupt
        AFE_TMR_ClearFlags_16(g_afe_timer_inst, HART_TIMER);

        g_afe_hart_timer_complete = 1;

        if (hart_irq_callback) {
            hart_irq_callback(E_ABORT);
        }
    }
}

static int afe_timer_validate_id(mxc_tmr_regs_t *tmr)
{
    uint8_t tmr_id = 0;
    MXC_ASSERT((tmr_id = MXC_TMR_GET_IDX(tmr)) >= 0);

#if (TARGET_NUM == 32675)
    if (tmr_id >= MAX32675_FIRST_LP_TIMER_INSTANCE_NUMBER) {
        return E_NOT_SUPPORTED;
    }
#elif (TARGET_NUM == 32680)
    if (tmr_id >= MAX32680_FIRST_LP_TIMER_INSTANCE_NUMBER) {
        return E_NOT_SUPPORTED;
    }
#else
#error "Selected TARGET is not known to have an AFE\n"
#endif

    g_afe_timer_inst = tmr;

    return E_NO_ERROR;
}

static int afe_timer_choose_prescaler(void)
{
    uint32_t timer_input_clk = 0;
    uint32_t scaled_clk = 0;
    int32_t scaled_diff_prev = 0;
    int32_t scaled_diff_curr = 0;

    g_timer_prescaler = 0;

    SystemCoreClockUpdate(); // Ensure Clock is up to date
    timer_input_clk = PeripheralClock;

    // Don't support input clock less than 1Mhz
    if (timer_input_clk < 1000000) {
        return E_NOT_SUPPORTED;
    }

    scaled_clk = timer_input_clk / 1000000;

    // Prescaler formula is input_clk/(2^prescaler) I.E. prescaler of 0 is div 1.
    for (g_timer_prescaler = 0; g_timer_prescaler < MXC_V_TMR_CTRL0_CLKDIV_A_DIV_BY_4096;
         g_timer_prescaler++) {
        if (scaled_clk <= (1 << g_timer_prescaler)) {
            break;
        }
    }

    // Should be close now, choose current prescaler or previous one based on diff to 1Mhz
    scaled_diff_curr = scaled_clk - (1 << g_timer_prescaler);
    scaled_diff_prev = scaled_clk - (1 << (g_timer_prescaler - 1));

    // Get absolute value
    if (scaled_diff_curr < 0) {
        scaled_diff_curr *= -1;
    }
    if (scaled_diff_prev < 0) {
        scaled_diff_prev *= -1;
    }

    if (scaled_diff_curr > scaled_diff_prev) {
        // Previous scaler was closer, use it
        g_timer_prescaler--;
    }

    return E_NO_ERROR;
}

int afe_timer_config(mxc_tmr_regs_t *tmr)
{
    mxc_tmr_cfg_t afe_tmr_cfg;
    mxc_tmr_cfg_t hart_tmr_cfg;
    int status = E_NO_ERROR;

    // We require TO detection for SPI transactions with AFE,
    //  and for accurately timed callbacks for HART UART transmitter.
    // So we will split a system timer into two 16 bit timers.
    // Doing so necessitates use of NON LP timers, and setting
    //  the prescaler to allow for proper timer range.

    // The AFE timer (Timer A) will need to time a range from about 800us-10ms.
    // The HART UART timer (Timer B) will need to time a range from about 1600us-19ms.
    // Due to having only 16 bits each for counting, we will target for the
    //  prescaled input clock to be ~1MHz which should provide adequate timer
    //  range for both timers. @1MHz input clock each tick is 1us giving a max
    //  timer value of 65.5ms.

    // First ensure specified timer is of correct type, that is NON LP timer.
    status = afe_timer_validate_id(tmr);
    if (status != E_NO_ERROR) {
        return status;
    }

    // Based on System Clock frequency choose divider to achieve input clock
    //  closest to 1MHz.
    // NOTE: Always using PCLK to simplify configurations.
    status = afe_timer_choose_prescaler();
    if (status != E_NO_ERROR) {
        return status;
    }

    // Setup AFE Timer driver configuration
    afe_tmr_cfg.pres = (mxc_tmr_pres_t)g_timer_prescaler;
    afe_tmr_cfg.mode = TMR_MODE_ONESHOT;
    afe_tmr_cfg.bitMode = AFE_SPI_TIMER;
    afe_tmr_cfg.clock = MXC_TMR_APB_CLK;
    afe_tmr_cfg.cmp_cnt = 0;
    afe_tmr_cfg.pol = 1;

    // Note: MXC_TMR_INIT does not properly handle split 16 bit modes
    //   Calling it any way here to handle baseline initialization
    status = MXC_TMR_Init(g_afe_timer_inst, &afe_tmr_cfg, FALSE);
    if (status != E_NO_ERROR) {
        return status;
    }

    // Actual 16bit configuration
    status = AFE_TMR_Config_16(g_afe_timer_inst, &afe_tmr_cfg);
    if (status != E_NO_ERROR) {
        return status;
    }

    // Setup HART Timer driver configuration
    hart_tmr_cfg.pres = (mxc_tmr_pres_t)g_timer_prescaler;
    hart_tmr_cfg.mode = TMR_MODE_ONESHOT;
    hart_tmr_cfg.bitMode = HART_TIMER;
    hart_tmr_cfg.clock = MXC_TMR_APB_CLK;
    hart_tmr_cfg.cmp_cnt = 0;
    hart_tmr_cfg.pol = 1;

    // Actual 16bit configuration
    status = AFE_TMR_Config_16(g_afe_timer_inst, &hart_tmr_cfg);
    if (status != E_NO_ERROR) {
        return status;
    }

    // Overwrite default Timer IRQ Vector with ours
    MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(g_afe_timer_inst)),
                       afe_timer_interrupt_handler);
    NVIC_EnableIRQ(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(g_afe_timer_inst)));

    return status;
}

static int afe_timer_calc_ticks(uint32_t microseconds, uint16_t *ticks)
{
    // Bounds check for MAX timeout and convert to actual ticks
    uint64_t timer_tick_period_ns = 0;
    uint64_t nanoseconds = microseconds * 1000;
    uint64_t ratioed_ticks = 0;

    timer_tick_period_ns = ((1 << g_timer_prescaler) * 1000) / (PeripheralClock / 1000000);

    ratioed_ticks = nanoseconds / timer_tick_period_ns;

    // Bounds check the results
    if (ratioed_ticks > UINT16_MAX) {
        // Timeout longer than supported with current prescaler
        *ticks = 0;
        return E_OVERFLOW;
    }

    if (ratioed_ticks == 0) {
        // Unexpectedly resulting timeout count is 0
        *ticks = 0;
        return E_UNKNOWN;
    }

    *ticks = ratioed_ticks;
    return E_NO_ERROR;
}

int afe_timer_delay_async(mxc_tmr_bit_mode_t timer_selection, uint32_t timeout_us,
                          afe_timeout_complete_t cb)
{
    uint16_t calculated_ticks = 0;
    int status = E_NO_ERROR;

    if ((timer_selection != AFE_SPI_TIMER) && (timer_selection != HART_TIMER)) {
        return E_BAD_PARAM;
    }

    if (timeout_us == 0) {
        return E_BAD_PARAM;
    }

    status = afe_timer_calc_ticks(timeout_us, &calculated_ticks);
    if (status != E_NO_ERROR) {
        return status;
    }

    // TMR Driver does not provide low level access for 16 bit mode
    AFE_TMR_Stop_16(g_afe_timer_inst, timer_selection);

    // Reset count every time, otherwise it will creep up and incur a false timeout
    AFE_TMR_SetCount_16(g_afe_timer_inst, timer_selection, 1);
    AFE_TMR_SetCompare_16(g_afe_timer_inst, timer_selection, calculated_ticks);
    AFE_TMR_ClearFlags_16(g_afe_timer_inst, timer_selection);

    // Set up interrupts, and completion irq globals
    if (timer_selection == AFE_SPI_TIMER) {
        afe_irq_callback = cb;
        g_afe_spi_timer_complete = 0;
    } else {
        hart_irq_callback = cb;
        g_afe_hart_timer_complete = 0;
    }

    AFE_TMR_EnableInt_16(g_afe_timer_inst, timer_selection);
    AFE_TMR_Start_16(g_afe_timer_inst, timer_selection);

    return E_SUCCESS;
}

int afe_timer_delay_check(mxc_tmr_bit_mode_t timer_selection)
{
    if ((timer_selection != AFE_SPI_TIMER) && (timer_selection != HART_TIMER)) {
        return E_BAD_PARAM;
    }

    if (timer_selection == AFE_SPI_TIMER) {
        if (g_afe_spi_timer_complete) {
            return E_NO_ERROR;
        }

        return E_BUSY;
    } else {
        if (g_afe_hart_timer_complete) {
            return E_NO_ERROR;
        }

        return E_BUSY;
    }
}

int afe_timer_delay_abort(mxc_tmr_bit_mode_t timer_selection)
{
    if ((timer_selection != AFE_SPI_TIMER) && (timer_selection != HART_TIMER)) {
        return E_BAD_PARAM;
    }

    AFE_TMR_Stop_16(g_afe_timer_inst, timer_selection);

    if (timer_selection == AFE_SPI_TIMER) {
        if (afe_irq_callback) {
            afe_irq_callback(E_ABORT);
            afe_irq_callback = NULL;
        }
    } else {
        if (hart_irq_callback) {
            hart_irq_callback(E_ABORT);
            hart_irq_callback = NULL;
        }
    }

    return E_SUCCESS;
}

/******************************************************************************
 *
 * Copyright (C) 2026 Analog Devices, Inc.
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
 * @file    main.c
 * @brief   Timer example
 * @details PWM Timers - Outputs a PWM signal (2Hz) on LED0 - ERTCO Clock and LED1 - IPO_APB Clock
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "gpio.h"
#include "board.h"
#include "tmr.h"
#include "led.h"
#include "pb.h"
#include "lp.h"
#include "lpgcr_regs.h"
#include "gcr_regs.h"
#include "pwrseq_regs.h"

#include "mxc_delay.h"

/***** Definitions *****/

// Parameters for PWM output
#define TOGGLE_FREQ 4 // (Hz)

/***** Globals *****/

/***** Functions *****/
void ERTCO_CLK_TMR4_LED0_Handler(void)
{
    // Clear interrupt
    MXC_TMR_ClearFlags(MXC_TMR4);
    MXC_GPIO_OutToggle(led_pin[0].port, led_pin[0].mask);
}

void Setup_ERTCO_CLK_TMR4_LED0(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(MXC_TMR4, MXC_TMR_ERTCO_CLK, 128, TOGGLE_FREQ);

    MXC_TMR_Shutdown(MXC_TMR4);

    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.clock = MXC_TMR_ERTCO_CLK;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;

    MXC_TMR_Init(MXC_TMR4, &tmr, false);
    MXC_TMR_EnableInt(MXC_TMR4);
}

void IPO_APB_CLK_TMR3_LED1_Handler(void)
{
    // Clear interrupt
    MXC_TMR_ClearFlags(MXC_TMR3);
    MXC_GPIO_OutToggle(led_pin[1].port, led_pin[1].mask);
}

void Setup_IPO_APB_CLK_TMR3_LED1(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(MXC_TMR3, MXC_TMR_APB_CLK, 128, TOGGLE_FREQ);

    MXC_TMR_Shutdown(MXC_TMR3);

    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.clock = MXC_TMR_APB_CLK;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;

    MXC_TMR_Init(MXC_TMR3, &tmr, false);
    MXC_TMR_EnableInt(MXC_TMR3);
}

// *****************************************************************************
int main(void)
{
    // Comment out this function call to disable the calibration feature
    MXC_SYS_ClockCalibrate(MXC_SYS_CLOCK_IPO);

    MXC_NVIC_SetVector(TMR4_IRQn, ERTCO_CLK_TMR4_LED0_Handler);
    NVIC_EnableIRQ(TMR4_IRQn);
    Setup_ERTCO_CLK_TMR4_LED0();

    MXC_NVIC_SetVector(TMR3_IRQn, IPO_APB_CLK_TMR3_LED1_Handler);
    NVIC_EnableIRQ(TMR3_IRQn);
    Setup_IPO_APB_CLK_TMR3_LED1();

    MXC_TMR_Start(MXC_TMR4);
    MXC_TMR_Start(MXC_TMR3);

    while (1) {}

    return 0;
}

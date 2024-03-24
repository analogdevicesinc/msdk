/**
 * @file    tmr.h
 * @brief   Timer (TMR) function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_TMR_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_TMR_H_

/* **** Includes **** */
#include <stdint.h>
#include <stdbool.h>
#include "mxc_device.h"
#include "tmr_regs.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "mcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup tmr Timer (TMR)
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief Timer prescaler values
 */
typedef enum {
    MXC_TMR_PRES_1 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_1, ///< Divide input clock by 1
    MXC_TMR_PRES_2 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_2, ///< Divide input clock by 2
    MXC_TMR_PRES_4 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_4, ///< Divide input clock by 4
    MXC_TMR_PRES_8 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_8, ///< Divide input clock by 8
    MXC_TMR_PRES_16 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_16, ///< Divide input clock by 16
    MXC_TMR_PRES_32 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_32, ///< Divide input clock by 32
    MXC_TMR_PRES_64 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_64, ///< Divide input clock by 64
    MXC_TMR_PRES_128 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_128, ///< Divide input clock by 128
    MXC_TMR_PRES_256 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_256, ///< Divide input clock by 256
    MXC_TMR_PRES_512 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_512, ///< Divide input clock by 512
    MXC_TMR_PRES_1024 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_1024, ///< Divide input clock by 1024
    MXC_TMR_PRES_2048 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_2048, ///< Divide input clock by 2048
    MXC_TMR_PRES_4096 = MXC_S_TMR_CTRL0_CLKDIV_A_DIV_BY_4096, ///< Divide input clock by 4096

    // Legacy names
    TMR_PRES_1 = MXC_TMR_PRES_1,
    TMR_PRES_2 = MXC_TMR_PRES_2,
    TMR_PRES_4 = MXC_TMR_PRES_4,
    TMR_PRES_8 = MXC_TMR_PRES_8,
    TMR_PRES_16 = MXC_TMR_PRES_16,
    TMR_PRES_32 = MXC_TMR_PRES_32,
    TMR_PRES_64 = MXC_TMR_PRES_64,
    TMR_PRES_128 = MXC_TMR_PRES_128,
    TMR_PRES_256 = MXC_TMR_PRES_256,
    TMR_PRES_512 = MXC_TMR_PRES_512,
    TMR_PRES_1024 = MXC_TMR_PRES_1024,
    TMR_PRES_2048 = MXC_TMR_PRES_2048,
    TMR_PRES_4096 = MXC_TMR_PRES_4096
} mxc_tmr_pres_t;

/**
 * @brief Timer modes
 */
typedef enum {
    MXC_TMR_MODE_ONESHOT = MXC_V_TMR_CTRL0_MODE_A_ONE_SHOT, ///< Timer Mode ONESHOT
    MXC_TMR_MODE_CONTINUOUS = MXC_V_TMR_CTRL0_MODE_A_CONTINUOUS, ///< Timer Mode CONTINUOUS
    MXC_TMR_MODE_COUNTER = MXC_V_TMR_CTRL0_MODE_A_COUNTER, ///< Timer Mode COUNTER
    MXC_TMR_MODE_PWM = MXC_V_TMR_CTRL0_MODE_A_PWM, ///< Timer Mode PWM
    MXC_TMR_MODE_CAPTURE = MXC_V_TMR_CTRL0_MODE_A_CAPTURE, ///< Timer Mode CAPTURE
    MXC_TMR_MODE_COMPARE = MXC_V_TMR_CTRL0_MODE_A_COMPARE, ///< Timer Mode COMPARE
    MXC_TMR_MODE_GATED = MXC_V_TMR_CTRL0_MODE_A_GATED, ///< Timer Mode GATED
    MXC_TMR_MODE_CAPTURE_COMPARE = MXC_V_TMR_CTRL0_MODE_A_CAPCOMP, ///< Timer Mode CAPTURECOMPARE
    MXC_TMR_MODE_DUAL_EDGE = MXC_V_TMR_CTRL0_MODE_A_DUAL_EDGE, ///< Timer Mode DUALEDGE

    // Legacy names
    TMR_MODE_ONESHOT = MXC_TMR_MODE_ONESHOT,
    TMR_MODE_CONTINUOUS = MXC_TMR_MODE_CONTINUOUS,
    TMR_MODE_COUNTER = MXC_TMR_MODE_COUNTER,
    TMR_MODE_PWM = MXC_TMR_MODE_PWM,
    TMR_MODE_CAPTURE = MXC_TMR_MODE_CAPTURE,
    TMR_MODE_COMPARE = MXC_TMR_MODE_COMPARE,
    TMR_MODE_GATED = MXC_TMR_MODE_GATED,
    TMR_MODE_CAPTURE_COMPARE = MXC_TMR_MODE_CAPTURE_COMPARE,
    TMR_MODE_DUAL_EDGE = MXC_TMR_MODE_DUAL_EDGE
} mxc_tmr_mode_t;

/**
 * @brief Timer bit mode 
 * 
 */
typedef enum {
    MXC_TMR_BIT_MODE_32, ///< Timer Mode 32 bit
    MXC_TMR_BIT_MODE_16A, ///< Timer Mode Lower 16 bit
    MXC_TMR_BIT_MODE_16B, ///< Timer Mode Upper 16 bit

    // Legacy names
    TMR_BIT_MODE_32 = MXC_TMR_BIT_MODE_32,
    TMR_BIT_MODE_16A = MXC_TMR_BIT_MODE_16A,
    TMR_BIT_MODE_16B = MXC_TMR_BIT_MODE_16B,
} mxc_tmr_bit_mode_t;

/**
 * @brief Timer units of time enumeration
 */
typedef enum {
    MXC_TMR_UNIT_NANOSEC, ///< Nanosecond Unit Indicator
    MXC_TMR_UNIT_MICROSEC, ///< Microsecond Unit Indicator
    MXC_TMR_UNIT_MILLISEC, ///< Millisecond Unit Indicator
    MXC_TMR_UNIT_SEC, ///< Second Unit Indicator

    // Legacy names
    TMR_UNIT_NANOSEC = MXC_TMR_UNIT_NANOSEC,
    TMR_UNIT_MICROSEC = MXC_TMR_UNIT_MICROSEC,
    TMR_UNIT_MILLISEC = MXC_TMR_UNIT_MILLISEC,
    TMR_UNIT_SEC = MXC_TMR_UNIT_SEC,
} mxc_tmr_unit_t;

/**
 * @brief       Clock settings 
 * @note        8M and 32M clocks can be used for Timers 0,1,2 and 3
 *              32K and 80K clocks can only be used for Timers 4 and 5
 */
typedef enum {
    MXC_TMR_APB_CLK = 0, ///< PCLK CLock
    MXC_TMR_EXT_CLK = 1, ///< External Clock
    MXC_TMR_IBRO_CLK = 2, ///< 7.3728MHz Clock
    MXC_TMR_ERFO_CLK = 3, ///< 32MHz Clock
    MXC_TMR_INRO_CLK = 4, ///< 80Khz Clock

    // Legacy names
    MXC_TMR_8M_CLK = MXC_TMR_IBRO_CLK, ///< 8MHz Clock
    MXC_TMR_32M_CLK = MXC_TMR_ERFO_CLK, ///< 32MHz Clock
    MXC_TMR_32K_CLK, ///< 32KHz Clock (NOT SUPPORTED as TMR clock source)
    MXC_TMR_80K_CLK = MXC_TMR_INRO_CLK, ///< 80KHz Clock
} mxc_tmr_clock_t;

/**
 * @brief Timer Configuration
 */
typedef struct {
    mxc_tmr_pres_t pres; ///< Desired timer prescaler
    mxc_tmr_mode_t mode; ///< Desired timer mode
    mxc_tmr_bit_mode_t bitMode; ///< Desired timer bits
    mxc_tmr_clock_t clock; ///< Desired clock source
    uint32_t cmp_cnt; ///< Compare register value in timer ticks
    unsigned pol; ///< Polarity (0 or 1)
} mxc_tmr_cfg_t;

/* **** Definitions **** */
typedef void (*mxc_tmr_complete_t)(int error);

/* **** Function Prototypes **** */

/**
 * @brief   Initialize timer module clock.
 * @note    On default this function enables TMR peripheral clock and related GPIOs.
 *          if you wish to manage clock and gpio related things in upper level instead of here.
 *          Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file. 
 *          By this flag this function will remove clock and gpio related codes from file.
 * 
 * @param   tmr        Pointer to timer module to initialize.
 * @param   cfg        System configuration object
 * @param   init_pins  True will initialize pins corresponding to the TMR and False will not if pins are pinned out otherwise it will not
 *                     be used, has no effect incase of MSDK_NO_GPIO_CLK_INIT has been defined. 
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TMR_Init(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg, bool init_pins);

/**
 * @brief   Shutdown timer module clock.
 * @param   tmr  Pointer to timer module to initialize.
 */
void MXC_TMR_Shutdown(mxc_tmr_regs_t *tmr);

/**
 * @brief   Start the timer counting.
 * @param   tmr  Pointer to timer module to initialize.
 */
void MXC_TMR_Start(mxc_tmr_regs_t *tmr);

/**
 * @brief   Stop the timer.
 * @param   tmr  Pointer to timer module to initialize.
 */
void MXC_TMR_Stop(mxc_tmr_regs_t *tmr);

/**
 * @brief   Set the value of the first transition in PWM mode
 * @param   tmr     Pointer to timer module to initialize.
 * @param   pwm     New pwm count.
 * @note    Will block until safe to change the period count.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TMR_SetPWM(mxc_tmr_regs_t *tmr, uint32_t pwm);

/**
 * @brief   Get the timer compare count.
 * @param   tmr     Pointer to timer module to initialize.
 * @return  Returns the current compare count.
 */
uint32_t MXC_TMR_GetCompare(mxc_tmr_regs_t *tmr);

/**
 * @brief   Get the timer capture count.
 * @param   tmr     Pointer to timer module to initialize.
 * @return  Returns the most recent capture count.
 */
uint32_t MXC_TMR_GetCapture(mxc_tmr_regs_t *tmr);

/**
 * @brief   Get the timer count.
 * @param   tmr     Pointer to timer module to initialize.
 * @return  Returns the current count.
 */
uint32_t MXC_TMR_GetCount(mxc_tmr_regs_t *tmr);

/**
 * @brief   Calculate count for required frequency.
 * @param   tmr         Timer
 * @param   clock       Clock source.
 * @param   prescalar   prescalar
 * @param   frequency   required frequency.
 * @return  Returns the period count.
 */
uint32_t MXC_TMR_GetPeriod(mxc_tmr_regs_t *tmr, mxc_tmr_clock_t clock, uint32_t prescalar,
                           uint32_t frequency);

/**
 * @brief   Clear the timer interrupt.
 * @param   tmr     Pointer to timer module to initialize.
 */
void MXC_TMR_ClearFlags(mxc_tmr_regs_t *tmr);

/**
 * @brief   Get the timer interrupt status.
 * @param   tmr     Pointer to timer module to initialize.
 * @return  Returns the interrupt status. 1 if interrupt has occured.
 */
uint32_t MXC_TMR_GetFlags(mxc_tmr_regs_t *tmr);

/**
 * @brief   enable interupt
 *
 * @param   tmr   Pointer to timer module to initialize.
 */
void MXC_TMR_EnableInt(mxc_tmr_regs_t *tmr);

/**
 * @brief   disable interupt
 *
 * @param   tmr   Pointer to timer module to initialize.
 */
void MXC_TMR_DisableInt(mxc_tmr_regs_t *tmr);

/**
 * @brief   Enable wakeup from sleep
 * 
 * @param   tmr   Pointer to timer module to initialize.
 * @param   cfg   System configuration object  
 */
void MXC_TMR_EnableWakeup(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg);

/**
 * @brief   Disable wakeup from sleep
 * 
 * @param   tmr   Pointer to timer module to initialize.
 * @param   cfg   System configuration object
 */
void MXC_TMR_DisableWakeup(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg);

/**
 * @brief   Set the timer compare count.
 * @param   tmr     Pointer to timer module to initialize.
 * @param   cmp_cnt New compare count.
 * @note    In PWM Mode use this to set the value of the second transition.
 */
void MXC_TMR_SetCompare(mxc_tmr_regs_t *tmr, uint32_t cmp_cnt);

/**
 * @brief   Set the timer count.
 * @param   tmr     Pointer to timer module to initialize.
 * @param   cnt     New count.
 */
void MXC_TMR_SetCount(mxc_tmr_regs_t *tmr, uint32_t cnt);

/**
 * @brief   Dealay for a set periord of time measured in microseconds
 *
 * @param   tmr   The timer
 * @param   us    microseconds to delay for
 */
void MXC_TMR_Delay(mxc_tmr_regs_t *tmr, uint32_t us);

/**
 * @brief   Start a timer that will time out after a certain number of microseconds
 * @note    This uses the 32-it Timer
 * 
 * @param   tmr   The timer
 * @param   us    microseconds to time out after
 */
void MXC_TMR_TO_Start(mxc_tmr_regs_t *tmr, uint32_t us);

/**
 * @brief   Check on time out timer
 *
 * @param   tmr   The timer
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TMR_TO_Check(mxc_tmr_regs_t *tmr);

/**
 * @brief   Stop the Timeout timer
 *
 * @param   tmr   The timer
 */
void MXC_TMR_TO_Stop(mxc_tmr_regs_t *tmr);

/**
 * @brief   Clear timeout timer back to zero
 *
 * @param   tmr   The timer
 */
void MXC_TMR_TO_Clear(mxc_tmr_regs_t *tmr);

/**
 * @brief   Get elapsed time of timeout timer
 *
 * @param   tmr   The timer
 *
 * @return  Time that has elapsed in timeout timer
 */
unsigned int MXC_TMR_TO_Elapsed(mxc_tmr_regs_t *tmr);

/**
 * @brief   Amount of time remaining until timeour
 *
 * @param   tmr   The timer
 *
 * @return  Time that is left until timeout
 */
unsigned int MXC_TMR_TO_Remaining(mxc_tmr_regs_t *tmr);

/**
 * @brief   Start stopwatch
 *
 * @param   tmr      The timer
 */
void MXC_TMR_SW_Start(mxc_tmr_regs_t *tmr);

/**
 * @brief   Stopwatch stop
 *
 * @param   tmr   The timer
 *
 * @return  the time when the stopwatch is stopped.
 */
unsigned int MXC_TMR_SW_Stop(mxc_tmr_regs_t *tmr);

/**
 * @brief   Get time from timer
 *
 * @param   tmr    The timer
 * @param   ticks  The ticks
 * @param   time   The time
 * @param   units  The units
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TMR_GetTime(mxc_tmr_regs_t *tmr, uint32_t ticks, uint32_t *time, mxc_tmr_unit_t *units);

/**@} end of group tmr */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_TMR_H_

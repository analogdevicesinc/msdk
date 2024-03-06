/**
 * @file    wdt.h
 * @brief   Watchdog timer (WDT) function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_WDT_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_WDT_H_

/* **** Includes **** */
#include <stdint.h>
#include "mxc_device.h"
#include "wdt_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup wdt WDT
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

/** @brief Watchdog upper limit period enumeration.
    Used to configure the period of the watchdog interrupt */
typedef enum {
    MXC_WDT_PERIOD_2_31 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW31, ///< Period 2^31
    MXC_WDT_PERIOD_2_30 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW30, ///< Period 2^30
    MXC_WDT_PERIOD_2_29 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW29, ///< Period 2^29
    MXC_WDT_PERIOD_2_28 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW28, ///< Period 2^28
    MXC_WDT_PERIOD_2_27 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW27, ///< Period 2^27
    MXC_WDT_PERIOD_2_26 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW26, ///< Period 2^26
    MXC_WDT_PERIOD_2_25 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW25, ///< Period 2^25
    MXC_WDT_PERIOD_2_24 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW24, ///< Period 2^24
    MXC_WDT_PERIOD_2_23 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW23, ///< Period 2^23
    MXC_WDT_PERIOD_2_22 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW22, ///< Period 2^22
    MXC_WDT_PERIOD_2_21 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW21, ///< Period 2^21
    MXC_WDT_PERIOD_2_20 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW20, ///< Period 2^20
    MXC_WDT_PERIOD_2_19 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW19, ///< Period 2^19
    MXC_WDT_PERIOD_2_18 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW18, ///< Period 2^18
    MXC_WDT_PERIOD_2_17 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW17, ///< Period 2^17
    MXC_WDT_PERIOD_2_16 = MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW16, ///< Period 2^16
} mxc_wdt_period_t;

/**
 * @brief Watchdog interrupt flag enumeration
 */
typedef enum {
    MXC_WDT_INT_TOO_LATE = MXC_F_WDT_CTRL_INT_LATE,
    MXC_WDT_INT_TOO_SOON = MXC_F_WDT_CTRL_INT_EARLY,
} mxc_wdt_int_t;

/**
 * @brief Watchdog reset flag enumeration
 */
typedef enum {
    MXC_WDT_RST_TOO_LATE = MXC_F_WDT_CTRL_RST_LATE,
    MXC_WDT_RST_TOO_SOON = MXC_F_WDT_CTRL_RST_EARLY,
} mxc_wdt_rst_t;

/**
 * @brief Watchdog mode enumeration
 */
typedef enum {
    MXC_WDT_COMPATIBILITY = 0,
    MXC_WDT_WINDOWED = 1,
} mxc_wdt_mode_t;

/**
 * @brief Peripheral Clock settings 
 */
typedef enum {
    MXC_WDT_PCLK = 0,
    MXC_WDT_IPO_CLK,
    MXC_WDT_IBRO_CLK,
    MXC_WDT_INRO_CLK,
    MXC_WDT_ERTCO_CLK,
    MXC_WDT_EXT_CLK,
    MXC_WDT_ERFO_CLK
} mxc_wdt_clock_t;

/**
 * @brief Timer Configuration
 */
typedef struct {
    mxc_wdt_mode_t mode; ///< WDT mode
    mxc_wdt_period_t upperResetPeriod; ///< Reset upper limit
    mxc_wdt_period_t lowerResetPeriod; ///< Reset lower limit
    mxc_wdt_period_t upperIntPeriod; ///< Interrupt upper limit
    mxc_wdt_period_t lowerIntPeriod; ///< Interrupt lower limit
} mxc_wdt_cfg_t;
/* **** Function Prototypes **** */

/**
 * @brief Initialize the Watchdog Timer
 * @note  On default this function enables WDT peripheral clock.
 *        if you wish to manage clock and gpio related things in upper level instead of here.
 *        Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file. 
 *        By this flag this function will remove clock and gpio related codes from file.
 * 
 * @param       wdt     Pointer to the watchdog registers
 * @param       cfg     watchdog configuration
 * @return      See \ref MXC_Error_Codes for the list of error codes.
 */
int MXC_WDT_Init(mxc_wdt_regs_t *wdt, mxc_wdt_cfg_t *cfg);

/**
 * @brief Shutdown the Watchdog Timer
 * @param       wdt     Pointer to the watchdog registers
 * @return      See \ref MXC_Error_Codes for the list of error codes.
 */
int MXC_WDT_Shutdown(mxc_wdt_regs_t *wdt);

/**
 * @brief       Set the period of the watchdog interrupt.
 * @param       wdt     Pointer to watchdog registers.
 * @param       cfg     watchdog configuration.
 */
void MXC_WDT_SetIntPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_cfg_t *cfg);

/**
 * @brief       Set the period of the watchdog reset.
 * @param       wdt     Pointer to watchdog registers.
 * @param       cfg     watchdog configuration.
 */
void MXC_WDT_SetResetPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_cfg_t *cfg);

/**
 * @brief       Enable the watchdog timer.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_Enable(mxc_wdt_regs_t *wdt);

/**
 * @brief       Disable the watchdog timer.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_Disable(mxc_wdt_regs_t *wdt);

/**
 * @brief       Enable the watchdog interrupt.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_EnableInt(mxc_wdt_regs_t *wdt);

/**
 * @brief       Disable the watchdog interrupt.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_DisableInt(mxc_wdt_regs_t *wdt);

/**
 * @brief       Enable the watchdog reset.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_EnableReset(mxc_wdt_regs_t *wdt);

/**
 * @brief       Disable the watchdog reset.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_DisableReset(mxc_wdt_regs_t *wdt);

/**
 * @brief       Reset the watchdog timer.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_ResetTimer(mxc_wdt_regs_t *wdt);

/**
 * @brief       Get the status of the reset flag.
 * @param       wdt     Pointer to watchdog registers.
 * @returns     1 if the previous reset was caused by the watchdog, 0 otherwise.
 */
int MXC_WDT_GetResetFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Clears the reset flag.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_ClearResetFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Get the status of the interrupt flag.
 * @param       wdt     Pointer to watchdog registers.
 * @returns     1 if the interrupt is pending, 0 otherwise.
 */
int MXC_WDT_GetIntFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Clears the interrupt flag.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_ClearIntFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Sets clock source.
 * @param       wdt             Pointer to watchdog registers.
 * @param       clock_source    Clock source.
 */
int MXC_WDT_SetClockSource(mxc_wdt_regs_t *wdt, mxc_wdt_clock_t clock_source);

/**@} end of group wdt */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_WDT_H_

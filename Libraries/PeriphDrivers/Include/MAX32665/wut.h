/**
 * @file    wut.h
 * @brief   Wakeup Timer (WUT) function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_WUT_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_WUT_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "wut_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup wut Wakeup Timer (WUT)
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief Wakeup Timer prescaler values
 */
typedef enum {
    MXC_WUT_PRES_1 = MXC_S_WUT_CTRL_PRES_DIV1, /// Divide input clock by 1
    MXC_WUT_PRES_2 = MXC_S_WUT_CTRL_PRES_DIV2, /// Divide input clock by 2
    MXC_WUT_PRES_4 = MXC_S_WUT_CTRL_PRES_DIV4, /// Divide input clock by 4
    MXC_WUT_PRES_8 = MXC_S_WUT_CTRL_PRES_DIV8, /// Divide input clock by 8
    MXC_WUT_PRES_16 = MXC_S_WUT_CTRL_PRES_DIV16, /// Divide input clock by 16
    MXC_WUT_PRES_32 = MXC_S_WUT_CTRL_PRES_DIV32, /// Divide input clock by 32
    MXC_WUT_PRES_64 = MXC_S_WUT_CTRL_PRES_DIV64, /// Divide input clock by 64
    MXC_WUT_PRES_128 = MXC_S_WUT_CTRL_PRES_DIV128, /// Divide input clock by 128
    MXC_WUT_PRES_256 = MXC_F_WUT_CTRL_PRES3 |
                       MXC_S_WUT_CTRL_PRES_DIV1, /// Divide input clock by 256
    MXC_WUT_PRES_512 = MXC_F_WUT_CTRL_PRES3 |
                       MXC_S_WUT_CTRL_PRES_DIV4, /// Divide input clock by 512
    MXC_WUT_PRES_1024 = MXC_F_WUT_CTRL_PRES3 |
                        MXC_S_WUT_CTRL_PRES_DIV8, /// Divide input clock by 1024
    MXC_WUT_PRES_2048 = MXC_F_WUT_CTRL_PRES3 |
                        MXC_S_WUT_CTRL_PRES_DIV16, /// Divide input clock by 2048
    MXC_WUT_PRES_4096 = MXC_F_WUT_CTRL_PRES3 |
                        MXC_S_WUT_CTRL_PRES_DIV32 /// Divide input clock by 4096
} mxc_wut_pres_t;

/**
 * @brief Wakeup Timer modes
 */
typedef enum {
    MXC_WUT_MODE_ONESHOT = MXC_V_WUT_CTRL_TMODE_ONESHOT, /// Wakeup Timer Mode ONESHOT
    MXC_WUT_MODE_CONTINUOUS = MXC_V_WUT_CTRL_TMODE_CONTINUOUS, /// Wakeup Timer Mode CONTINUOUS
    MXC_WUT_MODE_COUNTER = MXC_V_WUT_CTRL_TMODE_COUNTER, /// Wakeup Timer Mode COUNTER
    MXC_WUT_MODE_PWM = MXC_V_WUT_CTRL_TMODE_PWM, /// Wakeup Timer Mode PWM
    MXC_WUT_MODE_CAPTURE = MXC_V_WUT_CTRL_TMODE_CAPTURE, /// Wakeup Timer Mode CAPTURE
    MXC_WUT_MODE_COMPARE = MXC_V_WUT_CTRL_TMODE_COMPARE, /// Wakeup Timer Mode COMPARE
    MXC_WUT_MODE_GATED = MXC_V_WUT_CTRL_TMODE_GATED, /// Wakeup Timer Mode GATED
    MXC_WUT_MODE_CAPTURE_COMPARE =
        MXC_V_WUT_CTRL_TMODE_CAPTURECOMPARE /// Wakeup Timer Mode CAPTURECOMPARE
} mxc_wut_mode_t;

/**
 * @brief Wakeup Timer units of time enumeration
 */
typedef enum {
    MXC_WUT_UNIT_NANOSEC = 0, /**< Nanosecond Unit Indicator. */
    MXC_WUT_UNIT_MICROSEC, /**< Microsecond Unit Indicator. */
    MXC_WUT_UNIT_MILLISEC, /**< Millisecond Unit Indicator. */
    MXC_WUT_UNIT_SEC /**< Second Unit Indicator. */
} mxc_wut_unit_t;

/**
 * @brief Wakeup Timer Configuration
 */
typedef struct {
    mxc_wut_mode_t mode; /// Desired timer mode
    uint32_t cmp_cnt; /// Compare register value in timer ticks
} mxc_wut_cfg_t;

/**
 * @brief   The callback routine used by the MXC_WUT_TrimCrystalAsync()
 *          function to indicate the transaction has completed.
 *
 * @param   result      Error code.
 */
typedef void (*mxc_wut_complete_cb_t)(int result);

/* **** Definitions **** */

/* **** Function Prototypes **** */

/**
 * @brief      Initialize timer module clock.
 * @param      pres       Prescaler value. 
 */
void MXC_WUT_Init(mxc_wut_pres_t pres);

/**
 * @brief      Shutdown timer module clock.
 */
void MXC_WUT_Shutdown(void);

/**
 * @brief      Enable the timer. 
 */
void MXC_WUT_Enable(void);

/**
 * @brief      Disable the timer.
 */
void MXC_WUT_Disable(void);

/**
 * @brief      Configure the timer.
 * @param      cfg  Pointer to timer configuration struct.
 */
void MXC_WUT_Config(const mxc_wut_cfg_t *cfg);

/**
 * @brief   Get the timer compare count.
 * @return  Returns the current compare count.
 */
uint32_t MXC_WUT_GetCompare(void);

/**
 * @brief   Get the timer capture count.
 * @return  Returns the most recent capture count.
 */
uint32_t MXC_WUT_GetCapture(void);

/**
 * @brief   Get the timer count.
 * @return  Returns the current count.
 */
uint32_t MXC_WUT_GetCount(void);

/**
 * @brief   Clear the timer interrupt.
 */
__attribute__((deprecated("Use MXC_WUT_ClearFlags instead.  See wut.h for more details."))) void
MXC_WUT_IntClear(void);

/**
 * @brief   Clear the timer interrupt.
 */
void MXC_WUT_ClearFlags(void);

/**
 * @brief   Get the timer interrupt status.
 * @return  Returns the interrupt status. 1 if interrupt has occurred.
 */
__attribute__((deprecated("Use MXC_WUT_GetFlags instead.  See wut.h for more details."))) uint32_t
MXC_WUT_IntStatus(void);

/**
 * @brief   Get the timer interrupt status.
 * @return  Returns the interrupt status. 1 if interrupt has occurred.
 */
uint32_t MXC_WUT_GetFlags(void);

/**
 * @brief   Set the timer compare count.
 * @param   cmp_cnt New compare count.
 * @note    This function does not protect against output glitches in PWM mode.
 *          Use MXC_WUT_PWMSetPeriod when in PWM mode.
 */
void MXC_WUT_SetCompare(uint32_t cmp_cnt);

/**
 * @brief   Set the timer count.
 * @param   cnt     New count.
 */
void MXC_WUT_SetCount(uint32_t cnt);

/**
 * @brief   Convert real time to timer ticks.
 * @param   time    Number of units of time.
 * @param   units   Which units of time you want to convert.
 * @param   ticks   Pointer to store the number of ticks calculated.
 * @return     #E_NO_ERROR If everything is successful. 
 * @return     @ref MXC_Error_Codes If function is unsuccessful.
 */
int MXC_WUT_GetTicks(uint32_t time, mxc_wut_unit_t units, uint32_t *ticks);

/**
 * @brief   Convert timer ticks to real time.
 * @param   ticks   Number of ticks.
 * @param   time    Pointer to store number of units of time.
 * @param   units   Pointer to store the units that time represents.
 * @return     #E_NO_ERROR If everything is successful. 
 * @return     @ref MXC_Error_Codes If function is unsuccessful.
 */
int MXC_WUT_GetTime(uint32_t ticks, uint32_t *time, mxc_wut_unit_t *units);

/**
 * @brief   Wait for an edge of the WUT count register.
 */
__attribute__((deprecated("Use MXC_WUT_WaitForEdge instead.  See wut.h for more details."))) void
MXC_WUT_Edge(void);

/**
 * @brief   Wait for an edge of the WUT count register.
 */
void MXC_WUT_WaitForEdge(void);

/**
 * @brief   Store the count and snapshot values.
 */
__attribute__((deprecated("Use MXC_WUT_StoreCount instead.  See wut.h for more details."))) void
MXC_WUT_Store(void);

/**
 * @brief   Store the count and snapshot values.
 */
void MXC_WUT_StoreCount(void);

/**
 * @brief   Restore the DBB clock with the stored count and snapshot values.
 * @param   dbbFreq  Frequency of DBB clock.
 */
void MXC_WUT_RestoreBBClock(uint32_t dbbFreq);

/**
 * @brief   Get the difference between the stored counter value 
 *          and the current counter value.
 * @return  Returns the current counter value - stored counter value.
 */
uint32_t MXC_WUT_GetSleepTicks(void);

/**
 * @brief   Delays for the given number of milliseconds.
 * @param   waitMs  Number of milliseconds to wait.
 */
void MXC_WUT_Delay_MS(uint32_t waitMs);

/**
 * @brief   Trim the 32 kHz crystal load settings, blocks until complete.
 * @details This procedure uses the WUT and the BLE DBB, driven by the 32 MHz crystal,
 *          to trim the load settings of the 32 kHz crystal. This procedure will only
 *          work if the BLE DBB is initialized and running.
 *
 * @return  #E_NO_ERROR If everything is successful.
 */
int MXC_WUT_TrimCrystal(void);

/**
 * @brief   Trim the 32 kHz crystal load settings, non-blocking interrupt based.
 * @details This procedure uses the WUT and the BLE DBB, driven by the 32 MHz crystal,
 *          to trim the load settings of the 32 kHz crystal. This procedure will only
 *          work if the BLE DBB is initialized and running.
 *
 * @param   cb Callback for when the trim is complete.
 * @return  #E_NO_ERROR If everything is successful.
 */
int MXC_WUT_TrimCrystalAsync(mxc_wut_complete_cb_t cb);

/**
 * @brief   Check to see if the trim procedure is ongoing.
 * @details Must leave the 32 MHz clock and BLE DBB running while the trim procedure is pending.
 * @return  #E_NO_ERROR If trim is complete, E_BUSY if trim procedure is ongoing.
 */
int MXC_WUT_TrimPending(void);

/**
 * @brief   Interrupt handler for trim procedure.
 *
 * @return  #E_NO_ERROR If trim is complete, E_BUSY if trim procedure is ongoing.
 */
int MXC_WUT_Handler(void);

/**@} end of group wut */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_WUT_H_

/**
 * @file    afe_timer.h
 * @brief   Timer functions for AFE and HART UART interfacing.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_AFE_TIMER_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_AFE_TIMER_H_

/* **** Includes **** */
#include "tmr_regs.h"
#include "tmr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */
// AFE timer will be lower 16 bits
#define AFE_SPI_TIMER (TMR_BIT_MODE_16A)
#define HART_TIMER (TMR_BIT_MODE_16B)

/**
 * @brief   The callback routine used by afe_timer_delay_async() when the delay is complete
 *          or aborted early.
 *
 * @param   result      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*afe_timeout_complete_t)(int result);

/* **** Function Prototypes **** */
/**
 * @brief      Configure AFE Timer
 * @param      tmr   Pointer to selected timer registers
 * @return     #E_NO_ERROR if everything is successful. See \ref MXC_Error_Codes for the list of error codes.
 */
int afe_timer_config(mxc_tmr_regs_t *tmr);

/**
 * @brief      Starts a non-blocking delay for the specified number of
 *             microseconds.
 * @details    Uses the timer instance pass in /ref afe_timer_setup to time the
 *              requested delay. Supports two 16 bit timers in one, so must also
 *              specify which half to use.
 * @param      timer_selection    Which 16bit timer half to use. /ref AFE_SPI_TIMER, /ref HART_TIMER
 * @param      timeout_us    microseconds to delay
 * @param      cb  Function pointer to function called upon delay completion. /ref afe_timeout_complete_t
 * @return     #E_NO_ERROR if everything is successful. See \ref MXC_Error_Codes for the list of error codes.
 */
int afe_timer_delay_async(mxc_tmr_bit_mode_t timer_selection, uint32_t timeout_us,
                          afe_timeout_complete_t cb);

/**
 * @brief      Returns the status of a non-blocking delay request
 * @pre        Start the asynchronous delay by calling /ref afe_timer_delay_async.
 * @param      timer_selection    Which 16bit timer half to use. /ref AFE_SPI_TIMER, /ref HART_TIMER
 * @return     #E_BUSY until the requested delay time has expired.
 */
int afe_timer_delay_check(mxc_tmr_bit_mode_t timer_selection);

/**
 * @brief      Stops an asynchronous delay previously started. Calls CB if not NULL.
 * @pre        Start the asynchronous delay by calling /ref afe_timer_delay_async.
 * @param      timer_selection    Which 16bit timer half to use. /ref AFE_SPI_TIMER, /ref HART_TIMER
 */
int afe_timer_delay_abort(mxc_tmr_bit_mode_t timer_selection);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_AFE_TIMER_H_

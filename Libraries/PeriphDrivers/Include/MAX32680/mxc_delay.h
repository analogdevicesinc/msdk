/**
 * @file
 * @brief    Asynchronous delay routines based on the SysTick Timer.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_MXC_DELAY_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_MXC_DELAY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup devicelibs
 * @defgroup    MXC_delay Delay Utility Functions
 * @brief       Asynchronous delay routines based on the SysTick Timer
 * @{
 */

/***** Definitions *****/
/**
 * Macro used to specify a microsecond timing parameter in seconds.
 * \code
 * x = SEC(3) // 3 seconds -> x = 3,000,000
 * \endcode
 */
#define MXC_DELAY_SEC(s) (((uint32_t)s) * 1000000UL)
/**
 * Macro used to specify a microsecond timing parameter in milliseconds.
 * \code
 * x = MSEC(3) // 3ms -> x = 3,000
 * \endcode
 */
#define MXC_DELAY_MSEC(ms) (ms * 1000UL)
/**
 * Macro used to specify a microsecond timing parameter.
 * \code
 * x = USEC(3) // 3us -> x = 3
 * \endcode
 */
#define MXC_DELAY_USEC(us) (us)

/**
 * @brief   The callback routine used by MXC_DelayAsync() when the delay is complete
 *          or aborted early.
 *
 * @param   result      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_delay_complete_t)(int result);

/***** Function Prototypes *****/

/**
 * @brief      Blocks and delays for the specified number of microseconds.
 * @details    Uses the SysTick to create the requested delay. If the SysTick is
 *             running, the current settings will be used. If the SysTick is not
 *             running, it will be started.
 * @param      us    microseconds to delay
 * @return     #E_NO_ERROR if no errors, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_Delay(uint32_t us);

/**
 * @brief      Starts a non-blocking delay for the specified number of
 *             microseconds.
 * @details    Uses the SysTick to time the requested delay. If the SysTick is
 *             running, the current settings will be used. If the SysTick is not
 *             running, it will be started.
 * @note       MXC_Delay_handler() must be called from the SysTick interrupt service
 *             routine or at a rate greater than the SysTick overflow rate.
 * @param      us    microseconds to delay
 * @param      callback   Function pointer to the function to call after the delay has expired.
 * @return     #E_NO_ERROR if no errors, #E_BUSY if currently servicing another
 *             delay request.
 */
int MXC_DelayAsync(uint32_t us, mxc_delay_complete_t callback);

/**
 * @brief      Returns the status of a non-blocking delay request
 * @pre        Start the asynchronous delay by calling MXC_Delay_start().
 * @return     #E_BUSY until the requested delay time has expired.
 */
int MXC_DelayCheck(void);

/**
 * @brief      Stops an asynchronous delay previously started.
 * @pre        Start the asynchronous delay by calling MXC_Delay_start().
 */
void MXC_DelayAbort(void);

/**
 * @brief      Processes the delay interrupt.
 * @details    This function must be called from the SysTick IRQ or polled at a
 *             rate greater than the SysTick overflow rate.
 */
void MXC_DelayHandler(void);

/**@} end of group MXC_delay */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_MXC_DELAY_H_

/**
 * @file    rtc.h
 * @brief   Real Time Clock (RTC) functions and prototypes.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_RTC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_RTC_H_

/* **** Includes **** */
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "rtc_regs.h"
#include "tmr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup rtc Real Time Clock (RTC)
 * @ingroup periphlibs
 * @{
 */

#define MXC_RTC_MAX_SSEC (MXC_F_RTC_SSEC_SSEC + 1)
#define MXC_RTC_TRIM_TMR_IRQ MXC_F_TMR_INTR_IRQ

/* **** Definitions **** */
/**
 * @brief   Bitmasks for each of the RTC's Frequency.
 */
typedef enum {
    MXC_RTC_F_1HZ = MXC_S_RTC_CTRL_FT_FREQ1HZ, ///< 1Hz (Compensated)
    MXC_RTC_F_512HZ = MXC_S_RTC_CTRL_FT_FREQ512HZ, ///< 512Hz (Compensated)
    MXC_RTC_F_4KHZ = MXC_S_RTC_CTRL_FT_FREQ4KHZ, ///< 4Khz
    MXC_RTC_F_32KHZ = 32, ///< 32Khz
} mxc_rtc_freq_sel_t;

/**
 * @brief   Bitmasks for each of the RTC's interrupt enables.
 */
typedef enum {
    MXC_RTC_INT_EN_LONG = MXC_F_RTC_CTRL_ADE, ///< Long-interval alarm interrupt enable
    MXC_RTC_INT_EN_SHORT = MXC_F_RTC_CTRL_ASE, ///< Short-interval alarm interrupt enable
    MXC_RTC_INT_EN_READY = MXC_F_RTC_CTRL_RDYE, ///< Timer ready interrupt enable
} mxc_rtc_int_en_t;

/**
 * @brief     Bitmasks for each of the RTC's interrupt flags.
 */
typedef enum {
    MXC_RTC_INT_FL_LONG = MXC_F_RTC_CTRL_ALDF, ///< Long-interval alarm interrupt flag
    MXC_RTC_INT_FL_SHORT = MXC_F_RTC_CTRL_ALSF, ///< Short-interval alarm interrupt flag
    MXC_RTC_INT_FL_READY = MXC_F_RTC_CTRL_RDY, ///< Timer ready interrupt flag
} mxc_rtc_int_fl_t;

/**
 * @brief     Set Time-of-Day alarm value and enable Interrupt
 * @param     ras    20-bit value 0-0xFFFFF
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes 
 */
int MXC_RTC_SetTimeofdayAlarm(uint32_t ras);

/**
 * @brief     Set Sub-Second alarm value and enable interrupt,
 * @brief     this is to be called after the init_rtc() function
 * @param     rssa   32-bit value 0-0xFFFFFFFF
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_SetSubsecondAlarm(uint32_t rssa);

/**
 * @brief     Start the Real Time Clock (Blocking function)
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_Start(void);
/**
 * @brief     Stop the Real Time Clock (Blocking function)
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_Stop(void);

/**
 * @brief     Initialize the sec and ssec registers and enable RTC (Blocking function)
 * @param     sec    set the RTC Sec counter (32-bit)
 * @param     ssec   set the RTC Sub-second counter (12-bit)
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_Init(uint32_t sec, uint16_t ssec);

/**
 * @brief     Allow generation of Square Wave on the SQW pin (Blocking function)
 * @param     fq     Frequency output selection
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_SquareWaveStart(mxc_rtc_freq_sel_t fq);

/**
 * @brief     Stop the generation of square wave (Blocking function)
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_SquareWaveStop(void);

/**
 * @brief     Set Trim register value (Blocking function)
 * @param     trm    set the RTC Trim (8-bit, +/- 127)
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_Trim(int8_t trm);

/**
 * @brief     Enable Interurpts (Blocking function)
 * @param     mask   The bitwise OR of interrupts to enable.
 *                   See #mxc_rtc_int_en_t for available choices.
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_EnableInt(uint32_t mask);

/**
 * @brief     Disable Interurpts (Blocking function)
 * @param     mask   The mask of interrupts to disable.
 *                   See #mxc_rtc_int_en_t for available choices.
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_DisableInt(uint32_t mask);

/**
 * @brief     Gets interrupt flags.
 * @retval    The bitwise OR of any interrupts flags that are
 *            currently set. See \ref mxc_rtc_int_fl_t for the list
 *            of possible flags.
 */
int MXC_RTC_GetFlags(void);

/**
 * @brief     Clear interrupt flags.
 * @param     flags The bitwise OR of the interrupts flags to cleear.
 *            See #mxc_rtc_int_fl_t for the list of possible flags.
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_ClearFlags(int flags);

/**
 * @brief     Get SubSecond or E_BUSY, see /ref MXC_ERROR_CODES
 * @retval    Returns subsecond value
 */
#ifdef __GNUC__
__attribute__((deprecated("Use MXC_RTC_GetSubSeconds() instead.")))
#endif
int MXC_RTC_GetSubSecond(void);

/**
 * @brief     This function stores the current value of the sub-seconds counter into a
 *            pointer if the RTC is not busy. If the RTC is busy, an error is returned.
 * @param     ssec   Pointer to the variable to store the current sub-seconds value.
 * @retval    E_NO_ERROR if successful, otherwise an error code (see /ref MXC_ERROR_CODES).
 */
int MXC_RTC_GetSubSeconds(uint32_t *ssec);

/**
 * @brief     Get Second or E_BUSY, see /ref MXC_ERROR_CODES
 * @retval    returns second value
 */
#ifdef __GNUC__
__attribute__((deprecated("Use MXC_RTC_GetSeconds() instead.")))
#endif
int MXC_RTC_GetSecond(void);

/**
 * @brief     This function stores the current value of the seconds counter into a
 *            pointer if the RTC is not busy. If the RTC is busy, an error is returned.
 * @param     sec   Pointer to the variable to store the current seconds value.
 * @retval    E_NO_ERROR if successful, otherwise an error code (see /ref MXC_ERROR_CODES).
 */
int MXC_RTC_GetSeconds(uint32_t *sec);

/**
 * @brief     Get the current second and sub-second counts.
 * @param     sec pointer to store seconds value
 * @param     subsec pointer to store subseconds value
 * @retval    returns Success or Fail, see \ref MXC_Error_Codes
 */
int MXC_RTC_GetTime(uint32_t *sec, uint32_t *subsec);

/**
 * @brief     Get RTC busy flag.
 * @retval    returns Success or E_BUSY, see /ref MXC_ERROR_CODES
 */
int MXC_RTC_GetBusyFlag(void);

/**
 * @brief   Calculate and set the appropriate RTC trim value based on an accurate reference clock
 * 
 * @param   tmr     Timer available to be used to measure known time periods over which the RTC ticks are counted 
 * 
 * @retval  returns Success or Fail, see \ref MXC_Error_Codes
 * 
 * @note    If RTC running before calling this function and interrupts enabled, accuracy of trimming could be affected
 * @note    External 32MHz must be installed and calibrated properly for this function to be successful
 */
int MXC_RTC_TrimCrystal(mxc_tmr_regs_t *tmr);

/**@} end of group rtc */
#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_RTC_H_

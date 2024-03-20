/**
 * @file    htmr.h
 * @brief   High speed timer (HTMR) functions and prototypes.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_HTMR_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_HTMR_H_

/* **** Includes **** */
#include <stdint.h>
#include "mxc_device.h"
#include "htmr_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup htmr HTMR
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
#define IBRO_FREQ HIRC8_FREQ

/**
 * @brief   Bitmasks for each of the HTimer's interrupt enables.
 */
typedef enum {
    MXC_HTMR_INT_EN_LONG = MXC_F_HTMR_CTRL_ADE, ///< Long-interval alarm interrupt enable
    MXC_HTMR_INT_EN_SHORT = MXC_F_HTMR_CTRL_ASE, ///< Short-interval alarm interrupt enable
    MXC_HTMR_INT_EN_READY = MXC_F_HTMR_CTRL_RDYE, ///< Timer ready interrupt enable
} mxc_htmr_int_en_t;

/**
 * @brief   Bitmasks for each of the HTimer's interrupt flags.
 */
typedef enum {
    MXC_HTMR_INT_FL_LONG = MXC_F_HTMR_CTRL_ALDF, ///< Long-interval alarm interrupt flag
    MXC_HTMR_INT_FL_SHORT = MXC_F_HTMR_CTRL_ALSF, ///< Short-interval alarm interrupt flag
    MXC_HTMR_INT_FL_READY = MXC_F_HTMR_CTRL_RDY, ///< Timer ready interrupt flag
} mxc_htmr_int_fl_t;

/**
 * @brief     Enable Interurpts
 * @param     htmr   pointer to the htmr register structure
 * @param     mask   The bitwise OR of interrupts to enable.
 *                   See #mxc_htmr_int_en_t for available choices.
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_EnableInt(mxc_htmr_regs_t *htmr, uint32_t mask);

/**
 * @brief     Disable Interurpts
 * @param     htmr   pointer to the htmr register structure
 * @param     mask   The mask of interrupts to disable.
 *                   See #mxc_htmr_int_en_t for available choices.
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_DisableInt(mxc_htmr_regs_t *htmr, uint32_t mask);

/**
 * @brief     Set Long Interval alarm value and enable Interrupt
 * @param     htmr    pointer to the htmr register structure
 * @param     interval    20-bit value 0-0xFFFFF
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_SetLongAlarm(mxc_htmr_regs_t *htmr, uint32_t interval);

/**
 * @brief     Set Short Interval alarm value and enable interrupt,
 * @param     htmr    pointer to the htmr register structure
 * @param     interval   32-bit value 0-0xFFFFFFFF
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_SetShortAlarm(mxc_htmr_regs_t *htmr, uint32_t interval);

/**
 * @brief     Enable/Start the High Speed Timer
 * @param     htmr    pointer to the htmr register structure
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_Start(mxc_htmr_regs_t *htmr);

/**
 * @brief     Disable/Stop the High Speed Timer
 * @param     htmr    pointer to the htmr register structure
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_Stop(mxc_htmr_regs_t *htmr);

/**
 * @brief     Initialize the longInterval and shortInterval registers and enable MXC_HTMR
 * @param     htmr   pointer to the htmr register structure
 * @param     longInterval    set the MXC_HTMR long counter (32-bit)
 * @param     shortInterval   set the MXC_HTMR short counter (8-bit)
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_Init(mxc_htmr_regs_t *htmr, uint32_t longInterval, uint8_t shortInterval);

/**
 * @brief     Check if BUSY bit is 0.
 * @param     htmr   pointer to the htmr register structure
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_CheckBusy(mxc_htmr_regs_t *htmr);

/**
 * @brief     Get interrupt flags.
 * @param     htmr   pointer to the htmr register structure
 * @return    The bitwise OR of any interrupts flags that are
 *            currently set. See #mxc_htmr_int_fl_t for the list
 *            of possible flags.
 */
int MXC_HTMR_GetFlags(mxc_htmr_regs_t *htmr);

/**
 * @brief     Clear interrupt flags.
 * @param     htmr   pointer to the htmr register structure
 * @param     flags The bitwise OR of the interrupts flags to cleear.
 *            See #mxc_htmr_int_fl_t for the list of possible flags.
 * @retval    returns Success or Fail, see \ref MXC_ERROR_CODES
 */
int MXC_HTMR_ClearFlags(mxc_htmr_regs_t *htmr, int flags);

/**
 * @brief     Get value in short interval register
 * @param     htmr   pointer to the htmr register structure
 * @return    Returns short count value
 */
int MXC_HTMR_GetShortCount(mxc_htmr_regs_t *htmr);

/**
 * @brief     Get value in long interval register
 * @param     htmr   pointer to the htmr register structure
 * @return    returns long count value
 */
int MXC_HTMR_GetLongCount(mxc_htmr_regs_t *htmr);

#ifdef __cplusplus
}
#endif

/**@} end of group htmr */
#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_HTMR_H_

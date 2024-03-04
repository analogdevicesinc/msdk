/**
 * @file    qdec.h
 * @brief   Quadrature Encoder function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32672_QDEC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32672_QDEC_H_

/* **** Includes **** */
#include <stdint.h>
#include "qdec_regs.h"
#include "mcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup qdec QDEC
 * @ingroup periphlibs
 * @{
 */

/***************************************************************************************************************
                                    DATA STRUCTURES FOR QDEC INITIALIZATION
***************************************************************************************************************/

/**
  * @brief  Enumeration type for the QDEC Counter Modes
  */
typedef enum {
    MXC_QDEC_X1_MODE = 0, ///< Select Channel 0
    MXC_QDEC_X2_MODE = 1, ///< Select Channel 1
    MXC_QDEC_x4_MODE, ///< Select Channel 2
} mxc_qdec_counter_mode_t;

/**
 * @brief       Enumeration type for QDEC Phase Swap
 */
typedef enum {
    MXC_QDEC_SWAP_CW_A_LEADS_B, ///< QDEC Clockwise QEA leads QEB
    MXC_QDEC_SWAP_CW_B_LEADS_A, ///< QDEC Clockwise QEB leads QEA
} mxc_qdec_swap_t;

/**
 * @brief       Enumeration type for QDEC Filtering
 */
typedef enum {
    MXC_QDEC_FILTER_1_SAMPLE =
        MXC_S_QDEC_CTRL_FILTER_1_SAMPLE, ///< QDEC Pulse active for one clock cycle
    MXC_QDEC_FILTER_2_SAMPLES =
        MXC_S_QDEC_CTRL_FILTER_2_SAMPLES, ///< QDEC Pulse active for one clock cycle
    MXC_QDEC_FILTER_3_SAMPLES =
        MXC_S_QDEC_CTRL_FILTER_3_SAMPLES, ///< QDEC Pulse active for one clock cycle
    MXC_QDEC_FILTER_4_SAMPLES =
        MXC_S_QDEC_CTRL_FILTER_4_SAMPLES, ///< QDEC Pulse active for one clock cycle
} mxc_qdec_filter_t;

/**
 * @brief       Enumeration type for QDEC Sticky Condition
 */
typedef enum {
    MXC_QDEC_STICKY_PULSE = 0, ///< QDEC Pulse active for one clock cycle
    MXC_QDEC_STICKY_MIRROR, ///< QDEC Mirror state
} mxc_qdec_sticky_t;

/**
 * @brief       Enumeration type for QDEC clock divider
 */
typedef enum {
    MXC_QDEC_CLKDIV_1 = 0, ///< QDEC Scale by 1
    MXC_QDEC_CLKDIV_2, ///< QDEC Scale by 1/2
    MXC_QDEC_CLKDIV_4, ///< QDEC Scale by 1/4
    MXC_QDEC_CLKDIV_8, ///< QDEC Scale by 1/8
    MXC_QDEC_CLKDIV_16, ///< QDEC Scale by 1/16
    MXC_QDEC_CLKDIV_32, ///< QDEC Scale by 1/32
    MXC_QDEC_CLKDIV_64, ///< QDEC Scale by 1/64
    MXC_QDEC_CLKDIV_128, ///< QDEC Scale by 1/128
} mxc_qdec_clkdiv_t;

/**
 * @brief       Enumeration type for QDEC Reset on select
 */
typedef enum {
    MXC_QDEC_RST_ON_MAXCNT = 0,
    MXC_QDEC_RST_ON_INDEX,
} mxc_qdec_rst_on_t;

/**
 * @brief       Enumeration type for QDEC function states
 */
typedef enum {
    MXC_QDEC_NONE = 0,
    MXC_QDEC_CAPTURE,
    MXC_QDEC_COMPARE,
} mxc_qdec_function_t;

///< Callback used when interrupt occurs
typedef void (*mxc_qdec_cb_t)(void *req, int error);

typedef struct {
    mxc_qdec_counter_mode_t mode; ///< counter mode
    mxc_qdec_swap_t swap; ///< phase swap
    mxc_qdec_filter_t sample; ///< filter
    mxc_qdec_clkdiv_t clkdiv; ///< clock divider
    mxc_qdec_sticky_t sticky; ///< sticky condition
    mxc_qdec_rst_on_t rst; ///< reset on maxcnt or index
    mxc_qdec_function_t func; ///< compare, capture, none
    uint32_t maxcnt; ///< Maximum count value
    uint32_t initial; ///< Initial count value
    uint32_t compare; ///< Compare value
    mxc_qdec_cb_t callback;
} mxc_qdec_req_t;

/**
 * @brief   Performs the QDEC startup procedure
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_QDEC_Init(mxc_qdec_req_t *req);

/**
 * @brief   Shuts down the QDEC
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_QDEC_Shutdown(void);

/**
 * @brief   Enable specific QDEC interrupts
 *
 * @param   flags mask of interrupt flags to enables
 */
void MXC_QDEC_EnableInt(uint32_t flags);

/**
 * @brief   Disable specific QDEC interrupts
 *
 * @param   flags mask of interrupt flags to enables
 */
void MXC_QDEC_DisableInt(uint32_t flags);

/**
 * @brief   Retrieves current interrupt flag status
 *
 * @return  active flags
 */
int MXC_QDEC_GetFlags(void);

/**
 * @brief   Clear interrupt flags
 *
 * @param   flags mask of flags to clear
 */
void MXC_QDEC_ClearFlags(uint32_t flags);

/**
 * @brief   Sets the Maximum Poisiton Count for the QDEC
 *
 * @param   maximum position value
 */
void MXC_QDEC_SetMaxCount(uint32_t maxCount);

/**
 * @brief   Retrieves Maximum position value
 *
 * @return  maximum position value
 */
int MXC_QDEC_GetMaxCount(void);

/**
 * @brief   Sets the Initial (Minimum) Poisiton Count for the QDEC
 *
 * @param   minimum position value
 */
void MXC_QDEC_SetInitial(uint32_t initial);

/**
 * @brief   Retrieves Initial position value
 *
 * @return  minimum position value
 */
int MXC_QDEC_GetInitial(void);

/**
 * @brief   Sets the Compare value for the QDEC
 *
 * @param   compare register value
 */
void MXC_QDEC_SetCompare(uint32_t compare);

/**
 * @brief   Retrieves Compare value
 *
 * @return  value in compare register
 */
int MXC_QDEC_GetCompare(void);

/**
 * @brief   Retrieves Index value
 *
 * @return  value in index register
 */
int MXC_QDEC_GetIndex(void);

/**
 * @brief   Retrieves capture value
 *
 * @return  capture value latched from position register
 */
int MXC_QDEC_GetCapture(void);

/**
 * @brief   Call this function from the QDEC ISR when using Async API
 *             functions
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_QDEC_Handler(void);

/**
 * @brief   Gets the current position of the QDEC
 * 
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_QDEC_GetPosition(void);

/**
 * @brief   Gets the direction (Clockwise/Counter-Clockwise) of the QDEC
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_QDEC_GetDirection(void);

/**@} end of group qdec */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32672_QDEC_H_

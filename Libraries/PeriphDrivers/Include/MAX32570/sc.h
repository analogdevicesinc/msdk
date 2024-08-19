/**
 * @file    sc.h
 * @brief   Smart Card (SC) interface functions and data types.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SC_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "scn_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup scn Smart Card (SC)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/**
 * Bitmasks for each smart card device
 */
typedef enum {
    MXC_SC_DEV_MIN = 0,
    MXC_SC_DEV0 = MXC_SC_DEV_MIN, /**< Smart Card 0 */
    MXC_SC_DEV1, /**< Smart Card 1 */
    MXC_SC_DEV_MAX = MXC_SC_DEV1,
    MXC_SC_DEV_COUNT /**< Number of Smart Card Devices */
} mxc_sc_id_t;

/* **** Structures **** */
/**
  * @brief      Structure for smart card register
  *
  */
typedef struct {
    mxc_scn_regs_t *reg_sc;
} mxc_sc_info_t;

/**
  * @brief      Structure to save smart card state
  *
  */
typedef struct {
    unsigned char first_init;
    mxc_sc_info_t sc[MXC_SC_DEV_COUNT];
} mxc_sc_context_t;

/* **** Function Prototypes **** */

/**
 * @brief       Inititalize Smart Card Interface
 *
 * @param       id Smart Card interface id
 * @return      Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int mxc_sc_init(mxc_sc_id_t id);

#ifdef __cplusplus
}
#endif

/**@} end of group sc */
#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SC_H_

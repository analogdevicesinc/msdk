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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CAMERAIF_CAMERAIF_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CAMERAIF_CAMERAIF_REVA_H_

/****** Includes *******/
#include <stdint.h>
#include "cameraif_reva_regs.h"

/***** Definitions *****/

/**
 * @brief   The list of Camera Interface Datawith options supported
 *
 */
typedef enum {
    MXC_PCIF_REVA_DATAWITH_8_BIT = 0, ///<
    MXC_PCIF_REVA_DATAWITH_10_BIT, ///<
    MXC_PCIF_REVA_DATAWITH_12_BIT, ///<
} mxc_pcif_reva_datawith_t;

/**
 * @brief   The list of Camera Interface ReadMode options supported
 *
 */
typedef enum {
    MXC_PCIF_REVA_READMODE_SINGLE_MODE = 1, ///<
    MXC_PCIF_REVA_READMODE_CONTINUES_MODE, ///<
} mxc_pcif_reva_readmode_t;

/**
 * @brief   The list of Camera Interface TimingSel options supported
 *
 */
typedef enum {
    MXC_PCIF_REVA_TIMINGSEL_HSYNC_and_VSYNC = 0, ///<
    MXC_PCIF_REVA_TIMINGSEL_SAV_and_EAV, ///<
} mxc_pcif_reva_timingsel_t;

/******* Globals *******/

/****** Functions ******/
int MXC_PCIF_RevA_Init(void);
void MXC_PCIF_RevA_SetDatawidth(mxc_cameraif_reva_regs_t *cameraif,
                                mxc_pcif_reva_datawith_t datawith);
void MXC_PCIF_RevA_SetTimingSel(mxc_cameraif_reva_regs_t *cameraif,
                                mxc_pcif_reva_timingsel_t timingsel);
void MXC_PCIF_RevA_SetThreshold(mxc_cameraif_reva_regs_t *cameraif, int fifo_thrsh);
void MXC_PCIF_RevA_EnableInt(mxc_cameraif_reva_regs_t *cameraif, uint32_t flags);
void MXC_PCIF_RevA_DisableInt(mxc_cameraif_reva_regs_t *cameraif, uint32_t flags);
void MXC_PCIF_RevA_Start(mxc_cameraif_reva_regs_t *cameraif, mxc_pcif_reva_readmode_t readmode);
void MXC_PCIF_RevA_Stop(mxc_cameraif_reva_regs_t *cameraif);
unsigned int MXC_PCIF_RevA_GetData(mxc_cameraif_reva_regs_t *cameraif);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CAMERAIF_CAMERAIF_REVA_H_

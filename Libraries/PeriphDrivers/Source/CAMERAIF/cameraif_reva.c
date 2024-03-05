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

/* **** Includes **** */
#include <string.h>
#include "cameraif_reva.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_sys.h"

/* **** Definitions **** */
#define PCIF_REVA_IE_MASK                                                         \
    (MXC_F_CAMERAIF_REVA_INT_EN_IMG_DONE | MXC_F_CAMERAIF_REVA_INT_EN_FIFO_FULL | \
     MXC_F_CAMERAIF_REVA_INT_EN_FIFO_THRESH | MXC_F_CAMERAIF_REVA_INT_EN_FIFO_NOT_EMPTY)

/* **** Globals **** */

/* **** Functions **** */

int MXC_PCIF_RevA_Init(void)
{
    return 0;
}

void MXC_PCIF_RevA_SetDatawidth(mxc_cameraif_reva_regs_t *cameraif,
                                mxc_pcif_reva_datawith_t datawith)
{
    cameraif->ctrl &= ~(MXC_F_CAMERAIF_REVA_CTRL_DATA_WIDTH);
    cameraif->ctrl |= (datawith << MXC_F_CAMERAIF_REVA_CTRL_DATA_WIDTH_POS);
}

void MXC_PCIF_RevA_SetTimingSel(mxc_cameraif_reva_regs_t *cameraif,
                                mxc_pcif_reva_timingsel_t timingsel)
{
    cameraif->ctrl &= ~(MXC_F_CAMERAIF_REVA_CTRL_DS_TIMING_EN);
    cameraif->ctrl |= (timingsel << MXC_F_CAMERAIF_REVA_CTRL_DS_TIMING_EN_POS);
}

void MXC_PCIF_RevA_SetThreshold(mxc_cameraif_reva_regs_t *cameraif, int fifo_thrsh)
{
    cameraif->ctrl &= ~(MXC_F_CAMERAIF_REVA_CTRL_FIFO_THRSH);
    cameraif->ctrl |= ((fifo_thrsh << MXC_F_CAMERAIF_REVA_CTRL_FIFO_THRSH_POS) &
                       MXC_F_CAMERAIF_REVA_CTRL_FIFO_THRSH);
}

void MXC_PCIF_RevA_EnableInt(mxc_cameraif_reva_regs_t *cameraif, uint32_t flags)
{
    cameraif->int_en |= (flags & PCIF_REVA_IE_MASK);
}

void MXC_PCIF_RevA_DisableInt(mxc_cameraif_reva_regs_t *cameraif, uint32_t flags)
{
    cameraif->int_en &= ~(flags & PCIF_REVA_IE_MASK);
}

void MXC_PCIF_RevA_Start(mxc_cameraif_reva_regs_t *cameraif, mxc_pcif_reva_readmode_t readmode)
{
    cameraif->ctrl &= ~(MXC_F_CAMERAIF_REVA_CTRL_READ_MODE);
    cameraif->ctrl |= (readmode & MXC_F_CAMERAIF_REVA_CTRL_READ_MODE);
}

void MXC_PCIF_RevA_Stop(mxc_cameraif_reva_regs_t *cameraif)
{
    cameraif->ctrl &= ~(MXC_F_CAMERAIF_REVA_CTRL_READ_MODE);
}

unsigned int MXC_PCIF_RevA_GetData(mxc_cameraif_reva_regs_t *cameraif)
{
    return (unsigned int)(cameraif->fifo_data);
}

/**@} end of group cameraif */

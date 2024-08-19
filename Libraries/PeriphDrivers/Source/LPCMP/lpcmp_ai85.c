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

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_lock.h"
#include "mxc_pins.h"
#include "mcr_regs.h"
#include "lpcmp.h"
#include "lpcmp_regs.h"
#include "lpcmp_reva.h"

static int init = 0;
static const mxc_lpcmp_ctrl_reg_t lpcmp_ctrl_regs[] = { &MXC_MCR->cmp_ctrl, &MXC_LPCMP->ctrl[0],
                                                        &MXC_LPCMP->ctrl[1], &MXC_LPCMP->ctrl[2] };

static void initGPIOForComp(mxc_lpcmp_cmpsel_t cmp)
{
    switch (cmp) {
    case MXC_LPCMP_CMP0:
        MXC_GPIO_Config(&gpio_cfg_cmp0);
        break;
    case MXC_LPCMP_CMP1:
        MXC_GPIO_Config(&gpio_cfg_cmp1);
        break;
    case MXC_LPCMP_CMP2:
        MXC_GPIO_Config(&gpio_cfg_cmp2);
        break;
    case MXC_LPCMP_CMP3:
        MXC_GPIO_Config(&gpio_cfg_cmp3);
        break;
    default:
        break;
    }
}

int MXC_LPCMP_Init(mxc_lpcmp_cmpsel_t cmp)
{
    int ret;

    initGPIOForComp(cmp);

    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_LPCOMP);

    if ((ret = MXC_LPCMP_RevA_Init(lpcmp_ctrl_regs[cmp])) == E_NO_ERROR) {
        init++;
    }

    return ret;
}

int MXC_LPCMP_Shutdown(mxc_lpcmp_cmpsel_t cmp)
{
    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    init--;
    if (init == 0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_LPCOMP);
    }

    return MXC_LPCMP_RevA_Shutdown(lpcmp_ctrl_regs[cmp]);
}

int MXC_LPCMP_EnableInt(mxc_lpcmp_cmpsel_t cmp, mxc_lpcmp_polarity_t pol)
{
    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    MXC_LPCMP_ClearFlags(cmp);
    MXC_LPCMP_SelectPolarity(cmp, pol);

    return MXC_LPCMP_RevA_EnableInt(lpcmp_ctrl_regs[cmp]);
}

int MXC_LPCMP_DisableInt(mxc_lpcmp_cmpsel_t cmp)
{
    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    return MXC_LPCMP_RevA_DisableInt(lpcmp_ctrl_regs[cmp]);
}

int MXC_LPCMP_GetFlags(mxc_lpcmp_cmpsel_t cmp)
{
    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    return MXC_LPCMP_RevA_GetFlags(lpcmp_ctrl_regs[cmp]);
}

int MXC_LPCMP_ClearFlags(mxc_lpcmp_cmpsel_t cmp)
{
    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    return MXC_LPCMP_RevA_ClearFlags(lpcmp_ctrl_regs[cmp]);
}

int MXC_LPCMP_SelectPolarity(mxc_lpcmp_cmpsel_t cmp, mxc_lpcmp_polarity_t pol)
{
    if (cmp < MXC_LPCMP_CMP0 || cmp > MXC_LPCMP_CMP3) {
        return E_BAD_PARAM;
    }

    return MXC_LPCMP_RevA_SelectPolarity(lpcmp_ctrl_regs[cmp], pol);
}

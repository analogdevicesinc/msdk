/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *************************************************************************** */

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

static int init                                     = 0;
static const mxc_lpcmp_ctrl_reg_t lpcmp_ctrl_regs[] = {&MXC_MCR->cmp_ctrl, &MXC_LPCMP->ctrl[0],
                                                       &MXC_LPCMP->ctrl[1], &MXC_LPCMP->ctrl[2]};

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

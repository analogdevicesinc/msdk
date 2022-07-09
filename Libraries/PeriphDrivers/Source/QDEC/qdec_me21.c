/* ****************************************************************************
 * Copyright(C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
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
#include "qdec.h"
#include "qdec_regs.h"
#include "qdec_reva.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include "mxc_pins.h"


int MXC_QDEC_Init(mxc_qdec_req_t *req)
{
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_QDEC);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_QDEC);
    
    MXC_GPIO_Config(&gpio_cfg_qdec_in);
    MXC_GPIO_Config(&gpio_cfg_qdec_out);

    return MXC_QDEC_RevA_Init((mxc_qdec_reva_regs_t*) MXC_QDEC, req);
}

int MXC_QDEC_Shutdown(void)
{
    MXC_QDEC_RevA_Shutdown((mxc_qdec_reva_regs_t*) MXC_QDEC);
    
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_QDEC);
    
    return E_NO_ERROR;
}

void MXC_QDEC_EnableInt(uint32_t flags)
{
    MXC_QDEC_RevA_EnableInt((mxc_qdec_reva_regs_t*) MXC_QDEC, flags);
}

void MXC_QDEC_DisableInt(uint32_t flags)
{
    MXC_QDEC_RevA_DisableInt((mxc_qdec_reva_regs_t*) MXC_QDEC, flags);
}

int MXC_QDEC_GetFlags(void)
{
    return MXC_QDEC_RevA_GetFlags((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

void MXC_QDEC_ClearFlags(uint32_t flags)
{
    MXC_QDEC_RevA_ClearFlags((mxc_qdec_reva_regs_t*) MXC_QDEC, flags);
}

void MXC_QDEC_SetMaxCount(uint32_t maxCount)
{
    MXC_QDEC_RevA_SetMaxCount((mxc_qdec_reva_regs_t*) MXC_QDEC, maxCount);
}

int MXC_QDEC_GetMaxCount(void)
{
    return MXC_QDEC_RevA_GetMaxCount((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

void MXC_QDEC_SetInitial(uint32_t initial)
{
    MXC_QDEC_RevA_SetInitial((mxc_qdec_reva_regs_t*) MXC_QDEC, initial);
}

int MXC_QDEC_GetInitial(void)
{
    return MXC_QDEC_RevA_GetInitial((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

void MXC_QDEC_SetCompare(uint32_t compare)
{
    MXC_QDEC_RevA_SetCompare((mxc_qdec_reva_regs_t*) MXC_QDEC, compare);
}

int MXC_QDEC_GetCompare(void)
{
    return MXC_QDEC_RevA_GetCompare((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

int MXC_QDEC_GetIndex(void)
{
    return MXC_QDEC_RevA_GetIndex((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

int MXC_QDEC_GetCapture(void)
{
    return MXC_QDEC_RevA_GetCapture((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

int MXC_QDEC_Handler(void)
{
    return MXC_QDEC_RevA_Handler((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

int MXC_QDEC_GetDirection(void)
{
    return MXC_QDEC_RevA_GetDirection((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

int MXC_QDEC_GetPosition(void)
{
    return MXC_QDEC_RevA_GetPosition((mxc_qdec_reva_regs_t*) MXC_QDEC);
}

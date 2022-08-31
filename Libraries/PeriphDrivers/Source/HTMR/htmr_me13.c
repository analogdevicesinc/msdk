/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include "htmr.h"
#include "htmr_regs.h"
#include "htmr_reva.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* ****** Functions ****** */
int MXC_HTMR_Init(mxc_htmr_regs_t* htmr, uint32_t longInterval, uint8_t shortInterval)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO) != E_NO_ERROR) {
        return E_TIME_OUT;
    }

    if (htmr == MXC_HTMR0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HTMR0);
    } else if (htmr == MXC_HTMR1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HTMR1);
    } else {
        return E_BAD_PARAM;
    }

    return MXC_HTMR_RevA_Init((mxc_htmr_reva_regs_t*)htmr, longInterval, shortInterval);
}

int MXC_HTMR_Start(mxc_htmr_regs_t* htmr)
{
    return MXC_HTMR_RevA_Start((mxc_htmr_reva_regs_t*)htmr);
}

int MXC_HTMR_Stop(mxc_htmr_regs_t* htmr)
{
    return MXC_HTMR_RevA_Stop((mxc_htmr_reva_regs_t*)htmr);
}

int MXC_HTMR_GetShortCount(mxc_htmr_regs_t* htmr)
{
    return MXC_HTMR_RevA_GetShortCount((mxc_htmr_reva_regs_t*)htmr);
}

int MXC_HTMR_GetLongCount(mxc_htmr_regs_t* htmr)
{
    return MXC_HTMR_RevA_GetLongCount((mxc_htmr_reva_regs_t*)htmr);
}

int MXC_HTMR_SetLongAlarm(mxc_htmr_regs_t* htmr, uint32_t interval)
{
    return MXC_HTMR_RevA_SetLongAlarm((mxc_htmr_reva_regs_t*)htmr, interval);
}

int MXC_HTMR_SetShortAlarm(mxc_htmr_regs_t* htmr, uint32_t interval)
{
    return MXC_HTMR_RevA_SetShortAlarm((mxc_htmr_reva_regs_t*)htmr, interval);
}

int MXC_HTMR_CheckBusy(mxc_htmr_regs_t* htmr)
{
    return MXC_HTMR_RevA_CheckBusy((mxc_htmr_reva_regs_t*)htmr);
}

int MXC_HTMR_GetFlags(mxc_htmr_regs_t* htmr)
{
    return MXC_HTMR_RevA_GetFlags((mxc_htmr_reva_regs_t*)htmr);
}

int MXC_HTMR_ClearFlags(mxc_htmr_regs_t* htmr, int flags)
{
    return MXC_HTMR_RevA_ClearFlags((mxc_htmr_reva_regs_t*)htmr, flags);
}

int MXC_HTMR_EnableInt(mxc_htmr_regs_t* htmr, uint32_t mask)
{
    return MXC_HTMR_RevA_EnableInt((mxc_htmr_reva_regs_t*)htmr, mask);
}

int MXC_HTMR_DisableInt(mxc_htmr_regs_t* htmr, uint32_t mask)
{
    return MXC_HTMR_RevA_DisableInt((mxc_htmr_reva_regs_t*)htmr, mask);
}

/* *****************************************************************************
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
 **************************************************************************** */

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sema_reva.h"

/* ***** Functions ***** */

int MXC_SEMA_Init(void)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SMPHR);
    return MXC_SEMA_RevA_Init((mxc_sema_reva_regs_t*)MXC_SEMA);
}

int MXC_SEMA_InitBoxes(void)
{
    return MXC_SEMA_RevA_InitBoxes((mxc_sema_reva_regs_t*)MXC_SEMA);
}

int MXC_SEMA_GetSema(unsigned sema)
{
    return MXC_SEMA_RevA_GetSema((mxc_sema_reva_regs_t*)MXC_SEMA, sema);
}

int MXC_SEMA_CheckSema(unsigned sema)
{
    return MXC_SEMA_RevA_CheckSema((mxc_sema_reva_regs_t*)MXC_SEMA, sema);
}

uint32_t MXC_SEMA_Status(void)
{
    return MXC_SEMA_RevA_Status((mxc_sema_reva_regs_t*)MXC_SEMA);
}

void MXC_SEMA_FreeSema(unsigned sema)
{
    MXC_SEMA_RevA_FreeSema((mxc_sema_reva_regs_t*)MXC_SEMA, sema);
}

int MXC_SEMA_Shutdown(void)
{
    return E_NO_ERROR;
}

int MXC_SEMA_ReadBox(uint8_t* data, unsigned len)
{
    return MXC_SEMA_RevA_ReadBox((mxc_sema_reva_regs_t*)MXC_SEMA, data, len);
}

int MXC_SEMA_WriteBox(const uint8_t* data, unsigned len)
{
    return MXC_SEMA_RevA_WriteBox((mxc_sema_reva_regs_t*)MXC_SEMA, data, len);
}

int MXC_SEMA_Handler(void)
{
    return MXC_SEMA_RevA_Handler((mxc_sema_reva_regs_t*)MXC_SEMA);
}

int MXC_SEMA_ReadBoxAsync(mxc_sema_complete_cb_t cb, uint8_t* data, unsigned len)
{
    return MXC_SEMA_RevA_ReadBoxAsync((mxc_sema_reva_regs_t*)MXC_SEMA, cb, data, len);
}

int MXC_SEMA_WriteBoxAsync(mxc_sema_complete_cb_t cb, const uint8_t* data, unsigned len)
{
    return MXC_SEMA_RevA_WriteBoxAsync((mxc_sema_reva_regs_t*)MXC_SEMA, cb, data, len);
}

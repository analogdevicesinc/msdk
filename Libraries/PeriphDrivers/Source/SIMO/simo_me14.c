/**
 * @file       simo_me14.c
 * @brief      This file contains the function implementations for the
 *             SIMO peripheral module.
 */

/* *****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-08-28 17:03:02 -0500 (Tue, 28 Aug 2018) $
 * $Revision: 37424 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "simo_reva.h"

/**
 * @ingroup simo
 * @{
 */

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */
// All Voltages are in mV

void MXC_SIMO_SetVregO_A(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_A((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_B(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_B((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_C(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_C((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_D(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_D((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

uint32_t MXC_SIMO_GetOutReadyA(void)
{
    return MXC_SIMO_RevA_GetOutReadyA((mxc_simo_reva_regs_t *)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyB(void)
{
    return MXC_SIMO_RevA_GetOutReadyB((mxc_simo_reva_regs_t *)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyC(void)
{
    return MXC_SIMO_RevA_GetOutReadyC((mxc_simo_reva_regs_t *)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyD(void)
{
    return MXC_SIMO_RevA_GetOutReadyD((mxc_simo_reva_regs_t *)MXC_SIMO);
}

/**@} end of group simo */

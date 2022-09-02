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
 **************************************************************************** */

/* **** Includes **** */
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "simo_reva.h"

/* **** Functions **** */
void MXC_SIMO_SetVregO_A(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_A((mxc_simo_reva_regs_t*)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_B(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_B((mxc_simo_reva_regs_t*)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_C(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_C((mxc_simo_reva_regs_t*)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_D(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_D((mxc_simo_reva_regs_t*)MXC_SIMO, voltage);
}

// void MXC_SIMO_setIpkA(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkA(peak_current);
// }

// void MXC_SIMO_setIpkB(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkB(peak_current);
// }

// void MXC_SIMO_setIpkC(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkC(peak_current);
// }

// void MXC_SIMO_setIpkD(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkD(peak_current);
// }

// void MXC_SIMO_setMaxTon(uint32_t ontime)
// {
//     MXC_SIMO_RevA_setMaxTon(ontime);
// }

// void MXC_SIMO_setAlertThresholdA(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdA(threshold);
// }

// void MXC_SIMO_setAlertThresholdB(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdB(threshold);
// }

// void MXC_SIMO_setAlertThresholdC(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdC(threshold);
// }

// void MXC_SIMO_setAlertThresholdD(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdD(threshold);
// }

// void MXC_SIMO_setZeroCrossCalA(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalA(zerocross);
// }

// void MXC_SIMO_setZeroCrossCalB(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalB(zerocross);
// }

// void MXC_SIMO_setZeroCrossCalC(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalC(zerocross);
// }

// void MXC_SIMO_setZeroCrossCalD(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalD(zerocross);
// }

uint32_t MXC_SIMO_GetOutReadyA(void)
{
    return MXC_SIMO_RevA_GetOutReadyA((mxc_simo_reva_regs_t*)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyB(void)
{
    return MXC_SIMO_RevA_GetOutReadyB((mxc_simo_reva_regs_t*)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyC(void)
{
    return MXC_SIMO_RevA_GetOutReadyC((mxc_simo_reva_regs_t*)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyD(void)
{
    return MXC_SIMO_RevA_GetOutReadyD((mxc_simo_reva_regs_t*)MXC_SIMO);
}

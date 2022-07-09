/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
******************************************************************************/

/* **** Includes **** */
#include <string.h>
#include "cameraif.h"
#include "cameraif_reva.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

int MXC_PCIF_Init(void)
{
    return MXC_PCIF_RevA_Init();
}

void MXC_PCIF_SetDatawidth(mxc_pcif_datawidth_t  datawidth)
{
    MXC_PCIF_RevA_SetDatawidth ((mxc_cameraif_reva_regs_t*) MXC_PCIF, (mxc_pcif_reva_datawith_t)datawidth);
}

void MXC_PCIF_SetTimingSel(mxc_pcif_timingsel_t timingsel)
{
    MXC_PCIF_RevA_SetTimingSel ((mxc_cameraif_reva_regs_t*) MXC_PCIF, timingsel);
}

void MXC_PCIF_SetThreshold(int fifo_thrsh)
{
    MXC_PCIF_RevA_SetThreshold ((mxc_cameraif_reva_regs_t*) MXC_PCIF, fifo_thrsh);
}

void MXC_PCIF_EnableInt(uint32_t flags)
{
    MXC_PCIF_RevA_EnableInt ((mxc_cameraif_reva_regs_t*) MXC_PCIF, flags);
}

void MXC_PCIF_DisableInt(uint32_t flags)
{
    MXC_PCIF_RevA_DisableInt ((mxc_cameraif_reva_regs_t*) MXC_PCIF, flags);
}

void MXC_PCIF_Start(mxc_pcif_readmode_t  readmode)
{
    MXC_PCIF_RevA_Start ((mxc_cameraif_reva_regs_t*) MXC_PCIF, readmode);
}

void MXC_PCIF_Stop(void)
{
    MXC_PCIF_RevA_Stop((mxc_cameraif_reva_regs_t*) MXC_PCIF);
}

unsigned int MXC_PCIF_GetData(void)
{
    return MXC_PCIF_RevA_GetData((mxc_cameraif_reva_regs_t*) MXC_PCIF);
}

/**@} end of group cameraif */

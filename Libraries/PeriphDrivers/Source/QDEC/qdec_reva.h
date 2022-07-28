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
#include "qdec.h"
#include "qdec_reva_regs.h"

int MXC_QDEC_RevA_Init (mxc_qdec_reva_regs_t* qdec, mxc_qdec_req_t *req);
int MXC_QDEC_RevA_Shutdown (mxc_qdec_reva_regs_t* qdec);
void MXC_QDEC_RevA_EnableInt (mxc_qdec_reva_regs_t* qdec, uint32_t flags);
void MXC_QDEC_RevA_DisableInt (mxc_qdec_reva_regs_t* qdec, uint32_t flags);
int MXC_QDEC_RevA_GetFlags (mxc_qdec_reva_regs_t* qdec);
void MXC_QDEC_RevA_ClearFlags (mxc_qdec_reva_regs_t* qdec, uint32_t flags);
void MXC_QDEC_RevA_SetMaxCount (mxc_qdec_reva_regs_t* qdec, uint32_t maxCount);
int MXC_QDEC_RevA_GetMaxCount (mxc_qdec_reva_regs_t* qdec);
void MXC_QDEC_RevA_SetInitial (mxc_qdec_reva_regs_t* qdec, uint32_t initial);
int MXC_QDEC_RevA_GetInitial (mxc_qdec_reva_regs_t* qdec);
void MXC_QDEC_RevA_SetCompare (mxc_qdec_reva_regs_t* qdec, uint32_t compare);
int MXC_QDEC_RevA_GetCompare (mxc_qdec_reva_regs_t* qdec);
void MXC_QDEC_RevA_SetIndex (mxc_qdec_reva_regs_t* qdec, uint32_t index);
int MXC_QDEC_RevA_GetIndex (mxc_qdec_reva_regs_t* qdec);
int MXC_QDEC_RevA_GetCapture (mxc_qdec_reva_regs_t* qdec);
int MXC_QDEC_RevA_Handler (mxc_qdec_reva_regs_t* qdec);
int MXC_QDEC_RevA_GetPosition (mxc_qdec_reva_regs_t* qdec);
int MXC_QDEC_RevA_GetDirection (mxc_qdec_reva_regs_t* qdec);

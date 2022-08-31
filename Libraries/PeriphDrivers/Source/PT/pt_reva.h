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
#include "gcr_regs.h"
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "pt.h"
#include "pt_reva_regs.h"
#include "ptg_reva_regs.h"
#include <stdio.h>

void MXC_PT_RevA_Init(mxc_ptg_reva_regs_t* ptg, mxc_clk_scale_t clk_scale);
int MXC_PT_RevA_Shutdown(mxc_ptg_reva_regs_t* ptg, uint32_t pts);
int MXC_PT_RevA_Config(mxc_ptg_reva_regs_t* ptg, mxc_pt_cfg_t* cfg);
int MXC_PT_RevA_SqrWaveConfig(
    mxc_ptg_reva_regs_t* ptg, mxc_pt_cfg_t* sqwcfg, unsigned channel, uint32_t freq);
void MXC_PT_RevA_Start(mxc_ptg_reva_regs_t* ptg, unsigned pts);
void MXC_PT_RevA_Stop(mxc_ptg_reva_regs_t* ptg, unsigned pts);
uint32_t MXC_PT_RevA_IsActive(mxc_ptg_reva_regs_t* ptg, uint32_t pts);
void MXC_PT_RevA_SetPattern(unsigned pts, uint32_t pattern);
void MXC_PT_RevA_EnableInt(mxc_ptg_reva_regs_t* ptg, uint32_t pts);
void MXC_PT_RevA_DisableInt(mxc_ptg_reva_regs_t* ptg, uint32_t pts);
uint32_t MXC_PT_RevA_GetFlags(mxc_ptg_reva_regs_t* ptg);
void MXC_PT_RevA_ClearFlags(mxc_ptg_reva_regs_t* ptg, uint32_t flags);
void MXC_PT_RevA_EnableRestart(unsigned start, unsigned stop, uint8_t restartIndex);
void MXC_PT_RevA_DisableRestart(unsigned channel, uint8_t restartIndex);
void MXC_PT_RevA_Resync(mxc_ptg_reva_regs_t* ptg, uint32_t pts);

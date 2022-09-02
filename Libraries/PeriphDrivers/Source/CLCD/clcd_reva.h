/**
 * @file    clcd_reva.h
 * @brief   Color LCD function prototypes and data types.
 */

/* ****************************************************************************
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
 * $Date: 2017-06-08 09:55:03 -0500 (Thu, 08 Jun 2017) $
 * $Revision: 28436 $
 *
 *************************************************************************** */

/* Define to prevent redundant inclusion */
#ifndef _CLCD_REVA_H_
#define _CLCD_REVA_H_

/* **** Includes **** */
#include "clcd_reva_regs.h"
#include "clcd.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */

/* **** Function Prototypes **** */
int MXC_CLCD_RevA_Init(mxc_clcd_reva_regs_t* clcd, mxc_clcd_cfg_t* cfg);
int MXC_CLCD_RevA_Shutdown(mxc_clcd_reva_regs_t* clcd);
int MXC_CLCD_RevA_ConfigPanel(mxc_clcd_reva_regs_t* clcd, mxc_clcd_cfg_t* clcd_cfg);
int MXC_CLCD_RevA_Enable(mxc_clcd_reva_regs_t* clcd);
int MXC_CLCD_RevA_Disable(mxc_clcd_reva_regs_t* clcd);
int MXC_CLCD_RevA_SetFrameAddr(mxc_clcd_reva_regs_t* clcd, void* addr);

#ifdef __cplusplus
}
#endif

#endif /* _CLCD_REVA_H_ */

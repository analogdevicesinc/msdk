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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_EMCC_EMCC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_EMCC_EMCC_REVA_H_

/***** Includes *****/
#include "emcc_reva_regs.h"
#include "mxc_device.h"

/***** Definitions *****/

/**
 * @brief   Enumeration type for the EMCC Cache ID Register
 */
typedef enum {
    EMCC_REVA_CACHE_ID_RELNUM, ///< Release Number
    EMCC_REVA_CACHE_ID_PARTNUM, ///< Part Number
    EMCC_REVA_CACHE_ID_CCHID ///< Cache ID
} mxc_emcc_reva_info_t;

/***** Function Prototypes *****/
#if TARGET_NUM != 32650
#include "emcc.h"
uint32_t MXC_EMCC_RevA_ID(mxc_emcc_reva_regs_t *emcc, mxc_emcc_info_t id);
#endif

uint32_t MXC_EMCC_RevA_CacheSize(mxc_emcc_reva_regs_t *emcc);
uint32_t MXC_EMCC_RevA_MemSize(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_Enable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_Disable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_WriteAllocateEnable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_WriteAllocateDisable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_CriticalWordFirstEnable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_CriticalWordFirstDisable(mxc_emcc_reva_regs_t *emcc);
uint32_t MXC_EMCC_RevA_Ready(mxc_emcc_reva_regs_t *emcc);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_EMCC_EMCC_REVA_H_

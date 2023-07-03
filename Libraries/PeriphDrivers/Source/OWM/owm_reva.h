/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_H_

/* **** Includes **** */
#include "owm.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "owm_reva_regs.h"
#include "owm_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************* */
int MXC_OWM_RevA_Init(mxc_owm_reva_regs_t *owm, const mxc_owm_cfg_t *cfg);
void MXC_OWM_RevA_Shutdown(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_Reset(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_TouchByte(mxc_owm_reva_regs_t *owm, uint8_t data);
int MXC_OWM_RevA_WriteByte(uint8_t data);
int MXC_OWM_RevA_ReadByte(void);
int MXC_OWM_RevA_TouchBit(mxc_owm_reva_regs_t *owm, uint8_t bit);
int MXC_OWM_RevA_WriteBit(uint8_t bit);
int MXC_OWM_RevA_ReadBit(void);
int MXC_OWM_RevA_Write(mxc_owm_reva_regs_t *owm, uint8_t *data, int len);
int MXC_OWM_RevA_Read(mxc_owm_reva_regs_t *owm, uint8_t *data, int len);
int MXC_OWM_RevA_ReadROM(uint8_t *ROMCode);
int MXC_OWM_RevA_MatchROM(uint8_t *ROMCode);
int MXC_OWM_RevA_ODMatchROM(mxc_owm_reva_regs_t *owm, uint8_t *ROMCode);
int MXC_OWM_RevA_SkipROM(void);
int MXC_OWM_RevA_ODSkipROM(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_Resume(void);
int MXC_OWM_RevA_SearchROM(mxc_owm_reva_regs_t *owm, int newSearch, uint8_t *ROMCode);
void MXC_OWM_RevA_ClearFlags(mxc_owm_reva_regs_t *owm, uint32_t mask);
unsigned MXC_OWM_RevA_GetFlags(mxc_owm_reva_regs_t *owm);
void MXC_OWM_RevA_SetExtPullup(mxc_owm_reva_regs_t *owm, int enable);
void MXC_OWM_RevA_SetOverdrive(mxc_owm_reva_regs_t *owm, int enable);
void MXC_OWM_RevA_EnableInt(mxc_owm_reva_regs_t *owm, int flags);
void MXC_OWM_RevA_DisableInt(mxc_owm_reva_regs_t *owm, int flags);
int MXC_OWM_RevA_SetForcePresenceDetect(mxc_owm_reva_regs_t *owm, int enable);
int MXC_OWM_RevA_SetInternalPullup(mxc_owm_reva_regs_t *owm, int enable);
int MXC_OWM_RevA_SetExternalPullup(mxc_owm_reva_regs_t *owm, mxc_owm_ext_pu_t ext_pu_mode);
int MXC_OWM_RevA_SystemClockUpdated(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_SetSearchROMAccelerator(mxc_owm_reva_regs_t *owm, int enable);
int MXC_OWM_RevA_BitBang_Init(mxc_owm_reva_regs_t *owm, int initialState);
int MXC_OWM_RevA_BitBang_Read(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_BitBang_Write(mxc_owm_reva_regs_t *owm, int state);
int MXC_OWM_RevA_BitBang_Disable(mxc_owm_reva_regs_t *owm);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_H_

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
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "simo.h"
#include "simo_reva.h"

void MXC_SIMO_RevA_SetVregO_A(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
	uint32_t base_voltage = 0;
	if(simo->vrego_a & MXC_F_SIMO_REVA_VREGO_A_RANGEA) {
		base_voltage = VREGO_HIGH_RANGE_BASE;
	} else {
		base_voltage = VREGO_LOW_RANGE_BASE;
	}

	uint32_t setpoint = (voltage-base_voltage)/10;
	uint32_t value = (simo->vrego_a & ~MXC_F_SIMO_REVA_VREGO_A_VSETA) | (setpoint & MXC_F_SIMO_REVA_VREGO_A_VSETA);

	// Write the SIMO Registers twice due to clock glitch
	simo->vrego_a = value;
	simo->vrego_a = value;
}

void MXC_SIMO_RevA_SetVregO_B(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
	uint32_t base_voltage = 0;
	if(simo->vrego_b & MXC_F_SIMO_REVA_VREGO_B_RANGEB) {
		base_voltage = VREGO_HIGH_RANGE_BASE;
	} else {
		base_voltage = VREGO_LOW_RANGE_BASE;
	}

	uint32_t setpoint = (voltage-base_voltage)/10;
	uint32_t value = (simo->vrego_b & ~MXC_F_SIMO_REVA_VREGO_B_VSETB) | (setpoint & MXC_F_SIMO_REVA_VREGO_B_VSETB);

	// Write the SIMO Registers twice due to clock glitch
	simo->vrego_b = value;
	simo->vrego_b = value;
}

void MXC_SIMO_RevA_SetVregO_C(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
	uint32_t base_voltage = 0;
	if(simo->vrego_c & MXC_F_SIMO_REVA_VREGO_C_RANGEC) {
		base_voltage = VREGO_HIGH_RANGE_BASE;
	} else {
		base_voltage = VREGO_LOW_RANGE_BASE;
	}

	uint32_t setpoint = (voltage-base_voltage)/10;
	uint32_t value = (simo->vrego_c & ~MXC_F_SIMO_REVA_VREGO_C_VSETC) | (setpoint & MXC_F_SIMO_REVA_VREGO_C_VSETC);

	// Write the SIMO Registers twice due to clock glitch
	simo->vrego_c = value;
	simo->vrego_c = value;
}

void MXC_SIMO_RevA_SetVregO_D(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
	uint32_t base_voltage = 0;
	if(simo->vrego_d & MXC_F_SIMO_REVA_VREGO_D_RANGED) {
		base_voltage = VREGO_HIGH_RANGE_BASE;
	} else {
		base_voltage = VREGO_LOW_RANGE_BASE;
	}

	uint32_t setpoint = (voltage-base_voltage)/10;
	uint32_t value = (simo->vrego_d & ~MXC_F_SIMO_REVA_VREGO_D_VSETD) | (setpoint & MXC_F_SIMO_REVA_VREGO_D_VSETD);

	// Write the SIMO Registers twice due to clock glitch
	simo->vrego_d = value;
	simo->vrego_d = value;
}

uint32_t MXC_SIMO_RevA_GetOutReadyA(mxc_simo_reva_regs_t *simo)
{
	return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYA) ?
			E_NO_ERROR : E_BAD_STATE;
}

uint32_t MXC_SIMO_RevA_GetOutReadyB(mxc_simo_reva_regs_t *simo)
{
	return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYB) ?
			E_NO_ERROR : E_BAD_STATE;
}

uint32_t MXC_SIMO_RevA_GetOutReadyC(mxc_simo_reva_regs_t *simo)
{
	return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYC) ?
			E_NO_ERROR : E_BAD_STATE;
}

uint32_t MXC_SIMO_RevA_GetOutReadyD(mxc_simo_reva_regs_t *simo)
{
	return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYD) ?
			E_NO_ERROR : E_BAD_STATE;
}

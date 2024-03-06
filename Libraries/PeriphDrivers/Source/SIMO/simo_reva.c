/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
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
    if (simo->vrego_a & MXC_F_SIMO_REVA_VREGO_A_RANGEA) {
        base_voltage = VREGO_HIGH_RANGE_BASE;
    } else {
        base_voltage = VREGO_LOW_RANGE_BASE;
    }

    uint32_t setpoint = (voltage - base_voltage) / 10;
    uint32_t value = (simo->vrego_a & ~MXC_F_SIMO_REVA_VREGO_A_VSETA) |
                     (setpoint & MXC_F_SIMO_REVA_VREGO_A_VSETA);

    // Write the SIMO Registers twice due to clock glitch
    simo->vrego_a = value;
    simo->vrego_a = value;
}

void MXC_SIMO_RevA_SetVregO_B(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
    uint32_t base_voltage = 0;
    if (simo->vrego_b & MXC_F_SIMO_REVA_VREGO_B_RANGEB) {
        base_voltage = VREGO_HIGH_RANGE_BASE;
    } else {
        base_voltage = VREGO_LOW_RANGE_BASE;
    }

    uint32_t setpoint = (voltage - base_voltage) / 10;
    uint32_t value = (simo->vrego_b & ~MXC_F_SIMO_REVA_VREGO_B_VSETB) |
                     (setpoint & MXC_F_SIMO_REVA_VREGO_B_VSETB);

    // Write the SIMO Registers twice due to clock glitch
    simo->vrego_b = value;
    simo->vrego_b = value;
}

void MXC_SIMO_RevA_SetVregO_C(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
    uint32_t base_voltage = 0;
    if (simo->vrego_c & MXC_F_SIMO_REVA_VREGO_C_RANGEC) {
        base_voltage = VREGO_HIGH_RANGE_BASE;
    } else {
        base_voltage = VREGO_LOW_RANGE_BASE;
    }

    uint32_t setpoint = (voltage - base_voltage) / 10;
    uint32_t value = (simo->vrego_c & ~MXC_F_SIMO_REVA_VREGO_C_VSETC) |
                     (setpoint & MXC_F_SIMO_REVA_VREGO_C_VSETC);

    // Write the SIMO Registers twice due to clock glitch
    simo->vrego_c = value;
    simo->vrego_c = value;
}

void MXC_SIMO_RevA_SetVregO_D(mxc_simo_reva_regs_t *simo, uint32_t voltage)
{
    uint32_t base_voltage = 0;
    if (simo->vrego_d & MXC_F_SIMO_REVA_VREGO_D_RANGED) {
        base_voltage = VREGO_HIGH_RANGE_BASE;
    } else {
        base_voltage = VREGO_LOW_RANGE_BASE;
    }

    uint32_t setpoint = (voltage - base_voltage) / 10;
    uint32_t value = (simo->vrego_d & ~MXC_F_SIMO_REVA_VREGO_D_VSETD) |
                     (setpoint & MXC_F_SIMO_REVA_VREGO_D_VSETD);

    // Write the SIMO Registers twice due to clock glitch
    simo->vrego_d = value;
    simo->vrego_d = value;
}

uint32_t MXC_SIMO_RevA_GetOutReadyA(mxc_simo_reva_regs_t *simo)
{
    return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYA) ? E_NO_ERROR :
                                                                                 E_BAD_STATE;
}

uint32_t MXC_SIMO_RevA_GetOutReadyB(mxc_simo_reva_regs_t *simo)
{
    return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYB) ? E_NO_ERROR :
                                                                                 E_BAD_STATE;
}

uint32_t MXC_SIMO_RevA_GetOutReadyC(mxc_simo_reva_regs_t *simo)
{
    return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYC) ? E_NO_ERROR :
                                                                                 E_BAD_STATE;
}

uint32_t MXC_SIMO_RevA_GetOutReadyD(mxc_simo_reva_regs_t *simo)
{
    return (simo->buck_out_ready & MXC_F_SIMO_REVA_BUCK_OUT_READY_BUCKOUTRDYD) ? E_NO_ERROR :
                                                                                 E_BAD_STATE;
}

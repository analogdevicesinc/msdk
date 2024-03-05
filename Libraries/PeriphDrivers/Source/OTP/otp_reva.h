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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_OTP_OTP_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_OTP_OTP_REVA_H_

/* **** Includes **** */
#include <stdint.h>
#include "otp_reva_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Function Prototypes **** */

int MXC_OTP_RevA_Init(mxc_otp_reva_regs_t *otp, mxc_otp_clkdiv_t pclkdiv);

int MXC_OTP_RevA_IsLocked(mxc_otp_reva_regs_t *otp);

void MXC_OTP_RevA_Unlock(mxc_otp_reva_regs_t *otp);

void MXC_OTP_RevA_Lock(mxc_otp_reva_regs_t *otp);

int MXC_OTP_RevA_Write(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data, uint16_t size);

int MXC_OTP_RevA_Write32(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t data);

int MXC_OTP_RevA_Read(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data, uint16_t size);

int MXC_OTP_RevA_Read32(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data);

/**@} end of group otp */

#ifdef __cplusplus
}
#endif

#endif //LIBRARIES_PERIPHDRIVERS_SOURCE_OTP_OTP_REVA_H_

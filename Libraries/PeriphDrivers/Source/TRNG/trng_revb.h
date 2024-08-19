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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_TRNG_TRNG_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_TRNG_TRNG_REVB_H_

#include "trng.h"
#include "trng_revb_regs.h"

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_TRNG_RevB_Init(void);
void MXC_TRNG_RevB_EnableInt(mxc_trng_revb_regs_t *trng);
void MXC_TRNG_RevB_DisableInt(mxc_trng_revb_regs_t *trng);
int MXC_TRNG_RevB_Shutdown(void);
void MXC_TRNG_RevB_Handler(mxc_trng_revb_regs_t *trng);
int MXC_TRNG_RevB_RandomInt(mxc_trng_revb_regs_t *trng);
int MXC_TRNG_RevB_Random(uint8_t *data, uint32_t len);
void MXC_TRNG_RevB_RandomAsync(mxc_trng_revb_regs_t *trng, uint8_t *data, uint32_t len,
                               mxc_trng_complete_t callback);
void MXC_TRNG_RevB_GenerateKey(mxc_trng_revb_regs_t *trng);
int MXC_TRNG_RevB_HealthTest(mxc_trng_revb_regs_t *trng);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_TRNG_TRNG_REVB_H_

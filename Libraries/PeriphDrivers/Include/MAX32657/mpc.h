/**
 * @file    mpc.h
 * @brief   Memory Protection Controller (MPC) Header File.
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MPC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_TRUSTED_EXECUTION_SECURE

/**** Includes ****/
#include <stdint.h>
#include "mxc_device.h"
#include "mpc_regs.h"

/**** Defines ****/

/**** Function Prototypes ****/

/**
 * @brief   Checks whether a region fits within the constraints of physical
 *          memory (Flash or SRAM).
 * @param   start_addr  Starting address of region to check.
 * @param   end_addr    Ending address of region to check.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_MPC_CheckPhyBoundaries(uint32_t start_addr, uint32_t end_addr);

/**
 * @brief   Get the MPC register instance where the address is located.
 * @param   addr    Address within memory.
 * @return  Pointer to MPC registers associated with the address.
 */
mxc_mpc_regs_t *MXC_MPC_GetInstance(uint32_t addr);

/**
 * @brief   Gets the MPC block index that the register is located in which
 *          is associated to the bit location of MPC_BLK_LUT register.
 *          This function is NOT associated with the MPC_BLK_IDX register.
 * @param   mpc     Pointer to MPC region associated with the addr (param[1]).
 * @param   addr    Address within memory.
 * @return  The block index if successful. If Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_MPC_GetBlockIdx(mxc_mpc_regs_t *mpc, uint32_t addr);

/**
 * @brief   Sets the region to Secure.
 * @param   start_addr  Starting address of region to check.
 * @param   end_addr    Ending address of region to check.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_MPC_SetSecure(uint32_t start_addr, uint32_t end_addr);

/**
 * @brief   Sets the region to Non-Secure.
 * @param   start_addr  Starting address of region to check.
 * @param   end_addr    Ending address of region to check.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_MPC_SetNonSecure(uint32_t start_addr, uint32_t end_addr);

/**
 * @brief   Locks the MPC registers. Prevents write access.
 * @param   mpc     Pointer to MPC registers to lockdown.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_MPC_Lock(mxc_mpc_regs_t *mpc);

#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MPC_H_

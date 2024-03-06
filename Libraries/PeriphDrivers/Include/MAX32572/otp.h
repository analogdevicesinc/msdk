/**
 * @file    pt.h
 * @brief   Pulse Train data types, definitions and function prototypes.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_OTP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_OTP_H_

/* **** Includes **** */

#include "otp_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup otp One-Time Programmable Memory Controller
 * @ingroup periphlibs
 * @brief This is the high level API for the OTP Controller.
 * @{
 */

/**
 * @brief      OTP Controller Clock Divider values.
 * 
 * @note       There is no divide by 1 -> defaults to divide by 16.
 *             Default reset value of CLKDIV.pclkdiv is 0 -> divide by 16. 
 */
typedef enum {
    MXC_OTP_CLK_DIV2 = MXC_V_OTP_CLKDIV_PCLKDIV_DIV2, ///< Divide input clock by 2
    MXC_OTP_CLK_DIV4 = MXC_V_OTP_CLKDIV_PCLKDIV_DIV4, ///< Divide input clock by 4
    MXC_OTP_CLK_DIV8 = MXC_V_OTP_CLKDIV_PCLKDIV_DIV8, ///< Divide input clock by 8
    MXC_OTP_CLK_DIV16 = MXC_V_OTP_CLKDIV_PCLKDIV_DIV16, ///< Divide input clock by 16
    MXC_OTP_CLK_DIV32 = MXC_V_OTP_CLKDIV_PCLKDIV_DIV32, ///< Divide input clock by 32
} mxc_otp_clkdiv_t;

/**
 * @brief      Enumeration type for the OTP R/W Operations.
 */
typedef enum { MXC_OTP_READ_OP, MXC_OTP_WRITE_OP } mxc_otp_op_t;

/* **** Function Prototypes **** */

/**
 * @brief      Initializes the OTP Controller.
 * 
 * @param      pclkdiv   Desired peripheral clock divider.
 * 
 * @return     #E_NO_ERROR if everything is successful, \ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_OTP_Init(mxc_otp_clkdiv_t pclkdiv);

/**
 * @brief      Checks whether the OTP user block is locked/unlocked.
 * 
 * @return     (0) for Unlocked, (1) for Locked.
 */
int MXC_OTP_IsLocked(void);

/**
 * @brief      Unlocks the OTP user blocks (1-2k). Enables user block OTP program ability.
 */
void MXC_OTP_Unlock(void);

/**
 * @brief      Locks the OTP user blocks (1-2k). Disables user block OTP program ability.
 */
void MXC_OTP_Lock(void);

/**
 * @brief      Consecutively write multiple 32-bit values starting at specified address
 *             in OTP memory.
 * @note       This function unlocks the OTP block before writing and locks once finished.
 *
 * @param      addr    Starting location to write values in OTP memory.
 * @param      data    Pointer to buffer of data values to write.
 * @param      size    Size of buffer, number of values to write.
 * 
 * @return     #E_NO_ERROR if everything is successful, \ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_OTP_Write(uint16_t addr, uint32_t *data, uint16_t size);

/**
 * @brief      Write data at specified address in OTP memory.
 * @note       The OTP block where address is located should be unlocked before running
 *             this write function. This function doesn't lock the block after a write.
 *
 * @param      addr    Location to write data in OTP memory.
 * @param      data    32-bit Data value to write in memory.
 *
 * @return     #E_NO_ERROR if everything is successful, \ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_OTP_Write32(uint16_t addr, uint32_t data);

/**
 * @brief      Consecutively read multiple 32-bit values starting at specified address
 *             in OTP memory.
 * @note       This function unlocks the OTP block before reading and locks once finished.
 *
 * @param      addr    Starting location to read values in OTP memory.
 * @param      data    Pointer to buffer to store read values.
 * @param      size    Size of buffer, number of values to read.
 * 
 * @return     #E_NO_ERROR if everything is successful, \ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_OTP_Read(uint16_t addr, uint32_t *data, uint16_t size);

/**
 * @brief      Read data at specified address in OTP memory.
 * @note       The OTP block where address is located should be unlocked before running
 *             this read function. This function doesn't lock the block after reading.
 *             User block (1k-2k) is readable in normal mode.
 *
 * @param      addr    Location to read data in OTP memory.
 * @param      data    Pointer to store 32-bit value from OTP block.
 *
 * @return     #E_NO_ERROR if everything is successful, \ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_OTP_Read32(uint16_t addr, uint32_t *data);

/**@} end of group otp*/

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_OTP_H_

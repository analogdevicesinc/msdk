/**
 * @file    pt.h
 * @brief   Pulse Train data types, definitions and function prototypes.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

/* Define to prevent redundant inclusion */
#ifndef _MXC_OTP_H_
#define _MXC_OTP_H_

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

#endif /* _MXC_OTP_H_ */

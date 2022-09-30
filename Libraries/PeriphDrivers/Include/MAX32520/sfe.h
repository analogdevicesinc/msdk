/**
 * @file    sfe.h
 * @brief   Serial Flash Emulator function prototypes and data types.
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
 *************************************************************************** */

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_SFE_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_SFE_H_

/* **** Includes **** */
#include "sfe_regs.h"
#include "spi_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup sfe Serial Flash Emulator (SFE)
 * @ingroup periphlibs
 * @{
 */

/* **** Function Prototypes **** */

/**
 * @brief   Initialize serial flash emulator
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_Init(void);

/**
 * @brief   Shutdown serial flash emulator
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_Shutdown(void);

/**
 * @brief   Enable Flash Read
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_ReadEnable(void);

/**
 * @brief   Enable Flash Write 
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_WriteEnable(void);

/**
 * @brief   Set Flash Base and Top address
 * 
 * @param   baseAdd     Base address of Flash region
 * @param   topAdd      Top address of Flash region
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_SetFlashAddress(uint32_t baseAdd, uint32_t topAdd);

/**
 * @brief   Set RAM Base and Top address
 * 
 * @param   baseAdd     Base address of RAM region
 * @param   topAdd      Top address of RAM region
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_SetRAMAddress(uint32_t baseAdd, uint32_t topAdd);

/**
 * @brief   Set RAM and Flash base address for Host
 * 
 * @param   RAMAdd      Base address of RAM for Host device
 * @param   FLASHAdd    Base address of Flash for Host device
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes 
 */
int MXC_SFE_SetHostAddress(uint32_t RAMAdd, uint32_t FLASHAdd);

//-------------------------------------------------------------------------------------------
/**@} end of group sfe */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_SFE_H_

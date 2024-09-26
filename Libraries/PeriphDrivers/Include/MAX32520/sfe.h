/**
 * @file    sfe.h
 * @brief   Serial Flash Emulator function prototypes and data types.
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

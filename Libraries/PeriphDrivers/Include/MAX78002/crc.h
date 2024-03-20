/**
 * @file    crc.h
 * @brief   cyclic redundancy check driver.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_CRC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_CRC_H_

/***** Includes *****/
#include "crc_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup crc CRC
 * @ingroup periphlibs
 * @{
 */

/***** CRC Definitions *****/
/**
  * @brief  Structure used to set up CRC request
  *
  */
typedef struct _mxc_crc_req_t {
    uint32_t *dataBuffer; ///< Pointer to the data
    uint32_t dataLen; ///< Length of the data
    uint32_t resultCRC; ///< Calculated CRC value
} mxc_crc_req_t;

/** 
 * @brief CRC data bit order
 *  
 */
typedef enum { CRC_LSB_FIRST, CRC_MSB_FIRST } mxc_crc_bitorder_t;

/***** Function Prototypes *****/

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

/**
 * @brief   Enable portions of the CRC
 *
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CRC_Init(void);

/**
 * @brief   Disable and reset portions of the CRC
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CRC_Shutdown(void);

/**
 * @brief   This function should be called from the CRC ISR Handler
 *          when using Async functions
 * @param   ch      DMA channel
 * @param   error   error
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
// AI87-TODO: Changed because of redundancy in MXC_CRC_RevA_Handler?
void MXC_CRC_Handler(int ch, int error);

/**
 * @brief   Set the bit-order of CRC calculation
 *
 * @param   bitOrder  The direction to perform CRC calculation in
 */
void MXC_CRC_SetDirection(mxc_crc_bitorder_t bitOrder);

/**
 * @brief   Set the bit-order of CRC calculation
 *
 * @return  The direction of calculation, 1 for MSB first, 0 for LSB first
 */
mxc_crc_bitorder_t MXC_CRC_GetDirection(void);

/**
 * @brief   Byte Swap CRC Data Input
 *
 * @param   bitOrder  The direction to perform CRC calculation in
 */
void MXC_CRC_SwapDataIn(mxc_crc_bitorder_t bitOrder);

/**
 * @brief   Byte Swap CRC Data output
 *
 * @param   bitOrder  The direction to perform CRC calculation in
 */
void MXC_CRC_SwapDataOut(mxc_crc_bitorder_t bitOrder);

/**
 * @brief   Set the Polynomial for CRC calculation
 *
 * @param   poly  The polynomial to use for CRC calculation
 */
void MXC_CRC_SetPoly(uint32_t poly);

/**
 * @brief   Get the polynomial for CRC calculation
 *
 * @return  The polynomial used in calculation
 */
uint32_t MXC_CRC_GetPoly(void);

/**
 * @brief   Get the result of a CRC calculation
 *
 * @return  The calculated CRC value
 */
uint32_t MXC_CRC_GetResult(void);

/*******************************/
/* High Level Functions        */
/*******************************/

/**
 * @brief   Perform a CRC computation
 * @note    The result of the CRC calculation will be placed in the
 *          mxc_crc_req_t structure
 *
 * @param   req   Structure containing the data for calculation
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CRC_Compute(mxc_crc_req_t *req);

/**
 * @brief   Perform a CRC computation using DMA
 * @note    The result of the CRC calculation will be placed in the
 *          mxc_crc_req_t structure. The user must call
 *          MXC_DMA_Handler() in the ISR
 *
 * @param   req   Structure containing the data for calculation
 * 
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CRC_ComputeAsync(mxc_crc_req_t *req);

#ifdef __cplusplus
}
#endif
/**@} end of group crc */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_CRC_H_

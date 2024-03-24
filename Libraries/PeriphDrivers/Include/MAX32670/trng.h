/**
 * @file    trng.h
 * @brief   Random number generator driver.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_TRNG_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_TRNG_H_

/***** Includes *****/
#include "trng_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup trng TRNG
 * @ingroup periphlibs
 * @{
 */

/***** Function Prototypes *****/
typedef void (*mxc_trng_complete_t)(void *req, int result);

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

/**
 * @brief   Enable portions of the TRNG
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TRNG_Init(void);

/**
 * @brief   Enable TRNG Interrupts
 *
 */
void MXC_TRNG_EnableInt(void);

/**
 * @brief   Disable TRNG Interrupts
 *
 */
void MXC_TRNG_DisableInt(void);

/**
 * @brief   Disable and reset portions of the TRNG
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TRNG_Shutdown(void);

/**
 * @brief   This function should be called from the TRNG ISR Handler
 *          when using Async functions
 */
void MXC_TRNG_Handler(void);

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

/**
 * @brief   Get a random number
 *
 * @return  A random 32-bit number
 */
int MXC_TRNG_RandomInt(void);

/**
 * @brief   Get a random number of length len
 *
 * @param   data    Pointer to a location to store the number
 * @param   len     Length of random number in bytes
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TRNG_Random(uint8_t *data, uint32_t len);

/**
 * @brief   Get a random number of length len, do not block while generating data
 * @note    The user must call MXC_TRNG_Handler() in the ISR
 *
 * @param   data      Pointer to a location to store the number
 * @param   len       Length of random number in bytes
 * @param   callback  Function that will be called when all data has been generated
 *
 */
void MXC_TRNG_RandomAsync(uint8_t *data, uint32_t len, mxc_trng_complete_t callback);

/**
 * @brief   Generate an AES key and transfer to the AES block
 */
void MXC_TRNG_GenerateKey(void);

/**
 * @brief   Perform health test of the TRNG entropy source
 * 
 * @return  If test fails the function will return E_BAD_STATE (-7), otherwise it will return E_NO_ERROR.
 * 
 * @warning MAX32670 with Rev. A Silicon does not support health tests. (Check MXC_GCR->revision to see which revision your chip is.)
 */
int MXC_TRNG_HealthTest(void);

#ifdef __cplusplus
}
#endif
/**@} end of group trng */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_TRNG_H_

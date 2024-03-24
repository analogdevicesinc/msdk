/**
 * @file    sema_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SEMA Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sema_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SEMA_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SEMA_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     sema
 * @defgroup    sema_registers SEMA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SEMA Peripheral Module.
 * @details     The Semaphore peripheral allows multiple cores in a system to cooperate when accessing shred resources.
                                     The peripheral contains eight semaphores that can be atomically set and cleared. It is left to the discretion of the software
                                     architect to decide how and when the semaphores are used and how they are allocated. Existing hardware does not have to be
                                     
                                     modified for this type of cooperative sharing, and the use of semaphores is exclusively within the software domain.
 */

/**
 * @ingroup sema_registers
 * Structure type to access the SEMA Registers.
 */
typedef struct {
    __IO uint32_t semaphores[8];        /**< <tt>\b 0x000:</tt> SEMA SEMAPHORES Register */
    __R  uint32_t rsv_0x20_0xff[56];
    __IO uint32_t status;               /**< <tt>\b 0x100:</tt> SEMA STATUS Register */
} mxc_sema_regs_t;

/* Register offsets for module SEMA */
/**
 * @ingroup    sema_registers
 * @defgroup   SEMA_Register_Offsets Register Offsets
 * @brief      SEMA Peripheral Register Offsets from the SEMA Base Peripheral Address.
 * @{
 */
#define MXC_R_SEMA_SEMAPHORES              ((uint32_t)0x00000000UL) /**< Offset from SEMA Base Address: <tt> 0x0000</tt> */
#define MXC_R_SEMA_STATUS                  ((uint32_t)0x00000100UL) /**< Offset from SEMA Base Address: <tt> 0x0100</tt> */
/**@} end of group sema_registers */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_SEMAPHORES SEMA_SEMAPHORES
 * @brief    Read to test and set, returns prior value. Write 0 to clear semaphore.
 * @{
 */
#define MXC_F_SEMA_SEMAPHORES_SEMA_POS                 0 /**< SEMAPHORES_SEMA Position */
#define MXC_F_SEMA_SEMAPHORES_SEMA                     ((uint32_t)(0x1UL << MXC_F_SEMA_SEMAPHORES_SEMA_POS)) /**< SEMAPHORES_SEMA Mask */

/**@} end of group SEMA_SEMAPHORES_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_STATUS SEMA_STATUS
 * @brief    Semaphore status bits. 0 indicates the semaphore is free, 1 indicates taken.
 * @{
 */
#define MXC_F_SEMA_STATUS_STATUS_POS                   0 /**< STATUS_STATUS Position */
#define MXC_F_SEMA_STATUS_STATUS                       ((uint32_t)(0xFFUL << MXC_F_SEMA_STATUS_STATUS_POS)) /**< STATUS_STATUS Mask */

/**@} end of group SEMA_STATUS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SEMA_REGS_H_

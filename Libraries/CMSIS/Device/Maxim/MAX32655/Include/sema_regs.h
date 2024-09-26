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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_SEMA_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_SEMA_REGS_H_

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
    __IO uint32_t semaphores[8];        /**< <tt>\b 0x00:</tt> SEMA SEMAPHORES Register */
    __R  uint32_t rsv_0x20_0x3f[8];
    __IO uint32_t irq0;                 /**< <tt>\b 0x40:</tt> SEMA IRQ0 Register */
    __IO uint32_t mail0;                /**< <tt>\b 0x44:</tt> SEMA MAIL0 Register */
    __IO uint32_t irq1;                 /**< <tt>\b 0x48:</tt> SEMA IRQ1 Register */
    __IO uint32_t mail1;                /**< <tt>\b 0x4C:</tt> SEMA MAIL1 Register */
    __R  uint32_t rsv_0x50_0xff[44];
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
#define MXC_R_SEMA_IRQ0                    ((uint32_t)0x00000040UL) /**< Offset from SEMA Base Address: <tt> 0x0040</tt> */
#define MXC_R_SEMA_MAIL0                   ((uint32_t)0x00000044UL) /**< Offset from SEMA Base Address: <tt> 0x0044</tt> */
#define MXC_R_SEMA_IRQ1                    ((uint32_t)0x00000048UL) /**< Offset from SEMA Base Address: <tt> 0x0048</tt> */
#define MXC_R_SEMA_MAIL1                   ((uint32_t)0x0000004CUL) /**< Offset from SEMA Base Address: <tt> 0x004C</tt> */
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
 * @defgroup SEMA_IRQ0 SEMA_IRQ0
 * @brief    Semaphore IRQ0 register.
 * @{
 */
#define MXC_F_SEMA_IRQ0_EN_POS                         0 /**< IRQ0_EN Position */
#define MXC_F_SEMA_IRQ0_EN                             ((uint32_t)(0x1UL << MXC_F_SEMA_IRQ0_EN_POS)) /**< IRQ0_EN Mask */

#define MXC_F_SEMA_IRQ0_CM4_IRQ_POS                    16 /**< IRQ0_CM4_IRQ Position */
#define MXC_F_SEMA_IRQ0_CM4_IRQ                        ((uint32_t)(0x1UL << MXC_F_SEMA_IRQ0_CM4_IRQ_POS)) /**< IRQ0_CM4_IRQ Mask */

/**@} end of group SEMA_IRQ0_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_MAIL0 SEMA_MAIL0
 * @brief    Semaphore Mailbox 0 register.
 * @{
 */
#define MXC_F_SEMA_MAIL0_DATA_POS                      0 /**< MAIL0_DATA Position */
#define MXC_F_SEMA_MAIL0_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SEMA_MAIL0_DATA_POS)) /**< MAIL0_DATA Mask */

/**@} end of group SEMA_MAIL0_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_IRQ1 SEMA_IRQ1
 * @brief    Semaphore IRQ1 register.
 * @{
 */
#define MXC_F_SEMA_IRQ1_EN_POS                         0 /**< IRQ1_EN Position */
#define MXC_F_SEMA_IRQ1_EN                             ((uint32_t)(0x1UL << MXC_F_SEMA_IRQ1_EN_POS)) /**< IRQ1_EN Mask */

#define MXC_F_SEMA_IRQ1_RV32_IRQ_POS                   16 /**< IRQ1_RV32_IRQ Position */
#define MXC_F_SEMA_IRQ1_RV32_IRQ                       ((uint32_t)(0x1UL << MXC_F_SEMA_IRQ1_RV32_IRQ_POS)) /**< IRQ1_RV32_IRQ Mask */

/**@} end of group SEMA_IRQ1_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_MAIL1 SEMA_MAIL1
 * @brief    Semaphore Mailbox 1 register.
 * @{
 */
#define MXC_F_SEMA_MAIL1_DATA_POS                      0 /**< MAIL1_DATA Position */
#define MXC_F_SEMA_MAIL1_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SEMA_MAIL1_DATA_POS)) /**< MAIL1_DATA Mask */

/**@} end of group SEMA_MAIL1_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_STATUS SEMA_STATUS
 * @brief    Semaphore status bits. 0 indicates the semaphore is free, 1 indicates taken.
 * @{
 */
#define MXC_F_SEMA_STATUS_STATUS0_POS                  0 /**< STATUS_STATUS0 Position */
#define MXC_F_SEMA_STATUS_STATUS0                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS0_POS)) /**< STATUS_STATUS0 Mask */

#define MXC_F_SEMA_STATUS_STATUS1_POS                  1 /**< STATUS_STATUS1 Position */
#define MXC_F_SEMA_STATUS_STATUS1                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS1_POS)) /**< STATUS_STATUS1 Mask */

#define MXC_F_SEMA_STATUS_STATUS2_POS                  2 /**< STATUS_STATUS2 Position */
#define MXC_F_SEMA_STATUS_STATUS2                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS2_POS)) /**< STATUS_STATUS2 Mask */

#define MXC_F_SEMA_STATUS_STATUS3_POS                  3 /**< STATUS_STATUS3 Position */
#define MXC_F_SEMA_STATUS_STATUS3                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS3_POS)) /**< STATUS_STATUS3 Mask */

#define MXC_F_SEMA_STATUS_STATUS4_POS                  4 /**< STATUS_STATUS4 Position */
#define MXC_F_SEMA_STATUS_STATUS4                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS4_POS)) /**< STATUS_STATUS4 Mask */

#define MXC_F_SEMA_STATUS_STATUS5_POS                  5 /**< STATUS_STATUS5 Position */
#define MXC_F_SEMA_STATUS_STATUS5                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS5_POS)) /**< STATUS_STATUS5 Mask */

#define MXC_F_SEMA_STATUS_STATUS6_POS                  6 /**< STATUS_STATUS6 Position */
#define MXC_F_SEMA_STATUS_STATUS6                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS6_POS)) /**< STATUS_STATUS6 Mask */

#define MXC_F_SEMA_STATUS_STATUS7_POS                  7 /**< STATUS_STATUS7 Position */
#define MXC_F_SEMA_STATUS_STATUS7                      ((uint32_t)(0x1UL << MXC_F_SEMA_STATUS_STATUS7_POS)) /**< STATUS_STATUS7 Mask */

/**@} end of group SEMA_STATUS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_SEMA_REGS_H_

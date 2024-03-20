/**
 * @file    nbbfc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the NBBFC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup nbbfc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_NBBFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_NBBFC_REGS_H_

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
 * @ingroup     nbbfc
 * @defgroup    nbbfc_registers NBBFC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the NBBFC Peripheral Module.
 * @details     Non Battery-Backed Function Control Register.
 */

/**
 * @ingroup nbbfc_registers
 * Structure type to access the NBBFC Registers.
 */
typedef struct {
    __IO uint32_t reg0;                 /**< <tt>\b 0x00:</tt> NBBFC REG0 Register */
    __IO uint32_t reg1;                 /**< <tt>\b 0x04:</tt> NBBFC REG1 Register */
    __IO uint32_t reg2;                 /**< <tt>\b 0x08:</tt> NBBFC REG2 Register */
    __IO uint32_t reg3;                 /**< <tt>\b 0x0C:</tt> NBBFC REG3 Register */
} mxc_nbbfc_regs_t;

/* Register offsets for module NBBFC */
/**
 * @ingroup    nbbfc_registers
 * @defgroup   NBBFC_Register_Offsets Register Offsets
 * @brief      NBBFC Peripheral Register Offsets from the NBBFC Base Peripheral Address.
 * @{
 */
#define MXC_R_NBBFC_REG0                   ((uint32_t)0x00000000UL) /**< Offset from NBBFC Base Address: <tt> 0x0000</tt> */
#define MXC_R_NBBFC_REG1                   ((uint32_t)0x00000004UL) /**< Offset from NBBFC Base Address: <tt> 0x0004</tt> */
#define MXC_R_NBBFC_REG2                   ((uint32_t)0x00000008UL) /**< Offset from NBBFC Base Address: <tt> 0x0008</tt> */
#define MXC_R_NBBFC_REG3                   ((uint32_t)0x0000000CUL) /**< Offset from NBBFC Base Address: <tt> 0x000C</tt> */
/**@} end of group nbbfc_registers */

/**
 * @ingroup  nbbfc_registers
 * @defgroup NBBFC_REG0 NBBFC_REG0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_NBBFC_REG0_RDSGCSEL_POS                  0 /**< REG0_RDSGCSEL Position */
#define MXC_F_NBBFC_REG0_RDSGCSEL                      ((uint32_t)(0x3FUL << MXC_F_NBBFC_REG0_RDSGCSEL_POS)) /**< REG0_RDSGCSEL Mask */

#define MXC_F_NBBFC_REG0_RDSGCSET_POS                  6 /**< REG0_RDSGCSET Position */
#define MXC_F_NBBFC_REG0_RDSGCSET                      ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_RDSGCSET_POS)) /**< REG0_RDSGCSET Mask */
#define MXC_V_NBBFC_REG0_RDSGCSET_INTERNAL             ((uint32_t)0x0UL) /**< REG0_RDSGCSET_INTERNAL Value */
#define MXC_S_NBBFC_REG0_RDSGCSET_INTERNAL             (MXC_V_NBBFC_REG0_RDSGCSET_INTERNAL << MXC_F_NBBFC_REG0_RDSGCSET_POS) /**< REG0_RDSGCSET_INTERNAL Setting */
#define MXC_V_NBBFC_REG0_RDSGCSET_GRAY_CODE            ((uint32_t)0x1UL) /**< REG0_RDSGCSET_GRAY_CODE Value */
#define MXC_S_NBBFC_REG0_RDSGCSET_GRAY_CODE            (MXC_V_NBBFC_REG0_RDSGCSET_GRAY_CODE << MXC_F_NBBFC_REG0_RDSGCSET_POS) /**< REG0_RDSGCSET_GRAY_CODE Setting */

#define MXC_F_NBBFC_REG0_HYPCGDLY_POS                  8 /**< REG0_HYPCGDLY Position */
#define MXC_F_NBBFC_REG0_HYPCGDLY                      ((uint32_t)(0x3FUL << MXC_F_NBBFC_REG0_HYPCGDLY_POS)) /**< REG0_HYPCGDLY Mask */

#define MXC_F_NBBFC_REG0_USBRCKSEL_POS                 16 /**< REG0_USBRCKSEL Position */
#define MXC_F_NBBFC_REG0_USBRCKSEL                     ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_USBRCKSEL_POS)) /**< REG0_USBRCKSEL Mask */
#define MXC_V_NBBFC_REG0_USBRCKSEL_SYS                 ((uint32_t)0x0UL) /**< REG0_USBRCKSEL_SYS Value */
#define MXC_S_NBBFC_REG0_USBRCKSEL_SYS                 (MXC_V_NBBFC_REG0_USBRCKSEL_SYS << MXC_F_NBBFC_REG0_USBRCKSEL_POS) /**< REG0_USBRCKSEL_SYS Setting */
#define MXC_V_NBBFC_REG0_USBRCKSEL_DIG                 ((uint32_t)0x1UL) /**< REG0_USBRCKSEL_DIG Value */
#define MXC_S_NBBFC_REG0_USBRCKSEL_DIG                 (MXC_V_NBBFC_REG0_USBRCKSEL_DIG << MXC_F_NBBFC_REG0_USBRCKSEL_POS) /**< REG0_USBRCKSEL_DIG Setting */

#define MXC_F_NBBFC_REG0_QSPI0SEL_POS                  17 /**< REG0_QSPI0SEL Position */
#define MXC_F_NBBFC_REG0_QSPI0SEL                      ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_QSPI0SEL_POS)) /**< REG0_QSPI0SEL Mask */
#define MXC_V_NBBFC_REG0_QSPI0SEL_MED                  ((uint32_t)0x0UL) /**< REG0_QSPI0SEL_MED Value */
#define MXC_S_NBBFC_REG0_QSPI0SEL_MED                  (MXC_V_NBBFC_REG0_QSPI0SEL_MED << MXC_F_NBBFC_REG0_QSPI0SEL_POS) /**< REG0_QSPI0SEL_MED Setting */
#define MXC_V_NBBFC_REG0_QSPI0SEL_QSPI0                ((uint32_t)0x1UL) /**< REG0_QSPI0SEL_QSPI0 Value */
#define MXC_S_NBBFC_REG0_QSPI0SEL_QSPI0                (MXC_V_NBBFC_REG0_QSPI0SEL_QSPI0 << MXC_F_NBBFC_REG0_QSPI0SEL_POS) /**< REG0_QSPI0SEL_QSPI0 Setting */

#define MXC_F_NBBFC_REG0_I2C0DGEN0_POS                 20 /**< REG0_I2C0DGEN0 Position */
#define MXC_F_NBBFC_REG0_I2C0DGEN0                     ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_I2C0DGEN0_POS)) /**< REG0_I2C0DGEN0 Mask */
#define MXC_V_NBBFC_REG0_I2C0DGEN0_DIS                 ((uint32_t)0x0UL) /**< REG0_I2C0DGEN0_DIS Value */
#define MXC_S_NBBFC_REG0_I2C0DGEN0_DIS                 (MXC_V_NBBFC_REG0_I2C0DGEN0_DIS << MXC_F_NBBFC_REG0_I2C0DGEN0_POS) /**< REG0_I2C0DGEN0_DIS Setting */
#define MXC_V_NBBFC_REG0_I2C0DGEN0_EN                  ((uint32_t)0x1UL) /**< REG0_I2C0DGEN0_EN Value */
#define MXC_S_NBBFC_REG0_I2C0DGEN0_EN                  (MXC_V_NBBFC_REG0_I2C0DGEN0_EN << MXC_F_NBBFC_REG0_I2C0DGEN0_POS) /**< REG0_I2C0DGEN0_EN Setting */

#define MXC_F_NBBFC_REG0_I2C0DGEN1_POS                 21 /**< REG0_I2C0DGEN1 Position */
#define MXC_F_NBBFC_REG0_I2C0DGEN1                     ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_I2C0DGEN1_POS)) /**< REG0_I2C0DGEN1 Mask */
#define MXC_V_NBBFC_REG0_I2C0DGEN1_DIS                 ((uint32_t)0x0UL) /**< REG0_I2C0DGEN1_DIS Value */
#define MXC_S_NBBFC_REG0_I2C0DGEN1_DIS                 (MXC_V_NBBFC_REG0_I2C0DGEN1_DIS << MXC_F_NBBFC_REG0_I2C0DGEN1_POS) /**< REG0_I2C0DGEN1_DIS Setting */
#define MXC_V_NBBFC_REG0_I2C0DGEN1_EN                  ((uint32_t)0x1UL) /**< REG0_I2C0DGEN1_EN Value */
#define MXC_S_NBBFC_REG0_I2C0DGEN1_EN                  (MXC_V_NBBFC_REG0_I2C0DGEN1_EN << MXC_F_NBBFC_REG0_I2C0DGEN1_POS) /**< REG0_I2C0DGEN1_EN Setting */

#define MXC_F_NBBFC_REG0_I2C1DGEN0_POS                 22 /**< REG0_I2C1DGEN0 Position */
#define MXC_F_NBBFC_REG0_I2C1DGEN0                     ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_I2C1DGEN0_POS)) /**< REG0_I2C1DGEN0 Mask */
#define MXC_V_NBBFC_REG0_I2C1DGEN0_DIS                 ((uint32_t)0x0UL) /**< REG0_I2C1DGEN0_DIS Value */
#define MXC_S_NBBFC_REG0_I2C1DGEN0_DIS                 (MXC_V_NBBFC_REG0_I2C1DGEN0_DIS << MXC_F_NBBFC_REG0_I2C1DGEN0_POS) /**< REG0_I2C1DGEN0_DIS Setting */
#define MXC_V_NBBFC_REG0_I2C1DGEN0_EN                  ((uint32_t)0x1UL) /**< REG0_I2C1DGEN0_EN Value */
#define MXC_S_NBBFC_REG0_I2C1DGEN0_EN                  (MXC_V_NBBFC_REG0_I2C1DGEN0_EN << MXC_F_NBBFC_REG0_I2C1DGEN0_POS) /**< REG0_I2C1DGEN0_EN Setting */

#define MXC_F_NBBFC_REG0_I2C1DGEN1_POS                 23 /**< REG0_I2C1DGEN1 Position */
#define MXC_F_NBBFC_REG0_I2C1DGEN1                     ((uint32_t)(0x1UL << MXC_F_NBBFC_REG0_I2C1DGEN1_POS)) /**< REG0_I2C1DGEN1 Mask */
#define MXC_V_NBBFC_REG0_I2C1DGEN1_DIS                 ((uint32_t)0x0UL) /**< REG0_I2C1DGEN1_DIS Value */
#define MXC_S_NBBFC_REG0_I2C1DGEN1_DIS                 (MXC_V_NBBFC_REG0_I2C1DGEN1_DIS << MXC_F_NBBFC_REG0_I2C1DGEN1_POS) /**< REG0_I2C1DGEN1_DIS Setting */
#define MXC_V_NBBFC_REG0_I2C1DGEN1_EN                  ((uint32_t)0x1UL) /**< REG0_I2C1DGEN1_EN Value */
#define MXC_S_NBBFC_REG0_I2C1DGEN1_EN                  (MXC_V_NBBFC_REG0_I2C1DGEN1_EN << MXC_F_NBBFC_REG0_I2C1DGEN1_POS) /**< REG0_I2C1DGEN1_EN Setting */

/**@} end of group NBBFC_REG0_Register */

/**
 * @ingroup  nbbfc_registers
 * @defgroup NBBFC_REG1 NBBFC_REG1
 * @brief    Register 1.
 * @{
 */
#define MXC_F_NBBFC_REG1_ACEN_POS                      0 /**< REG1_ACEN Position */
#define MXC_F_NBBFC_REG1_ACEN                          ((uint32_t)(0x1UL << MXC_F_NBBFC_REG1_ACEN_POS)) /**< REG1_ACEN Mask */
#define MXC_V_NBBFC_REG1_ACEN_DIS                      ((uint32_t)0x0UL) /**< REG1_ACEN_DIS Value */
#define MXC_S_NBBFC_REG1_ACEN_DIS                      (MXC_V_NBBFC_REG1_ACEN_DIS << MXC_F_NBBFC_REG1_ACEN_POS) /**< REG1_ACEN_DIS Setting */
#define MXC_V_NBBFC_REG1_ACEN_EN                       ((uint32_t)0x1UL) /**< REG1_ACEN_EN Value */
#define MXC_S_NBBFC_REG1_ACEN_EN                       (MXC_V_NBBFC_REG1_ACEN_EN << MXC_F_NBBFC_REG1_ACEN_POS) /**< REG1_ACEN_EN Setting */

#define MXC_F_NBBFC_REG1_ACRUN_POS                     1 /**< REG1_ACRUN Position */
#define MXC_F_NBBFC_REG1_ACRUN                         ((uint32_t)(0x1UL << MXC_F_NBBFC_REG1_ACRUN_POS)) /**< REG1_ACRUN Mask */
#define MXC_V_NBBFC_REG1_ACRUN_NOT                     ((uint32_t)0x0UL) /**< REG1_ACRUN_NOT Value */
#define MXC_S_NBBFC_REG1_ACRUN_NOT                     (MXC_V_NBBFC_REG1_ACRUN_NOT << MXC_F_NBBFC_REG1_ACRUN_POS) /**< REG1_ACRUN_NOT Setting */
#define MXC_V_NBBFC_REG1_ACRUN_RUN                     ((uint32_t)0x1UL) /**< REG1_ACRUN_RUN Value */
#define MXC_S_NBBFC_REG1_ACRUN_RUN                     (MXC_V_NBBFC_REG1_ACRUN_RUN << MXC_F_NBBFC_REG1_ACRUN_POS) /**< REG1_ACRUN_RUN Setting */

#define MXC_F_NBBFC_REG1_LDTRM_POS                     2 /**< REG1_LDTRM Position */
#define MXC_F_NBBFC_REG1_LDTRM                         ((uint32_t)(0x1UL << MXC_F_NBBFC_REG1_LDTRM_POS)) /**< REG1_LDTRM Mask */

#define MXC_F_NBBFC_REG1_GAININV_POS                   3 /**< REG1_GAININV Position */
#define MXC_F_NBBFC_REG1_GAININV                       ((uint32_t)(0x1UL << MXC_F_NBBFC_REG1_GAININV_POS)) /**< REG1_GAININV Mask */
#define MXC_V_NBBFC_REG1_GAININV_NOT                   ((uint32_t)0x0UL) /**< REG1_GAININV_NOT Value */
#define MXC_S_NBBFC_REG1_GAININV_NOT                   (MXC_V_NBBFC_REG1_GAININV_NOT << MXC_F_NBBFC_REG1_GAININV_POS) /**< REG1_GAININV_NOT Setting */
#define MXC_V_NBBFC_REG1_GAININV_RUN                   ((uint32_t)0x1UL) /**< REG1_GAININV_RUN Value */
#define MXC_S_NBBFC_REG1_GAININV_RUN                   (MXC_V_NBBFC_REG1_GAININV_RUN << MXC_F_NBBFC_REG1_GAININV_POS) /**< REG1_GAININV_RUN Setting */

#define MXC_F_NBBFC_REG1_ATOMIC_POS                    4 /**< REG1_ATOMIC Position */
#define MXC_F_NBBFC_REG1_ATOMIC                        ((uint32_t)(0x1UL << MXC_F_NBBFC_REG1_ATOMIC_POS)) /**< REG1_ATOMIC Mask */
#define MXC_V_NBBFC_REG1_ATOMIC_NOT                    ((uint32_t)0x0UL) /**< REG1_ATOMIC_NOT Value */
#define MXC_S_NBBFC_REG1_ATOMIC_NOT                    (MXC_V_NBBFC_REG1_ATOMIC_NOT << MXC_F_NBBFC_REG1_ATOMIC_POS) /**< REG1_ATOMIC_NOT Setting */
#define MXC_V_NBBFC_REG1_ATOMIC_RUN                    ((uint32_t)0x1UL) /**< REG1_ATOMIC_RUN Value */
#define MXC_S_NBBFC_REG1_ATOMIC_RUN                    (MXC_V_NBBFC_REG1_ATOMIC_RUN << MXC_F_NBBFC_REG1_ATOMIC_POS) /**< REG1_ATOMIC_RUN Setting */

#define MXC_F_NBBFC_REG1_MU_POS                        8 /**< REG1_MU Position */
#define MXC_F_NBBFC_REG1_MU                            ((uint32_t)(0xFFFUL << MXC_F_NBBFC_REG1_MU_POS)) /**< REG1_MU Mask */

/**@} end of group NBBFC_REG1_Register */

/**
 * @ingroup  nbbfc_registers
 * @defgroup NBBFC_REG2 NBBFC_REG2
 * @brief    Register 2.
 * @{
 */
#define MXC_F_NBBFC_REG2_INTTRIM_POS                   0 /**< REG2_INTTRIM Position */
#define MXC_F_NBBFC_REG2_INTTRIM                       ((uint32_t)(0x1FFUL << MXC_F_NBBFC_REG2_INTTRIM_POS)) /**< REG2_INTTRIM Mask */

#define MXC_F_NBBFC_REG2_MINTRM_POS                    10 /**< REG2_MINTRM Position */
#define MXC_F_NBBFC_REG2_MINTRM                        ((uint32_t)(0x1FFUL << MXC_F_NBBFC_REG2_MINTRM_POS)) /**< REG2_MINTRM Mask */

#define MXC_F_NBBFC_REG2_MAXTRM_POS                    20 /**< REG2_MAXTRM Position */
#define MXC_F_NBBFC_REG2_MAXTRM                        ((uint32_t)(0x1FFUL << MXC_F_NBBFC_REG2_MAXTRM_POS)) /**< REG2_MAXTRM Mask */

/**@} end of group NBBFC_REG2_Register */

/**
 * @ingroup  nbbfc_registers
 * @defgroup NBBFC_REG3 NBBFC_REG3
 * @brief    Register 3.
 * @{
 */
#define MXC_F_NBBFC_REG3_DONECNT_POS                   0 /**< REG3_DONECNT Position */
#define MXC_F_NBBFC_REG3_DONECNT                       ((uint32_t)(0xFFUL << MXC_F_NBBFC_REG3_DONECNT_POS)) /**< REG3_DONECNT Mask */

/**@} end of group NBBFC_REG3_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_NBBFC_REGS_H_

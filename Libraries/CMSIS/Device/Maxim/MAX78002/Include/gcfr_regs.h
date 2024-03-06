/**
 * @file    gcfr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the GCFR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup gcfr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_GCFR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_GCFR_REGS_H_

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
 * @ingroup     gcfr
 * @defgroup    gcfr_registers GCFR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the GCFR Peripheral Module.
 * @details     Global Control Function Register.
 */

/**
 * @ingroup gcfr_registers
 * Structure type to access the GCFR Registers.
 */
typedef struct {
    __IO uint32_t reg0;                 /**< <tt>\b 0x00:</tt> GCFR REG0 Register */
    __IO uint32_t reg1;                 /**< <tt>\b 0x04:</tt> GCFR REG1 Register */
    __IO uint32_t reg2;                 /**< <tt>\b 0x08:</tt> GCFR REG2 Register */
    __IO uint32_t reg3;                 /**< <tt>\b 0x0C:</tt> GCFR REG3 Register */
} mxc_gcfr_regs_t;

/* Register offsets for module GCFR */
/**
 * @ingroup    gcfr_registers
 * @defgroup   GCFR_Register_Offsets Register Offsets
 * @brief      GCFR Peripheral Register Offsets from the GCFR Base Peripheral Address.
 * @{
 */
#define MXC_R_GCFR_REG0                    ((uint32_t)0x00000000UL) /**< Offset from GCFR Base Address: <tt> 0x0000</tt> */
#define MXC_R_GCFR_REG1                    ((uint32_t)0x00000004UL) /**< Offset from GCFR Base Address: <tt> 0x0004</tt> */
#define MXC_R_GCFR_REG2                    ((uint32_t)0x00000008UL) /**< Offset from GCFR Base Address: <tt> 0x0008</tt> */
#define MXC_R_GCFR_REG3                    ((uint32_t)0x0000000CUL) /**< Offset from GCFR Base Address: <tt> 0x000C</tt> */
/**@} end of group gcfr_registers */

/**
 * @ingroup  gcfr_registers
 * @defgroup GCFR_REG0 GCFR_REG0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_GCFR_REG0_CNNX16_0_PWR_EN_POS            0 /**< REG0_CNNX16_0_PWR_EN Position */
#define MXC_F_GCFR_REG0_CNNX16_0_PWR_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG0_CNNX16_0_PWR_EN_POS)) /**< REG0_CNNX16_0_PWR_EN Mask */

#define MXC_F_GCFR_REG0_CNNX16_1_PWR_EN_POS            1 /**< REG0_CNNX16_1_PWR_EN Position */
#define MXC_F_GCFR_REG0_CNNX16_1_PWR_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG0_CNNX16_1_PWR_EN_POS)) /**< REG0_CNNX16_1_PWR_EN Mask */

#define MXC_F_GCFR_REG0_CNNX16_2_PWR_EN_POS            2 /**< REG0_CNNX16_2_PWR_EN Position */
#define MXC_F_GCFR_REG0_CNNX16_2_PWR_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG0_CNNX16_2_PWR_EN_POS)) /**< REG0_CNNX16_2_PWR_EN Mask */

#define MXC_F_GCFR_REG0_CNNX16_3_PWR_EN_POS            3 /**< REG0_CNNX16_3_PWR_EN Position */
#define MXC_F_GCFR_REG0_CNNX16_3_PWR_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG0_CNNX16_3_PWR_EN_POS)) /**< REG0_CNNX16_3_PWR_EN Mask */

/**@} end of group GCFR_REG0_Register */

/**
 * @ingroup  gcfr_registers
 * @defgroup GCFR_REG1 GCFR_REG1
 * @brief    Register 1.
 * @{
 */
#define MXC_F_GCFR_REG1_CNNX16_0_RAM_EN_POS            0 /**< REG1_CNNX16_0_RAM_EN Position */
#define MXC_F_GCFR_REG1_CNNX16_0_RAM_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG1_CNNX16_0_RAM_EN_POS)) /**< REG1_CNNX16_0_RAM_EN Mask */

#define MXC_F_GCFR_REG1_CNNX16_1_RAM_EN_POS            1 /**< REG1_CNNX16_1_RAM_EN Position */
#define MXC_F_GCFR_REG1_CNNX16_1_RAM_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG1_CNNX16_1_RAM_EN_POS)) /**< REG1_CNNX16_1_RAM_EN Mask */

#define MXC_F_GCFR_REG1_CNNX16_2_RAM_EN_POS            2 /**< REG1_CNNX16_2_RAM_EN Position */
#define MXC_F_GCFR_REG1_CNNX16_2_RAM_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG1_CNNX16_2_RAM_EN_POS)) /**< REG1_CNNX16_2_RAM_EN Mask */

#define MXC_F_GCFR_REG1_CNNX16_3_RAM_EN_POS            3 /**< REG1_CNNX16_3_RAM_EN Position */
#define MXC_F_GCFR_REG1_CNNX16_3_RAM_EN                ((uint32_t)(0x1UL << MXC_F_GCFR_REG1_CNNX16_3_RAM_EN_POS)) /**< REG1_CNNX16_3_RAM_EN Mask */

/**@} end of group GCFR_REG1_Register */

/**
 * @ingroup  gcfr_registers
 * @defgroup GCFR_REG2 GCFR_REG2
 * @brief    Register 2.
 * @{
 */
#define MXC_F_GCFR_REG2_CNNX16_0_ISO_POS               0 /**< REG2_CNNX16_0_ISO Position */
#define MXC_F_GCFR_REG2_CNNX16_0_ISO                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_0_ISO_POS)) /**< REG2_CNNX16_0_ISO Mask */

#define MXC_F_GCFR_REG2_CNNX16_1_ISO_POS               1 /**< REG2_CNNX16_1_ISO Position */
#define MXC_F_GCFR_REG2_CNNX16_1_ISO                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_1_ISO_POS)) /**< REG2_CNNX16_1_ISO Mask */

#define MXC_F_GCFR_REG2_CNNX16_2_ISO_POS               2 /**< REG2_CNNX16_2_ISO Position */
#define MXC_F_GCFR_REG2_CNNX16_2_ISO                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_2_ISO_POS)) /**< REG2_CNNX16_2_ISO Mask */

#define MXC_F_GCFR_REG2_CNNX16_3_ISO_POS               3 /**< REG2_CNNX16_3_ISO Position */
#define MXC_F_GCFR_REG2_CNNX16_3_ISO                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_3_ISO_POS)) /**< REG2_CNNX16_3_ISO Mask */

#define MXC_F_GCFR_REG2_CNNX16_0_DATA_RET_EN_POS       16 /**< REG2_CNNX16_0_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_0_DATA_RET_EN           ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_0_DATA_RET_EN_POS)) /**< REG2_CNNX16_0_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_1_DATA_RET_EN_POS       17 /**< REG2_CNNX16_1_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_1_DATA_RET_EN           ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_1_DATA_RET_EN_POS)) /**< REG2_CNNX16_1_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_2_DATA_RET_EN_POS       18 /**< REG2_CNNX16_2_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_2_DATA_RET_EN           ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_2_DATA_RET_EN_POS)) /**< REG2_CNNX16_2_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_3_DATA_RET_EN_POS       19 /**< REG2_CNNX16_3_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_3_DATA_RET_EN           ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_3_DATA_RET_EN_POS)) /**< REG2_CNNX16_3_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_0_RAM_DATA_RET_EN_POS   20 /**< REG2_CNNX16_0_RAM_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_0_RAM_DATA_RET_EN       ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_0_RAM_DATA_RET_EN_POS)) /**< REG2_CNNX16_0_RAM_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_1_RAM_DATA_RET_EN_POS   21 /**< REG2_CNNX16_1_RAM_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_1_RAM_DATA_RET_EN       ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_1_RAM_DATA_RET_EN_POS)) /**< REG2_CNNX16_1_RAM_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_2_RAM_DATA_RET_EN_POS   22 /**< REG2_CNNX16_2_RAM_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_2_RAM_DATA_RET_EN       ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_2_RAM_DATA_RET_EN_POS)) /**< REG2_CNNX16_2_RAM_DATA_RET_EN Mask */

#define MXC_F_GCFR_REG2_CNNX16_3_RAM_DATA_RET_EN_POS   23 /**< REG2_CNNX16_3_RAM_DATA_RET_EN Position */
#define MXC_F_GCFR_REG2_CNNX16_3_RAM_DATA_RET_EN       ((uint32_t)(0x1UL << MXC_F_GCFR_REG2_CNNX16_3_RAM_DATA_RET_EN_POS)) /**< REG2_CNNX16_3_RAM_DATA_RET_EN Mask */

/**@} end of group GCFR_REG2_Register */

/**
 * @ingroup  gcfr_registers
 * @defgroup GCFR_REG3 GCFR_REG3
 * @brief    Register 3.
 * @{
 */
#define MXC_F_GCFR_REG3_CNNX16_0_RST_POS               0 /**< REG3_CNNX16_0_RST Position */
#define MXC_F_GCFR_REG3_CNNX16_0_RST                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG3_CNNX16_0_RST_POS)) /**< REG3_CNNX16_0_RST Mask */

#define MXC_F_GCFR_REG3_CNNX16_1_RST_POS               1 /**< REG3_CNNX16_1_RST Position */
#define MXC_F_GCFR_REG3_CNNX16_1_RST                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG3_CNNX16_1_RST_POS)) /**< REG3_CNNX16_1_RST Mask */

#define MXC_F_GCFR_REG3_CNNX16_2_RST_POS               2 /**< REG3_CNNX16_2_RST Position */
#define MXC_F_GCFR_REG3_CNNX16_2_RST                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG3_CNNX16_2_RST_POS)) /**< REG3_CNNX16_2_RST Mask */

#define MXC_F_GCFR_REG3_CNNX16_3_RST_POS               3 /**< REG3_CNNX16_3_RST Position */
#define MXC_F_GCFR_REG3_CNNX16_3_RST                   ((uint32_t)(0x1UL << MXC_F_GCFR_REG3_CNNX16_3_RST_POS)) /**< REG3_CNNX16_3_RST Mask */

/**@} end of group GCFR_REG3_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_GCFR_REGS_H_

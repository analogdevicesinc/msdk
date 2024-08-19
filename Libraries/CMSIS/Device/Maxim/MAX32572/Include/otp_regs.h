/**
 * @file    otp_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the OTP Peripheral Module.
 * @note    This file is @generated.
 * @ingroup otp_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_OTP_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_OTP_REGS_H_

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
 * @ingroup     otp
 * @defgroup    otp_registers OTP_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the OTP Peripheral Module.
 * @details     One-Time Programmable (OTP) Memory Controller.
 */

/**
 * @ingroup otp_registers
 * Structure type to access the OTP Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> OTP CTRL Register */
    __IO uint32_t clkdiv;               /**< <tt>\b 0x04:</tt> OTP CLKDIV Register */
    __IO uint32_t rdata;                /**< <tt>\b 0x08:</tt> OTP RDATA Register */
    __IO uint32_t status;               /**< <tt>\b 0x0C:</tt> OTP STATUS Register */
    __R  uint32_t rsv_0x10_0x2f[8];
    __IO uint32_t wdata;                /**< <tt>\b 0x30:</tt> OTP WDATA Register */
    __R  uint32_t rsv_0x34_0x3b[2];
    __IO uint32_t actrl0;               /**< <tt>\b 0x3C:</tt> OTP ACTRL0 Register */
    __IO uint32_t actrl1;               /**< <tt>\b 0x40:</tt> OTP ACTRL1 Register */
} mxc_otp_regs_t;

/* Register offsets for module OTP */
/**
 * @ingroup    otp_registers
 * @defgroup   OTP_Register_Offsets Register Offsets
 * @brief      OTP Peripheral Register Offsets from the OTP Base Peripheral Address.
 * @{
 */
#define MXC_R_OTP_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from OTP Base Address: <tt> 0x0000</tt> */
#define MXC_R_OTP_CLKDIV                   ((uint32_t)0x00000004UL) /**< Offset from OTP Base Address: <tt> 0x0004</tt> */
#define MXC_R_OTP_RDATA                    ((uint32_t)0x00000008UL) /**< Offset from OTP Base Address: <tt> 0x0008</tt> */
#define MXC_R_OTP_STATUS                   ((uint32_t)0x0000000CUL) /**< Offset from OTP Base Address: <tt> 0x000C</tt> */
#define MXC_R_OTP_WDATA                    ((uint32_t)0x00000030UL) /**< Offset from OTP Base Address: <tt> 0x0030</tt> */
#define MXC_R_OTP_ACTRL0                   ((uint32_t)0x0000003CUL) /**< Offset from OTP Base Address: <tt> 0x003C</tt> */
#define MXC_R_OTP_ACTRL1                   ((uint32_t)0x00000040UL) /**< Offset from OTP Base Address: <tt> 0x0040</tt> */
/**@} end of group otp_registers */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_CTRL OTP_CTRL
 * @brief    OTP Control Register.
 * @{
 */
#define MXC_F_OTP_CTRL_ADDR_POS                        0 /**< CTRL_ADDR Position */
#define MXC_F_OTP_CTRL_ADDR                            ((uint32_t)(0xFFFFUL << MXC_F_OTP_CTRL_ADDR_POS)) /**< CTRL_ADDR Mask */

#define MXC_F_OTP_CTRL_READ_POS                        24 /**< CTRL_READ Position */
#define MXC_F_OTP_CTRL_READ                            ((uint32_t)(0x1UL << MXC_F_OTP_CTRL_READ_POS)) /**< CTRL_READ Mask */

#define MXC_F_OTP_CTRL_WRITE_POS                       25 /**< CTRL_WRITE Position */
#define MXC_F_OTP_CTRL_WRITE                           ((uint32_t)(0x1UL << MXC_F_OTP_CTRL_WRITE_POS)) /**< CTRL_WRITE Mask */

/**@} end of group OTP_CTRL_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_CLKDIV OTP_CLKDIV
 * @brief    OTP Clock Divide Register.
 * @{
 */
#define MXC_F_OTP_CLKDIV_PCLKDIV_POS                   0 /**< CLKDIV_PCLKDIV Position */
#define MXC_F_OTP_CLKDIV_PCLKDIV                       ((uint32_t)(0x3FUL << MXC_F_OTP_CLKDIV_PCLKDIV_POS)) /**< CLKDIV_PCLKDIV Mask */
#define MXC_V_OTP_CLKDIV_PCLKDIV_DIV2                  ((uint32_t)0x1UL) /**< CLKDIV_PCLKDIV_DIV2 Value */
#define MXC_S_OTP_CLKDIV_PCLKDIV_DIV2                  (MXC_V_OTP_CLKDIV_PCLKDIV_DIV2 << MXC_F_OTP_CLKDIV_PCLKDIV_POS) /**< CLKDIV_PCLKDIV_DIV2 Setting */
#define MXC_V_OTP_CLKDIV_PCLKDIV_DIV4                  ((uint32_t)0x3UL) /**< CLKDIV_PCLKDIV_DIV4 Value */
#define MXC_S_OTP_CLKDIV_PCLKDIV_DIV4                  (MXC_V_OTP_CLKDIV_PCLKDIV_DIV4 << MXC_F_OTP_CLKDIV_PCLKDIV_POS) /**< CLKDIV_PCLKDIV_DIV4 Setting */
#define MXC_V_OTP_CLKDIV_PCLKDIV_DIV8                  ((uint32_t)0x7UL) /**< CLKDIV_PCLKDIV_DIV8 Value */
#define MXC_S_OTP_CLKDIV_PCLKDIV_DIV8                  (MXC_V_OTP_CLKDIV_PCLKDIV_DIV8 << MXC_F_OTP_CLKDIV_PCLKDIV_POS) /**< CLKDIV_PCLKDIV_DIV8 Setting */
#define MXC_V_OTP_CLKDIV_PCLKDIV_DIV16                 ((uint32_t)0xFUL) /**< CLKDIV_PCLKDIV_DIV16 Value */
#define MXC_S_OTP_CLKDIV_PCLKDIV_DIV16                 (MXC_V_OTP_CLKDIV_PCLKDIV_DIV16 << MXC_F_OTP_CLKDIV_PCLKDIV_POS) /**< CLKDIV_PCLKDIV_DIV16 Setting */
#define MXC_V_OTP_CLKDIV_PCLKDIV_DIV32                 ((uint32_t)0x1FUL) /**< CLKDIV_PCLKDIV_DIV32 Value */
#define MXC_S_OTP_CLKDIV_PCLKDIV_DIV32                 (MXC_V_OTP_CLKDIV_PCLKDIV_DIV32 << MXC_F_OTP_CLKDIV_PCLKDIV_POS) /**< CLKDIV_PCLKDIV_DIV32 Setting */

#define MXC_F_OTP_CLKDIV_SPWE_POS                      8 /**< CLKDIV_SPWE Position */
#define MXC_F_OTP_CLKDIV_SPWE                          ((uint32_t)(0x1UL << MXC_F_OTP_CLKDIV_SPWE_POS)) /**< CLKDIV_SPWE Mask */

#define MXC_F_OTP_CLKDIV_PD_POS                        9 /**< CLKDIV_PD Position */
#define MXC_F_OTP_CLKDIV_PD                            ((uint32_t)(0x1UL << MXC_F_OTP_CLKDIV_PD_POS)) /**< CLKDIV_PD Mask */

#define MXC_F_OTP_CLKDIV_HCLKDIV_POS                   16 /**< CLKDIV_HCLKDIV Position */
#define MXC_F_OTP_CLKDIV_HCLKDIV                       ((uint32_t)(0x3FUL << MXC_F_OTP_CLKDIV_HCLKDIV_POS)) /**< CLKDIV_HCLKDIV Mask */

/**@} end of group OTP_CLKDIV_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_RDATA OTP_RDATA
 * @brief    GPIO Clear Function Enable Register. Writing a 1 to one or more bits in this
 *           register clears the bits in the same positions in GPIO_EN to 0, without
 *           affecting other bits in that register.
 * @{
 */
#define MXC_F_OTP_RDATA_DATA_POS                       0 /**< RDATA_DATA Position */
#define MXC_F_OTP_RDATA_DATA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_RDATA_DATA_POS)) /**< RDATA_DATA Mask */

/**@} end of group OTP_RDATA_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_STATUS OTP_STATUS
 * @brief    OTP Status Register.
 * @{
 */
#define MXC_F_OTP_STATUS_BUSY_POS                      0 /**< STATUS_BUSY Position */
#define MXC_F_OTP_STATUS_BUSY                          ((uint32_t)(0x1UL << MXC_F_OTP_STATUS_BUSY_POS)) /**< STATUS_BUSY Mask */

#define MXC_F_OTP_STATUS_FAIL_POS                      1 /**< STATUS_FAIL Position */
#define MXC_F_OTP_STATUS_FAIL                          ((uint32_t)(0x1UL << MXC_F_OTP_STATUS_FAIL_POS)) /**< STATUS_FAIL Mask */

#define MXC_F_OTP_STATUS_UNLOCK1_POS                   8 /**< STATUS_UNLOCK1 Position */
#define MXC_F_OTP_STATUS_UNLOCK1                       ((uint32_t)(0x1UL << MXC_F_OTP_STATUS_UNLOCK1_POS)) /**< STATUS_UNLOCK1 Mask */

#define MXC_F_OTP_STATUS_UNLOCK3_POS                   9 /**< STATUS_UNLOCK3 Position */
#define MXC_F_OTP_STATUS_UNLOCK3                       ((uint32_t)(0x1UL << MXC_F_OTP_STATUS_UNLOCK3_POS)) /**< STATUS_UNLOCK3 Mask */

#define MXC_F_OTP_STATUS_PWR_RDY_POS                   16 /**< STATUS_PWR_RDY Position */
#define MXC_F_OTP_STATUS_PWR_RDY                       ((uint32_t)(0x1UL << MXC_F_OTP_STATUS_PWR_RDY_POS)) /**< STATUS_PWR_RDY Mask */

/**@} end of group OTP_STATUS_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_WDATA OTP_WDATA
 * @brief    OTP Write Data Register.
 * @{
 */
#define MXC_F_OTP_WDATA_DATA_POS                       0 /**< WDATA_DATA Position */
#define MXC_F_OTP_WDATA_DATA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_WDATA_DATA_POS)) /**< WDATA_DATA Mask */

/**@} end of group OTP_WDATA_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_ACTRL0 OTP_ACTRL0
 * @brief    Access Control for user block.
 * @{
 */
#define MXC_F_OTP_ACTRL0_ADATA_POS                     0 /**< ACTRL0_ADATA Position */
#define MXC_F_OTP_ACTRL0_ADATA                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_ACTRL0_ADATA_POS)) /**< ACTRL0_ADATA Mask */

/**@} end of group OTP_ACTRL0_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_ACTRL1 OTP_ACTRL1
 * @brief    Access Control for sys and user block.
 * @{
 */
#define MXC_F_OTP_ACTRL1_ADATA_POS                     0 /**< ACTRL1_ADATA Position */
#define MXC_F_OTP_ACTRL1_ADATA                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_ACTRL1_ADATA_POS)) /**< ACTRL1_ADATA Mask */

/**@} end of group OTP_ACTRL1_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_OTP_REGS_H_

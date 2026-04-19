/**
 * @file    trng_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
 * @note    This file is @generated.
 * @ingroup trng_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TRNG_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TRNG_REGS_H_

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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
 * @ingroup     trng
 * @defgroup    trng_registers TRNG_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
 * @details     Random Number Generator.
 */

/**
 * @ingroup trng_registers
 * Structure type to access the TRNG Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> TRNG CTRL Register */
    __IO uint32_t status;               /**< <tt>\b 0x04:</tt> TRNG STATUS Register */
    __I  uint32_t data;                 /**< <tt>\b 0x08:</tt> TRNG DATA Register */
    __R  uint32_t rsv_0xc_0x37[11];
    __IO uint32_t data_nist;            /**< <tt>\b 0x38:</tt> TRNG DATA_NIST Register */
} mxc_trng_regs_t;

/* Register offsets for module TRNG */
/**
 * @ingroup    trng_registers
 * @defgroup   TRNG_Register_Offsets Register Offsets
 * @brief      TRNG Peripheral Register Offsets from the TRNG Base Peripheral Address.
 * @{
 */
#define MXC_R_TRNG_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from TRNG Base Address: <tt> 0x0000</tt> */
#define MXC_R_TRNG_STATUS                  ((uint32_t)0x00000004UL) /**< Offset from TRNG Base Address: <tt> 0x0004</tt> */
#define MXC_R_TRNG_DATA                    ((uint32_t)0x00000008UL) /**< Offset from TRNG Base Address: <tt> 0x0008</tt> */
#define MXC_R_TRNG_DATA_NIST               ((uint32_t)0x00000038UL) /**< Offset from TRNG Base Address: <tt> 0x0038</tt> */
/**@} end of group trng_registers */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_CTRL TRNG_CTRL
 * @brief    TRNG Control Register.
 * @{
 */
#define MXC_F_TRNG_CTRL_OD_HEALTH_POS                  0 /**< CTRL_OD_HEALTH Position */
#define MXC_F_TRNG_CTRL_OD_HEALTH                      ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_OD_HEALTH_POS)) /**< CTRL_OD_HEALTH Mask */

#define MXC_F_TRNG_CTRL_RND_IE_POS                     1 /**< CTRL_RND_IE Position */
#define MXC_F_TRNG_CTRL_RND_IE                         ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_RND_IE_POS)) /**< CTRL_RND_IE Mask */

#define MXC_F_TRNG_CTRL_HEALTH_IE_POS                  2 /**< CTRL_HEALTH_IE Position */
#define MXC_F_TRNG_CTRL_HEALTH_IE                      ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_HEALTH_IE_POS)) /**< CTRL_HEALTH_IE Mask */

#define MXC_F_TRNG_CTRL_OD_ROMON_POS                   6 /**< CTRL_OD_ROMON Position */
#define MXC_F_TRNG_CTRL_OD_ROMON                       ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_OD_ROMON_POS)) /**< CTRL_OD_ROMON Mask */

#define MXC_F_TRNG_CTRL_OD_EE_POS                      7 /**< CTRL_OD_EE Position */
#define MXC_F_TRNG_CTRL_OD_EE                          ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_OD_EE_POS)) /**< CTRL_OD_EE Mask */

#define MXC_F_TRNG_CTRL_ROMON_EE_FOE_POS               8 /**< CTRL_ROMON_EE_FOE Position */
#define MXC_F_TRNG_CTRL_ROMON_EE_FOE                   ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_ROMON_EE_FOE_POS)) /**< CTRL_ROMON_EE_FOE Mask */

#define MXC_F_TRNG_CTRL_ROMON_EE_FOD_POS               9 /**< CTRL_ROMON_EE_FOD Position */
#define MXC_F_TRNG_CTRL_ROMON_EE_FOD                   ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_ROMON_EE_FOD_POS)) /**< CTRL_ROMON_EE_FOD Mask */

#define MXC_F_TRNG_CTRL_EBLS_POS                       10 /**< CTRL_EBLS Position */
#define MXC_F_TRNG_CTRL_EBLS                           ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_EBLS_POS)) /**< CTRL_EBLS Mask */

#define MXC_F_TRNG_CTRL_GET_TERO_CNT_POS               16 /**< CTRL_GET_TERO_CNT Position */
#define MXC_F_TRNG_CTRL_GET_TERO_CNT                   ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_GET_TERO_CNT_POS)) /**< CTRL_GET_TERO_CNT Mask */

#define MXC_F_TRNG_CTRL_EE_DONE_IE_POS                 23 /**< CTRL_EE_DONE_IE Position */
#define MXC_F_TRNG_CTRL_EE_DONE_IE                     ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_EE_DONE_IE_POS)) /**< CTRL_EE_DONE_IE Mask */

#define MXC_F_TRNG_CTRL_ROMON_DIS_POS                  24 /**< CTRL_ROMON_DIS Position */
#define MXC_F_TRNG_CTRL_ROMON_DIS                      ((uint32_t)(0x7UL << MXC_F_TRNG_CTRL_ROMON_DIS_POS)) /**< CTRL_ROMON_DIS Mask */
#define MXC_V_TRNG_CTRL_ROMON_DIS_RO_0                 ((uint32_t)0x1UL) /**< CTRL_ROMON_DIS_RO_0 Value */
#define MXC_S_TRNG_CTRL_ROMON_DIS_RO_0                 (MXC_V_TRNG_CTRL_ROMON_DIS_RO_0 << MXC_F_TRNG_CTRL_ROMON_DIS_POS) /**< CTRL_ROMON_DIS_RO_0 Setting */
#define MXC_V_TRNG_CTRL_ROMON_DIS_RO_1                 ((uint32_t)0x2UL) /**< CTRL_ROMON_DIS_RO_1 Value */
#define MXC_S_TRNG_CTRL_ROMON_DIS_RO_1                 (MXC_V_TRNG_CTRL_ROMON_DIS_RO_1 << MXC_F_TRNG_CTRL_ROMON_DIS_POS) /**< CTRL_ROMON_DIS_RO_1 Setting */
#define MXC_V_TRNG_CTRL_ROMON_DIS_RO_2                 ((uint32_t)0x4UL) /**< CTRL_ROMON_DIS_RO_2 Value */
#define MXC_S_TRNG_CTRL_ROMON_DIS_RO_2                 (MXC_V_TRNG_CTRL_ROMON_DIS_RO_2 << MXC_F_TRNG_CTRL_ROMON_DIS_POS) /**< CTRL_ROMON_DIS_RO_2 Setting */

#define MXC_F_TRNG_CTRL_ROMON_DIV2_POS                 28 /**< CTRL_ROMON_DIV2 Position */
#define MXC_F_TRNG_CTRL_ROMON_DIV2                     ((uint32_t)(0x7UL << MXC_F_TRNG_CTRL_ROMON_DIV2_POS)) /**< CTRL_ROMON_DIV2 Mask */
#define MXC_V_TRNG_CTRL_ROMON_DIV2_RO_0                ((uint32_t)0x0UL) /**< CTRL_ROMON_DIV2_RO_0 Value */
#define MXC_S_TRNG_CTRL_ROMON_DIV2_RO_0                (MXC_V_TRNG_CTRL_ROMON_DIV2_RO_0 << MXC_F_TRNG_CTRL_ROMON_DIV2_POS) /**< CTRL_ROMON_DIV2_RO_0 Setting */
#define MXC_V_TRNG_CTRL_ROMON_DIV2_RO_1                ((uint32_t)0x1UL) /**< CTRL_ROMON_DIV2_RO_1 Value */
#define MXC_S_TRNG_CTRL_ROMON_DIV2_RO_1                (MXC_V_TRNG_CTRL_ROMON_DIV2_RO_1 << MXC_F_TRNG_CTRL_ROMON_DIV2_POS) /**< CTRL_ROMON_DIV2_RO_1 Setting */
#define MXC_V_TRNG_CTRL_ROMON_DIV2_RO_2                ((uint32_t)0x2UL) /**< CTRL_ROMON_DIV2_RO_2 Value */
#define MXC_S_TRNG_CTRL_ROMON_DIV2_RO_2                (MXC_V_TRNG_CTRL_ROMON_DIV2_RO_2 << MXC_F_TRNG_CTRL_ROMON_DIV2_POS) /**< CTRL_ROMON_DIV2_RO_2 Setting */

/**@} end of group TRNG_CTRL_Register */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_STATUS TRNG_STATUS
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
#define MXC_F_TRNG_STATUS_RDY_POS                      0 /**< STATUS_RDY Position */
#define MXC_F_TRNG_STATUS_RDY                          ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_RDY_POS)) /**< STATUS_RDY Mask */

#define MXC_F_TRNG_STATUS_OD_HEALTH_POS                1 /**< STATUS_OD_HEALTH Position */
#define MXC_F_TRNG_STATUS_OD_HEALTH                    ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_OD_HEALTH_POS)) /**< STATUS_OD_HEALTH Mask */

#define MXC_F_TRNG_STATUS_HEALTH_POS                   2 /**< STATUS_HEALTH Position */
#define MXC_F_TRNG_STATUS_HEALTH                       ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_HEALTH_POS)) /**< STATUS_HEALTH Mask */

#define MXC_F_TRNG_STATUS_SRCFAIL_POS                  3 /**< STATUS_SRCFAIL Position */
#define MXC_F_TRNG_STATUS_SRCFAIL                      ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_SRCFAIL_POS)) /**< STATUS_SRCFAIL Mask */

#define MXC_F_TRNG_STATUS_OD_ROMON_POS                 6 /**< STATUS_OD_ROMON Position */
#define MXC_F_TRNG_STATUS_OD_ROMON                     ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_OD_ROMON_POS)) /**< STATUS_OD_ROMON Mask */

#define MXC_F_TRNG_STATUS_OD_EE_POS                    7 /**< STATUS_OD_EE Position */
#define MXC_F_TRNG_STATUS_OD_EE                        ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_OD_EE_POS)) /**< STATUS_OD_EE Mask */

#define MXC_F_TRNG_STATUS_PP_ERR_POS                   8 /**< STATUS_PP_ERR Position */
#define MXC_F_TRNG_STATUS_PP_ERR                       ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_PP_ERR_POS)) /**< STATUS_PP_ERR Mask */

#define MXC_F_TRNG_STATUS_ROMON_0_ERR_POS              9 /**< STATUS_ROMON_0_ERR Position */
#define MXC_F_TRNG_STATUS_ROMON_0_ERR                  ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_ROMON_0_ERR_POS)) /**< STATUS_ROMON_0_ERR Mask */

#define MXC_F_TRNG_STATUS_ROMON_1_ERR_POS              10 /**< STATUS_ROMON_1_ERR Position */
#define MXC_F_TRNG_STATUS_ROMON_1_ERR                  ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_ROMON_1_ERR_POS)) /**< STATUS_ROMON_1_ERR Mask */

#define MXC_F_TRNG_STATUS_ROMON_2_ERR_POS              11 /**< STATUS_ROMON_2_ERR Position */
#define MXC_F_TRNG_STATUS_ROMON_2_ERR                  ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_ROMON_2_ERR_POS)) /**< STATUS_ROMON_2_ERR Mask */

#define MXC_F_TRNG_STATUS_EE_ERR_THR_POS               12 /**< STATUS_EE_ERR_THR Position */
#define MXC_F_TRNG_STATUS_EE_ERR_THR                   ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_EE_ERR_THR_POS)) /**< STATUS_EE_ERR_THR Mask */

#define MXC_F_TRNG_STATUS_EE_ERR_OOB_POS               13 /**< STATUS_EE_ERR_OOB Position */
#define MXC_F_TRNG_STATUS_EE_ERR_OOB                   ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_EE_ERR_OOB_POS)) /**< STATUS_EE_ERR_OOB Mask */

#define MXC_F_TRNG_STATUS_EE_ERR_LOCK_POS              14 /**< STATUS_EE_ERR_LOCK Position */
#define MXC_F_TRNG_STATUS_EE_ERR_LOCK                  ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_EE_ERR_LOCK_POS)) /**< STATUS_EE_ERR_LOCK Mask */

#define MXC_F_TRNG_STATUS_TERO_CNT_RDY_POS             16 /**< STATUS_TERO_CNT_RDY Position */
#define MXC_F_TRNG_STATUS_TERO_CNT_RDY                 ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_TERO_CNT_RDY_POS)) /**< STATUS_TERO_CNT_RDY Mask */

#define MXC_F_TRNG_STATUS_RC_ERR_POS                   17 /**< STATUS_RC_ERR Position */
#define MXC_F_TRNG_STATUS_RC_ERR                       ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_RC_ERR_POS)) /**< STATUS_RC_ERR Mask */

#define MXC_F_TRNG_STATUS_AP_ERR_POS                   18 /**< STATUS_AP_ERR Position */
#define MXC_F_TRNG_STATUS_AP_ERR                       ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_AP_ERR_POS)) /**< STATUS_AP_ERR Mask */

#define MXC_F_TRNG_STATUS_DATA_DONE_POS                19 /**< STATUS_DATA_DONE Position */
#define MXC_F_TRNG_STATUS_DATA_DONE                    ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_DATA_DONE_POS)) /**< STATUS_DATA_DONE Mask */

#define MXC_F_TRNG_STATUS_DATA_NIST_DONE_POS           20 /**< STATUS_DATA_NIST_DONE Position */
#define MXC_F_TRNG_STATUS_DATA_NIST_DONE               ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_DATA_NIST_DONE_POS)) /**< STATUS_DATA_NIST_DONE Mask */

#define MXC_F_TRNG_STATUS_HEALTH_DONE_POS              21 /**< STATUS_HEALTH_DONE Position */
#define MXC_F_TRNG_STATUS_HEALTH_DONE                  ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_HEALTH_DONE_POS)) /**< STATUS_HEALTH_DONE Mask */

#define MXC_F_TRNG_STATUS_ROMON_DONE_POS               22 /**< STATUS_ROMON_DONE Position */
#define MXC_F_TRNG_STATUS_ROMON_DONE                   ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_ROMON_DONE_POS)) /**< STATUS_ROMON_DONE Mask */

#define MXC_F_TRNG_STATUS_EE_DONE_POS                  23 /**< STATUS_EE_DONE Position */
#define MXC_F_TRNG_STATUS_EE_DONE                      ((uint32_t)(0x1UL << MXC_F_TRNG_STATUS_EE_DONE_POS)) /**< STATUS_EE_DONE Mask */

/**@} end of group TRNG_STATUS_Register */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_DATA TRNG_DATA
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
#define MXC_F_TRNG_DATA_DATA_POS                       0 /**< DATA_DATA Position */
#define MXC_F_TRNG_DATA_DATA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_TRNG_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group TRNG_DATA_Register */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_DATA_NIST TRNG_DATA_NIST
 * @brief    Data NIST Register.
 * @{
 */
#define MXC_F_TRNG_DATA_NIST_DATA_POS                  0 /**< DATA_NIST_DATA Position */
#define MXC_F_TRNG_DATA_NIST_DATA                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_TRNG_DATA_NIST_DATA_POS)) /**< DATA_NIST_DATA Mask */

/**@} end of group TRNG_DATA_NIST_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TRNG_REGS_H_

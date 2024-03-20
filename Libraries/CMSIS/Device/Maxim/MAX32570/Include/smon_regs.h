/**
 * @file    smon_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SMON Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SMON_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SMON_REGS_H_

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
 * @ingroup     smon
 * @defgroup    smon_registers SMON_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SMON Peripheral Module.
 * @details     The Security Monitor block used to monitor system threat conditions.
 */

/**
 * @ingroup smon_registers
 * Structure type to access the SMON Registers.
 */
typedef struct {
    __IO uint32_t extscn;               /**< <tt>\b 0x00:</tt> SMON EXTSCN Register */
    __IO uint32_t intscn;               /**< <tt>\b 0x04:</tt> SMON INTSCN Register */
    __IO uint32_t secalm;               /**< <tt>\b 0x08:</tt> SMON SECALM Register */
    __IO uint32_t secdiag;              /**< <tt>\b 0x0C:</tt> SMON SECDIAG Register */
    __I  uint32_t dlrtc;                /**< <tt>\b 0x10:</tt> SMON DLRTC Register */
    __R  uint32_t rsv_0x14_0x23[4];
    __IO uint32_t meucfg;               /**< <tt>\b 0x24:</tt> SMON MEUCFG Register */
    __R  uint32_t rsv_0x28_0x33[3];
    __I  uint32_t secst;                /**< <tt>\b 0x34:</tt> SMON SECST Register */
    __IO uint32_t sdbe;                 /**< <tt>\b 0x38:</tt> SMON SDBE Register */
} mxc_smon_regs_t;

/* Register offsets for module SMON */
/**
 * @ingroup    smon_registers
 * @defgroup   SMON_Register_Offsets Register Offsets
 * @brief      SMON Peripheral Register Offsets from the SMON Base Peripheral Address.
 * @{
 */
#define MXC_R_SMON_EXTSCN                  ((uint32_t)0x00000000UL) /**< Offset from SMON Base Address: <tt> 0x0000</tt> */
#define MXC_R_SMON_INTSCN                  ((uint32_t)0x00000004UL) /**< Offset from SMON Base Address: <tt> 0x0004</tt> */
#define MXC_R_SMON_SECALM                  ((uint32_t)0x00000008UL) /**< Offset from SMON Base Address: <tt> 0x0008</tt> */
#define MXC_R_SMON_SECDIAG                 ((uint32_t)0x0000000CUL) /**< Offset from SMON Base Address: <tt> 0x000C</tt> */
#define MXC_R_SMON_DLRTC                   ((uint32_t)0x00000010UL) /**< Offset from SMON Base Address: <tt> 0x0010</tt> */
#define MXC_R_SMON_MEUCFG                  ((uint32_t)0x00000024UL) /**< Offset from SMON Base Address: <tt> 0x0024</tt> */
#define MXC_R_SMON_SECST                   ((uint32_t)0x00000034UL) /**< Offset from SMON Base Address: <tt> 0x0034</tt> */
#define MXC_R_SMON_SDBE                    ((uint32_t)0x00000038UL) /**< Offset from SMON Base Address: <tt> 0x0038</tt> */
/**@} end of group smon_registers */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_EXTSCN SMON_EXTSCN
 * @brief    External Sensor Control Register.
 * @{
 */
#define MXC_F_SMON_EXTSCN_EXTS_EN0_POS                 0 /**< EXTSCN_EXTS_EN0 Position */
#define MXC_F_SMON_EXTSCN_EXTS_EN0                     ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_EXTS_EN0_POS)) /**< EXTSCN_EXTS_EN0 Mask */

#define MXC_F_SMON_EXTSCN_EXTS_EN1_POS                 1 /**< EXTSCN_EXTS_EN1 Position */
#define MXC_F_SMON_EXTSCN_EXTS_EN1                     ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_EXTS_EN1_POS)) /**< EXTSCN_EXTS_EN1 Mask */

#define MXC_F_SMON_EXTSCN_EXTS_EN2_POS                 2 /**< EXTSCN_EXTS_EN2 Position */
#define MXC_F_SMON_EXTSCN_EXTS_EN2                     ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_EXTS_EN2_POS)) /**< EXTSCN_EXTS_EN2 Mask */

#define MXC_F_SMON_EXTSCN_EXTS_EN3_POS                 3 /**< EXTSCN_EXTS_EN3 Position */
#define MXC_F_SMON_EXTSCN_EXTS_EN3                     ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_EXTS_EN3_POS)) /**< EXTSCN_EXTS_EN3 Mask */

#define MXC_F_SMON_EXTSCN_EXTS_EN4_POS                 4 /**< EXTSCN_EXTS_EN4 Position */
#define MXC_F_SMON_EXTSCN_EXTS_EN4                     ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_EXTS_EN4_POS)) /**< EXTSCN_EXTS_EN4 Mask */

#define MXC_F_SMON_EXTSCN_EXTS_EN5_POS                 5 /**< EXTSCN_EXTS_EN5 Position */
#define MXC_F_SMON_EXTSCN_EXTS_EN5                     ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_EXTS_EN5_POS)) /**< EXTSCN_EXTS_EN5 Mask */

#define MXC_F_SMON_EXTSCN_EXTCNT_POS                   16 /**< EXTSCN_EXTCNT Position */
#define MXC_F_SMON_EXTSCN_EXTCNT                       ((uint32_t)(0x1FUL << MXC_F_SMON_EXTSCN_EXTCNT_POS)) /**< EXTSCN_EXTCNT Mask */

#define MXC_F_SMON_EXTSCN_EXTFRQ_POS                   21 /**< EXTSCN_EXTFRQ Position */
#define MXC_F_SMON_EXTSCN_EXTFRQ                       ((uint32_t)(0x7UL << MXC_F_SMON_EXTSCN_EXTFRQ_POS)) /**< EXTSCN_EXTFRQ Mask */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ2000HZ            ((uint32_t)0x0UL) /**< EXTSCN_EXTFRQ_FREQ2000HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ2000HZ            (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ2000HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ2000HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ1000HZ            ((uint32_t)0x1UL) /**< EXTSCN_EXTFRQ_FREQ1000HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ1000HZ            (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ1000HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ1000HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ500HZ             ((uint32_t)0x2UL) /**< EXTSCN_EXTFRQ_FREQ500HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ500HZ             (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ500HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ500HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ250HZ             ((uint32_t)0x3UL) /**< EXTSCN_EXTFRQ_FREQ250HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ250HZ             (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ250HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ250HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ125HZ             ((uint32_t)0x4UL) /**< EXTSCN_EXTFRQ_FREQ125HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ125HZ             (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ125HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ125HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ63HZ              ((uint32_t)0x5UL) /**< EXTSCN_EXTFRQ_FREQ63HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ63HZ              (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ63HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ63HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_FREQ31HZ              ((uint32_t)0x6UL) /**< EXTSCN_EXTFRQ_FREQ31HZ Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_FREQ31HZ              (MXC_V_SMON_EXTSCN_EXTFRQ_FREQ31HZ << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_FREQ31HZ Setting */
#define MXC_V_SMON_EXTSCN_EXTFRQ_RFU                   ((uint32_t)0x7UL) /**< EXTSCN_EXTFRQ_RFU Value */
#define MXC_S_SMON_EXTSCN_EXTFRQ_RFU                   (MXC_V_SMON_EXTSCN_EXTFRQ_RFU << MXC_F_SMON_EXTSCN_EXTFRQ_POS) /**< EXTSCN_EXTFRQ_RFU Setting */

#define MXC_F_SMON_EXTSCN_DIVCLK_POS                   24 /**< EXTSCN_DIVCLK Position */
#define MXC_F_SMON_EXTSCN_DIVCLK                       ((uint32_t)(0x7UL << MXC_F_SMON_EXTSCN_DIVCLK_POS)) /**< EXTSCN_DIVCLK Mask */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV1                  ((uint32_t)0x0UL) /**< EXTSCN_DIVCLK_DIV1 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV1                  (MXC_V_SMON_EXTSCN_DIVCLK_DIV1 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV1 Setting */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV2                  ((uint32_t)0x1UL) /**< EXTSCN_DIVCLK_DIV2 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV2                  (MXC_V_SMON_EXTSCN_DIVCLK_DIV2 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV2 Setting */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV4                  ((uint32_t)0x2UL) /**< EXTSCN_DIVCLK_DIV4 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV4                  (MXC_V_SMON_EXTSCN_DIVCLK_DIV4 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV4 Setting */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV8                  ((uint32_t)0x3UL) /**< EXTSCN_DIVCLK_DIV8 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV8                  (MXC_V_SMON_EXTSCN_DIVCLK_DIV8 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV8 Setting */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV16                 ((uint32_t)0x4UL) /**< EXTSCN_DIVCLK_DIV16 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV16                 (MXC_V_SMON_EXTSCN_DIVCLK_DIV16 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV16 Setting */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV32                 ((uint32_t)0x5UL) /**< EXTSCN_DIVCLK_DIV32 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV32                 (MXC_V_SMON_EXTSCN_DIVCLK_DIV32 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV32 Setting */
#define MXC_V_SMON_EXTSCN_DIVCLK_DIV64                 ((uint32_t)0x6UL) /**< EXTSCN_DIVCLK_DIV64 Value */
#define MXC_S_SMON_EXTSCN_DIVCLK_DIV64                 (MXC_V_SMON_EXTSCN_DIVCLK_DIV64 << MXC_F_SMON_EXTSCN_DIVCLK_POS) /**< EXTSCN_DIVCLK_DIV64 Setting */

#define MXC_F_SMON_EXTSCN_BUSY_POS                     30 /**< EXTSCN_BUSY Position */
#define MXC_F_SMON_EXTSCN_BUSY                         ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_BUSY_POS)) /**< EXTSCN_BUSY Mask */

#define MXC_F_SMON_EXTSCN_LOCK_POS                     31 /**< EXTSCN_LOCK Position */
#define MXC_F_SMON_EXTSCN_LOCK                         ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCN_LOCK_POS)) /**< EXTSCN_LOCK Mask */

/**@} end of group SMON_EXTSCN_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_INTSCN SMON_INTSCN
 * @brief    Internal Sensor Control Register.
 * @{
 */
#define MXC_F_SMON_INTSCN_SHIELD_EN_POS                0 /**< INTSCN_SHIELD_EN Position */
#define MXC_F_SMON_INTSCN_SHIELD_EN                    ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_SHIELD_EN_POS)) /**< INTSCN_SHIELD_EN Mask */

#define MXC_F_SMON_INTSCN_TEMP_EN_POS                  1 /**< INTSCN_TEMP_EN Position */
#define MXC_F_SMON_INTSCN_TEMP_EN                      ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_TEMP_EN_POS)) /**< INTSCN_TEMP_EN Mask */

#define MXC_F_SMON_INTSCN_VBAT_EN_POS                  2 /**< INTSCN_VBAT_EN Position */
#define MXC_F_SMON_INTSCN_VBAT_EN                      ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_VBAT_EN_POS)) /**< INTSCN_VBAT_EN Mask */

#define MXC_F_SMON_INTSCN_DFD_EN_POS                   3 /**< INTSCN_DFD_EN Position */
#define MXC_F_SMON_INTSCN_DFD_EN                       ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_DFD_EN_POS)) /**< INTSCN_DFD_EN Mask */

#define MXC_F_SMON_INTSCN_DFD_NMI_POS                  4 /**< INTSCN_DFD_NMI Position */
#define MXC_F_SMON_INTSCN_DFD_NMI                      ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_DFD_NMI_POS)) /**< INTSCN_DFD_NMI Mask */

#define MXC_F_SMON_INTSCN_DFD_STDBY_POS                8 /**< INTSCN_DFD_STDBY Position */
#define MXC_F_SMON_INTSCN_DFD_STDBY                    ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_DFD_STDBY_POS)) /**< INTSCN_DFD_STDBY Mask */

#define MXC_F_SMON_INTSCN_LOTEMP_SEL_POS               16 /**< INTSCN_LOTEMP_SEL Position */
#define MXC_F_SMON_INTSCN_LOTEMP_SEL                   ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_LOTEMP_SEL_POS)) /**< INTSCN_LOTEMP_SEL Mask */

#define MXC_F_SMON_INTSCN_VTM_LOTHSEL_POS              18 /**< INTSCN_VTM_LOTHSEL Position */
#define MXC_F_SMON_INTSCN_VTM_LOTHSEL                  ((uint32_t)(0x3UL << MXC_F_SMON_INTSCN_VTM_LOTHSEL_POS)) /**< INTSCN_VTM_LOTHSEL Mask */
#define MXC_V_SMON_INTSCN_VTM_LOTHSEL_1_6V             ((uint32_t)0x0UL) /**< INTSCN_VTM_LOTHSEL_1_6V Value */
#define MXC_S_SMON_INTSCN_VTM_LOTHSEL_1_6V             (MXC_V_SMON_INTSCN_VTM_LOTHSEL_1_6V << MXC_F_SMON_INTSCN_VTM_LOTHSEL_POS) /**< INTSCN_VTM_LOTHSEL_1_6V Setting */
#define MXC_V_SMON_INTSCN_VTM_LOTHSEL_2_2V             ((uint32_t)0x1UL) /**< INTSCN_VTM_LOTHSEL_2_2V Value */
#define MXC_S_SMON_INTSCN_VTM_LOTHSEL_2_2V             (MXC_V_SMON_INTSCN_VTM_LOTHSEL_2_2V << MXC_F_SMON_INTSCN_VTM_LOTHSEL_POS) /**< INTSCN_VTM_LOTHSEL_2_2V Setting */
#define MXC_V_SMON_INTSCN_VTM_LOTHSEL_2_8V             ((uint32_t)0x2UL) /**< INTSCN_VTM_LOTHSEL_2_8V Value */
#define MXC_S_SMON_INTSCN_VTM_LOTHSEL_2_8V             (MXC_V_SMON_INTSCN_VTM_LOTHSEL_2_8V << MXC_F_SMON_INTSCN_VTM_LOTHSEL_POS) /**< INTSCN_VTM_LOTHSEL_2_8V Setting */

#define MXC_F_SMON_INTSCN_LOCK_POS                     31 /**< INTSCN_LOCK Position */
#define MXC_F_SMON_INTSCN_LOCK                         ((uint32_t)(0x1UL << MXC_F_SMON_INTSCN_LOCK_POS)) /**< INTSCN_LOCK Mask */

/**@} end of group SMON_INTSCN_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SECALM SMON_SECALM
 * @brief    Security Alarm Register.
 * @{
 */
#define MXC_F_SMON_SECALM_DRS_POS                      0 /**< SECALM_DRS Position */
#define MXC_F_SMON_SECALM_DRS                          ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_DRS_POS)) /**< SECALM_DRS Mask */

#define MXC_F_SMON_SECALM_KEYWIPE_POS                  1 /**< SECALM_KEYWIPE Position */
#define MXC_F_SMON_SECALM_KEYWIPE                      ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_KEYWIPE_POS)) /**< SECALM_KEYWIPE Mask */

#define MXC_F_SMON_SECALM_SHIELDF_POS                  2 /**< SECALM_SHIELDF Position */
#define MXC_F_SMON_SECALM_SHIELDF                      ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_SHIELDF_POS)) /**< SECALM_SHIELDF Mask */

#define MXC_F_SMON_SECALM_LOTEMP_POS                   3 /**< SECALM_LOTEMP Position */
#define MXC_F_SMON_SECALM_LOTEMP                       ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_LOTEMP_POS)) /**< SECALM_LOTEMP Mask */

#define MXC_F_SMON_SECALM_HITEMP_POS                   4 /**< SECALM_HITEMP Position */
#define MXC_F_SMON_SECALM_HITEMP                       ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_HITEMP_POS)) /**< SECALM_HITEMP Mask */

#define MXC_F_SMON_SECALM_BATLO_POS                    5 /**< SECALM_BATLO Position */
#define MXC_F_SMON_SECALM_BATLO                        ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_BATLO_POS)) /**< SECALM_BATLO Mask */

#define MXC_F_SMON_SECALM_BATHI_POS                    6 /**< SECALM_BATHI Position */
#define MXC_F_SMON_SECALM_BATHI                        ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_BATHI_POS)) /**< SECALM_BATHI Mask */

#define MXC_F_SMON_SECALM_EXTF_POS                     7 /**< SECALM_EXTF Position */
#define MXC_F_SMON_SECALM_EXTF                         ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTF_POS)) /**< SECALM_EXTF Mask */

#define MXC_F_SMON_SECALM_DFD_POS                      8 /**< SECALM_DFD Position */
#define MXC_F_SMON_SECALM_DFD                          ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_DFD_POS)) /**< SECALM_DFD Mask */

#define MXC_F_SMON_SECALM_VMAINPF_POS                  9 /**< SECALM_VMAINPF Position */
#define MXC_F_SMON_SECALM_VMAINPF                      ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_VMAINPF_POS)) /**< SECALM_VMAINPF Mask */

#define MXC_F_SMON_SECALM_EXTSTAT0_POS                 16 /**< SECALM_EXTSTAT0 Position */
#define MXC_F_SMON_SECALM_EXTSTAT0                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT0_POS)) /**< SECALM_EXTSTAT0 Mask */

#define MXC_F_SMON_SECALM_EXTSTAT1_POS                 17 /**< SECALM_EXTSTAT1 Position */
#define MXC_F_SMON_SECALM_EXTSTAT1                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT1_POS)) /**< SECALM_EXTSTAT1 Mask */

#define MXC_F_SMON_SECALM_EXTSTAT2_POS                 18 /**< SECALM_EXTSTAT2 Position */
#define MXC_F_SMON_SECALM_EXTSTAT2                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT2_POS)) /**< SECALM_EXTSTAT2 Mask */

#define MXC_F_SMON_SECALM_EXTSTAT3_POS                 19 /**< SECALM_EXTSTAT3 Position */
#define MXC_F_SMON_SECALM_EXTSTAT3                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT3_POS)) /**< SECALM_EXTSTAT3 Mask */

#define MXC_F_SMON_SECALM_EXTSTAT4_POS                 20 /**< SECALM_EXTSTAT4 Position */
#define MXC_F_SMON_SECALM_EXTSTAT4                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT4_POS)) /**< SECALM_EXTSTAT4 Mask */

#define MXC_F_SMON_SECALM_EXTSTAT5_POS                 21 /**< SECALM_EXTSTAT5 Position */
#define MXC_F_SMON_SECALM_EXTSTAT5                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT5_POS)) /**< SECALM_EXTSTAT5 Mask */

#define MXC_F_SMON_SECALM_EXTSWARN0_POS                24 /**< SECALM_EXTSWARN0 Position */
#define MXC_F_SMON_SECALM_EXTSWARN0                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN0_POS)) /**< SECALM_EXTSWARN0 Mask */

#define MXC_F_SMON_SECALM_EXTSWARN1_POS                25 /**< SECALM_EXTSWARN1 Position */
#define MXC_F_SMON_SECALM_EXTSWARN1                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN1_POS)) /**< SECALM_EXTSWARN1 Mask */

#define MXC_F_SMON_SECALM_EXTSWARN2_POS                26 /**< SECALM_EXTSWARN2 Position */
#define MXC_F_SMON_SECALM_EXTSWARN2                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN2_POS)) /**< SECALM_EXTSWARN2 Mask */

#define MXC_F_SMON_SECALM_EXTSWARN3_POS                27 /**< SECALM_EXTSWARN3 Position */
#define MXC_F_SMON_SECALM_EXTSWARN3                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN3_POS)) /**< SECALM_EXTSWARN3 Mask */

#define MXC_F_SMON_SECALM_EXTSWARN4_POS                28 /**< SECALM_EXTSWARN4 Position */
#define MXC_F_SMON_SECALM_EXTSWARN4                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN4_POS)) /**< SECALM_EXTSWARN4 Mask */

#define MXC_F_SMON_SECALM_EXTSWARN5_POS                29 /**< SECALM_EXTSWARN5 Position */
#define MXC_F_SMON_SECALM_EXTSWARN5                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN5_POS)) /**< SECALM_EXTSWARN5 Mask */

/**@} end of group SMON_SECALM_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SECDIAG SMON_SECDIAG
 * @brief    Security Diagnostic Register.
 * @{
 */
#define MXC_F_SMON_SECDIAG_PORF_POS                    0 /**< SECDIAG_PORF Position */
#define MXC_F_SMON_SECDIAG_PORF                        ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_PORF_POS)) /**< SECDIAG_PORF Mask */

#define MXC_F_SMON_SECDIAG_SHIELDF_POS                 2 /**< SECDIAG_SHIELDF Position */
#define MXC_F_SMON_SECDIAG_SHIELDF                     ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_SHIELDF_POS)) /**< SECDIAG_SHIELDF Mask */

#define MXC_F_SMON_SECDIAG_LOTEMP_POS                  3 /**< SECDIAG_LOTEMP Position */
#define MXC_F_SMON_SECDIAG_LOTEMP                      ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_LOTEMP_POS)) /**< SECDIAG_LOTEMP Mask */

#define MXC_F_SMON_SECDIAG_HITEMP_POS                  4 /**< SECDIAG_HITEMP Position */
#define MXC_F_SMON_SECDIAG_HITEMP                      ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_HITEMP_POS)) /**< SECDIAG_HITEMP Mask */

#define MXC_F_SMON_SECDIAG_BATLO_POS                   5 /**< SECDIAG_BATLO Position */
#define MXC_F_SMON_SECDIAG_BATLO                       ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_BATLO_POS)) /**< SECDIAG_BATLO Mask */

#define MXC_F_SMON_SECDIAG_BATHI_POS                   6 /**< SECDIAG_BATHI Position */
#define MXC_F_SMON_SECDIAG_BATHI                       ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_BATHI_POS)) /**< SECDIAG_BATHI Mask */

#define MXC_F_SMON_SECDIAG_DYNF_POS                    7 /**< SECDIAG_DYNF Position */
#define MXC_F_SMON_SECDIAG_DYNF                        ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_DYNF_POS)) /**< SECDIAG_DYNF Mask */

#define MXC_F_SMON_SECDIAG_AESK_MDU_POS                9 /**< SECDIAG_AESK_MDU Position */
#define MXC_F_SMON_SECDIAG_AESK_MDU                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_AESK_MDU_POS)) /**< SECDIAG_AESK_MDU Mask */

#define MXC_F_SMON_SECDIAG_AESK_NVSRAM_POS             10 /**< SECDIAG_AESK_NVSRAM Position */
#define MXC_F_SMON_SECDIAG_AESK_NVSRAM                 ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_AESK_NVSRAM_POS)) /**< SECDIAG_AESK_NVSRAM Mask */

#define MXC_F_SMON_SECDIAG_AESK_SPIXF_POS              11 /**< SECDIAG_AESK_SPIXF Position */
#define MXC_F_SMON_SECDIAG_AESK_SPIXF                  ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_AESK_SPIXF_POS)) /**< SECDIAG_AESK_SPIXF Mask */

#define MXC_F_SMON_SECDIAG_AESK_SPIXR_POS              12 /**< SECDIAG_AESK_SPIXR Position */
#define MXC_F_SMON_SECDIAG_AESK_SPIXR                  ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_AESK_SPIXR_POS)) /**< SECDIAG_AESK_SPIXR Mask */

#define MXC_F_SMON_SECDIAG_EXTSTAT0_POS                16 /**< SECDIAG_EXTSTAT0 Position */
#define MXC_F_SMON_SECDIAG_EXTSTAT0                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTSTAT0_POS)) /**< SECDIAG_EXTSTAT0 Mask */

#define MXC_F_SMON_SECDIAG_EXTSTAT1_POS                17 /**< SECDIAG_EXTSTAT1 Position */
#define MXC_F_SMON_SECDIAG_EXTSTAT1                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTSTAT1_POS)) /**< SECDIAG_EXTSTAT1 Mask */

#define MXC_F_SMON_SECDIAG_EXTSTAT2_POS                18 /**< SECDIAG_EXTSTAT2 Position */
#define MXC_F_SMON_SECDIAG_EXTSTAT2                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTSTAT2_POS)) /**< SECDIAG_EXTSTAT2 Mask */

#define MXC_F_SMON_SECDIAG_EXTSTAT3_POS                19 /**< SECDIAG_EXTSTAT3 Position */
#define MXC_F_SMON_SECDIAG_EXTSTAT3                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTSTAT3_POS)) /**< SECDIAG_EXTSTAT3 Mask */

#define MXC_F_SMON_SECDIAG_EXTSTAT4_POS                20 /**< SECDIAG_EXTSTAT4 Position */
#define MXC_F_SMON_SECDIAG_EXTSTAT4                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTSTAT4_POS)) /**< SECDIAG_EXTSTAT4 Mask */

#define MXC_F_SMON_SECDIAG_EXTSTAT5_POS                21 /**< SECDIAG_EXTSTAT5 Position */
#define MXC_F_SMON_SECDIAG_EXTSTAT5                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTSTAT5_POS)) /**< SECDIAG_EXTSTAT5 Mask */

/**@} end of group SMON_SECDIAG_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_DLRTC SMON_DLRTC
 * @brief    DRS Log RTC Value. This register contains the 32 bit value in the RTC second
 *           register when the last DRS event occurred.
 * @{
 */
#define MXC_F_SMON_DLRTC_DLRTC_POS                     0 /**< DLRTC_DLRTC Position */
#define MXC_F_SMON_DLRTC_DLRTC                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_SMON_DLRTC_DLRTC_POS)) /**< DLRTC_DLRTC Mask */

/**@} end of group SMON_DLRTC_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_MEUCFG SMON_MEUCFG
 * @brief    MEU Configuration
 * @{
 */
#define MXC_F_SMON_MEUCFG_ENC_REG0_POS                 0 /**< MEUCFG_ENC_REG0 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG0                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG0_POS)) /**< MEUCFG_ENC_REG0 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG1_POS                 1 /**< MEUCFG_ENC_REG1 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG1                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG1_POS)) /**< MEUCFG_ENC_REG1 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG2_POS                 2 /**< MEUCFG_ENC_REG2 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG2                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG2_POS)) /**< MEUCFG_ENC_REG2 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG3_POS                 3 /**< MEUCFG_ENC_REG3 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG3                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG3_POS)) /**< MEUCFG_ENC_REG3 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG4_POS                 4 /**< MEUCFG_ENC_REG4 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG4                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG4_POS)) /**< MEUCFG_ENC_REG4 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG5_POS                 5 /**< MEUCFG_ENC_REG5 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG5                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG5_POS)) /**< MEUCFG_ENC_REG5 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG6_POS                 6 /**< MEUCFG_ENC_REG6 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG6                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG6_POS)) /**< MEUCFG_ENC_REG6 Mask */

#define MXC_F_SMON_MEUCFG_ENC_REG7_POS                 7 /**< MEUCFG_ENC_REG7 Position */
#define MXC_F_SMON_MEUCFG_ENC_REG7                     ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_ENC_REG7_POS)) /**< MEUCFG_ENC_REG7 Mask */

#define MXC_F_SMON_MEUCFG_LOCK_POS                     31 /**< MEUCFG_LOCK Position */
#define MXC_F_SMON_MEUCFG_LOCK                         ((uint32_t)(0x1UL << MXC_F_SMON_MEUCFG_LOCK_POS)) /**< MEUCFG_LOCK Mask */

/**@} end of group SMON_MEUCFG_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SECST SMON_SECST
 * @brief    Security Monitor Status Register.
 * @{
 */
#define MXC_F_SMON_SECST_EXTSRS_POS                    0 /**< SECST_EXTSRS Position */
#define MXC_F_SMON_SECST_EXTSRS                        ((uint32_t)(0x1UL << MXC_F_SMON_SECST_EXTSRS_POS)) /**< SECST_EXTSRS Mask */

#define MXC_F_SMON_SECST_INTSRS_POS                    1 /**< SECST_INTSRS Position */
#define MXC_F_SMON_SECST_INTSRS                        ((uint32_t)(0x1UL << MXC_F_SMON_SECST_INTSRS_POS)) /**< SECST_INTSRS Mask */

#define MXC_F_SMON_SECST_SECALRS_POS                   2 /**< SECST_SECALRS Position */
#define MXC_F_SMON_SECST_SECALRS                       ((uint32_t)(0x1UL << MXC_F_SMON_SECST_SECALRS_POS)) /**< SECST_SECALRS Mask */

#define MXC_F_SMON_SECST_MEUCFG_POS                    4 /**< SECST_MEUCFG Position */
#define MXC_F_SMON_SECST_MEUCFG                        ((uint32_t)(0x1UL << MXC_F_SMON_SECST_MEUCFG_POS)) /**< SECST_MEUCFG Mask */

/**@} end of group SMON_SECST_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SDBE SMON_SDBE
 * @brief    Security Monitor Self Destruct Byte.
 * @{
 */
#define MXC_F_SMON_SDBE_DBYTE_POS                      0 /**< SDBE_DBYTE Position */
#define MXC_F_SMON_SDBE_DBYTE                          ((uint32_t)(0xFFUL << MXC_F_SMON_SDBE_DBYTE_POS)) /**< SDBE_DBYTE Mask */

#define MXC_F_SMON_SDBE_SBDEN_POS                      31 /**< SDBE_SBDEN Position */
#define MXC_F_SMON_SDBE_SBDEN                          ((uint32_t)(0x1UL << MXC_F_SMON_SDBE_SBDEN_POS)) /**< SDBE_SBDEN Mask */

/**@} end of group SMON_SDBE_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SMON_REGS_H_

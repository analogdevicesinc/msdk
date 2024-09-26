/**
 * @file    smon_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SMON Peripheral Module.
 * @note    This file is @generated.
 * @ingroup smon_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SMON_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SMON_REGS_H_

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
    __IO uint32_t extsctrl;             /**< <tt>\b 0x00:</tt> SMON EXTSCTRL Register */
    __IO uint32_t intsctrl;             /**< <tt>\b 0x04:</tt> SMON INTSCTRL Register */
    __IO uint32_t secalm;               /**< <tt>\b 0x08:</tt> SMON SECALM Register */
    __IO uint32_t secdiag;              /**< <tt>\b 0x0C:</tt> SMON SECDIAG Register */
    __I  uint32_t dlrtc;                /**< <tt>\b 0x10:</tt> SMON DLRTC Register */
    __R  uint32_t rsv_0x14_0x23[4];
    __IO uint32_t meuctrl;              /**< <tt>\b 0x24:</tt> SMON MEUCTRL Register */
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
#define MXC_R_SMON_EXTSCTRL                ((uint32_t)0x00000000UL) /**< Offset from SMON Base Address: <tt> 0x0000</tt> */
#define MXC_R_SMON_INTSCTRL                ((uint32_t)0x00000004UL) /**< Offset from SMON Base Address: <tt> 0x0004</tt> */
#define MXC_R_SMON_SECALM                  ((uint32_t)0x00000008UL) /**< Offset from SMON Base Address: <tt> 0x0008</tt> */
#define MXC_R_SMON_SECDIAG                 ((uint32_t)0x0000000CUL) /**< Offset from SMON Base Address: <tt> 0x000C</tt> */
#define MXC_R_SMON_DLRTC                   ((uint32_t)0x00000010UL) /**< Offset from SMON Base Address: <tt> 0x0010</tt> */
#define MXC_R_SMON_MEUCTRL                 ((uint32_t)0x00000024UL) /**< Offset from SMON Base Address: <tt> 0x0024</tt> */
#define MXC_R_SMON_SECST                   ((uint32_t)0x00000034UL) /**< Offset from SMON Base Address: <tt> 0x0034</tt> */
#define MXC_R_SMON_SDBE                    ((uint32_t)0x00000038UL) /**< Offset from SMON Base Address: <tt> 0x0038</tt> */
/**@} end of group smon_registers */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_EXTSCTRL SMON_EXTSCTRL
 * @brief    External Sensor Control Register.
 * @{
 */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN0_POS               0 /**< EXTSCTRL_EXTS_EN0 Position */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN0                   ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_EXTS_EN0_POS)) /**< EXTSCTRL_EXTS_EN0 Mask */

#define MXC_F_SMON_EXTSCTRL_EXTS_EN1_POS               1 /**< EXTSCTRL_EXTS_EN1 Position */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN1                   ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_EXTS_EN1_POS)) /**< EXTSCTRL_EXTS_EN1 Mask */

#define MXC_F_SMON_EXTSCTRL_EXTS_EN2_POS               2 /**< EXTSCTRL_EXTS_EN2 Position */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN2                   ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_EXTS_EN2_POS)) /**< EXTSCTRL_EXTS_EN2 Mask */

#define MXC_F_SMON_EXTSCTRL_EXTS_EN3_POS               3 /**< EXTSCTRL_EXTS_EN3 Position */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN3                   ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_EXTS_EN3_POS)) /**< EXTSCTRL_EXTS_EN3 Mask */

#define MXC_F_SMON_EXTSCTRL_EXTS_EN4_POS               4 /**< EXTSCTRL_EXTS_EN4 Position */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN4                   ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_EXTS_EN4_POS)) /**< EXTSCTRL_EXTS_EN4 Mask */

#define MXC_F_SMON_EXTSCTRL_EXTS_EN5_POS               5 /**< EXTSCTRL_EXTS_EN5 Position */
#define MXC_F_SMON_EXTSCTRL_EXTS_EN5                   ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_EXTS_EN5_POS)) /**< EXTSCTRL_EXTS_EN5 Mask */

#define MXC_F_SMON_EXTSCTRL_EXTCNT_POS                 16 /**< EXTSCTRL_EXTCNT Position */
#define MXC_F_SMON_EXTSCTRL_EXTCNT                     ((uint32_t)(0x1FUL << MXC_F_SMON_EXTSCTRL_EXTCNT_POS)) /**< EXTSCTRL_EXTCNT Mask */

#define MXC_F_SMON_EXTSCTRL_EXTFRQ_POS                 21 /**< EXTSCTRL_EXTFRQ Position */
#define MXC_F_SMON_EXTSCTRL_EXTFRQ                     ((uint32_t)(0x7UL << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS)) /**< EXTSCTRL_EXTFRQ Mask */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ2000HZ          ((uint32_t)0x0UL) /**< EXTSCTRL_EXTFRQ_FREQ2000HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ2000HZ          (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ2000HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ2000HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ1000HZ          ((uint32_t)0x1UL) /**< EXTSCTRL_EXTFRQ_FREQ1000HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ1000HZ          (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ1000HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ1000HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ500HZ           ((uint32_t)0x2UL) /**< EXTSCTRL_EXTFRQ_FREQ500HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ500HZ           (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ500HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ500HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ250HZ           ((uint32_t)0x3UL) /**< EXTSCTRL_EXTFRQ_FREQ250HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ250HZ           (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ250HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ250HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ125HZ           ((uint32_t)0x4UL) /**< EXTSCTRL_EXTFRQ_FREQ125HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ125HZ           (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ125HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ125HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ63HZ            ((uint32_t)0x5UL) /**< EXTSCTRL_EXTFRQ_FREQ63HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ63HZ            (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ63HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ63HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ31HZ            ((uint32_t)0x6UL) /**< EXTSCTRL_EXTFRQ_FREQ31HZ Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_FREQ31HZ            (MXC_V_SMON_EXTSCTRL_EXTFRQ_FREQ31HZ << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_FREQ31HZ Setting */
#define MXC_V_SMON_EXTSCTRL_EXTFRQ_RFU                 ((uint32_t)0x7UL) /**< EXTSCTRL_EXTFRQ_RFU Value */
#define MXC_S_SMON_EXTSCTRL_EXTFRQ_RFU                 (MXC_V_SMON_EXTSCTRL_EXTFRQ_RFU << MXC_F_SMON_EXTSCTRL_EXTFRQ_POS) /**< EXTSCTRL_EXTFRQ_RFU Setting */

#define MXC_F_SMON_EXTSCTRL_CLKDIV_POS                 24 /**< EXTSCTRL_CLKDIV Position */
#define MXC_F_SMON_EXTSCTRL_CLKDIV                     ((uint32_t)(0x7UL << MXC_F_SMON_EXTSCTRL_CLKDIV_POS)) /**< EXTSCTRL_CLKDIV Mask */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV1                ((uint32_t)0x0UL) /**< EXTSCTRL_CLKDIV_DIV1 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV1                (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV1 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV1 Setting */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV2                ((uint32_t)0x1UL) /**< EXTSCTRL_CLKDIV_DIV2 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV2                (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV2 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV2 Setting */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV4                ((uint32_t)0x2UL) /**< EXTSCTRL_CLKDIV_DIV4 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV4                (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV4 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV4 Setting */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV8                ((uint32_t)0x3UL) /**< EXTSCTRL_CLKDIV_DIV8 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV8                (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV8 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV8 Setting */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV16               ((uint32_t)0x4UL) /**< EXTSCTRL_CLKDIV_DIV16 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV16               (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV16 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV16 Setting */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV32               ((uint32_t)0x5UL) /**< EXTSCTRL_CLKDIV_DIV32 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV32               (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV32 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV32 Setting */
#define MXC_V_SMON_EXTSCTRL_CLKDIV_DIV64               ((uint32_t)0x6UL) /**< EXTSCTRL_CLKDIV_DIV64 Value */
#define MXC_S_SMON_EXTSCTRL_CLKDIV_DIV64               (MXC_V_SMON_EXTSCTRL_CLKDIV_DIV64 << MXC_F_SMON_EXTSCTRL_CLKDIV_POS) /**< EXTSCTRL_CLKDIV_DIV64 Setting */

#define MXC_F_SMON_EXTSCTRL_BUSY_POS                   30 /**< EXTSCTRL_BUSY Position */
#define MXC_F_SMON_EXTSCTRL_BUSY                       ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_BUSY_POS)) /**< EXTSCTRL_BUSY Mask */

#define MXC_F_SMON_EXTSCTRL_LOCK_POS                   31 /**< EXTSCTRL_LOCK Position */
#define MXC_F_SMON_EXTSCTRL_LOCK                       ((uint32_t)(0x1UL << MXC_F_SMON_EXTSCTRL_LOCK_POS)) /**< EXTSCTRL_LOCK Mask */

/**@} end of group SMON_EXTSCTRL_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_INTSCTRL SMON_INTSCTRL
 * @brief    Internal Sensor Control Register.
 * @{
 */
#define MXC_F_SMON_INTSCTRL_SHIELD_EN_POS              0 /**< INTSCTRL_SHIELD_EN Position */
#define MXC_F_SMON_INTSCTRL_SHIELD_EN                  ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_SHIELD_EN_POS)) /**< INTSCTRL_SHIELD_EN Mask */

#define MXC_F_SMON_INTSCTRL_TEMP_EN_POS                1 /**< INTSCTRL_TEMP_EN Position */
#define MXC_F_SMON_INTSCTRL_TEMP_EN                    ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_TEMP_EN_POS)) /**< INTSCTRL_TEMP_EN Mask */

#define MXC_F_SMON_INTSCTRL_VBAT_EN_POS                2 /**< INTSCTRL_VBAT_EN Position */
#define MXC_F_SMON_INTSCTRL_VBAT_EN                    ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_VBAT_EN_POS)) /**< INTSCTRL_VBAT_EN Mask */

#define MXC_F_SMON_INTSCTRL_DFD_EN_POS                 3 /**< INTSCTRL_DFD_EN Position */
#define MXC_F_SMON_INTSCTRL_DFD_EN                     ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_DFD_EN_POS)) /**< INTSCTRL_DFD_EN Mask */

#define MXC_F_SMON_INTSCTRL_DFD_NMI_EN_POS             4 /**< INTSCTRL_DFD_NMI_EN Position */
#define MXC_F_SMON_INTSCTRL_DFD_NMI_EN                 ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_DFD_NMI_EN_POS)) /**< INTSCTRL_DFD_NMI_EN Mask */

#define MXC_F_SMON_INTSCTRL_TAMPOUT_EN_POS             7 /**< INTSCTRL_TAMPOUT_EN Position */
#define MXC_F_SMON_INTSCTRL_TAMPOUT_EN                 ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_TAMPOUT_EN_POS)) /**< INTSCTRL_TAMPOUT_EN Mask */

#define MXC_F_SMON_INTSCTRL_LOTEMP_SEL_POS             16 /**< INTSCTRL_LOTEMP_SEL Position */
#define MXC_F_SMON_INTSCTRL_LOTEMP_SEL                 ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_LOTEMP_SEL_POS)) /**< INTSCTRL_LOTEMP_SEL Mask */

#define MXC_F_SMON_INTSCTRL_HITEMP_SEL_POS             17 /**< INTSCTRL_HITEMP_SEL Position */
#define MXC_F_SMON_INTSCTRL_HITEMP_SEL                 ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_HITEMP_SEL_POS)) /**< INTSCTRL_HITEMP_SEL Mask */

#define MXC_F_SMON_INTSCTRL_VCORELO_EN_POS             18 /**< INTSCTRL_VCORELO_EN Position */
#define MXC_F_SMON_INTSCTRL_VCORELO_EN                 ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_VCORELO_EN_POS)) /**< INTSCTRL_VCORELO_EN Mask */

#define MXC_F_SMON_INTSCTRL_VCOREHI_EN_POS             19 /**< INTSCTRL_VCOREHI_EN Position */
#define MXC_F_SMON_INTSCTRL_VCOREHI_EN                 ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_VCOREHI_EN_POS)) /**< INTSCTRL_VCOREHI_EN Mask */

#define MXC_F_SMON_INTSCTRL_VDDLO_EN_POS               20 /**< INTSCTRL_VDDLO_EN Position */
#define MXC_F_SMON_INTSCTRL_VDDLO_EN                   ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_VDDLO_EN_POS)) /**< INTSCTRL_VDDLO_EN Mask */

#define MXC_F_SMON_INTSCTRL_VDDHI_EN_POS               21 /**< INTSCTRL_VDDHI_EN Position */
#define MXC_F_SMON_INTSCTRL_VDDHI_EN                   ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_VDDHI_EN_POS)) /**< INTSCTRL_VDDHI_EN Mask */

#define MXC_F_SMON_INTSCTRL_VGL_EN_POS                 22 /**< INTSCTRL_VGL_EN Position */
#define MXC_F_SMON_INTSCTRL_VGL_EN                     ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_VGL_EN_POS)) /**< INTSCTRL_VGL_EN Mask */

#define MXC_F_SMON_INTSCTRL_LOCK_POS                   31 /**< INTSCTRL_LOCK Position */
#define MXC_F_SMON_INTSCTRL_LOCK                       ((uint32_t)(0x1UL << MXC_F_SMON_INTSCTRL_LOCK_POS)) /**< INTSCTRL_LOCK Mask */

/**@} end of group SMON_INTSCTRL_Register */

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

#define MXC_F_SMON_SECALM_SHIELD_FL_POS                2 /**< SECALM_SHIELD_FL Position */
#define MXC_F_SMON_SECALM_SHIELD_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_SHIELD_FL_POS)) /**< SECALM_SHIELD_FL Mask */

#define MXC_F_SMON_SECALM_LOTEMP_FL_POS                3 /**< SECALM_LOTEMP_FL Position */
#define MXC_F_SMON_SECALM_LOTEMP_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_LOTEMP_FL_POS)) /**< SECALM_LOTEMP_FL Mask */

#define MXC_F_SMON_SECALM_HITEMP_FL_POS                4 /**< SECALM_HITEMP_FL Position */
#define MXC_F_SMON_SECALM_HITEMP_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_HITEMP_FL_POS)) /**< SECALM_HITEMP_FL Mask */

#define MXC_F_SMON_SECALM_BATLO_FL_POS                 5 /**< SECALM_BATLO_FL Position */
#define MXC_F_SMON_SECALM_BATLO_FL                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_BATLO_FL_POS)) /**< SECALM_BATLO_FL Mask */

#define MXC_F_SMON_SECALM_BATHI_FL_POS                 6 /**< SECALM_BATHI_FL Position */
#define MXC_F_SMON_SECALM_BATHI_FL                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_BATHI_FL_POS)) /**< SECALM_BATHI_FL Mask */

#define MXC_F_SMON_SECALM_EXTS_FL_POS                  7 /**< SECALM_EXTS_FL Position */
#define MXC_F_SMON_SECALM_EXTS_FL                      ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTS_FL_POS)) /**< SECALM_EXTS_FL Mask */

#define MXC_F_SMON_SECALM_DFD_FL_POS                   8 /**< SECALM_DFD_FL Position */
#define MXC_F_SMON_SECALM_DFD_FL                       ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_DFD_FL_POS)) /**< SECALM_DFD_FL Mask */

#define MXC_F_SMON_SECALM_VMAINPF_FL_POS               9 /**< SECALM_VMAINPF_FL Position */
#define MXC_F_SMON_SECALM_VMAINPF_FL                   ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_VMAINPF_FL_POS)) /**< SECALM_VMAINPF_FL Mask */

#define MXC_F_SMON_SECALM_VCOREHI_FL_POS               10 /**< SECALM_VCOREHI_FL Position */
#define MXC_F_SMON_SECALM_VCOREHI_FL                   ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_VCOREHI_FL_POS)) /**< SECALM_VCOREHI_FL Mask */

#define MXC_F_SMON_SECALM_VDDHI_FL_POS                 11 /**< SECALM_VDDHI_FL Position */
#define MXC_F_SMON_SECALM_VDDHI_FL                     ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_VDDHI_FL_POS)) /**< SECALM_VDDHI_FL Mask */

#define MXC_F_SMON_SECALM_VGL_FL_POS                   12 /**< SECALM_VGL_FL Position */
#define MXC_F_SMON_SECALM_VGL_FL                       ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_VGL_FL_POS)) /**< SECALM_VGL_FL Mask */

#define MXC_F_SMON_SECALM_EXTSTAT0_FL_POS              16 /**< SECALM_EXTSTAT0_FL Position */
#define MXC_F_SMON_SECALM_EXTSTAT0_FL                  ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT0_FL_POS)) /**< SECALM_EXTSTAT0_FL Mask */

#define MXC_F_SMON_SECALM_EXTSTAT1_FL_POS              17 /**< SECALM_EXTSTAT1_FL Position */
#define MXC_F_SMON_SECALM_EXTSTAT1_FL                  ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT1_FL_POS)) /**< SECALM_EXTSTAT1_FL Mask */

#define MXC_F_SMON_SECALM_EXTSTAT2_FL_POS              18 /**< SECALM_EXTSTAT2_FL Position */
#define MXC_F_SMON_SECALM_EXTSTAT2_FL                  ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT2_FL_POS)) /**< SECALM_EXTSTAT2_FL Mask */

#define MXC_F_SMON_SECALM_EXTSTAT3_FL_POS              19 /**< SECALM_EXTSTAT3_FL Position */
#define MXC_F_SMON_SECALM_EXTSTAT3_FL                  ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT3_FL_POS)) /**< SECALM_EXTSTAT3_FL Mask */

#define MXC_F_SMON_SECALM_EXTSTAT4_FL_POS              20 /**< SECALM_EXTSTAT4_FL Position */
#define MXC_F_SMON_SECALM_EXTSTAT4_FL                  ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT4_FL_POS)) /**< SECALM_EXTSTAT4_FL Mask */

#define MXC_F_SMON_SECALM_EXTSTAT5_FL_POS              21 /**< SECALM_EXTSTAT5_FL Position */
#define MXC_F_SMON_SECALM_EXTSTAT5_FL                  ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSTAT5_FL_POS)) /**< SECALM_EXTSTAT5_FL Mask */

#define MXC_F_SMON_SECALM_EXTSWARN0_FL_POS             24 /**< SECALM_EXTSWARN0_FL Position */
#define MXC_F_SMON_SECALM_EXTSWARN0_FL                 ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN0_FL_POS)) /**< SECALM_EXTSWARN0_FL Mask */

#define MXC_F_SMON_SECALM_EXTSWARN1_FL_POS             25 /**< SECALM_EXTSWARN1_FL Position */
#define MXC_F_SMON_SECALM_EXTSWARN1_FL                 ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN1_FL_POS)) /**< SECALM_EXTSWARN1_FL Mask */

#define MXC_F_SMON_SECALM_EXTSWARN2_FL_POS             26 /**< SECALM_EXTSWARN2_FL Position */
#define MXC_F_SMON_SECALM_EXTSWARN2_FL                 ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN2_FL_POS)) /**< SECALM_EXTSWARN2_FL Mask */

#define MXC_F_SMON_SECALM_EXTSWARN3_FL_POS             27 /**< SECALM_EXTSWARN3_FL Position */
#define MXC_F_SMON_SECALM_EXTSWARN3_FL                 ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN3_FL_POS)) /**< SECALM_EXTSWARN3_FL Mask */

#define MXC_F_SMON_SECALM_EXTSWARN4_FL_POS             28 /**< SECALM_EXTSWARN4_FL Position */
#define MXC_F_SMON_SECALM_EXTSWARN4_FL                 ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN4_FL_POS)) /**< SECALM_EXTSWARN4_FL Mask */

#define MXC_F_SMON_SECALM_EXTSWARN5_FL_POS             29 /**< SECALM_EXTSWARN5_FL Position */
#define MXC_F_SMON_SECALM_EXTSWARN5_FL                 ((uint32_t)(0x1UL << MXC_F_SMON_SECALM_EXTSWARN5_FL_POS)) /**< SECALM_EXTSWARN5_FL Mask */

/**@} end of group SMON_SECALM_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SECDIAG SMON_SECDIAG
 * @brief    Security Diagnostic Register.
 * @{
 */
#define MXC_F_SMON_SECDIAG_POR_FL_POS                  0 /**< SECDIAG_POR_FL Position */
#define MXC_F_SMON_SECDIAG_POR_FL                      ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_POR_FL_POS)) /**< SECDIAG_POR_FL Mask */

#define MXC_F_SMON_SECDIAG_SHIELD_FL_POS               2 /**< SECDIAG_SHIELD_FL Position */
#define MXC_F_SMON_SECDIAG_SHIELD_FL                   ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_SHIELD_FL_POS)) /**< SECDIAG_SHIELD_FL Mask */

#define MXC_F_SMON_SECDIAG_LOTEMP_FL_POS               3 /**< SECDIAG_LOTEMP_FL Position */
#define MXC_F_SMON_SECDIAG_LOTEMP_FL                   ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_LOTEMP_FL_POS)) /**< SECDIAG_LOTEMP_FL Mask */

#define MXC_F_SMON_SECDIAG_HITEMP_FL_POS               4 /**< SECDIAG_HITEMP_FL Position */
#define MXC_F_SMON_SECDIAG_HITEMP_FL                   ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_HITEMP_FL_POS)) /**< SECDIAG_HITEMP_FL Mask */

#define MXC_F_SMON_SECDIAG_BATLO_FL_POS                5 /**< SECDIAG_BATLO_FL Position */
#define MXC_F_SMON_SECDIAG_BATLO_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_BATLO_FL_POS)) /**< SECDIAG_BATLO_FL Mask */

#define MXC_F_SMON_SECDIAG_BATHI_FL_POS                6 /**< SECDIAG_BATHI_FL Position */
#define MXC_F_SMON_SECDIAG_BATHI_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_BATHI_FL_POS)) /**< SECDIAG_BATHI_FL Mask */

#define MXC_F_SMON_SECDIAG_DYNS_FL_POS                 7 /**< SECDIAG_DYNS_FL Position */
#define MXC_F_SMON_SECDIAG_DYNS_FL                     ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_DYNS_FL_POS)) /**< SECDIAG_DYNS_FL Mask */

#define MXC_F_SMON_SECDIAG_AESKT_MEU_POS               8 /**< SECDIAG_AESKT_MEU Position */
#define MXC_F_SMON_SECDIAG_AESKT_MEU                   ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_AESKT_MEU_POS)) /**< SECDIAG_AESKT_MEU Mask */

#define MXC_F_SMON_SECDIAG_AESKT_MEMPROT_XIP_POS       9 /**< SECDIAG_AESKT_MEMPROT_XIP Position */
#define MXC_F_SMON_SECDIAG_AESKT_MEMPROT_XIP           ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_AESKT_MEMPROT_XIP_POS)) /**< SECDIAG_AESKT_MEMPROT_XIP Mask */

#define MXC_F_SMON_SECDIAG_KEY0_ZERO_POS               10 /**< SECDIAG_KEY0_ZERO Position */
#define MXC_F_SMON_SECDIAG_KEY0_ZERO                   ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_KEY0_ZERO_POS)) /**< SECDIAG_KEY0_ZERO Mask */

#define MXC_F_SMON_SECDIAG_KEY1_ZERO_POS               11 /**< SECDIAG_KEY1_ZERO Position */
#define MXC_F_SMON_SECDIAG_KEY1_ZERO                   ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_KEY1_ZERO_POS)) /**< SECDIAG_KEY1_ZERO Mask */

#define MXC_F_SMON_SECDIAG_DFD_FL_POS                  15 /**< SECDIAG_DFD_FL Position */
#define MXC_F_SMON_SECDIAG_DFD_FL                      ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_DFD_FL_POS)) /**< SECDIAG_DFD_FL Mask */

#define MXC_F_SMON_SECDIAG_EXTS0_FL_POS                16 /**< SECDIAG_EXTS0_FL Position */
#define MXC_F_SMON_SECDIAG_EXTS0_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTS0_FL_POS)) /**< SECDIAG_EXTS0_FL Mask */

#define MXC_F_SMON_SECDIAG_EXTS1_FL_POS                17 /**< SECDIAG_EXTS1_FL Position */
#define MXC_F_SMON_SECDIAG_EXTS1_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTS1_FL_POS)) /**< SECDIAG_EXTS1_FL Mask */

#define MXC_F_SMON_SECDIAG_EXTS2_FL_POS                18 /**< SECDIAG_EXTS2_FL Position */
#define MXC_F_SMON_SECDIAG_EXTS2_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTS2_FL_POS)) /**< SECDIAG_EXTS2_FL Mask */

#define MXC_F_SMON_SECDIAG_EXTS3_FL_POS                19 /**< SECDIAG_EXTS3_FL Position */
#define MXC_F_SMON_SECDIAG_EXTS3_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTS3_FL_POS)) /**< SECDIAG_EXTS3_FL Mask */

#define MXC_F_SMON_SECDIAG_EXTS4_FL_POS                20 /**< SECDIAG_EXTS4_FL Position */
#define MXC_F_SMON_SECDIAG_EXTS4_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTS4_FL_POS)) /**< SECDIAG_EXTS4_FL Mask */

#define MXC_F_SMON_SECDIAG_EXTS5_FL_POS                21 /**< SECDIAG_EXTS5_FL Position */
#define MXC_F_SMON_SECDIAG_EXTS5_FL                    ((uint32_t)(0x1UL << MXC_F_SMON_SECDIAG_EXTS5_FL_POS)) /**< SECDIAG_EXTS5_FL Mask */

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
 * @defgroup SMON_MEUCTRL SMON_MEUCTRL
 * @brief    MEU Configuration
 * @{
 */
#define MXC_F_SMON_MEUCTRL_ENC_EN_POS                  0 /**< MEUCTRL_ENC_EN Position */
#define MXC_F_SMON_MEUCTRL_ENC_EN                      ((uint32_t)(0x7FUL << MXC_F_SMON_MEUCTRL_ENC_EN_POS)) /**< MEUCTRL_ENC_EN Mask */

#define MXC_F_SMON_MEUCTRL_LOCK_POS                    31 /**< MEUCTRL_LOCK Position */
#define MXC_F_SMON_MEUCTRL_LOCK                        ((uint32_t)(0x1UL << MXC_F_SMON_MEUCTRL_LOCK_POS)) /**< MEUCTRL_LOCK Mask */

/**@} end of group SMON_MEUCTRL_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SECST SMON_SECST
 * @brief    Security Monitor Status Register.
 * @{
 */
#define MXC_F_SMON_SECST_EXTSCTRL_POS                  0 /**< SECST_EXTSCTRL Position */
#define MXC_F_SMON_SECST_EXTSCTRL                      ((uint32_t)(0x1UL << MXC_F_SMON_SECST_EXTSCTRL_POS)) /**< SECST_EXTSCTRL Mask */

#define MXC_F_SMON_SECST_INTSCTRL_POS                  1 /**< SECST_INTSCTRL Position */
#define MXC_F_SMON_SECST_INTSCTRL                      ((uint32_t)(0x1UL << MXC_F_SMON_SECST_INTSCTRL_POS)) /**< SECST_INTSCTRL Mask */

#define MXC_F_SMON_SECST_SECALM_POS                    2 /**< SECST_SECALM Position */
#define MXC_F_SMON_SECST_SECALM                        ((uint32_t)(0x1UL << MXC_F_SMON_SECST_SECALM_POS)) /**< SECST_SECALM Mask */

#define MXC_F_SMON_SECST_MEUCTRL_POS                   7 /**< SECST_MEUCTRL Position */
#define MXC_F_SMON_SECST_MEUCTRL                       ((uint32_t)(0x1UL << MXC_F_SMON_SECST_MEUCTRL_POS)) /**< SECST_MEUCTRL Mask */

/**@} end of group SMON_SECST_Register */

/**
 * @ingroup  smon_registers
 * @defgroup SMON_SDBE SMON_SDBE
 * @brief    Security Monitor Self Destruct Byte.
 * @{
 */
#define MXC_F_SMON_SDBE_SDBYTE_POS                     0 /**< SDBE_SDBYTE Position */
#define MXC_F_SMON_SDBE_SDBYTE                         ((uint32_t)(0xFFUL << MXC_F_SMON_SDBE_SDBYTE_POS)) /**< SDBE_SDBYTE Mask */

#define MXC_F_SMON_SDBE_SDBYTE_EN_POS                  31 /**< SDBE_SDBYTE_EN Position */
#define MXC_F_SMON_SDBE_SDBYTE_EN                      ((uint32_t)(0x1UL << MXC_F_SMON_SDBE_SDBYTE_EN_POS)) /**< SDBE_SDBYTE_EN Mask */

/**@} end of group SMON_SDBE_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SMON_REGS_H_

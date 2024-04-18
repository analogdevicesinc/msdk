/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_

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
 * @ingroup     mcr
 * @defgroup    mcr_registers MCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @details     Misc Control.
 */

/**
 * @ingroup mcr_registers
 * Structure type to access the MCR Registers.
 */
typedef struct {
    __IO uint32_t eccen;                /**< <tt>\b 0x00:</tt> MCR ECCEN Register */
    __IO uint32_t ipotrim;              /**< <tt>\b 0x04:</tt> MCR IPOTRIM Register */
    __IO uint32_t outen;                /**< <tt>\b 0x08:</tt> MCR OUTEN Register */
    __IO uint32_t cmp_ctrl;             /**< <tt>\b 0x0C:</tt> MCR CMP_CTRL Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __R  uint32_t rsv_0x14;
    __IO uint32_t rtcrst;               /**< <tt>\b 0x18:</tt> MCR RTCRST Register */
    __IO uint32_t rtctrim;              /**< <tt>\b 0x1C:</tt> MCR RTCTRIM Register */
    __IO uint32_t gpio3_ctrl;           /**< <tt>\b 0x20:</tt> MCR GPIO3_CTRL Register */
    __R  uint32_t rsv_0x24_0x5f[15];
    __IO uint32_t ldoctrl;              /**< <tt>\b 0x60:</tt> MCR LDOCTRL Register */
    __IO uint32_t pwrmonst;             /**< <tt>\b 0x64:</tt> MCR PWRMONST Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_MCR_IPOTRIM                  ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_OUTEN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CMP_CTRL                 ((uint32_t)0x0000000CUL) /**< Offset from MCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_RTCRST                   ((uint32_t)0x00000018UL) /**< Offset from MCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_MCR_RTCTRIM                  ((uint32_t)0x0000001CUL) /**< Offset from MCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_MCR_GPIO3_CTRL               ((uint32_t)0x00000020UL) /**< Offset from MCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_MCR_LDOCTRL                  ((uint32_t)0x00000060UL) /**< Offset from MCR Base Address: <tt> 0x0060</tt> */
#define MXC_R_MCR_PWRMONST                 ((uint32_t)0x00000064UL) /**< Offset from MCR Base Address: <tt> 0x0064</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ECCEN MCR_ECCEN
 * @brief    ECC Enable Register
 * @{
 */
#define MXC_F_MCR_ECCEN_RAM0_POS                       0 /**< ECCEN_RAM0 Position */
#define MXC_F_MCR_ECCEN_RAM0                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM0_POS)) /**< ECCEN_RAM0 Mask */

/**@} end of group MCR_ECCEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_IPOTRIM MCR_IPOTRIM
 * @brief    IPO Manual Trim Register
 * @{
 */
#define MXC_F_MCR_IPOTRIM_VAL_POS                      0 /**< IPOTRIM_VAL Position */
#define MXC_F_MCR_IPOTRIM_VAL                          ((uint32_t)(0xFFUL << MXC_F_MCR_IPOTRIM_VAL_POS)) /**< IPOTRIM_VAL Mask */

#define MXC_F_MCR_IPOTRIM_RANGE_POS                    8 /**< IPOTRIM_RANGE Position */
#define MXC_F_MCR_IPOTRIM_RANGE                        ((uint32_t)(0x1UL << MXC_F_MCR_IPOTRIM_RANGE_POS)) /**< IPOTRIM_RANGE Mask */

/**@} end of group MCR_IPOTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_OUTEN MCR_OUTEN
 * @brief    Output Enable Register.
 * @{
 */
#define MXC_F_MCR_OUTEN_SQWOUT_EN_POS                  0 /**< OUTEN_SQWOUT_EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT_EN_POS)) /**< OUTEN_SQWOUT_EN Mask */

#define MXC_F_MCR_OUTEN_PDOWN_EN_POS                   1 /**< OUTEN_PDOWN_EN Position */
#define MXC_F_MCR_OUTEN_PDOWN_EN                       ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_PDOWN_EN_POS)) /**< OUTEN_PDOWN_EN Mask */

/**@} end of group MCR_OUTEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CMP_CTRL MCR_CMP_CTRL
 * @brief    Comparator Control Register.
 * @{
 */
#define MXC_F_MCR_CMP_CTRL_EN_POS                      0 /**< CMP_CTRL_EN Position */
#define MXC_F_MCR_CMP_CTRL_EN                          ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_EN_POS)) /**< CMP_CTRL_EN Mask */

#define MXC_F_MCR_CMP_CTRL_POL_POS                     5 /**< CMP_CTRL_POL Position */
#define MXC_F_MCR_CMP_CTRL_POL                         ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_POL_POS)) /**< CMP_CTRL_POL Mask */

#define MXC_F_MCR_CMP_CTRL_INTR_EN_POS                 6 /**< CMP_CTRL_INTR_EN Position */
#define MXC_F_MCR_CMP_CTRL_INTR_EN                     ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_INTR_EN_POS)) /**< CMP_CTRL_INTR_EN Mask */

#define MXC_F_MCR_CMP_CTRL_OUT_POS                     14 /**< CMP_CTRL_OUT Position */
#define MXC_F_MCR_CMP_CTRL_OUT                         ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_OUT_POS)) /**< CMP_CTRL_OUT Mask */

#define MXC_F_MCR_CMP_CTRL_INTR_FL_POS                 15 /**< CMP_CTRL_INTR_FL Position */
#define MXC_F_MCR_CMP_CTRL_INTR_FL                     ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_INTR_FL_POS)) /**< CMP_CTRL_INTR_FL Mask */

/**@} end of group MCR_CMP_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_CMP_HYST_POS                    0 /**< CTRL_CMP_HYST Position */
#define MXC_F_MCR_CTRL_CMP_HYST                        ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_CMP_HYST_POS)) /**< CTRL_CMP_HYST Mask */

#define MXC_F_MCR_CTRL_INRO_EN_POS                     2 /**< CTRL_INRO_EN Position */
#define MXC_F_MCR_CTRL_INRO_EN                         ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_INRO_EN_POS)) /**< CTRL_INRO_EN Mask */

#define MXC_F_MCR_CTRL_ERTCO_EN_POS                    3 /**< CTRL_ERTCO_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_EN                        ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_EN_POS)) /**< CTRL_ERTCO_EN Mask */

#define MXC_F_MCR_CTRL_IBRO_EN_POS                     4 /**< CTRL_IBRO_EN Position */
#define MXC_F_MCR_CTRL_IBRO_EN                         ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_IBRO_EN_POS)) /**< CTRL_IBRO_EN Mask */

#define MXC_F_MCR_CTRL_ERTCO_LP_EN_POS                 5 /**< CTRL_ERTCO_LP_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_LP_EN                     ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_LP_EN_POS)) /**< CTRL_ERTCO_LP_EN Mask */

#define MXC_F_MCR_CTRL_PADPUP_RST_POS                  9 /**< CTRL_PADPUP_RST Position */
#define MXC_F_MCR_CTRL_PADPUP_RST                      ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_PADPUP_RST_POS)) /**< CTRL_PADPUP_RST Mask */

#define MXC_F_MCR_CTRL_PADVDDIOHSEL_RST_POS            10 /**< CTRL_PADVDDIOHSEL_RST Position */
#define MXC_F_MCR_CTRL_PADVDDIOHSEL_RST                ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_PADVDDIOHSEL_RST_POS)) /**< CTRL_PADVDDIOHSEL_RST Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RTCRST MCR_RTCRST
 * @brief    Reset Register.
 * @{
 */
#define MXC_F_MCR_RTCRST_RTC_POS                       0 /**< RTCRST_RTC Position */
#define MXC_F_MCR_RTCRST_RTC                           ((uint32_t)(0x1UL << MXC_F_MCR_RTCRST_RTC_POS)) /**< RTCRST_RTC Mask */

/**@} end of group MCR_RTCRST_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RTCTRIM MCR_RTCTRIM
 * @brief    RTC Trim Register.
 * @{
 */
#define MXC_F_MCR_RTCTRIM_X1_POS                       0 /**< RTCTRIM_X1 Position */
#define MXC_F_MCR_RTCTRIM_X1                           ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_X1_POS)) /**< RTCTRIM_X1 Mask */

#define MXC_F_MCR_RTCTRIM_X2_POS                       8 /**< RTCTRIM_X2 Position */
#define MXC_F_MCR_RTCTRIM_X2                           ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_X2_POS)) /**< RTCTRIM_X2 Mask */

/**@} end of group MCR_RTCTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_GPIO3_CTRL MCR_GPIO3_CTRL
 * @brief    GPIO3 Pin Control Register.
 * @{
 */
#define MXC_F_MCR_GPIO3_CTRL_P30_OUT_POS               0 /**< GPIO3_CTRL_P30_OUT Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_OUT                   ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_OUT_POS)) /**< GPIO3_CTRL_P30_OUT Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_OUTEN_POS             1 /**< GPIO3_CTRL_P30_OUTEN Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_OUTEN                 ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_OUTEN_POS)) /**< GPIO3_CTRL_P30_OUTEN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_PUPEN_POS             2 /**< GPIO3_CTRL_P30_PUPEN Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_PUPEN                 ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_PUPEN_POS)) /**< GPIO3_CTRL_P30_PUPEN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_IN_POS                3 /**< GPIO3_CTRL_P30_IN Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_IN_POS)) /**< GPIO3_CTRL_P30_IN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_OUT_POS               4 /**< GPIO3_CTRL_P31_OUT Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_OUT                   ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_OUT_POS)) /**< GPIO3_CTRL_P31_OUT Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_OUTEN_POS             5 /**< GPIO3_CTRL_P31_OUTEN Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_OUTEN                 ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_OUTEN_POS)) /**< GPIO3_CTRL_P31_OUTEN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_PUPEN_POS             6 /**< GPIO3_CTRL_P31_PUPEN Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_PUPEN                 ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_PUPEN_POS)) /**< GPIO3_CTRL_P31_PUPEN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_IN_POS                7 /**< GPIO3_CTRL_P31_IN Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_IN_POS)) /**< GPIO3_CTRL_P31_IN Mask */

/**@} end of group MCR_GPIO3_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LDOCTRL MCR_LDOCTRL
 * @brief    LDO Control Register.
 * @{
 */
#define MXC_F_MCR_LDOCTRL_0P9V_EN_POS                  0 /**< LDOCTRL_0P9V_EN Position */
#define MXC_F_MCR_LDOCTRL_0P9V_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_LDOCTRL_0P9V_EN_POS)) /**< LDOCTRL_0P9V_EN Mask */

/**@} end of group MCR_LDOCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PWRMONST MCR_PWRMONST
 * @brief    LDO Control Register.
 * @{
 */
#define MXC_F_MCR_PWRMONST_PORZ_VLOSS_POS              0 /**< PWRMONST_PORZ_VLOSS Position */
#define MXC_F_MCR_PWRMONST_PORZ_VLOSS                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VLOSS_POS)) /**< PWRMONST_PORZ_VLOSS Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VBAT_POS               1 /**< PWRMONST_PORZ_VBAT Position */
#define MXC_F_MCR_PWRMONST_PORZ_VBAT                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VBAT_POS)) /**< PWRMONST_PORZ_VBAT Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VBB_POS                2 /**< PWRMONST_PORZ_VBB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VBB                    ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VBB_POS)) /**< PWRMONST_PORZ_VBB Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDCA_POS              4 /**< PWRMONST_PORZ_VDDCA Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDCA                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDCA_POS)) /**< PWRMONST_PORZ_VDDCA Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDCB_POS              5 /**< PWRMONST_PORZ_VDDCB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDCB                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDCB_POS)) /**< PWRMONST_PORZ_VDDCB Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDA_POS               6 /**< PWRMONST_PORZ_VDDA Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDA                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDA_POS)) /**< PWRMONST_PORZ_VDDA Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDB_POS               7 /**< PWRMONST_PORZ_VDDB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDB                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDB_POS)) /**< PWRMONST_PORZ_VDDB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDCB_POS              9 /**< PWRMONST_RSTZ_VDDCB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDCB                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDCB_POS)) /**< PWRMONST_RSTZ_VDDCB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDA_POS               10 /**< PWRMONST_RSTZ_VDDA Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDA                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDA_POS)) /**< PWRMONST_RSTZ_VDDA Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDB_POS               11 /**< PWRMONST_RSTZ_VDDB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDB                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDB_POS)) /**< PWRMONST_RSTZ_VDDB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIO_POS              12 /**< PWRMONST_RSTZ_VDDIO Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIO                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIO_POS)) /**< PWRMONST_RSTZ_VDDIO Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOH_POS             13 /**< PWRMONST_RSTZ_VDDIOH Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOH                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOH_POS)) /**< PWRMONST_RSTZ_VDDIOH Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VBB_POS                14 /**< PWRMONST_RSTZ_VBB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VBB                    ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VBB_POS)) /**< PWRMONST_RSTZ_VBB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_LDO0P9V_POS            16 /**< PWRMONST_RSTZ_LDO0P9V Position */
#define MXC_F_MCR_PWRMONST_RSTZ_LDO0P9V                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_LDO0P9V_POS)) /**< PWRMONST_RSTZ_LDO0P9V Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDCA_POS              17 /**< PWRMONST_RSTZ_VDDCA Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDCA                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDCA_POS)) /**< PWRMONST_RSTZ_VDDCA Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VCOREHV_POS            18 /**< PWRMONST_RSTZ_VCOREHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VCOREHV                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VCOREHV_POS)) /**< PWRMONST_RSTZ_VCOREHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV_POS            19 /**< PWRMONST_RSTZ_VDDIOHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV_POS)) /**< PWRMONST_RSTZ_VDDIOHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV_POS           20 /**< PWRMONST_RSTZ_VDDIOHHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV               ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV_POS)) /**< PWRMONST_RSTZ_VDDIOHHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VNFCRX_POS             21 /**< PWRMONST_RSTZ_VNFCRX Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VNFCRX                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VNFCRX_POS)) /**< PWRMONST_RSTZ_VNFCRX Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VNFCTX_POS             22 /**< PWRMONST_RSTZ_VNFCTX Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VNFCTX                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VNFCTX_POS)) /**< PWRMONST_RSTZ_VNFCTX Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VNFC1V_POS             23 /**< PWRMONST_RSTZ_VNFC1V Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VNFC1V                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VNFC1V_POS)) /**< PWRMONST_RSTZ_VNFC1V Mask */

/**@} end of group MCR_PWRMONST_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_

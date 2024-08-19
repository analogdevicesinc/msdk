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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_MCR_REGS_H_

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
    __IO uint32_t ipo_mtrim;            /**< <tt>\b 0x04:</tt> MCR IPO_MTRIM Register */
    __IO uint32_t outen;                /**< <tt>\b 0x08:</tt> MCR OUTEN Register */
    __IO uint32_t cmp_ctrl;             /**< <tt>\b 0x0C:</tt> MCR CMP_CTRL Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __R  uint32_t rsv_0x14_0x1f[3];
    __IO uint32_t gpio3_ctrl;           /**< <tt>\b 0x20:</tt> MCR GPIO3_CTRL Register */
    __R  uint32_t rsv_0x24_0x3f[7];
    __IO uint32_t cwd0;                 /**< <tt>\b 0x40:</tt> MCR CWD0 Register */
    __IO uint32_t cwd1;                 /**< <tt>\b 0x44:</tt> MCR CWD1 Register */
    __R  uint32_t rsv_0x48_0x4f[2];
    __IO uint32_t adccfg0;              /**< <tt>\b 0x50:</tt> MCR ADCCFG0 Register */
    __IO uint32_t adccfg1;              /**< <tt>\b 0x54:</tt> MCR ADCCFG1 Register */
    __IO uint32_t adccfg2;              /**< <tt>\b 0x58:</tt> MCR ADCCFG2 Register */
    __R  uint32_t rsv_0x5c;
    __IO uint32_t ldoctrl;              /**< <tt>\b 0x60:</tt> MCR LDOCTRL Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_MCR_IPO_MTRIM                ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_OUTEN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CMP_CTRL                 ((uint32_t)0x0000000CUL) /**< Offset from MCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_GPIO3_CTRL               ((uint32_t)0x00000020UL) /**< Offset from MCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_MCR_CWD0                     ((uint32_t)0x00000040UL) /**< Offset from MCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_MCR_CWD1                     ((uint32_t)0x00000044UL) /**< Offset from MCR Base Address: <tt> 0x0044</tt> */
#define MXC_R_MCR_ADCCFG0                  ((uint32_t)0x00000050UL) /**< Offset from MCR Base Address: <tt> 0x0050</tt> */
#define MXC_R_MCR_ADCCFG1                  ((uint32_t)0x00000054UL) /**< Offset from MCR Base Address: <tt> 0x0054</tt> */
#define MXC_R_MCR_ADCCFG2                  ((uint32_t)0x00000058UL) /**< Offset from MCR Base Address: <tt> 0x0058</tt> */
#define MXC_R_MCR_LDOCTRL                  ((uint32_t)0x00000060UL) /**< Offset from MCR Base Address: <tt> 0x0060</tt> */
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
 * @defgroup MCR_IPO_MTRIM MCR_IPO_MTRIM
 * @brief    IPO Manual Register
 * @{
 */
#define MXC_F_MCR_IPO_MTRIM_MTRIM_POS                  0 /**< IPO_MTRIM_MTRIM Position */
#define MXC_F_MCR_IPO_MTRIM_MTRIM                      ((uint32_t)(0xFFUL << MXC_F_MCR_IPO_MTRIM_MTRIM_POS)) /**< IPO_MTRIM_MTRIM Mask */

#define MXC_F_MCR_IPO_MTRIM_TRIM_RANGE_POS             8 /**< IPO_MTRIM_TRIM_RANGE Position */
#define MXC_F_MCR_IPO_MTRIM_TRIM_RANGE                 ((uint32_t)(0x1UL << MXC_F_MCR_IPO_MTRIM_TRIM_RANGE_POS)) /**< IPO_MTRIM_TRIM_RANGE Mask */

/**@} end of group MCR_IPO_MTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_OUTEN MCR_OUTEN
 * @brief    Output Enable Register
 * @{
 */
#define MXC_F_MCR_OUTEN_SQWOUT_EN_POS                  0 /**< OUTEN_SQWOUT_EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT_EN_POS)) /**< OUTEN_SQWOUT_EN Mask */

#define MXC_F_MCR_OUTEN_PDOWN_OUT_EN_POS               1 /**< OUTEN_PDOWN_OUT_EN Position */
#define MXC_F_MCR_OUTEN_PDOWN_OUT_EN                   ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_PDOWN_OUT_EN_POS)) /**< OUTEN_PDOWN_OUT_EN Mask */

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

#define MXC_F_MCR_CMP_CTRL_INT_EN_POS                  6 /**< CMP_CTRL_INT_EN Position */
#define MXC_F_MCR_CMP_CTRL_INT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_INT_EN_POS)) /**< CMP_CTRL_INT_EN Mask */

#define MXC_F_MCR_CMP_CTRL_OUT_POS                     14 /**< CMP_CTRL_OUT Position */
#define MXC_F_MCR_CMP_CTRL_OUT                         ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_OUT_POS)) /**< CMP_CTRL_OUT Mask */

#define MXC_F_MCR_CMP_CTRL_INT_FL_POS                  15 /**< CMP_CTRL_INT_FL Position */
#define MXC_F_MCR_CMP_CTRL_INT_FL                      ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_INT_FL_POS)) /**< CMP_CTRL_INT_FL Mask */

/**@} end of group MCR_CMP_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Miscellaneous Control Register.
 * @{
 */
#define MXC_F_MCR_CTRL_CMPHYST_POS                     0 /**< CTRL_CMPHYST Position */
#define MXC_F_MCR_CTRL_CMPHYST                         ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_CMPHYST_POS)) /**< CTRL_CMPHYST Mask */

#define MXC_F_MCR_CTRL_INRO_EN_POS                     2 /**< CTRL_INRO_EN Position */
#define MXC_F_MCR_CTRL_INRO_EN                         ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_INRO_EN_POS)) /**< CTRL_INRO_EN Mask */

#define MXC_F_MCR_CTRL_ERTCO_EN_POS                    3 /**< CTRL_ERTCO_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_EN                        ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_EN_POS)) /**< CTRL_ERTCO_EN Mask */

#define MXC_F_MCR_CTRL_IBRO_EN_POS                     4 /**< CTRL_IBRO_EN Position */
#define MXC_F_MCR_CTRL_IBRO_EN                         ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_IBRO_EN_POS)) /**< CTRL_IBRO_EN Mask */

#define MXC_F_MCR_CTRL_SIMO_CLKSCL_EN_POS              8 /**< CTRL_SIMO_CLKSCL_EN Position */
#define MXC_F_MCR_CTRL_SIMO_CLKSCL_EN                  ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_SIMO_CLKSCL_EN_POS)) /**< CTRL_SIMO_CLKSCL_EN Mask */

#define MXC_F_MCR_CTRL_SIMO_RSTD_POS                   9 /**< CTRL_SIMO_RSTD Position */
#define MXC_F_MCR_CTRL_SIMO_RSTD                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_SIMO_RSTD_POS)) /**< CTRL_SIMO_RSTD Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_GPIO3_CTRL MCR_GPIO3_CTRL
 * @brief    GPIO3 Pin Control Register.
 * @{
 */
#define MXC_F_MCR_GPIO3_CTRL_P30_DO_POS                0 /**< GPIO3_CTRL_P30_DO Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_DO                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_DO_POS)) /**< GPIO3_CTRL_P30_DO Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_OE_POS                1 /**< GPIO3_CTRL_P30_OE Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_OE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_OE_POS)) /**< GPIO3_CTRL_P30_OE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_PE_POS                2 /**< GPIO3_CTRL_P30_PE Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_PE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_PE_POS)) /**< GPIO3_CTRL_P30_PE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_IN_POS                3 /**< GPIO3_CTRL_P30_IN Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_IN_POS)) /**< GPIO3_CTRL_P30_IN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_DO_POS                4 /**< GPIO3_CTRL_P31_DO Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_DO                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_DO_POS)) /**< GPIO3_CTRL_P31_DO Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_OE_POS                5 /**< GPIO3_CTRL_P31_OE Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_OE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_OE_POS)) /**< GPIO3_CTRL_P31_OE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_PE_POS                6 /**< GPIO3_CTRL_P31_PE Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_PE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_PE_POS)) /**< GPIO3_CTRL_P31_PE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_IN_POS                7 /**< GPIO3_CTRL_P31_IN Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_IN_POS)) /**< GPIO3_CTRL_P31_IN Mask */

/**@} end of group MCR_GPIO3_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CWD0 MCR_CWD0
 * @brief    Code Word Data0
 * @{
 */
#define MXC_F_MCR_CWD0_DATA_POS                        0 /**< CWD0_DATA Position */
#define MXC_F_MCR_CWD0_DATA                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_MCR_CWD0_DATA_POS)) /**< CWD0_DATA Mask */

/**@} end of group MCR_CWD0_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CWD1 MCR_CWD1
 * @brief    Code Word Data1
 * @{
 */
#define MXC_F_MCR_CWD1_DATA_POS                        0 /**< CWD1_DATA Position */
#define MXC_F_MCR_CWD1_DATA                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_MCR_CWD1_DATA_POS)) /**< CWD1_DATA Mask */

/**@} end of group MCR_CWD1_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADCCFG0 MCR_ADCCFG0
 * @brief    ADC Config 0
 * @{
 */
#define MXC_F_MCR_ADCCFG0_LP_5K_DIS_POS                0 /**< ADCCFG0_LP_5K_DIS Position */
#define MXC_F_MCR_ADCCFG0_LP_5K_DIS                    ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_LP_5K_DIS_POS)) /**< ADCCFG0_LP_5K_DIS Mask */

#define MXC_F_MCR_ADCCFG0_LP_50K_DIS_POS               1 /**< ADCCFG0_LP_50K_DIS Position */
#define MXC_F_MCR_ADCCFG0_LP_50K_DIS                   ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_LP_50K_DIS_POS)) /**< ADCCFG0_LP_50K_DIS Mask */

#define MXC_F_MCR_ADCCFG0_EXT_REF_POS                  2 /**< ADCCFG0_EXT_REF Position */
#define MXC_F_MCR_ADCCFG0_EXT_REF                      ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_EXT_REF_POS)) /**< ADCCFG0_EXT_REF Mask */

#define MXC_F_MCR_ADCCFG0_REF_SEL_POS                  3 /**< ADCCFG0_REF_SEL Position */
#define MXC_F_MCR_ADCCFG0_REF_SEL                      ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_REF_SEL_POS)) /**< ADCCFG0_REF_SEL Mask */

/**@} end of group MCR_ADCCFG0_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADCCFG1 MCR_ADCCFG1
 * @brief    ADC Config 1
 * @{
 */
#define MXC_F_MCR_ADCCFG1_CHX_PU_DYN_POS               0 /**< ADCCFG1_CHX_PU_DYN Position */
#define MXC_F_MCR_ADCCFG1_CHX_PU_DYN                   ((uint32_t)(0x1FFFUL << MXC_F_MCR_ADCCFG1_CHX_PU_DYN_POS)) /**< ADCCFG1_CHX_PU_DYN Mask */

/**@} end of group MCR_ADCCFG1_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADCCFG2 MCR_ADCCFG2
 * @brief    ADC Config 2
 * @{
 */
#define MXC_F_MCR_ADCCFG2_CH0_POS                      0 /**< ADCCFG2_CH0 Position */
#define MXC_F_MCR_ADCCFG2_CH0                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH0_POS)) /**< ADCCFG2_CH0 Mask */
#define MXC_V_MCR_ADCCFG2_CH0_DIV1                     ((uint32_t)0x0UL) /**< ADCCFG2_CH0_DIV1 Value */
#define MXC_S_MCR_ADCCFG2_CH0_DIV1                     (MXC_V_MCR_ADCCFG2_CH0_DIV1 << MXC_F_MCR_ADCCFG2_CH0_POS) /**< ADCCFG2_CH0_DIV1 Setting */
#define MXC_V_MCR_ADCCFG2_CH0_DIV2_5K                  ((uint32_t)0x1UL) /**< ADCCFG2_CH0_DIV2_5K Value */
#define MXC_S_MCR_ADCCFG2_CH0_DIV2_5K                  (MXC_V_MCR_ADCCFG2_CH0_DIV2_5K << MXC_F_MCR_ADCCFG2_CH0_POS) /**< ADCCFG2_CH0_DIV2_5K Setting */
#define MXC_V_MCR_ADCCFG2_CH0_DIV2_50K                 ((uint32_t)0x2UL) /**< ADCCFG2_CH0_DIV2_50K Value */
#define MXC_S_MCR_ADCCFG2_CH0_DIV2_50K                 (MXC_V_MCR_ADCCFG2_CH0_DIV2_50K << MXC_F_MCR_ADCCFG2_CH0_POS) /**< ADCCFG2_CH0_DIV2_50K Setting */

#define MXC_F_MCR_ADCCFG2_CH1_POS                      2 /**< ADCCFG2_CH1 Position */
#define MXC_F_MCR_ADCCFG2_CH1                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH1_POS)) /**< ADCCFG2_CH1 Mask */

#define MXC_F_MCR_ADCCFG2_CH2_POS                      4 /**< ADCCFG2_CH2 Position */
#define MXC_F_MCR_ADCCFG2_CH2                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH2_POS)) /**< ADCCFG2_CH2 Mask */

#define MXC_F_MCR_ADCCFG2_CH3_POS                      6 /**< ADCCFG2_CH3 Position */
#define MXC_F_MCR_ADCCFG2_CH3                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH3_POS)) /**< ADCCFG2_CH3 Mask */

#define MXC_F_MCR_ADCCFG2_CH4_POS                      8 /**< ADCCFG2_CH4 Position */
#define MXC_F_MCR_ADCCFG2_CH4                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH4_POS)) /**< ADCCFG2_CH4 Mask */

#define MXC_F_MCR_ADCCFG2_CH5_POS                      10 /**< ADCCFG2_CH5 Position */
#define MXC_F_MCR_ADCCFG2_CH5                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH5_POS)) /**< ADCCFG2_CH5 Mask */

#define MXC_F_MCR_ADCCFG2_CH6_POS                      12 /**< ADCCFG2_CH6 Position */
#define MXC_F_MCR_ADCCFG2_CH6                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH6_POS)) /**< ADCCFG2_CH6 Mask */

#define MXC_F_MCR_ADCCFG2_CH7_POS                      14 /**< ADCCFG2_CH7 Position */
#define MXC_F_MCR_ADCCFG2_CH7                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_CH7_POS)) /**< ADCCFG2_CH7 Mask */

/**@} end of group MCR_ADCCFG2_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LDOCTRL MCR_LDOCTRL
 * @brief    LDO Control
 * @{
 */
#define MXC_F_MCR_LDOCTRL_0P9EN_POS                    0 /**< LDOCTRL_0P9EN Position */
#define MXC_F_MCR_LDOCTRL_0P9EN                        ((uint32_t)(0x1UL << MXC_F_MCR_LDOCTRL_0P9EN_POS)) /**< LDOCTRL_0P9EN Mask */

#define MXC_F_MCR_LDOCTRL_2P5EN_POS                    1 /**< LDOCTRL_2P5EN Position */
#define MXC_F_MCR_LDOCTRL_2P5EN                        ((uint32_t)(0x1UL << MXC_F_MCR_LDOCTRL_2P5EN_POS)) /**< LDOCTRL_2P5EN Mask */

/**@} end of group MCR_LDOCTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_MCR_REGS_H_

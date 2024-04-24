/**
 * @file    test_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TEST Peripheral Module.
 * @note    This file is @generated.
 * @ingroup test_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TEST_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TEST_REGS_H_

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
 * @ingroup     test
 * @defgroup    test_registers TEST_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TEST Peripheral Module.
 * @details     Function Control Register.
 */

/**
 * @ingroup test_registers
 * Structure type to access the TEST Registers.
 */
typedef struct {
    __IO uint32_t tmr;                  /**< <tt>\b 0x00:</tt> TEST TMR Register */
    __R  uint32_t rsv_0x4;
    __IO uint32_t tclk;                 /**< <tt>\b 0x08:</tt> TEST TCLK Register */
    __IO uint32_t tmr0;                 /**< <tt>\b 0x0C:</tt> TEST TMR0 Register */
    __IO uint32_t tmr1;                 /**< <tt>\b 0x10:</tt> TEST TMR1 Register */
    __IO uint32_t tmr2;                 /**< <tt>\b 0x14:</tt> TEST TMR2 Register */
    __IO uint32_t tmr3;                 /**< <tt>\b 0x18:</tt> TEST TMR3 Register */
    __R  uint32_t rsv_0x1c;
    __IO uint32_t mbcn;                 /**< <tt>\b 0x20:</tt> TEST MBCN Register */
    __IO uint32_t mbstat;               /**< <tt>\b 0x24:</tt> TEST MBSTAT Register */
} mxc_test_regs_t;

/* Register offsets for module TEST */
/**
 * @ingroup    test_registers
 * @defgroup   TEST_Register_Offsets Register Offsets
 * @brief      TEST Peripheral Register Offsets from the TEST Base Peripheral Address.
 * @{
 */
#define MXC_R_TEST_TMR                     ((uint32_t)0x00000000UL) /**< Offset from TEST Base Address: <tt> 0x0000</tt> */
#define MXC_R_TEST_TCLK                    ((uint32_t)0x00000008UL) /**< Offset from TEST Base Address: <tt> 0x0008</tt> */
#define MXC_R_TEST_TMR0                    ((uint32_t)0x0000000CUL) /**< Offset from TEST Base Address: <tt> 0x000C</tt> */
#define MXC_R_TEST_TMR1                    ((uint32_t)0x00000010UL) /**< Offset from TEST Base Address: <tt> 0x0010</tt> */
#define MXC_R_TEST_TMR2                    ((uint32_t)0x00000014UL) /**< Offset from TEST Base Address: <tt> 0x0014</tt> */
#define MXC_R_TEST_TMR3                    ((uint32_t)0x00000018UL) /**< Offset from TEST Base Address: <tt> 0x0018</tt> */
#define MXC_R_TEST_MBCN                    ((uint32_t)0x00000020UL) /**< Offset from TEST Base Address: <tt> 0x0020</tt> */
#define MXC_R_TEST_MBSTAT                  ((uint32_t)0x00000024UL) /**< Offset from TEST Base Address: <tt> 0x0024</tt> */
/**@} end of group test_registers */

/**
 * @ingroup  test_registers
 * @defgroup TEST_TMR TEST_TMR
 * @brief    Test mode register.
 * @{
 */
#define MXC_F_TEST_TMR_TME_POS                         0 /**< TMR_TME Position */
#define MXC_F_TEST_TMR_TME                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_TME_POS)) /**< TMR_TME Mask */

#define MXC_F_TEST_TMR_TTEN_POS                        1 /**< TMR_TTEN Position */
#define MXC_F_TEST_TMR_TTEN                            ((uint32_t)(0x1UL << MXC_F_TEST_TMR_TTEN_POS)) /**< TMR_TTEN Mask */

#define MXC_F_TEST_TMR_SCANE_POS                       3 /**< TMR_SCANE Position */
#define MXC_F_TEST_TMR_SCANE                           ((uint32_t)(0x1UL << MXC_F_TEST_TMR_SCANE_POS)) /**< TMR_SCANE Mask */

#define MXC_F_TEST_TMR_SRT_POS                         5 /**< TMR_SRT Position */
#define MXC_F_TEST_TMR_STR                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_SRT_POS)) /**< TMR_SRT Mask */

#define MXC_F_TEST_TMR_XTALSEL_POS                     7 /**< TMR_XTALSEL Position */
#define MXC_F_TEST_TMR_XTALSEL                         ((uint32_t)(0x1UL << MXC_F_TEST_TMR_XTALSEL_POS)) /**< TMR_XTALSEL Mask */
#define MXC_V_TEST_TMR_XTALSEL_INTREF                  ((uint32_t)0x0UL) /**< TMR_XTALSEL_INTREF Value */
#define MXC_S_TEST_TMR_XTALSEL_INTREF                  (MXC_V_TEST_TMR_XTALSEL_INTREF << MXC_F_TEST_TMR_XTALSEL_POS) /**< TMR_XTALSEL_INTREF Setting */
#define MXC_V_TEST_TMR_XTALSEL_GPIO                    ((uint32_t)0x1UL) /**< TMR_XTALSEL_GPIO Value */
#define MXC_S_TEST_TMR_XTALSEL_GPIO                    (MXC_V_TEST_TMR_XTALSEL_GPIO << MXC_F_TEST_TMR_XTALSEL_POS) /**< TMR_XTALSEL_GPIO Setting */

#define MXC_F_TEST_TMR_FTM_POS                         10 /**< TMR_FTM Position */
#define MXC_F_TEST_TMR_FTM                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_FTM_POS)) /**< TMR_FTM Mask */

#define MXC_F_TEST_TMR_FBIST_POS                       12 /**< TMR_FBIST Position */
#define MXC_F_TEST_TMR_FBIST                           ((uint32_t)(0x1UL << MXC_F_TEST_TMR_FBIST_POS)) /**< TMR_FBIST Mask */

#define MXC_F_TEST_TMR_POR_POS                         14 /**< TMR_POR Position */
#define MXC_F_TEST_TMR_POR                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_POR_POS)) /**< TMR_POR Mask */

#define MXC_F_TEST_TMR_BOR_POS                         15 /**< TMR_BOR Position */
#define MXC_F_TEST_TMR_BOR                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_BOR_POS)) /**< TMR_BOR Mask */

#define MXC_F_TEST_TMR_SCANMD_POS                      16 /**< TMR_SCANMD Position */
#define MXC_F_TEST_TMR_SCANMD                          ((uint32_t)(0x7UL << MXC_F_TEST_TMR_SCANMD_POS)) /**< TMR_SCANMD Mask */

#define MXC_F_TEST_TMR_CKT_POS                         20 /**< TMR_CKT Position */
#define MXC_F_TEST_TMR_CKT                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_CKT_POS)) /**< TMR_CKT Mask */

#define MXC_F_TEST_TMR_DCW_POS                         22 /**< TMR_DCW Position */
#define MXC_F_TEST_TMR_DCW                             ((uint32_t)(0x1UL << MXC_F_TEST_TMR_DCW_POS)) /**< TMR_DCW Mask */

#define MXC_F_TEST_TMR_RETLDOE_POS                     28 /**< TMR_RETLDOE Position */
#define MXC_F_TEST_TMR_RETLDOE                         ((uint32_t)(0x1UL << MXC_F_TEST_TMR_RETLDOE_POS)) /**< TMR_RETLDOE Mask */

/**@} end of group TEST_TMR_Register */

/**
 * @ingroup  test_registers
 * @defgroup TEST_TCLK TEST_TCLK
 * @brief    Test clock register.
 * @{
 */
#define MXC_F_TEST_TCLK_TCLK_POS                       0 /**< TCLK_TCLK Position */
#define MXC_F_TEST_TCLK_TCLK                           ((uint32_t)(0xFFUL << MXC_F_TEST_TCLK_TCLK_POS)) /**< TCLK_TCLK Mask */
#define MXC_V_TEST_TCLK_TCLK_CLKXTAL                   ((uint32_t)0x0UL) /**< TCLK_TCLK_CLKXTAL Value */
#define MXC_S_TEST_TCLK_TCLK_CLKXTAL                   (MXC_V_TEST_TCLK_TCLK_CLKXTAL << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_CLKXTAL Setting */
#define MXC_V_TEST_TCLK_TCLK_HCLK                      ((uint32_t)0x2UL) /**< TCLK_TCLK_HCLK Value */
#define MXC_S_TEST_TCLK_TCLK_HCLK                      (MXC_V_TEST_TCLK_TCLK_HCLK << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_HCLK Setting */
#define MXC_V_TEST_TCLK_TCLK_PCLK                      ((uint32_t)0x3UL) /**< TCLK_TCLK_PCLK Value */
#define MXC_S_TEST_TCLK_TCLK_PCLK                      (MXC_V_TEST_TCLK_TCLK_PCLK << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_PCLK Setting */
#define MXC_V_TEST_TCLK_TCLK_PCLKEN                    ((uint32_t)0x4UL) /**< TCLK_TCLK_PCLKEN Value */
#define MXC_S_TEST_TCLK_TCLK_PCLKEN                    (MXC_V_TEST_TCLK_TCLK_PCLKEN << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_PCLKEN Setting */
#define MXC_V_TEST_TCLK_TCLK_IPO                       ((uint32_t)0x5UL) /**< TCLK_TCLK_IPO Value */
#define MXC_S_TEST_TCLK_TCLK_IPO                       (MXC_V_TEST_TCLK_TCLK_IPO << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_IPO Setting */
#define MXC_V_TEST_TCLK_TCLK_IBRO                      ((uint32_t)0x7UL) /**< TCLK_TCLK_IBRO Value */
#define MXC_S_TEST_TCLK_TCLK_IBRO                      (MXC_V_TEST_TCLK_TCLK_IBRO << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_IBRO Setting */
#define MXC_V_TEST_TCLK_TCLK_ERTCO_4KHZ                ((uint32_t)0x8UL) /**< TCLK_TCLK_ERTCO_4KHZ Value */
#define MXC_S_TEST_TCLK_TCLK_ERTCO_4KHZ                (MXC_V_TEST_TCLK_TCLK_ERTCO_4KHZ << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_ERTCO_4KHZ Setting */
#define MXC_V_TEST_TCLK_TCLK_ERTCO_32KHZ               ((uint32_t)0x9UL) /**< TCLK_TCLK_ERTCO_32KHZ Value */
#define MXC_S_TEST_TCLK_TCLK_ERTCO_32KHZ               (MXC_V_TEST_TCLK_TCLK_ERTCO_32KHZ << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_ERTCO_32KHZ Setting */
#define MXC_V_TEST_TCLK_TCLK_INRO                      ((uint32_t)0xAUL) /**< TCLK_TCLK_INRO Value */
#define MXC_S_TEST_TCLK_TCLK_INRO                      (MXC_V_TEST_TCLK_TCLK_INRO << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_INRO Setting */
#define MXC_V_TEST_TCLK_TCLK_XO16M                     ((uint32_t)0x1EUL) /**< TCLK_TCLK_XO16M Value */
#define MXC_S_TEST_TCLK_TCLK_XO16M                     (MXC_V_TEST_TCLK_TCLK_XO16M << MXC_F_TEST_TCLK_TCLK_POS) /**< TCLK_TCLK_XO16M Setting */

#define MXC_F_TEST_TCLK_TCLKDIV_POS                    8 /**< TCLK_TCLKDIV Position */
#define MXC_F_TEST_TCLK_TCLKDIV                        ((uint32_t)(0xFUL << MXC_F_TEST_TCLK_TCLKDIV_POS)) /**< TCLK_TCLKDIV Mask */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV1                   ((uint32_t)0x1UL) /**< TCLK_TCLKDIV_DIV1 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV1                   (MXC_V_TEST_TCLK_TCLKDIV_DIV1 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV1 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV2                   ((uint32_t)0x2UL) /**< TCLK_TCLKDIV_DIV2 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV2                   (MXC_V_TEST_TCLK_TCLKDIV_DIV2 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV2 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV3                   ((uint32_t)0x3UL) /**< TCLK_TCLKDIV_DIV3 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV3                   (MXC_V_TEST_TCLK_TCLKDIV_DIV3 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV3 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV4                   ((uint32_t)0x4UL) /**< TCLK_TCLKDIV_DIV4 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV4                   (MXC_V_TEST_TCLK_TCLKDIV_DIV4 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV4 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV5                   ((uint32_t)0x5UL) /**< TCLK_TCLKDIV_DIV5 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV5                   (MXC_V_TEST_TCLK_TCLKDIV_DIV5 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV5 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV6                   ((uint32_t)0x6UL) /**< TCLK_TCLKDIV_DIV6 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV6                   (MXC_V_TEST_TCLK_TCLKDIV_DIV6 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV6 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV7                   ((uint32_t)0x7UL) /**< TCLK_TCLKDIV_DIV7 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV7                   (MXC_V_TEST_TCLK_TCLKDIV_DIV7 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV7 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV8                   ((uint32_t)0x8UL) /**< TCLK_TCLKDIV_DIV8 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV8                   (MXC_V_TEST_TCLK_TCLKDIV_DIV8 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV8 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV9                   ((uint32_t)0x9UL) /**< TCLK_TCLKDIV_DIV9 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV9                   (MXC_V_TEST_TCLK_TCLKDIV_DIV9 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV9 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV10                  ((uint32_t)0xAUL) /**< TCLK_TCLKDIV_DIV10 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV10                  (MXC_V_TEST_TCLK_TCLKDIV_DIV10 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV10 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV11                  ((uint32_t)0xBUL) /**< TCLK_TCLKDIV_DIV11 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV11                  (MXC_V_TEST_TCLK_TCLKDIV_DIV11 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV11 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV12                  ((uint32_t)0xCUL) /**< TCLK_TCLKDIV_DIV12 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV12                  (MXC_V_TEST_TCLK_TCLKDIV_DIV12 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV12 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV13                  ((uint32_t)0xDUL) /**< TCLK_TCLKDIV_DIV13 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV13                  (MXC_V_TEST_TCLK_TCLKDIV_DIV13 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV13 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV14                  ((uint32_t)0xEUL) /**< TCLK_TCLKDIV_DIV14 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV14                  (MXC_V_TEST_TCLK_TCLKDIV_DIV14 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV14 Setting */
#define MXC_V_TEST_TCLK_TCLKDIV_DIV15                  ((uint32_t)0xFUL) /**< TCLK_TCLKDIV_DIV15 Value */
#define MXC_S_TEST_TCLK_TCLKDIV_DIV15                  (MXC_V_TEST_TCLK_TCLKDIV_DIV15 << MXC_F_TEST_TCLK_TCLKDIV_POS) /**< TCLK_TCLKDIV_DIV15 Setting */

#define MXC_F_TEST_TCLK_TCLKEN_POS                     15 /**< TCLK_TCLKEN Position */
#define MXC_F_TEST_TCLK_TCLKEN                         ((uint32_t)(0x1UL << MXC_F_TEST_TCLK_TCLKEN_POS)) /**< TCLK_TCLKEN Mask */

#define MXC_F_TEST_TCLK_TEVTSEL_POS                    16 /**< TCLK_TEVTSEL Position */
#define MXC_F_TEST_TCLK_TEVTSEL                        ((uint32_t)(0x1UL << MXC_F_TEST_TCLK_TEVTSEL_POS)) /**< TCLK_TEVTSEL Mask */

#define MXC_F_TEST_TCLK_TEVTDIV_POS                    24 /**< TCLK_TEVTDIV Position */
#define MXC_F_TEST_TCLK_TEVTDIV                        ((uint32_t)(0x1UL << MXC_F_TEST_TCLK_TEVTDIV_POS)) /**< TCLK_TEVTDIV Mask */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV1                   ((uint32_t)0x1UL) /**< TCLK_TEVTDIV_DIV1 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV1                   (MXC_V_TEST_TCLK_TEVTDIV_DIV1 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV1 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV2                   ((uint32_t)0x2UL) /**< TCLK_TEVTDIV_DIV2 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV2                   (MXC_V_TEST_TCLK_TEVTDIV_DIV2 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV2 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV3                   ((uint32_t)0x3UL) /**< TCLK_TEVTDIV_DIV3 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV3                   (MXC_V_TEST_TCLK_TEVTDIV_DIV3 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV3 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV4                   ((uint32_t)0x4UL) /**< TCLK_TEVTDIV_DIV4 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV4                   (MXC_V_TEST_TCLK_TEVTDIV_DIV4 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV4 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV5                   ((uint32_t)0x5UL) /**< TCLK_TEVTDIV_DIV5 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV5                   (MXC_V_TEST_TCLK_TEVTDIV_DIV5 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV5 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV6                   ((uint32_t)0x6UL) /**< TCLK_TEVTDIV_DIV6 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV6                   (MXC_V_TEST_TCLK_TEVTDIV_DIV6 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV6 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV7                   ((uint32_t)0x7UL) /**< TCLK_TEVTDIV_DIV7 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV7                   (MXC_V_TEST_TCLK_TEVTDIV_DIV7 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV7 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV8                   ((uint32_t)0x8UL) /**< TCLK_TEVTDIV_DIV8 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV8                   (MXC_V_TEST_TCLK_TEVTDIV_DIV8 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV8 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV9                   ((uint32_t)0x9UL) /**< TCLK_TEVTDIV_DIV9 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV9                   (MXC_V_TEST_TCLK_TEVTDIV_DIV9 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV9 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV10                  ((uint32_t)0xAUL) /**< TCLK_TEVTDIV_DIV10 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV10                  (MXC_V_TEST_TCLK_TEVTDIV_DIV10 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV10 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV11                  ((uint32_t)0xBUL) /**< TCLK_TEVTDIV_DIV11 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV11                  (MXC_V_TEST_TCLK_TEVTDIV_DIV11 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV11 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV12                  ((uint32_t)0xCUL) /**< TCLK_TEVTDIV_DIV12 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV12                  (MXC_V_TEST_TCLK_TEVTDIV_DIV12 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV12 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV13                  ((uint32_t)0xDUL) /**< TCLK_TEVTDIV_DIV13 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV13                  (MXC_V_TEST_TCLK_TEVTDIV_DIV13 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV13 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV14                  ((uint32_t)0xEUL) /**< TCLK_TEVTDIV_DIV14 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV14                  (MXC_V_TEST_TCLK_TEVTDIV_DIV14 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV14 Setting */
#define MXC_V_TEST_TCLK_TEVTDIV_DIV15                  ((uint32_t)0xFUL) /**< TCLK_TEVTDIV_DIV15 Value */
#define MXC_S_TEST_TCLK_TEVTDIV_DIV15                  (MXC_V_TEST_TCLK_TEVTDIV_DIV15 << MXC_F_TEST_TCLK_TEVTDIV_POS) /**< TCLK_TEVTDIV_DIV15 Setting */

#define MXC_F_TEST_TCLK_TEVTE_POS                      31 /**< TCLK_TEVTE Position */
#define MXC_F_TEST_TCLK_TEVTE                          ((uint32_t)(0x1UL << MXC_F_TEST_TCLK_TEVTE_POS)) /**< TCLK_TEVTE Mask */

/**@} end of group TEST_TCLK_Register */

/**
 * @ingroup  test_registers
 * @defgroup TEST_TMR1 TEST_TMR1
 * @brief    Test mode register 1.
 * @{
 */
#define MXC_F_TEST_TMR1_XTALTM1_POS                    0 /**< TMR1_XTALTM1 Position */
#define MXC_F_TEST_TMR1_XTALTM1                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALTM1_POS)) /**< TMR1_XTALTM1 Mask */

#define MXC_F_TEST_TMR1_XTALTM2_POS                    1 /**< TMR1_XTALTM2 Position */
#define MXC_F_TEST_TMR1_XTALTM2                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALTM2_POS)) /**< TMR1_XTALTM2 Mask */

#define MXC_F_TEST_TMR1_XTALPD_POS                     2 /**< TMR1_XTALPD Position */
#define MXC_F_TEST_TMR1_XTALPD                         ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALPD_POS)) /**< TMR1_XTALPD Mask */

#define MXC_F_TEST_TMR1_XTALBP_POS                     3 /**< TMR1_XTALBP Position */
#define MXC_F_TEST_TMR1_XTALBP                         ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALBP_POS)) /**< TMR1_XTALBP Mask */

#define MXC_F_TEST_TMR1_IPOPD_POS                      11 /**< TMR1_IPOPD Position */
#define MXC_F_TEST_TMR1_IPOPD                          ((uint32_t)(0x1 << MXC_F_TEST_TMR1_IPOPD_POS)) /**< TMR1_IPOPD Mask */

#define MXC_F_TEST_TMR1_ERTCPD_POS                     16 /**< TMR1_ERTCPD Position */
#define MXC_F_TEST_TMR1_ERTCPD                         ((uint32_t)(0x1 << MXC_F_TEST_TMR1_ERTCPD_POS)) /**< TMR1_ERTCPD Mask */

#define MXC_F_TEST_TMR1_ERTCBYP_POS                    17 /**< TMR1_ERTCBYP Position */
#define MXC_F_TEST_TMR1_ERTCBYP                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_ERTCBYP_POS)) /**< TMR1_ERTCBYP Mask */

#define MXC_F_TEST_TMR1_XTALDISCLK_POS                 26 /**< TMR1_XTALDISCLK Position */
#define MXC_F_TEST_TMR1_XTALDISCLK                     ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALDISCLK_POS)) /**< TMR1_XTALDISCLK Mask */

#define MXC_F_TEST_TMR1_XTALDISCLK2_POS                27 /**< TMR1_XTALDISCLK2 Position */
#define MXC_F_TEST_TMR1_XTALDISCLK2                    ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALDISCLK2_POS)) /**< TMR1_XTALDISCLK2 Mask */

#define MXC_F_TEST_TMR1_XTALDISCLKANA_POS              28 /**< TMR1_XTALDISCLKANA Position */
#define MXC_F_TEST_TMR1_XTALDISCLKANA                  ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTALDISCLKANA_POS)) /**< TMR1_XTALDISCLKANA Mask */

/**@} end of group TEST_TMR1_Register */

/**
 * @ingroup  test_registers
 * @defgroup TEST_TMR2 TEST_TMR2
 * @brief    Test mode register 2.
 * @{
 */
#define MXC_F_TEST_TMR2_I2CBE_POS                      0 /**< TMR2_I2CBE Position */
#define MXC_F_TEST_TMR2_I2CBE                          ((uint32_t)(0x1 << MXC_F_TEST_TMR2_I2CBE_POS)) /**< TMR2_I2CBE Mask */

#define MXC_F_TEST_TMR2_I2CCE_POS                      2 /**< TMR2_I2CCE Position */
#define MXC_F_TEST_TMR2_I2CCE                          ((uint32_t)(0x1 << MXC_F_TEST_TMR2_I2CCE_POS)) /**< TMR2_I2CCE Mask */

#define MXC_F_TEST_TMR2_SDAOE_POS                      4 /**< TMR2_SDAOE Position */
#define MXC_F_TEST_TMR2_SDAOE                          ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDAOE_POS)) /**< TMR2_SDAOE Mask */

#define MXC_F_TEST_TMR2_SCLOE_POS                      5 /**< TMR2_SCLOE Position */
#define MXC_F_TEST_TMR2_SCLOE                          ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCLOE_POS)) /**< TMR2_SCLOE Mask */

#define MXC_F_TEST_TMR2_SDADGEN_POS                    8 /**< TMR2_SDADGEN Position */
#define MXC_F_TEST_TMR2_SDADGEN                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDADGEN_POS)) /**< TMR2_SDADGEN Mask */

#define MXC_F_TEST_TMR2_SCLDGE_POS                     9 /**< TMR2_SCLDGE Position */
#define MXC_F_TEST_TMR2_SCLDGE                         ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCLDGE_POS)) /**< TMR2_SCLDGE Mask */

#define MXC_F_TEST_TMR2_SDADGYBP_POS                   12 /**< TMR2_SDADGYBP Position */
#define MXC_F_TEST_TMR2_SDADGYBP                       ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDADGYBP_POS)) /**< TMR2_SDADGYBP Mask */

#define MXC_F_TEST_TMR2_SCLDGBYP_POS                   13 /**< TMR2_SCLDGBYP Position */
#define MXC_F_TEST_TMR2_SCLDGBYP                       ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCLDGBYP_POS)) /**< TMR2_SCLDGBYP Mask */

#define MXC_F_TEST_TMR2_SDADGFE_POS                    16 /**< TMR2_SDADGFE Position */
#define MXC_F_TEST_TMR2_SDADGFE                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDADGFE_POS)) /**< TMR2_SDADGFE Mask */

#define MXC_F_TEST_TMR2_SCLDGFE_POS                    17 /**< TMR2_SCLDGFE Position */
#define MXC_F_TEST_TMR2_SCLDGFE                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCLDGFE_POS)) /**< TMR2_SCLDGFE Mask */

#define MXC_F_TEST_TMR2_SCLDGFE_POS                    20 /**< TMR2_SDAPADE Position */
#define MXC_F_TEST_TMR2_SDAPADE                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDAPADE_POS)) /**< TMR2_SDAPADE Mask */

#define MXC_F_TEST_TMR2_SCLPADE_POS                    21 /**< TMR2_SCLPADE Position */
#define MXC_F_TEST_TMR2_SCLPADE                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCLPADE_POS)) /**< TMR2_SCLPADE Mask */

/**@} end of group TEST_TMR2_Register */

/**
 * @ingroup  test_registers
 * @defgroup TEST_TMR3 TEST_TMR3
 * @brief    Test mode register 3.
 * @{
 */
#define MXC_F_TEST_TMR3_ROMTEST1_POS                   0 /**< TMR3_ROMTEST1 Position */
#define MXC_F_TEST_TMR3_ROMTEST1                       ((uint32_t)(0x1 << MXC_F_TEST_TMR3_ROMTEST1_POS)) /**< TMR3_ROMTEST1 Mask */

#define MXC_F_TEST_TMR3_RAMTEST1_POS                   1 /**< TMR3_RAMTEST1 Position */
#define MXC_F_TEST_TMR3_RAMTEST1                       ((uint32_t)(0x1 << MXC_F_TEST_TMR3_RAMTEST1_POS)) /**< TMR3_RAMTEST1 Mask */

#define MXC_F_TEST_TMR3_TRNGOUT_POS                    28 /**< TMR3_TRNGOUT Position */
#define MXC_F_TEST_TMR3_TRNGOUT                        ((uint32_t)(0x1 << MXC_F_TEST_TMR3_TRNGOUT_POS)) /**< TMR3_TRNGOUT Mask */

/**@} end of group TEST_TMR3_Register */

/**
 * @ingroup  test_registers
 * @defgroup TEST_MBCN TEST_MBCN
 * @brief    Memory BIST control.
 * @{
 */
#define MXC_F_TEST_MBCN_SRAM0BE_POS                    0 /**< MBCN_SRAM0BE Position */
#define MXC_F_TEST_MBCN_SRAM0BE                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SRAM0BE_POS)) /**< MBCN_SRAM0BE Mask */

#define MXC_F_TEST_MBCN_SRAM1BE_POS                    1 /**< MBCN_SRAM1BE Position */
#define MXC_F_TEST_MBCN_SRAM1BE                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SRAM1BE_POS)) /**< MBCN_SRAM1BE Mask */

#define MXC_F_TEST_MBCN_SRAM2BE_POS                    2 /**< MBCN_SRAM2BE Position */
#define MXC_F_TEST_MBCN_SRAM2BE                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SRAM2BE_POS)) /**< MBCN_SRAM2BE Mask */

#define MXC_F_TEST_MBCN_SRAM3BE_POS                    3 /**< MBCN_SRAM3BE Position */
#define MXC_F_TEST_MBCN_SRAM3BE                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SRAM3BE_POS)) /**< MBCN_SRAM3BE Mask */

#define MXC_F_TEST_MBCN_SRAM4BE_POS                    4 /**< MBCN_SRAM4BE Position */
#define MXC_F_TEST_MBCN_SRAM4BE                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SRAM4BE_POS)) /**< MBCN_SRAM4BE Mask */

#define MXC_F_TEST_MBCN_ICBE_POS                       5 /**< MBCN_ICBE Position */
#define MXC_F_TEST_MBCN_ICBE                           ((uint32_t)(0x1 << MXC_F_TEST_MBCN_ICBE_POS)) /**< MBCN_ICBE Mask */

/**@} end of group TEST_MBCN_Register */

/**
 * @ingroup  test_registers
 * @defgroup TEST_MBSTAT TEST_MBSTAT
 * @brief    Memory BIST control.
 * @{
 */
#define MXC_F_TEST_MBSTAT_SRAM0BST_POS                 0 /**< MBSTAT_SRAM0BST Position */
#define MXC_F_TEST_MBSTAT_SRAM0BST                     ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SRAM0BST_POS)) /**< MBSTAT_SRAM0BST Mask */

#define MXC_F_TEST_MBSTAT_SRAM1BST_POS                 1 /**< MBSTAT_SRAM1BST Position */
#define MXC_F_TEST_MBSTAT_SRAM1BST                     ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SRAM1BST_POS)) /**< MBSTAT_SRAM1BST Mask */

#define MXC_F_TEST_MBSTAT_SRAM2BST_POS                 2 /**< MBSTAT_SRAM2BST Position */
#define MXC_F_TEST_MBSTAT_SRAM2BST                     ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SRAM2BST_POS)) /**< MBSTAT_SRAM2BST Mask */

#define MXC_F_TEST_MBSTAT_SRAM3BST_POS                 3 /**< MBSTAT_SRAM3BST Position */
#define MXC_F_TEST_MBSTAT_SRAM3BST                     ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SRAM3BST_POS)) /**< MBSTAT_SRAM3BST Mask */

#define MXC_F_TEST_MBSTAT_SRAM4BST_POS                 4 /**< MBSTAT_SRAM4BST Position */
#define MXC_F_TEST_MBSTAT_SRAM4BST                     ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SRAM4BST_POS)) /**< MBSTAT_SRAM4BST Mask */

#define MXC_F_TEST_MBSTAT_ICBST_POS                    5 /**< MBSTAT_ICBST Position */
#define MXC_F_TEST_MBSTAT_ICBST                        ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_ICBST_POS)) /**< MBSTAT_ICBST Mask */

/**@} end of group TEST_MBSTAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TEST_REGS_H_

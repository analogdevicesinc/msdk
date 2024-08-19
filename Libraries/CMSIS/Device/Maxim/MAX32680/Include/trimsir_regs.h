/**
 * @file    trimsir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup trimsir_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_TRIMSIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_TRIMSIR_REGS_H_

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
 * @ingroup     trimsir
 * @defgroup    trimsir_registers TRIMSIR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @details     Trim System Initilazation Registers
 */

/**
 * @ingroup trimsir_registers
 * Structure type to access the TRIMSIR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t rtc;                  /**< <tt>\b 0x08:</tt> TRIMSIR RTC Register */
    __R  uint32_t rsv_0xc_0x33[10];
    __I  uint32_t simo;                 /**< <tt>\b 0x34:</tt> TRIMSIR SIMO Register */
    __R  uint32_t rsv_0x38;
    __I  uint32_t ipolo;                /**< <tt>\b 0x3C:</tt> TRIMSIR IPOLO Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x40:</tt> TRIMSIR CTRL Register */
    __IO uint32_t inro;                 /**< <tt>\b 0x44:</tt> TRIMSIR INRO Register */
} mxc_trimsir_regs_t;

/* Register offsets for module TRIMSIR */
/**
 * @ingroup    trimsir_registers
 * @defgroup   TRIMSIR_Register_Offsets Register Offsets
 * @brief      TRIMSIR Peripheral Register Offsets from the TRIMSIR Base Peripheral Address.
 * @{
 */
#define MXC_R_TRIMSIR_RTC                  ((uint32_t)0x00000008UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0008</tt> */
#define MXC_R_TRIMSIR_SIMO                 ((uint32_t)0x00000034UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0034</tt> */
#define MXC_R_TRIMSIR_IPOLO                ((uint32_t)0x0000003CUL) /**< Offset from TRIMSIR Base Address: <tt> 0x003C</tt> */
#define MXC_R_TRIMSIR_CTRL                 ((uint32_t)0x00000040UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0040</tt> */
#define MXC_R_TRIMSIR_INRO                 ((uint32_t)0x00000044UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0044</tt> */
/**@} end of group trimsir_registers */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_RTC TRIMSIR_RTC
 * @brief    RTC Trim System Initialization Register.
 * @{
 */
#define MXC_F_TRIMSIR_RTC_X1TRIM_POS                   16 /**< RTC_X1TRIM Position */
#define MXC_F_TRIMSIR_RTC_X1TRIM                       ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTC_X1TRIM_POS)) /**< RTC_X1TRIM Mask */

#define MXC_F_TRIMSIR_RTC_X2TRIM_POS                   21 /**< RTC_X2TRIM Position */
#define MXC_F_TRIMSIR_RTC_X2TRIM                       ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTC_X2TRIM_POS)) /**< RTC_X2TRIM Mask */

#define MXC_F_TRIMSIR_RTC_LOCK_POS                     31 /**< RTC_LOCK Position */
#define MXC_F_TRIMSIR_RTC_LOCK                         ((uint32_t)(0x1UL << MXC_F_TRIMSIR_RTC_LOCK_POS)) /**< RTC_LOCK Mask */

/**@} end of group TRIMSIR_RTC_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_SIMO TRIMSIR_SIMO
 * @brief    SIMO Trim System Initialization Register.
 * @{
 */
#define MXC_F_TRIMSIR_SIMO_CLKDIV_POS                  0 /**< SIMO_CLKDIV Position */
#define MXC_F_TRIMSIR_SIMO_CLKDIV                      ((uint32_t)(0x7UL << MXC_F_TRIMSIR_SIMO_CLKDIV_POS)) /**< SIMO_CLKDIV Mask */
#define MXC_V_TRIMSIR_SIMO_CLKDIV_DIV1                 ((uint32_t)0x0UL) /**< SIMO_CLKDIV_DIV1 Value */
#define MXC_S_TRIMSIR_SIMO_CLKDIV_DIV1                 (MXC_V_TRIMSIR_SIMO_CLKDIV_DIV1 << MXC_F_TRIMSIR_SIMO_CLKDIV_POS) /**< SIMO_CLKDIV_DIV1 Setting */
#define MXC_V_TRIMSIR_SIMO_CLKDIV_DIV16                ((uint32_t)0x1UL) /**< SIMO_CLKDIV_DIV16 Value */
#define MXC_S_TRIMSIR_SIMO_CLKDIV_DIV16                (MXC_V_TRIMSIR_SIMO_CLKDIV_DIV16 << MXC_F_TRIMSIR_SIMO_CLKDIV_POS) /**< SIMO_CLKDIV_DIV16 Setting */
#define MXC_V_TRIMSIR_SIMO_CLKDIV_DIV32                ((uint32_t)0x3UL) /**< SIMO_CLKDIV_DIV32 Value */
#define MXC_S_TRIMSIR_SIMO_CLKDIV_DIV32                (MXC_V_TRIMSIR_SIMO_CLKDIV_DIV32 << MXC_F_TRIMSIR_SIMO_CLKDIV_POS) /**< SIMO_CLKDIV_DIV32 Setting */
#define MXC_V_TRIMSIR_SIMO_CLKDIV_DIV64                ((uint32_t)0x5UL) /**< SIMO_CLKDIV_DIV64 Value */
#define MXC_S_TRIMSIR_SIMO_CLKDIV_DIV64                (MXC_V_TRIMSIR_SIMO_CLKDIV_DIV64 << MXC_F_TRIMSIR_SIMO_CLKDIV_POS) /**< SIMO_CLKDIV_DIV64 Setting */
#define MXC_V_TRIMSIR_SIMO_CLKDIV_DIV128               ((uint32_t)0x7UL) /**< SIMO_CLKDIV_DIV128 Value */
#define MXC_S_TRIMSIR_SIMO_CLKDIV_DIV128               (MXC_V_TRIMSIR_SIMO_CLKDIV_DIV128 << MXC_F_TRIMSIR_SIMO_CLKDIV_POS) /**< SIMO_CLKDIV_DIV128 Setting */

/**@} end of group TRIMSIR_SIMO_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_IPOLO TRIMSIR_IPOLO
 * @brief    IPO Low Trim System Initialization Register.
 * @{
 */
#define MXC_F_TRIMSIR_IPOLO_IPO_LIMITLO_POS            0 /**< IPOLO_IPO_LIMITLO Position */
#define MXC_F_TRIMSIR_IPOLO_IPO_LIMITLO                ((uint32_t)(0xFFUL << MXC_F_TRIMSIR_IPOLO_IPO_LIMITLO_POS)) /**< IPOLO_IPO_LIMITLO Mask */

/**@} end of group TRIMSIR_IPOLO_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_CTRL TRIMSIR_CTRL
 * @brief    Control Trim System Initialization Register.
 * @{
 */
#define MXC_F_TRIMSIR_CTRL_VDDA_LIMITLO_POS            0 /**< CTRL_VDDA_LIMITLO Position */
#define MXC_F_TRIMSIR_CTRL_VDDA_LIMITLO                ((uint32_t)(0x7FUL << MXC_F_TRIMSIR_CTRL_VDDA_LIMITLO_POS)) /**< CTRL_VDDA_LIMITLO Mask */

#define MXC_F_TRIMSIR_CTRL_VDDA_LIMITHI_POS            8 /**< CTRL_VDDA_LIMITHI Position */
#define MXC_F_TRIMSIR_CTRL_VDDA_LIMITHI                ((uint32_t)(0x7FUL << MXC_F_TRIMSIR_CTRL_VDDA_LIMITHI_POS)) /**< CTRL_VDDA_LIMITHI Mask */

#define MXC_F_TRIMSIR_CTRL_IPO_LIMITHI_POS             15 /**< CTRL_IPO_LIMITHI Position */
#define MXC_F_TRIMSIR_CTRL_IPO_LIMITHI                 ((uint32_t)(0x1FFUL << MXC_F_TRIMSIR_CTRL_IPO_LIMITHI_POS)) /**< CTRL_IPO_LIMITHI Mask */

#define MXC_F_TRIMSIR_CTRL_INRO_SEL_POS                24 /**< CTRL_INRO_SEL Position */
#define MXC_F_TRIMSIR_CTRL_INRO_SEL                    ((uint32_t)(0x3UL << MXC_F_TRIMSIR_CTRL_INRO_SEL_POS)) /**< CTRL_INRO_SEL Mask */
#define MXC_V_TRIMSIR_CTRL_INRO_SEL_8KHZ               ((uint32_t)0x0UL) /**< CTRL_INRO_SEL_8KHZ Value */
#define MXC_S_TRIMSIR_CTRL_INRO_SEL_8KHZ               (MXC_V_TRIMSIR_CTRL_INRO_SEL_8KHZ << MXC_F_TRIMSIR_CTRL_INRO_SEL_POS) /**< CTRL_INRO_SEL_8KHZ Setting */
#define MXC_V_TRIMSIR_CTRL_INRO_SEL_16KHZ              ((uint32_t)0x1UL) /**< CTRL_INRO_SEL_16KHZ Value */
#define MXC_S_TRIMSIR_CTRL_INRO_SEL_16KHZ              (MXC_V_TRIMSIR_CTRL_INRO_SEL_16KHZ << MXC_F_TRIMSIR_CTRL_INRO_SEL_POS) /**< CTRL_INRO_SEL_16KHZ Setting */
#define MXC_V_TRIMSIR_CTRL_INRO_SEL_30KHZ              ((uint32_t)0x2UL) /**< CTRL_INRO_SEL_30KHZ Value */
#define MXC_S_TRIMSIR_CTRL_INRO_SEL_30KHZ              (MXC_V_TRIMSIR_CTRL_INRO_SEL_30KHZ << MXC_F_TRIMSIR_CTRL_INRO_SEL_POS) /**< CTRL_INRO_SEL_30KHZ Setting */

#define MXC_F_TRIMSIR_CTRL_INRO_TRIM_POS               29 /**< CTRL_INRO_TRIM Position */
#define MXC_F_TRIMSIR_CTRL_INRO_TRIM                   ((uint32_t)(0x7UL << MXC_F_TRIMSIR_CTRL_INRO_TRIM_POS)) /**< CTRL_INRO_TRIM Mask */

/**@} end of group TRIMSIR_CTRL_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_INRO TRIMSIR_INRO
 * @brief    RTC Trim System Initialization Register.
 * @{
 */
#define MXC_F_TRIMSIR_INRO_TRIM16K_POS                 0 /**< INRO_TRIM16K Position */
#define MXC_F_TRIMSIR_INRO_TRIM16K                     ((uint32_t)(0x7UL << MXC_F_TRIMSIR_INRO_TRIM16K_POS)) /**< INRO_TRIM16K Mask */

#define MXC_F_TRIMSIR_INRO_TRIM30K_POS                 3 /**< INRO_TRIM30K Position */
#define MXC_F_TRIMSIR_INRO_TRIM30K                     ((uint32_t)(0x7UL << MXC_F_TRIMSIR_INRO_TRIM30K_POS)) /**< INRO_TRIM30K Mask */

#define MXC_F_TRIMSIR_INRO_LPCLKSEL_POS                6 /**< INRO_LPCLKSEL Position */
#define MXC_F_TRIMSIR_INRO_LPCLKSEL                    ((uint32_t)(0x3UL << MXC_F_TRIMSIR_INRO_LPCLKSEL_POS)) /**< INRO_LPCLKSEL Mask */
#define MXC_V_TRIMSIR_INRO_LPCLKSEL_8KHZ               ((uint32_t)0x0UL) /**< INRO_LPCLKSEL_8KHZ Value */
#define MXC_S_TRIMSIR_INRO_LPCLKSEL_8KHZ               (MXC_V_TRIMSIR_INRO_LPCLKSEL_8KHZ << MXC_F_TRIMSIR_INRO_LPCLKSEL_POS) /**< INRO_LPCLKSEL_8KHZ Setting */
#define MXC_V_TRIMSIR_INRO_LPCLKSEL_16KHZ              ((uint32_t)0x1UL) /**< INRO_LPCLKSEL_16KHZ Value */
#define MXC_S_TRIMSIR_INRO_LPCLKSEL_16KHZ              (MXC_V_TRIMSIR_INRO_LPCLKSEL_16KHZ << MXC_F_TRIMSIR_INRO_LPCLKSEL_POS) /**< INRO_LPCLKSEL_16KHZ Setting */
#define MXC_V_TRIMSIR_INRO_LPCLKSEL_30KHZ              ((uint32_t)0x2UL) /**< INRO_LPCLKSEL_30KHZ Value */
#define MXC_S_TRIMSIR_INRO_LPCLKSEL_30KHZ              (MXC_V_TRIMSIR_INRO_LPCLKSEL_30KHZ << MXC_F_TRIMSIR_INRO_LPCLKSEL_POS) /**< INRO_LPCLKSEL_30KHZ Setting */

/**@} end of group TRIMSIR_INRO_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_TRIMSIR_REGS_H_

/**
 * @file    trimsir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @note    This file is @generated.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_TRIMSIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_TRIMSIR_REGS_H_

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
    __IO uint32_t bb_sir2;              /**< <tt>\b 0x08:</tt> TRIMSIR BB_SIR2 Register */
    __IO uint32_t bb_sir3;              /**< <tt>\b 0x0C:</tt> TRIMSIR BB_SIR3 Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __I  uint32_t bb_sir6;              /**< <tt>\b 0x18:</tt> TRIMSIR BB_SIR6 Register */
} mxc_trimsir_regs_t;

/* Register offsets for module TRIMSIR */
/**
 * @ingroup    trimsir_registers
 * @defgroup   TRIMSIR_Register_Offsets Register Offsets
 * @brief      TRIMSIR Peripheral Register Offsets from the TRIMSIR Base Peripheral Address.
 * @{
 */
#define MXC_R_TRIMSIR_BB_SIR2              ((uint32_t)0x00000008UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0008</tt> */
#define MXC_R_TRIMSIR_BB_SIR3              ((uint32_t)0x0000000CUL) /**< Offset from TRIMSIR Base Address: <tt> 0x000C</tt> */
#define MXC_R_TRIMSIR_BB_SIR6              ((uint32_t)0x00000018UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0018</tt> */
/**@} end of group trimsir_registers */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_BB_SIR2 TRIMSIR_BB_SIR2
 * @brief    System Init. Configuration Register 2.
 * @{
 */
#define MXC_F_TRIMSIR_BB_SIR2_TRIM_IBRO_RBIAS_POS      0 /**< BB_SIR2_TRIM_IBRO_RBIAS Position */
#define MXC_F_TRIMSIR_BB_SIR2_TRIM_IBRO_RBIAS          ((uint32_t)(0x3FUL << MXC_F_TRIMSIR_BB_SIR2_TRIM_IBRO_RBIAS_POS)) /**< BB_SIR2_TRIM_IBRO_RBIAS Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM0_1ECCEN_POS          8 /**< BB_SIR2_RAM0_1ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM0_1ECCEN              ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM0_1ECCEN_POS)) /**< BB_SIR2_RAM0_1ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM2ECCEN_POS            9 /**< BB_SIR2_RAM2ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM2ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM2ECCEN_POS)) /**< BB_SIR2_RAM2ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM3ECCEN_POS            10 /**< BB_SIR2_RAM3ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM3ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM3ECCEN_POS)) /**< BB_SIR2_RAM3ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_ICC0ECCEN_POS            11 /**< BB_SIR2_ICC0ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_ICC0ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_ICC0ECCEN_POS)) /**< BB_SIR2_ICC0ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN_POS             12 /**< BB_SIR2_FL0ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN                 ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN_POS)) /**< BB_SIR2_FL0ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_FL1ECCEN_POS             13 /**< BB_SIR2_FL1ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_FL1ECCEN                 ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_FL1ECCEN_POS)) /**< BB_SIR2_FL1ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_TRIM_IBRO_POS            16 /**< BB_SIR2_TRIM_IBRO Position */
#define MXC_F_TRIMSIR_BB_SIR2_TRIM_IBRO                ((uint32_t)(0xFFFFUL << MXC_F_TRIMSIR_BB_SIR2_TRIM_IBRO_POS)) /**< BB_SIR2_TRIM_IBRO Mask */

/**@} end of group TRIMSIR_BB_SIR2_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_BB_SIR6 TRIMSIR_BB_SIR6
 * @brief    System Init. Configuration Register 6.
 * @{
 */
#define MXC_F_TRIMSIR_BB_SIR6_RTCX1TRIM_POS            4 /**< BB_SIR6_RTCX1TRIM Position */
#define MXC_F_TRIMSIR_BB_SIR6_RTCX1TRIM                ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_BB_SIR6_RTCX1TRIM_POS)) /**< BB_SIR6_RTCX1TRIM Mask */

#define MXC_F_TRIMSIR_BB_SIR6_RTCX2TRIM_POS            9 /**< BB_SIR6_RTCX2TRIM Position */
#define MXC_F_TRIMSIR_BB_SIR6_RTCX2TRIM                ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_BB_SIR6_RTCX2TRIM_POS)) /**< BB_SIR6_RTCX2TRIM Mask */

/**@} end of group TRIMSIR_BB_SIR6_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_TRIMSIR_REGS_H_

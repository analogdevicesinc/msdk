/**
 * @file    msr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MSR Peripheral Module.
 */

/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *
 *************************************************************************** */

#ifndef _MSR_REGS_H_
#define _MSR_REGS_H_

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
 * @ingroup     msr
 * @defgroup    msr_registers MSR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MSR Peripheral Module.
 * @details Magnetic Stripe Reader
 */

/**
 * @ingroup msr_registers
 * Structure type to access the MSR Registers.
 */
typedef struct {
    __IO uint32_t dspaddr;              /**< <tt>\b 0x00:</tt> MSR DSPADDR Register */
    __IO uint32_t dspctrl;              /**< <tt>\b 0x04:</tt> MSR DSPCTRL Register */
    __IO uint32_t dspdata;              /**< <tt>\b 0x08:</tt> MSR DSPDATA Register */
    __I  uint32_t t%sfifo[3];           /**< <tt>\b 0x0C:</tt> MSR T%SFIFO Register */
} mxc_msr_regs_t;

/* Register offsets for module MSR */
/**
 * @ingroup    msr_registers
 * @defgroup   MSR_Register_Offsets Register Offsets
 * @brief      MSR Peripheral Register Offsets from the MSR Base Peripheral Address.
 * @{
 */
 #define MXC_R_MSR_DSPADDR                  ((uint32_t)0x00000000UL) /**< Offset from MSR Base Address: <tt> 0x0000</tt> */
 #define MXC_R_MSR_DSPCTRL                  ((uint32_t)0x00000004UL) /**< Offset from MSR Base Address: <tt> 0x0004</tt> */
 #define MXC_R_MSR_DSPDATA                  ((uint32_t)0x00000008UL) /**< Offset from MSR Base Address: <tt> 0x0008</tt> */
 #define MXC_R_MSR_T%SFIFO                  ((uint32_t)0x0000000CUL) /**< Offset from MSR Base Address: <tt> 0x000C</tt> */
/**@} end of group msr_registers */

/**
 * @ingroup  msr_registers
 * @defgroup MSR_DSPADDR MSR_DSPADDR
 * @brief    MSR Indirect Address Register Access. Indirectly access MSR settings registers
 *           by writing the index to this register. The register values are accessible by
 *           reading the DSPDATA register.
 * @{
 */
 #define MXC_F_MSR_DSPADDR_ADDR_POS                     0 /**< DSPADDR_ADDR Position */
 #define MXC_F_MSR_DSPADDR_ADDR                         ((uint32_t)(0xFFUL << MXC_F_MSR_DSPADDR_ADDR_POS)) /**< DSPADDR_ADDR Mask */
 #define MXC_V_MSR_DSPADDR_ADDR_T13SCALE                ((uint32_t)0x0UL) /**< DSPADDR_ADDR_T13SCALE Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T13SCALE                (MXC_V_MSR_DSPADDR_ADDR_T13SCALE << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T13SCALE Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T2SCALE                 ((uint32_t)0x1UL) /**< DSPADDR_ADDR_T2SCALE Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T2SCALE                 (MXC_V_MSR_DSPADDR_ADDR_T2SCALE << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T2SCALE Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T13ZCTFAST              ((uint32_t)0x2UL) /**< DSPADDR_ADDR_T13ZCTFAST Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T13ZCTFAST              (MXC_V_MSR_DSPADDR_ADDR_T13ZCTFAST << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T13ZCTFAST Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T13ZCTMID               ((uint32_t)0x3UL) /**< DSPADDR_ADDR_T13ZCTMID Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T13ZCTMID               (MXC_V_MSR_DSPADDR_ADDR_T13ZCTMID << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T13ZCTMID Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T13ZCTSLOW              ((uint32_t)0x4UL) /**< DSPADDR_ADDR_T13ZCTSLOW Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T13ZCTSLOW              (MXC_V_MSR_DSPADDR_ADDR_T13ZCTSLOW << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T13ZCTSLOW Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T2ZCTFAST               ((uint32_t)0x5UL) /**< DSPADDR_ADDR_T2ZCTFAST Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T2ZCTFAST               (MXC_V_MSR_DSPADDR_ADDR_T2ZCTFAST << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T2ZCTFAST Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T2ZCTMID                ((uint32_t)0x6UL) /**< DSPADDR_ADDR_T2ZCTMID Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T2ZCTMID                (MXC_V_MSR_DSPADDR_ADDR_T2ZCTMID << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T2ZCTMID Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T2ZCTSLOW               ((uint32_t)0x7UL) /**< DSPADDR_ADDR_T2ZCTSLOW Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T2ZCTSLOW               (MXC_V_MSR_DSPADDR_ADDR_T2ZCTSLOW << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T2ZCTSLOW Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_STARTPCNT               ((uint32_t)0x8UL) /**< DSPADDR_ADDR_STARTPCNT Value */
 #define MXC_S_MSR_DSPADDR_ADDR_STARTPCNT               (MXC_V_MSR_DSPADDR_ADDR_STARTPCNT << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_STARTPCNT Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T1DATACNT               ((uint32_t)0x9UL) /**< DSPADDR_ADDR_T1DATACNT Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T1DATACNT               (MXC_V_MSR_DSPADDR_ADDR_T1DATACNT << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T1DATACNT Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T2DATACNT               ((uint32_t)0xAUL) /**< DSPADDR_ADDR_T2DATACNT Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T2DATACNT               (MXC_V_MSR_DSPADDR_ADDR_T2DATACNT << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T2DATACNT Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_T3DATACNT               ((uint32_t)0xBUL) /**< DSPADDR_ADDR_T3DATACNT Value */
 #define MXC_S_MSR_DSPADDR_ADDR_T3DATACNT               (MXC_V_MSR_DSPADDR_ADDR_T3DATACNT << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_T3DATACNT Setting */
 #define MXC_V_MSR_DSPADDR_ADDR_ADCCFG                  ((uint32_t)0x12UL) /**< DSPADDR_ADDR_ADCCFG Value */
 #define MXC_S_MSR_DSPADDR_ADDR_ADCCFG                  (MXC_V_MSR_DSPADDR_ADDR_ADCCFG << MXC_F_MSR_DSPADDR_ADDR_POS) /**< DSPADDR_ADDR_ADCCFG Setting */

/**@} end of group MSR_DSPADDR_Register */

/**
 * @ingroup  msr_registers
 * @defgroup MSR_DSPCTRL MSR_DSPCTRL
 * @brief    MSR DSP Control Register
 * @{
 */
 #define MXC_F_MSR_DSPCTRL_DSPEN_POS                    0 /**< DSPCTRL_DSPEN Position */
 #define MXC_F_MSR_DSPCTRL_DSPEN                        ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_DSPEN_POS)) /**< DSPCTRL_DSPEN Mask */

 #define MXC_F_MSR_DSPCTRL_DSP_PKDETECT_POS             1 /**< DSPCTRL_DSP_PKDETECT Position */
 #define MXC_F_MSR_DSPCTRL_DSP_PKDETECT                 ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_DSP_PKDETECT_POS)) /**< DSPCTRL_DSP_PKDETECT Mask */

 #define MXC_F_MSR_DSPCTRL_FIFO_INT_EN_POS              2 /**< DSPCTRL_FIFO_INT_EN Position */
 #define MXC_F_MSR_DSPCTRL_FIFO_INT_EN                  ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_FIFO_INT_EN_POS)) /**< DSPCTRL_FIFO_INT_EN Mask */

 #define MXC_F_MSR_DSPCTRL_DSP_INTF_POS                 4 /**< DSPCTRL_DSP_INTF Position */
 #define MXC_F_MSR_DSPCTRL_DSP_INTF                     ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_DSP_INTF_POS)) /**< DSPCTRL_DSP_INTF Mask */

 #define MXC_F_MSR_DSPCTRL_T1_INTF_POS                  5 /**< DSPCTRL_T1_INTF Position */
 #define MXC_F_MSR_DSPCTRL_T1_INTF                      ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_T1_INTF_POS)) /**< DSPCTRL_T1_INTF Mask */

 #define MXC_F_MSR_DSPCTRL_T2_INTF_POS                  6 /**< DSPCTRL_T2_INTF Position */
 #define MXC_F_MSR_DSPCTRL_T2_INTF                      ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_T2_INTF_POS)) /**< DSPCTRL_T2_INTF Mask */

 #define MXC_F_MSR_DSPCTRL_T3_INTF_POS                  7 /**< DSPCTRL_T3_INTF Position */
 #define MXC_F_MSR_DSPCTRL_T3_INTF                      ((uint32_t)(0x1UL << MXC_F_MSR_DSPCTRL_T3_INTF_POS)) /**< DSPCTRL_T3_INTF Mask */

 #define MXC_F_MSR_DSPCTRL_DIV_RATIO_POS                8 /**< DSPCTRL_DIV_RATIO Position */
 #define MXC_F_MSR_DSPCTRL_DIV_RATIO                    ((uint32_t)(0x7FUL << MXC_F_MSR_DSPCTRL_DIV_RATIO_POS)) /**< DSPCTRL_DIV_RATIO Mask */

/**@} end of group MSR_DSPCTRL_Register */

/**
 * @ingroup  msr_registers
 * @defgroup MSR_T%SFIFO MSR_T%SFIFO
 * @brief    Track FIFO Data Output Register
 * @{
 */
 #define MXC_F_MSR_T%SFIFO_TNDATA_POS                   0 /**< T%SFIFO_TNDATA Position */
 #define MXC_F_MSR_T%SFIFO_TNDATA                       ((uint32_t)(0xFFFFUL << MXC_F_MSR_T%SFIFO_TNDATA_POS)) /**< T%SFIFO_TNDATA Mask */

/**@} end of group MSR_T%SFIFO_Register */

#ifdef __cplusplus
}
#endif

#endif /* _MSR_REGS_H_ */

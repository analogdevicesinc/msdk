/**
 * @file    fcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
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

#ifndef _FCR_REGS_H_
#define _FCR_REGS_H_

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
 * @ingroup     fcr
 * @defgroup    fcr_registers FCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
 * @details Function Control.
 */

/**
 * @ingroup fcr_registers
 * Structure type to access the FCR Registers.
 */
typedef struct {
    __IO uint32_t fcr;                  /**< <tt>\b 0x00:</tt> FCR FCR Register */
} mxc_fcr_regs_t;

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCR FCR_FCR
 * @brief    Register 0.
 * @{
 */
 #define MXC_F_FCR_FCR_USB_CLK_SEL_POS                  16 /**< FCR_USB_CLK_SEL Position */
 #define MXC_F_FCR_FCR_USB_CLK_SEL                      ((uint32_t)(0x1UL << MXC_F_FCR_FCR_USB_CLK_SEL_POS)) /**< FCR_USB_CLK_SEL Mask */

 #define MXC_F_FCR_FCR_QSPI0_FNC_SEL_POS                17 /**< FCR_QSPI0_FNC_SEL Position */
 #define MXC_F_FCR_FCR_QSPI0_FNC_SEL                    ((uint32_t)(0x1UL << MXC_F_FCR_FCR_QSPI0_FNC_SEL_POS)) /**< FCR_QSPI0_FNC_SEL Mask */

 #define MXC_F_FCR_FCR_I2C0_SDA_FILTER_EN_POS           20 /**< FCR_I2C0_SDA_FILTER_EN Position */
 #define MXC_F_FCR_FCR_I2C0_SDA_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C0_SDA_FILTER_EN_POS)) /**< FCR_I2C0_SDA_FILTER_EN Mask */

 #define MXC_F_FCR_FCR_I2C0_SCL_FILTER_EN_POS           21 /**< FCR_I2C0_SCL_FILTER_EN Position */
 #define MXC_F_FCR_FCR_I2C0_SCL_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C0_SCL_FILTER_EN_POS)) /**< FCR_I2C0_SCL_FILTER_EN Mask */

 #define MXC_F_FCR_FCR_I2C1_SDA_FILTER_EN_POS           22 /**< FCR_I2C1_SDA_FILTER_EN Position */
 #define MXC_F_FCR_FCR_I2C1_SDA_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C1_SDA_FILTER_EN_POS)) /**< FCR_I2C1_SDA_FILTER_EN Mask */

 #define MXC_F_FCR_FCR_I2C1_SCL_FILTER_EN_POS           23 /**< FCR_I2C1_SCL_FILTER_EN Position */
 #define MXC_F_FCR_FCR_I2C1_SCL_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C1_SCL_FILTER_EN_POS)) /**< FCR_I2C1_SCL_FILTER_EN Mask */

/**@} end of group FCR_FCR_Register */

#ifdef __cplusplus
}
#endif

#endif /* _FCR_REGS_H_ */

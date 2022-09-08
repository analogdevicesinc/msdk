/**
 * @file    fcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
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
 * @details Function Control Register.
 */

/**
 * @ingroup fcr_registers
 * Structure type to access the FCR Registers.
 */
typedef struct {
    __IO uint32_t fctrl0;               /**< <tt>\b 0x00:</tt> FCR FCTRL0 Register */
    __IO uint32_t fctrl1;               /**< <tt>\b 0x04:</tt> FCR FCTRL1 Register */
    __IO uint32_t fctrl2;               /**< <tt>\b 0x08:</tt> FCR FCTRL2 Register */
    __IO uint32_t fctrl3;               /**< <tt>\b 0x0C:</tt> FCR FCTRL3 Register */
    __IO uint32_t urvbootaddr;          /**< <tt>\b 0x10:</tt> FCR URVBOOTADDR Register */
    __IO uint32_t urvctrl;              /**< <tt>\b 0x14:</tt> FCR URVCTRL Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t gp;                   /**< <tt>\b 0x1C:</tt> FCR GP Register */
} mxc_fcr_regs_t;

/* Register offsets for module FCR */
/**
 * @ingroup    fcr_registers
 * @defgroup   FCR_Register_Offsets Register Offsets
 * @brief      FCR Peripheral Register Offsets from the FCR Base Peripheral Address.
 * @{
 */
 #define MXC_R_FCR_FCTRL0                   ((uint32_t)0x00000000UL) /**< Offset from FCR Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_FCR_FCTRL1                   ((uint32_t)0x00000004UL) /**< Offset from FCR Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_FCR_FCTRL2                   ((uint32_t)0x00000008UL) /**< Offset from FCR Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_FCR_FCTRL3                   ((uint32_t)0x0000000CUL) /**< Offset from FCR Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_FCR_URVBOOTADDR              ((uint32_t)0x00000010UL) /**< Offset from FCR Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_FCR_URVCTRL                  ((uint32_t)0x00000014UL) /**< Offset from FCR Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_FCR_GP                       ((uint32_t)0x0000001CUL) /**< Offset from FCR Base Address: <tt> 0x001C</tt> */ 
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL0 FCR_FCTRL0
 * @brief    Register 0.
 * @{
 */
 #define MXC_F_FCR_FCTRL0_USBCLKSEL_POS                 16 /**< FCTRL0_USBCLKSEL Position */
 #define MXC_F_FCR_FCTRL0_USBCLKSEL                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_USBCLKSEL_POS)) /**< FCTRL0_USBCLKSEL Mask */

 #define MXC_F_FCR_FCTRL0_I2C0DGEN0_POS                 20 /**< FCTRL0_I2C0DGEN0 Position */
 #define MXC_F_FCR_FCTRL0_I2C0DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN0_POS)) /**< FCTRL0_I2C0DGEN0 Mask */

 #define MXC_F_FCR_FCTRL0_I2C0DGEN1_POS                 21 /**< FCTRL0_I2C0DGEN1 Position */
 #define MXC_F_FCR_FCTRL0_I2C0DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN1_POS)) /**< FCTRL0_I2C0DGEN1 Mask */

 #define MXC_F_FCR_FCTRL0_I2C1DGEN0_POS                 22 /**< FCTRL0_I2C1DGEN0 Position */
 #define MXC_F_FCR_FCTRL0_I2C1DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN0_POS)) /**< FCTRL0_I2C1DGEN0 Mask */

 #define MXC_F_FCR_FCTRL0_I2C1DGEN1_POS                 23 /**< FCTRL0_I2C1DGEN1 Position */
 #define MXC_F_FCR_FCTRL0_I2C1DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN1_POS)) /**< FCTRL0_I2C1DGEN1 Mask */

/**@} end of group FCR_FCTRL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL1 FCR_FCTRL1
 * @brief    Register 1.
 * @{
 */
 #define MXC_F_FCR_FCTRL1_ACEN_POS                      0 /**< FCTRL1_ACEN Position */
 #define MXC_F_FCR_FCTRL1_ACEN                          ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_ACEN_POS)) /**< FCTRL1_ACEN Mask */

 #define MXC_F_FCR_FCTRL1_ACRUN_POS                     1 /**< FCTRL1_ACRUN Position */
 #define MXC_F_FCR_FCTRL1_ACRUN                         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_ACRUN_POS)) /**< FCTRL1_ACRUN Mask */

 #define MXC_F_FCR_FCTRL1_LOAD_POS                      2 /**< FCTRL1_LOAD Position */
 #define MXC_F_FCR_FCTRL1_LOAD                          ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_LOAD_POS)) /**< FCTRL1_LOAD Mask */

 #define MXC_F_FCR_FCTRL1_INV_GAIN_POS                  3 /**< FCTRL1_INV_GAIN Position */
 #define MXC_F_FCR_FCTRL1_INV_GAIN                      ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_INV_GAIN_POS)) /**< FCTRL1_INV_GAIN Mask */

 #define MXC_F_FCR_FCTRL1_ATOMIC_POS                    4 /**< FCTRL1_ATOMIC Position */
 #define MXC_F_FCR_FCTRL1_ATOMIC                        ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_ATOMIC_POS)) /**< FCTRL1_ATOMIC Mask */

 #define MXC_F_FCR_FCTRL1_MU_POS                        8 /**< FCTRL1_MU Position */
 #define MXC_F_FCR_FCTRL1_MU                            ((uint32_t)(0xFFFUL << MXC_F_FCR_FCTRL1_MU_POS)) /**< FCTRL1_MU Mask */

 #define MXC_F_FCR_FCTRL1_TRIM_POS                      23 /**< FCTRL1_TRIM Position */
 #define MXC_F_FCR_FCTRL1_TRIM                          ((uint32_t)(0x1FFUL << MXC_F_FCR_FCTRL1_TRIM_POS)) /**< FCTRL1_TRIM Mask */

/**@} end of group FCR_FCTRL1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL2 FCR_FCTRL2
 * @brief    Register 2.
 * @{
 */
 #define MXC_F_FCR_FCTRL2_XO_EN_DGL_POS                 3 /**< FCTRL2_XO_EN_DGL Position */
 #define MXC_F_FCR_FCTRL2_XO_EN_DGL                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL2_XO_EN_DGL_POS)) /**< FCTRL2_XO_EN_DGL Mask */

/**@} end of group FCR_FCTRL2_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL3 FCR_FCTRL3
 * @brief    Register 3.
 * @{
 */
 #define MXC_F_FCR_FCTRL3_DONECNT_POS                   0 /**< FCTRL3_DONECNT Position */
 #define MXC_F_FCR_FCTRL3_DONECNT                       ((uint32_t)(0xFFUL << MXC_F_FCR_FCTRL3_DONECNT_POS)) /**< FCTRL3_DONECNT Mask */

/**@} end of group FCR_FCTRL3_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_URVBOOTADDR FCR_URVBOOTADDR
 * @brief    Register 4.
 * @{
 */
 #define MXC_F_FCR_URVBOOTADDR_BOOTADDR_POS             0 /**< URVBOOTADDR_BOOTADDR Position */
 #define MXC_F_FCR_URVBOOTADDR_BOOTADDR                 ((uint32_t)(0xFFFFFFFFUL << MXC_F_FCR_URVBOOTADDR_BOOTADDR_POS)) /**< URVBOOTADDR_BOOTADDR Mask */

/**@} end of group FCR_URVBOOTADDR_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_URVCTRL FCR_URVCTRL
 * @brief    Register 5.
 * @{
 */
 #define MXC_F_FCR_URVCTRL_SLEEP_REQ_POS                0 /**< URVCTRL_SLEEP_REQ Position */
 #define MXC_F_FCR_URVCTRL_SLEEP_REQ                    ((uint32_t)(0x1UL << MXC_F_FCR_URVCTRL_SLEEP_REQ_POS)) /**< URVCTRL_SLEEP_REQ Mask */

 #define MXC_F_FCR_URVCTRL_SLEEP_ACK_POS                1 /**< URVCTRL_SLEEP_ACK Position */
 #define MXC_F_FCR_URVCTRL_SLEEP_ACK                    ((uint32_t)(0x1UL << MXC_F_FCR_URVCTRL_SLEEP_ACK_POS)) /**< URVCTRL_SLEEP_ACK Mask */

/**@} end of group FCR_URVCTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_GP FCR_GP
 * @brief    General Purpose Register.
 * @{
 */
 #define MXC_F_FCR_GP_GP_POS                            0 /**< GP_GP Position */
 #define MXC_F_FCR_GP_GP                                ((uint32_t)(0xFFFFFFFFUL << MXC_F_FCR_GP_GP_POS)) /**< GP_GP Mask */

/**@} end of group FCR_GP_Register */

#ifdef __cplusplus
}
#endif

#endif /* _FCR_REGS_H_ */

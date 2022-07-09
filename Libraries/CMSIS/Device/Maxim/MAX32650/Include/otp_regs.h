/**
 * @file    otp_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the OTP Peripheral Module.
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

#ifndef _OTP_REGS_H_
#define _OTP_REGS_H_

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
 * @ingroup     otp
 * @defgroup    otp_registers OTP_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the OTP Peripheral Module.
 * @details One Time Programmable Memory controller.
 */

/**
 * @ingroup otp_registers
 * Structure type to access the OTP Registers.
 */
typedef struct {
    __IO uint32_t ocntl;                /**< <tt>\b 0x00:</tt> OTP OCNTL Register */
    __IO uint32_t ckdiv;                /**< <tt>\b 0x04:</tt> OTP CKDIV Register */
    __IO uint32_t otprdata;             /**< <tt>\b 0x08:</tt> OTP OTPRDATA Register */
    __I  uint32_t stat;                 /**< <tt>\b 0x0C:</tt> OTP STAT Register */
    __R  uint32_t rsv_0x10_0x2f[8];
    __O  uint32_t odata;                /**< <tt>\b 0x30:</tt> OTP ODATA Register */
    __R  uint32_t rsv_0x34_0x3f[3];
    __IO uint32_t acntl;                /**< <tt>\b 0x40:</tt> OTP ACNTL Register */
} mxc_otp_regs_t;

/* Register offsets for module OTP */
/**
 * @ingroup    otp_registers
 * @defgroup   OTP_Register_Offsets Register Offsets
 * @brief      OTP Peripheral Register Offsets from the OTP Base Peripheral Address. 
 * @{
 */
 #define MXC_R_OTP_OCNTL                    ((uint32_t)0x00000000UL) /**< Offset from OTP Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_OTP_CKDIV                    ((uint32_t)0x00000004UL) /**< Offset from OTP Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_OTP_OTPRDATA                 ((uint32_t)0x00000008UL) /**< Offset from OTP Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_OTP_STAT                     ((uint32_t)0x0000000CUL) /**< Offset from OTP Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_OTP_ODATA                    ((uint32_t)0x00000030UL) /**< Offset from OTP Base Address: <tt> 0x0030</tt> */ 
 #define MXC_R_OTP_ACNTL                    ((uint32_t)0x00000040UL) /**< Offset from OTP Base Address: <tt> 0x0040</tt> */ 
/**@} end of group otp_registers */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_OCNTL OTP_OCNTL
 * @brief    OTP Control Register.
 * @{
 */
 #define MXC_F_OTP_OCNTL_PA_POS                         0 /**< OCNTL_PA Position */
 #define MXC_F_OTP_OCNTL_PA                             ((uint32_t)(0x7FUL << MXC_F_OTP_OCNTL_PA_POS)) /**< OCNTL_PA Mask */

 #define MXC_F_OTP_OCNTL_RD_OP_POS                      8 /**< OCNTL_RD_OP Position */
 #define MXC_F_OTP_OCNTL_RD_OP                          ((uint32_t)(0x1UL << MXC_F_OTP_OCNTL_RD_OP_POS)) /**< OCNTL_RD_OP Mask */
 #define MXC_V_OTP_OCNTL_RD_OP_NO_OPERATION             ((uint32_t)0x0UL) /**< OCNTL_RD_OP_NO_OPERATION Value */
 #define MXC_S_OTP_OCNTL_RD_OP_NO_OPERATION             (MXC_V_OTP_OCNTL_RD_OP_NO_OPERATION << MXC_F_OTP_OCNTL_RD_OP_POS) /**< OCNTL_RD_OP_NO_OPERATION Setting */
 #define MXC_V_OTP_OCNTL_RD_OP_INITIATE_READ_OP         ((uint32_t)0x1UL) /**< OCNTL_RD_OP_INITIATE_READ_OP Value */
 #define MXC_S_OTP_OCNTL_RD_OP_INITIATE_READ_OP         (MXC_V_OTP_OCNTL_RD_OP_INITIATE_READ_OP << MXC_F_OTP_OCNTL_RD_OP_POS) /**< OCNTL_RD_OP_INITIATE_READ_OP Setting */

 #define MXC_F_OTP_OCNTL_PROG_OP_POS                    9 /**< OCNTL_PROG_OP Position */
 #define MXC_F_OTP_OCNTL_PROG_OP                        ((uint32_t)(0x1UL << MXC_F_OTP_OCNTL_PROG_OP_POS)) /**< OCNTL_PROG_OP Mask */
 #define MXC_V_OTP_OCNTL_PROG_OP_NO_OPERATION           ((uint32_t)0x0UL) /**< OCNTL_PROG_OP_NO_OPERATION Value */
 #define MXC_S_OTP_OCNTL_PROG_OP_NO_OPERATION           (MXC_V_OTP_OCNTL_PROG_OP_NO_OPERATION << MXC_F_OTP_OCNTL_PROG_OP_POS) /**< OCNTL_PROG_OP_NO_OPERATION Setting */
 #define MXC_V_OTP_OCNTL_PROG_OP_INITIATE_WRITE_OP      ((uint32_t)0x1UL) /**< OCNTL_PROG_OP_INITIATE_WRITE_OP Value */
 #define MXC_S_OTP_OCNTL_PROG_OP_INITIATE_WRITE_OP      (MXC_V_OTP_OCNTL_PROG_OP_INITIATE_WRITE_OP << MXC_F_OTP_OCNTL_PROG_OP_POS) /**< OCNTL_PROG_OP_INITIATE_WRITE_OP Setting */

/**@} end of group OTP_OCNTL_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_CKDIV OTP_CKDIV
 * @brief    Clock Divider Register.
 * @{
 */
 #define MXC_F_OTP_CKDIV_CKDIV_POS                      0 /**< CKDIV_CKDIV Position */
 #define MXC_F_OTP_CKDIV_CKDIV                          ((uint32_t)(0x3UL << MXC_F_OTP_CKDIV_CKDIV_POS)) /**< CKDIV_CKDIV Mask */
 #define MXC_V_OTP_CKDIV_CKDIV_DIV_BY_2                 ((uint32_t)0x0UL) /**< CKDIV_CKDIV_DIV_BY_2 Value */
 #define MXC_S_OTP_CKDIV_CKDIV_DIV_BY_2                 (MXC_V_OTP_CKDIV_CKDIV_DIV_BY_2 << MXC_F_OTP_CKDIV_CKDIV_POS) /**< CKDIV_CKDIV_DIV_BY_2 Setting */
 #define MXC_V_OTP_CKDIV_CKDIV_DIV_BY_4                 ((uint32_t)0x1UL) /**< CKDIV_CKDIV_DIV_BY_4 Value */
 #define MXC_S_OTP_CKDIV_CKDIV_DIV_BY_4                 (MXC_V_OTP_CKDIV_CKDIV_DIV_BY_4 << MXC_F_OTP_CKDIV_CKDIV_POS) /**< CKDIV_CKDIV_DIV_BY_4 Setting */
 #define MXC_V_OTP_CKDIV_CKDIV_DIV_BY_8                 ((uint32_t)0x2UL) /**< CKDIV_CKDIV_DIV_BY_8 Value */
 #define MXC_S_OTP_CKDIV_CKDIV_DIV_BY_8                 (MXC_V_OTP_CKDIV_CKDIV_DIV_BY_8 << MXC_F_OTP_CKDIV_CKDIV_POS) /**< CKDIV_CKDIV_DIV_BY_8 Setting */
 #define MXC_V_OTP_CKDIV_CKDIV_DIV_BY_16                ((uint32_t)0x3UL) /**< CKDIV_CKDIV_DIV_BY_16 Value */
 #define MXC_S_OTP_CKDIV_CKDIV_DIV_BY_16                (MXC_V_OTP_CKDIV_CKDIV_DIV_BY_16 << MXC_F_OTP_CKDIV_CKDIV_POS) /**< CKDIV_CKDIV_DIV_BY_16 Setting */

/**@} end of group OTP_CKDIV_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_OTPRDATA OTP_OTPRDATA
 * @brief    Read Data Register.
 * @{
 */
 #define MXC_F_OTP_OTPRDATA_RDATA_POS                   0 /**< OTPRDATA_RDATA Position */
 #define MXC_F_OTP_OTPRDATA_RDATA                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_OTPRDATA_RDATA_POS)) /**< OTPRDATA_RDATA Mask */

/**@} end of group OTP_OTPRDATA_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_STAT OTP_STAT
 * @brief    OTP Status Register.
 * @{
 */
 #define MXC_F_OTP_STAT_BUSY_POS                        0 /**< STAT_BUSY Position */
 #define MXC_F_OTP_STAT_BUSY                            ((uint32_t)(0x1UL << MXC_F_OTP_STAT_BUSY_POS)) /**< STAT_BUSY Mask */
 #define MXC_V_OTP_STAT_BUSY_NOT_BUSY                   ((uint32_t)0x0UL) /**< STAT_BUSY_NOT_BUSY Value */
 #define MXC_S_OTP_STAT_BUSY_NOT_BUSY                   (MXC_V_OTP_STAT_BUSY_NOT_BUSY << MXC_F_OTP_STAT_BUSY_POS) /**< STAT_BUSY_NOT_BUSY Setting */
 #define MXC_V_OTP_STAT_BUSY_BUSY                       ((uint32_t)0x1UL) /**< STAT_BUSY_BUSY Value */
 #define MXC_S_OTP_STAT_BUSY_BUSY                       (MXC_V_OTP_STAT_BUSY_BUSY << MXC_F_OTP_STAT_BUSY_POS) /**< STAT_BUSY_BUSY Setting */

 #define MXC_F_OTP_STAT_FAIL_POS                        1 /**< STAT_FAIL Position */
 #define MXC_F_OTP_STAT_FAIL                            ((uint32_t)(0x1UL << MXC_F_OTP_STAT_FAIL_POS)) /**< STAT_FAIL Mask */
 #define MXC_V_OTP_STAT_FAIL_NFAIL                      ((uint32_t)0x0UL) /**< STAT_FAIL_NFAIL Value */
 #define MXC_S_OTP_STAT_FAIL_NFAIL                      (MXC_V_OTP_STAT_FAIL_NFAIL << MXC_F_OTP_STAT_FAIL_POS) /**< STAT_FAIL_NFAIL Setting */
 #define MXC_V_OTP_STAT_FAIL_FAIL                       ((uint32_t)0x1UL) /**< STAT_FAIL_FAIL Value */
 #define MXC_S_OTP_STAT_FAIL_FAIL                       (MXC_V_OTP_STAT_FAIL_FAIL << MXC_F_OTP_STAT_FAIL_POS) /**< STAT_FAIL_FAIL Setting */

/**@} end of group OTP_STAT_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_ODATA OTP_ODATA
 * @brief    Write Data Register.
 * @{
 */
 #define MXC_F_OTP_ODATA_OTPDATA_POS                    0 /**< ODATA_OTPDATA Position */
 #define MXC_F_OTP_ODATA_OTPDATA                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_ODATA_OTPDATA_POS)) /**< ODATA_OTPDATA Mask */

/**@} end of group OTP_ODATA_Register */

/**
 * @ingroup  otp_registers
 * @defgroup OTP_ACNTL OTP_ACNTL
 * @brief    Access Control Register.
 * @{
 */
 #define MXC_F_OTP_ACNTL_ADATA_POS                      0 /**< ACNTL_ADATA Position */
 #define MXC_F_OTP_ACNTL_ADATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_OTP_ACNTL_ADATA_POS)) /**< ACNTL_ADATA Mask */

/**@} end of group OTP_ACNTL_Register */

#ifdef __cplusplus
}
#endif

#endif /* _OTP_REGS_H_ */

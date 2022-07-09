/**
 * @file    emcc_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the EMCC_REVA Peripheral Module.
 */

/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef _EMCC_REVA_REGS_H_
#define _EMCC_REVA_REGS_H_

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
 * @ingroup     emcc_reva
 * @defgroup    emcc_reva_registers EMCC_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the EMCC_REVA Peripheral Module.
 * @details External Memory Cache Controller Registers.
 */

/**
 * @ingroup emcc_reva_registers
 * Structure type to access the EMCC_REVA Registers.
 */
typedef struct {
    __I  uint32_t info;                 /**< <tt>\b 0x0000:</tt> EMCC_REVA INFO Register */
    __I  uint32_t sz;                   /**< <tt>\b 0x0004:</tt> EMCC_REVA SZ Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0100:</tt> EMCC_REVA CTRL Register */
    __R  uint32_t rsv_0x104_0x6ff[383];
    __IO uint32_t invalidate;           /**< <tt>\b 0x0700:</tt> EMCC_REVA INVALIDATE Register */
} mxc_emcc_reva_regs_t;

/* Register offsets for module EMCC_REVA */
/**
 * @ingroup    emcc_reva_registers
 * @defgroup   EMCC_REVA_Register_Offsets Register Offsets
 * @brief      EMCC_REVA Peripheral Register Offsets from the EMCC_REVA Base Peripheral Address.
 * @{
 */
 #define MXC_R_EMCC_REVA_INFO               ((uint32_t)0x00000000UL) /**< Offset from EMCC_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_EMCC_REVA_SZ                 ((uint32_t)0x00000004UL) /**< Offset from EMCC_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_EMCC_REVA_CTRL               ((uint32_t)0x00000100UL) /**< Offset from EMCC_REVA Base Address: <tt> 0x0100</tt> */ 
 #define MXC_R_EMCC_REVA_INVALIDATE         ((uint32_t)0x00000700UL) /**< Offset from EMCC_REVA Base Address: <tt> 0x0700</tt> */ 
/**@} end of group emcc_reva_registers */

/**
 * @ingroup  emcc_reva_registers
 * @defgroup EMCC_REVA_INFO EMCC_REVA_INFO
 * @brief    Cache ID Register.
 * @{
 */
 #define MXC_F_EMCC_REVA_INFO_RELNUM_POS                0 /**< INFO_RELNUM Position */
 #define MXC_F_EMCC_REVA_INFO_RELNUM                    ((uint32_t)(0x3FUL << MXC_F_EMCC_REVA_INFO_RELNUM_POS)) /**< INFO_RELNUM Mask */

 #define MXC_F_EMCC_REVA_INFO_PARTNUM_POS               6 /**< INFO_PARTNUM Position */
 #define MXC_F_EMCC_REVA_INFO_PARTNUM                   ((uint32_t)(0xFUL << MXC_F_EMCC_REVA_INFO_PARTNUM_POS)) /**< INFO_PARTNUM Mask */

 #define MXC_F_EMCC_REVA_INFO_ID_POS                    10 /**< INFO_ID Position */
 #define MXC_F_EMCC_REVA_INFO_ID                        ((uint32_t)(0x3FUL << MXC_F_EMCC_REVA_INFO_ID_POS)) /**< INFO_ID Mask */

/**@} end of group EMCC_REVA_INFO_Register */

/**
 * @ingroup  emcc_reva_registers
 * @defgroup EMCC_REVA_SZ EMCC_REVA_SZ
 * @brief    Memory Configuration Register.
 * @{
 */
 #define MXC_F_EMCC_REVA_SZ_CCH_POS                     0 /**< SZ_CCH Position */
 #define MXC_F_EMCC_REVA_SZ_CCH                         ((uint32_t)(0xFFFFUL << MXC_F_EMCC_REVA_SZ_CCH_POS)) /**< SZ_CCH Mask */

 #define MXC_F_EMCC_REVA_SZ_MEM_POS                     16 /**< SZ_MEM Position */
 #define MXC_F_EMCC_REVA_SZ_MEM                         ((uint32_t)(0xFFFFUL << MXC_F_EMCC_REVA_SZ_MEM_POS)) /**< SZ_MEM Mask */

/**@} end of group EMCC_REVA_SZ_Register */

/**
 * @ingroup  emcc_reva_registers
 * @defgroup EMCC_REVA_CTRL EMCC_REVA_CTRL
 * @brief    Cache Control and Status Register.
 * @{
 */
 #define MXC_F_EMCC_REVA_CTRL_EN_POS                    0 /**< CTRL_EN Position */
 #define MXC_F_EMCC_REVA_CTRL_EN                        ((uint32_t)(0x1UL << MXC_F_EMCC_REVA_CTRL_EN_POS)) /**< CTRL_EN Mask */

 #define MXC_F_EMCC_REVA_CTRL_WRITE_ALLOC_POS           1 /**< CTRL_WRITE_ALLOC Position */
 #define MXC_F_EMCC_REVA_CTRL_WRITE_ALLOC               ((uint32_t)(0x1UL << MXC_F_EMCC_REVA_CTRL_WRITE_ALLOC_POS)) /**< CTRL_WRITE_ALLOC Mask */

 #define MXC_F_EMCC_REVA_CTRL_CWFST_DIS_POS             2 /**< CTRL_CWFST_DIS Position */
 #define MXC_F_EMCC_REVA_CTRL_CWFST_DIS                 ((uint32_t)(0x1UL << MXC_F_EMCC_REVA_CTRL_CWFST_DIS_POS)) /**< CTRL_CWFST_DIS Mask */

 #define MXC_F_EMCC_REVA_CTRL_RDY_POS                   16 /**< CTRL_RDY Position */
 #define MXC_F_EMCC_REVA_CTRL_RDY                       ((uint32_t)(0x1UL << MXC_F_EMCC_REVA_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

/**@} end of group EMCC_REVA_CTRL_Register */

/**
 * @ingroup  emcc_reva_registers
 * @defgroup EMCC_REVA_INVALIDATE EMCC_REVA_INVALIDATE
 * @brief    Invalidate All Cache Contents. Any time this register location is written
 *           (regardless of the data value), the cache controller immediately begins
 *           invalidating the entire contents of the cache memory. The cache will be in
 *           bypass mode until the invalidate operation is complete. System software can
 *           examine the Cache Ready bit (CACHE_CTRL.CACHE_RDY) to determine when the
 *           invalidate operation is complete. Note that it is not necessary to disable the
 *           cache controller prior to beginning this operation. Reads from this register
 *           always return 0.
 * @{
 */
 #define MXC_F_EMCC_REVA_INVALIDATE_IA_POS              0 /**< INVALIDATE_IA Position */
 #define MXC_F_EMCC_REVA_INVALIDATE_IA                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMCC_REVA_INVALIDATE_IA_POS)) /**< INVALIDATE_IA Mask */

/**@} end of group EMCC_REVA_INVALIDATE_Register */

#ifdef __cplusplus
}
#endif

#endif /* _EMCC_REVA_REGS_H_ */

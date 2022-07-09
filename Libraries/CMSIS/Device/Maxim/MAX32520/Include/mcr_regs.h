/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
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

#ifndef _MCR_REGS_H_
#define _MCR_REGS_H_

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
 * @details Misc Control.
 */

/**
 * @ingroup mcr_registers
 * Structure type to access the MCR Registers.
 */
typedef struct {
    __IO uint32_t eccen;                /**< <tt>\b 0x00:</tt> MCR ECCEN Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address. 
 * @{
 */
 #define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */ 
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ECCEN MCR_ECCEN
 * @brief    ECC Enable Register
 * @{
 */
 #define MXC_F_MCR_ECCEN_SYSRAM0ECCEN_POS               0 /**< ECCEN_SYSRAM0ECCEN Position */
 #define MXC_F_MCR_ECCEN_SYSRAM0ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM0ECCEN_POS)) /**< ECCEN_SYSRAM0ECCEN Mask */

 #define MXC_F_MCR_ECCEN_SYSRAM1ECCEN_POS               1 /**< ECCEN_SYSRAM1ECCEN Position */
 #define MXC_F_MCR_ECCEN_SYSRAM1ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM1ECCEN_POS)) /**< ECCEN_SYSRAM1ECCEN Mask */

 #define MXC_F_MCR_ECCEN_SYSRAM2ECCEN_POS               2 /**< ECCEN_SYSRAM2ECCEN Position */
 #define MXC_F_MCR_ECCEN_SYSRAM2ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM2ECCEN_POS)) /**< ECCEN_SYSRAM2ECCEN Mask */

 #define MXC_F_MCR_ECCEN_SYSRAM3ECCEN_POS               3 /**< ECCEN_SYSRAM3ECCEN Position */
 #define MXC_F_MCR_ECCEN_SYSRAM3ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM3ECCEN_POS)) /**< ECCEN_SYSRAM3ECCEN Mask */

 #define MXC_F_MCR_ECCEN_SYSRAM4ECCEN_POS               4 /**< ECCEN_SYSRAM4ECCEN Position */
 #define MXC_F_MCR_ECCEN_SYSRAM4ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM4ECCEN_POS)) /**< ECCEN_SYSRAM4ECCEN Mask */

 #define MXC_F_MCR_ECCEN_FL0ECCEN_POS                   11 /**< ECCEN_FL0ECCEN Position */
 #define MXC_F_MCR_ECCEN_FL0ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL0ECCEN_POS)) /**< ECCEN_FL0ECCEN Mask */

 #define MXC_F_MCR_ECCEN_FL1ECCEN_POS                   12 /**< ECCEN_FL1ECCEN Position */
 #define MXC_F_MCR_ECCEN_FL1ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL1ECCEN_POS)) /**< ECCEN_FL1ECCEN Mask */

/**@} end of group MCR_ECCEN_Register */

#ifdef __cplusplus
}
#endif

#endif /* _MCR_REGS_H_ */

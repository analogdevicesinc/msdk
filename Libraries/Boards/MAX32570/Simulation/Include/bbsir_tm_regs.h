/**
 * @file    bbsir_tm_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the BBSIR Peripheral Module.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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

#ifndef _BBSIR_TM_REGS_H_
#define _BBSIR_TM_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
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
 * @ingroup     bbsir
 * @defgroup    bbsir_registers Registers
 * @brief       Registers, Bit Masks and Bit Positions for the BBSIR Peripheral Module.
 * @description Battery Backed System Initialization Registers (For Simulation)
 */

/**
 * @ingroup bbsir_registers
 * Structure type to access the BBSIR Registers.
 */
typedef struct {
    __IO uint32_t bbconf0;              /**< <tt>\b 0x00:<\tt> BBSIR BBCONF0 Register */
    __IO uint32_t bbconf1;              /**< <tt>\b 0x04:<\tt> BBSIR BBCONF1 Register */
    __IO uint32_t bbshr2;               /**< <tt>\b 0x08:<\tt> BBSIR BBSHR2 Register */
    __IO uint32_t bbshr3;               /**< <tt>\b 0x0C:<\tt> BBSIR BBSHR3 Register */
    __IO uint32_t bbshr4;               /**< <tt>\b 0x10:<\tt> BBSIR BBSHR4 Register */
    __IO uint32_t bbshr5;               /**< <tt>\b 0x14:<\tt> BBSIR BBSHR5 Register */
    __IO uint32_t bbshr6;               /**< <tt>\b 0x18:<\tt> BBSIR BBSHR6 Register */
    __IO uint32_t bbshr7;               /**< <tt>\b 0x1C:<\tt> BBSIR BBSHR7 Register */
    __IO uint32_t bbshr8;               /**< <tt>\b 0x20:<\tt> BBSIR BBSHR8 Register */
    __IO uint32_t bbshr9;               /**< <tt>\b 0x24:<\tt> BBSIR BBSHR9 Register */
    __IO uint32_t bbshr10;              /**< <tt>\b 0x28:<\tt> BBSIR BBSHR10 Register */
    __IO uint32_t bbshr11;              /**< <tt>\b 0x2C:<\tt> BBSIR BBSHR11 Register */
} mxc_bbsir_tm_regs_t;

/*******************************************************************************/
/*                                                                       BBSIR */
#define MXC_BASE_BBSIR_TM                ((uint32_t)0x40005400UL)
#define MXC_BBSIR_TM                     ((mxc_bbsir_tm_regs_t*)MXC_BASE_BBSIR_TM)
#define MXC_BBSIR_TM_INSTANCES           (1)

#ifdef __cplusplus
}
#endif

#endif /* _BBSIR_TM_REGS_H_ */


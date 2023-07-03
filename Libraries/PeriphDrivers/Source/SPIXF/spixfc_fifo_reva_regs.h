/**
 * @file    spixfc_fifo_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFC_FIFO_REVA Peripheral Module.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef _SPIXFC_FIFO_REVA_REGS_H_
#define _SPIXFC_FIFO_REVA_REGS_H_

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
 * @ingroup     spixfc_fifo_reva
 * @defgroup    spixfc_fifo_reva_registers SPIXFC_FIFO_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFC_FIFO_REVA Peripheral Module.
 * @details SPI XiP Master Controller FIFO.
 */

/**
 * @ingroup spixfc_fifo_reva_registers
 * Structure type to access the SPIXFC_FIFO_REVA Registers.
 */
typedef struct {
  union{
    __IO uint8_t  tx_8;                 /**< <tt>\b 0x00:</tt> SPIXFC_FIFO_REVA TX_8 Register */
    __IO uint16_t tx_16;                /**< <tt>\b 0x00:</tt> SPIXFC_FIFO_REVA TX_16 Register */
    __IO uint32_t tx_32;                /**< <tt>\b 0x00:</tt> SPIXFC_FIFO_REVA TX_32 Register */
  };
  union{
    __IO uint8_t  rx_8;                 /**< <tt>\b 0x04:</tt> SPIXFC_FIFO_REVA RX_8 Register */
    __IO uint16_t rx_16;                /**< <tt>\b 0x04:</tt> SPIXFC_FIFO_REVA RX_16 Register */
    __IO uint32_t rx_32;                /**< <tt>\b 0x04:</tt> SPIXFC_FIFO_REVA RX_32 Register */
  };
} mxc_spixfc_fifo_reva_regs_t;

/* Register offsets for module SPIXFC_FIFO_REVA */
/**
 * @ingroup    spixfc_fifo_reva_registers
 * @defgroup   SPIXFC_FIFO_REVA_Register_Offsets Register Offsets
 * @brief      SPIXFC_FIFO_REVA Peripheral Register Offsets from the SPIXFC_FIFO_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_SPIXFC_FIFO_REVA_TX_8        ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_FIFO_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SPIXFC_FIFO_REVA_TX_16       ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_FIFO_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SPIXFC_FIFO_REVA_TX_32       ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_FIFO_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SPIXFC_FIFO_REVA_RX_8        ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_FIFO_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_SPIXFC_FIFO_REVA_RX_16       ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_FIFO_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_SPIXFC_FIFO_REVA_RX_32       ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_FIFO_REVA Base Address: <tt> 0x0004</tt> */ 
/**@} end of group spixfc_fifo_reva_registers */

#ifdef __cplusplus
}
#endif

#endif /* _SPIXFC_FIFO_REVA_REGS_H_ */

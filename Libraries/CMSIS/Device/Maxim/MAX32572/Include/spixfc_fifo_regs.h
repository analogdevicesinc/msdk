/**
 * @file    spixfc_fifo_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFC_FIFO Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixfc_fifo_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFC_FIFO_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFC_FIFO_REGS_H_

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
 * @ingroup     spixfc_fifo
 * @defgroup    spixfc_fifo_registers SPIXFC_FIFO_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFC_FIFO Peripheral Module.
 * @details     SPI XiP Master Controller FIFO.
 */

/**
 * @ingroup spixfc_fifo_registers
 * Structure type to access the SPIXFC_FIFO Registers.
 */
typedef struct {
    union {
        __IO uint8_t  tx_8;             /**< <tt>\b 0x00:</tt> SPIXFC_FIFO TX_8 Register */
        __IO uint16_t tx_16;            /**< <tt>\b 0x00:</tt> SPIXFC_FIFO TX_16 Register */
        __IO uint32_t tx_32;            /**< <tt>\b 0x00:</tt> SPIXFC_FIFO TX_32 Register */
    };
    union {
        __IO uint8_t  rx_8;             /**< <tt>\b 0x04:</tt> SPIXFC_FIFO RX_8 Register */
        __IO uint16_t rx_16;            /**< <tt>\b 0x04:</tt> SPIXFC_FIFO RX_16 Register */
        __IO uint32_t rx_32;            /**< <tt>\b 0x04:</tt> SPIXFC_FIFO RX_32 Register */
    };
} mxc_spixfc_fifo_regs_t;

/* Register offsets for module SPIXFC_FIFO */
/**
 * @ingroup    spixfc_fifo_registers
 * @defgroup   SPIXFC_FIFO_Register_Offsets Register Offsets
 * @brief      SPIXFC_FIFO Peripheral Register Offsets from the SPIXFC_FIFO Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXFC_FIFO_TX_8             ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_FIFO Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFC_FIFO_TX_16            ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_FIFO Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFC_FIFO_TX_32            ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_FIFO Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFC_FIFO_RX_8             ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_FIFO Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXFC_FIFO_RX_16            ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_FIFO Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXFC_FIFO_RX_32            ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_FIFO Base Address: <tt> 0x0004</tt> */
/**@} end of group spixfc_fifo_registers */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFC_FIFO_REGS_H_

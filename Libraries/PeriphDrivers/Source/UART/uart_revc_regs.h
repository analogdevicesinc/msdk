/**
 * @file    uart_revc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the UART_REVC Peripheral Module.
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

#ifndef _UART_REVC_REGS_H_
#define _UART_REVC_REGS_H_

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
 * @ingroup     uart_revc
 * @defgroup    uart_revc_registers UART_REVC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the UART_REVC Peripheral Module.
 * @details UART
 */

/**
 * @ingroup uart_revc_registers
 * Structure type to access the UART_REVC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> UART_REVC CTRL Register */
    __I  uint32_t status;               /**< <tt>\b 0x04:</tt> UART_REVC STATUS Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x08:</tt> UART_REVC INT_EN Register */
    __IO uint32_t int_fl;               /**< <tt>\b 0x0C:</tt> UART_REVC INT_FL Register */
    __IO uint32_t baud0;                /**< <tt>\b 0x10:</tt> UART_REVC BAUD0 Register */
    __IO uint32_t baud1;                /**< <tt>\b 0x14:</tt> UART_REVC BAUD1 Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t pin;                  /**< <tt>\b 0x1C:</tt> UART_REVC PIN Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x20:</tt> UART_REVC FIFO Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t dma;                  /**< <tt>\b 0x30:</tt> UART_REVC DMA Register */
} mxc_uart_revc_regs_t;

/* Register offsets for module UART_REVC */
/**
 * @ingroup    uart_revc_registers
 * @defgroup   UART_REVC_Register_Offsets Register Offsets
 * @brief      UART_REVC Peripheral Register Offsets from the UART_REVC Base Peripheral Address. 
 * @{
 */
 #define MXC_R_UART_REVC_CTRL               ((uint32_t)0x00000000UL) /**< Offset from UART_REVC Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_UART_REVC_STATUS             ((uint32_t)0x00000004UL) /**< Offset from UART_REVC Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_UART_REVC_INT_EN             ((uint32_t)0x00000008UL) /**< Offset from UART_REVC Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_UART_REVC_INT_FL             ((uint32_t)0x0000000CUL) /**< Offset from UART_REVC Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_UART_REVC_BAUD0              ((uint32_t)0x00000010UL) /**< Offset from UART_REVC Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_UART_REVC_BAUD1              ((uint32_t)0x00000014UL) /**< Offset from UART_REVC Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_UART_REVC_PIN                ((uint32_t)0x0000001CUL) /**< Offset from UART_REVC Base Address: <tt> 0x001C</tt> */ 
 #define MXC_R_UART_REVC_FIFO               ((uint32_t)0x00000020UL) /**< Offset from UART_REVC Base Address: <tt> 0x0020</tt> */ 
 #define MXC_R_UART_REVC_DMA                ((uint32_t)0x00000030UL) /**< Offset from UART_REVC Base Address: <tt> 0x0030</tt> */ 
/**@} end of group uart_revc_registers */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_CTRL UART_REVC_CTRL
 * @brief    Control Register.
 * @{
 */
 #define MXC_F_UART_REVC_CTRL_RXTHD_POS                 0 /**< CTRL_RXTHD Position */
 #define MXC_F_UART_REVC_CTRL_RXTHD                     ((uint32_t)(0xFUL << MXC_F_UART_REVC_CTRL_RXTHD_POS)) /**< CTRL_RXTHD Mask */

 #define MXC_F_UART_REVC_CTRL_PARITY_EN_POS             4 /**< CTRL_PARITY_EN Position */
 #define MXC_F_UART_REVC_CTRL_PARITY_EN                 ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_PARITY_EN_POS)) /**< CTRL_PARITY_EN Mask */

 #define MXC_F_UART_REVC_CTRL_PARITY_POS                5 /**< CTRL_PARITY Position */
 #define MXC_F_UART_REVC_CTRL_PARITY                    ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_PARITY_POS)) /**< CTRL_PARITY Mask */

 #define MXC_F_UART_REVC_CTRL_PARMD_POS                 6 /**< CTRL_PARMD Position */
 #define MXC_F_UART_REVC_CTRL_PARMD                     ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_PARMD_POS)) /**< CTRL_PARMD Mask */

 #define MXC_F_UART_REVC_CTRL_TX_FLUSH_POS              8 /**< CTRL_TX_FLUSH Position */
 #define MXC_F_UART_REVC_CTRL_TX_FLUSH                  ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_TX_FLUSH_POS)) /**< CTRL_TX_FLUSH Mask */

 #define MXC_F_UART_REVC_CTRL_RX_FLUSH_POS              9 /**< CTRL_RX_FLUSH Position */
 #define MXC_F_UART_REVC_CTRL_RX_FLUSH                  ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_RX_FLUSH_POS)) /**< CTRL_RX_FLUSH Mask */

 #define MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS             10 /**< CTRL_CHAR_SIZE Position */
 #define MXC_F_UART_REVC_CTRL_CHAR_SIZE                 ((uint32_t)(0x3UL << MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS)) /**< CTRL_CHAR_SIZE Mask */
 #define MXC_V_UART_REVC_CTRL_CHAR_SIZE_5               ((uint32_t)0x0UL) /**< CTRL_CHAR_SIZE_5 Value */
 #define MXC_S_UART_REVC_CTRL_CHAR_SIZE_5               (MXC_V_UART_REVC_CTRL_CHAR_SIZE_5 << MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_5 Setting */
 #define MXC_V_UART_REVC_CTRL_CHAR_SIZE_6               ((uint32_t)0x1UL) /**< CTRL_CHAR_SIZE_6 Value */
 #define MXC_S_UART_REVC_CTRL_CHAR_SIZE_6               (MXC_V_UART_REVC_CTRL_CHAR_SIZE_6 << MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_6 Setting */
 #define MXC_V_UART_REVC_CTRL_CHAR_SIZE_7               ((uint32_t)0x2UL) /**< CTRL_CHAR_SIZE_7 Value */
 #define MXC_S_UART_REVC_CTRL_CHAR_SIZE_7               (MXC_V_UART_REVC_CTRL_CHAR_SIZE_7 << MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_7 Setting */
 #define MXC_V_UART_REVC_CTRL_CHAR_SIZE_8               ((uint32_t)0x3UL) /**< CTRL_CHAR_SIZE_8 Value */
 #define MXC_S_UART_REVC_CTRL_CHAR_SIZE_8               (MXC_V_UART_REVC_CTRL_CHAR_SIZE_8 << MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_8 Setting */

 #define MXC_F_UART_REVC_CTRL_STOPBITS_POS              12 /**< CTRL_STOPBITS Position */
 #define MXC_F_UART_REVC_CTRL_STOPBITS                  ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_STOPBITS_POS)) /**< CTRL_STOPBITS Mask */

 #define MXC_F_UART_REVC_CTRL_FLOW_CTRL_POS             13 /**< CTRL_FLOW_CTRL Position */
 #define MXC_F_UART_REVC_CTRL_FLOW_CTRL                 ((uint32_t)(0x1UL << MXC_F_UART_REVC_CTRL_FLOW_CTRL_POS)) /**< CTRL_FLOW_CTRL Mask */

/**@} end of group UART_REVC_CTRL_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_STATUS UART_REVC_STATUS
 * @brief    Status Register.
 * @{
 */
 #define MXC_F_UART_REVC_STATUS_TX_BUSY_POS             0 /**< STATUS_TX_BUSY Position */
 #define MXC_F_UART_REVC_STATUS_TX_BUSY                 ((uint32_t)(0x1UL << MXC_F_UART_REVC_STATUS_TX_BUSY_POS)) /**< STATUS_TX_BUSY Mask */

 #define MXC_F_UART_REVC_STATUS_RX_BUSY_POS             1 /**< STATUS_RX_BUSY Position */
 #define MXC_F_UART_REVC_STATUS_RX_BUSY                 ((uint32_t)(0x1UL << MXC_F_UART_REVC_STATUS_RX_BUSY_POS)) /**< STATUS_RX_BUSY Mask */

 #define MXC_F_UART_REVC_STATUS_RX_EMPTY_POS            4 /**< STATUS_RX_EMPTY Position */
 #define MXC_F_UART_REVC_STATUS_RX_EMPTY                ((uint32_t)(0x1UL << MXC_F_UART_REVC_STATUS_RX_EMPTY_POS)) /**< STATUS_RX_EMPTY Mask */

 #define MXC_F_UART_REVC_STATUS_RX_FULL_POS             5 /**< STATUS_RX_FULL Position */
 #define MXC_F_UART_REVC_STATUS_RX_FULL                 ((uint32_t)(0x1UL << MXC_F_UART_REVC_STATUS_RX_FULL_POS)) /**< STATUS_RX_FULL Mask */

 #define MXC_F_UART_REVC_STATUS_TX_EMPTY_POS            6 /**< STATUS_TX_EMPTY Position */
 #define MXC_F_UART_REVC_STATUS_TX_EMPTY                ((uint32_t)(0x1UL << MXC_F_UART_REVC_STATUS_TX_EMPTY_POS)) /**< STATUS_TX_EMPTY Mask */

 #define MXC_F_UART_REVC_STATUS_TX_FULL_POS             7 /**< STATUS_TX_FULL Position */
 #define MXC_F_UART_REVC_STATUS_TX_FULL                 ((uint32_t)(0x1UL << MXC_F_UART_REVC_STATUS_TX_FULL_POS)) /**< STATUS_TX_FULL Mask */

 #define MXC_F_UART_REVC_STATUS_RX_FIFO_CNT_POS         8 /**< STATUS_RX_FIFO_CNT Position */
 #define MXC_F_UART_REVC_STATUS_RX_FIFO_CNT             ((uint32_t)(0xFUL << MXC_F_UART_REVC_STATUS_RX_FIFO_CNT_POS)) /**< STATUS_RX_FIFO_CNT Mask */

 #define MXC_F_UART_REVC_STATUS_TX_FIFO_CNT_POS         12 /**< STATUS_TX_FIFO_CNT Position */
 #define MXC_F_UART_REVC_STATUS_TX_FIFO_CNT             ((uint32_t)(0xFUL << MXC_F_UART_REVC_STATUS_TX_FIFO_CNT_POS)) /**< STATUS_TX_FIFO_CNT Mask */

/**@} end of group UART_REVC_STATUS_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_INT_EN UART_REVC_INT_EN
 * @brief    Interrupt Enable Register.
 * @{
 */
 #define MXC_F_UART_REVC_INT_EN_RX_FRAME_ERROR_POS      0 /**< INT_EN_RX_FRAME_ERROR Position */
 #define MXC_F_UART_REVC_INT_EN_RX_FRAME_ERROR          ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_EN_RX_FRAME_ERROR_POS)) /**< INT_EN_RX_FRAME_ERROR Mask */

 #define MXC_F_UART_REVC_INT_EN_RX_PARITY_ERROR_POS     1 /**< INT_EN_RX_PARITY_ERROR Position */
 #define MXC_F_UART_REVC_INT_EN_RX_PARITY_ERROR         ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_EN_RX_PARITY_ERROR_POS)) /**< INT_EN_RX_PARITY_ERROR Mask */

 #define MXC_F_UART_REVC_INT_EN_RX_OVERRUN_POS          3 /**< INT_EN_RX_OVERRUN Position */
 #define MXC_F_UART_REVC_INT_EN_RX_OVERRUN              ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_EN_RX_OVERRUN_POS)) /**< INT_EN_RX_OVERRUN Mask */

 #define MXC_F_UART_REVC_INT_EN_RX_FIFO_THRESH_POS      4 /**< INT_EN_RX_FIFO_THRESH Position */
 #define MXC_F_UART_REVC_INT_EN_RX_FIFO_THRESH          ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_EN_RX_FIFO_THRESH_POS)) /**< INT_EN_RX_FIFO_THRESH Mask */

 #define MXC_F_UART_REVC_INT_EN_TX_FIFO_ALMOST_EMPTY_POS 5 /**< INT_EN_TX_FIFO_ALMOST_EMPTY Position */
 #define MXC_F_UART_REVC_INT_EN_TX_FIFO_ALMOST_EMPTY    ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_EN_TX_FIFO_ALMOST_EMPTY_POS)) /**< INT_EN_TX_FIFO_ALMOST_EMPTY Mask */

 #define MXC_F_UART_REVC_INT_EN_TX_FIFO_HALF_EMPTY_POS  6 /**< INT_EN_TX_FIFO_HALF_EMPTY Position */
 #define MXC_F_UART_REVC_INT_EN_TX_FIFO_HALF_EMPTY      ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_EN_TX_FIFO_HALF_EMPTY_POS)) /**< INT_EN_TX_FIFO_HALF_EMPTY Mask */

/**@} end of group UART_REVC_INT_EN_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_INT_FL UART_REVC_INT_FL
 * @brief    Interrupt Status Flags.
 * @{
 */
 #define MXC_F_UART_REVC_INT_FL_RX_FRAME_ERROR_POS      0 /**< INT_FL_RX_FRAME_ERROR Position */
 #define MXC_F_UART_REVC_INT_FL_RX_FRAME_ERROR          ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_FL_RX_FRAME_ERROR_POS)) /**< INT_FL_RX_FRAME_ERROR Mask */

 #define MXC_F_UART_REVC_INT_FL_RX_PARITY_ERROR_POS     1 /**< INT_FL_RX_PARITY_ERROR Position */
 #define MXC_F_UART_REVC_INT_FL_RX_PARITY_ERROR         ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_FL_RX_PARITY_ERROR_POS)) /**< INT_FL_RX_PARITY_ERROR Mask */

 #define MXC_F_UART_REVC_INT_FL_RX_OVERRUN_POS          3 /**< INT_FL_RX_OVERRUN Position */
 #define MXC_F_UART_REVC_INT_FL_RX_OVERRUN              ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_FL_RX_OVERRUN_POS)) /**< INT_FL_RX_OVERRUN Mask */

 #define MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH_POS      4 /**< INT_FL_RX_FIFO_THRESH Position */
 #define MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH          ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH_POS)) /**< INT_FL_RX_FIFO_THRESH Mask */

 #define MXC_F_UART_REVC_INT_FL_TX_FIFO_ALMOST_EMPTY_POS 5 /**< INT_FL_TX_FIFO_ALMOST_EMPTY Position */
 #define MXC_F_UART_REVC_INT_FL_TX_FIFO_ALMOST_EMPTY    ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_FL_TX_FIFO_ALMOST_EMPTY_POS)) /**< INT_FL_TX_FIFO_ALMOST_EMPTY Mask */

 #define MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY_POS  6 /**< INT_FL_TX_FIFO_HALF_EMPTY Position */
 #define MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY      ((uint32_t)(0x1UL << MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY_POS)) /**< INT_FL_TX_FIFO_HALF_EMPTY Mask */

/**@} end of group UART_REVC_INT_FL_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_BAUD0 UART_REVC_BAUD0
 * @brief    Baud rate register. Integer portion.
 * @{
 */
 #define MXC_F_UART_REVC_BAUD0_IBAUD_POS                0 /**< BAUD0_IBAUD Position */
 #define MXC_F_UART_REVC_BAUD0_IBAUD                    ((uint32_t)(0xFFFUL << MXC_F_UART_REVC_BAUD0_IBAUD_POS)) /**< BAUD0_IBAUD Mask */

/**@} end of group UART_REVC_BAUD0_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_BAUD1 UART_REVC_BAUD1
 * @brief    Baud rate register. Decimal Setting.
 * @{
 */
 #define MXC_F_UART_REVC_BAUD1_DBAUD_POS                0 /**< BAUD1_DBAUD Position */
 #define MXC_F_UART_REVC_BAUD1_DBAUD                    ((uint32_t)(0x7FUL << MXC_F_UART_REVC_BAUD1_DBAUD_POS)) /**< BAUD1_DBAUD Mask */

/**@} end of group UART_REVC_BAUD1_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_PIN UART_REVC_PIN
 * @brief    UART Pin Control Register.
 * @{
 */
 #define MXC_F_UART_REVC_PIN_CTS_POS                    0 /**< PIN_CTS Position */
 #define MXC_F_UART_REVC_PIN_CTS                        ((uint32_t)(0x1UL << MXC_F_UART_REVC_PIN_CTS_POS)) /**< PIN_CTS Mask */

 #define MXC_F_UART_REVC_PIN_RTS_POS                    1 /**< PIN_RTS Position */
 #define MXC_F_UART_REVC_PIN_RTS                        ((uint32_t)(0x1UL << MXC_F_UART_REVC_PIN_RTS_POS)) /**< PIN_RTS Mask */

/**@} end of group UART_REVC_PIN_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_FIFO UART_REVC_FIFO
 * @brief    FIFO Data buffer.
 * @{
 */
 #define MXC_F_UART_REVC_FIFO_FIFO_POS                  0 /**< FIFO_FIFO Position */
 #define MXC_F_UART_REVC_FIFO_FIFO                      ((uint32_t)(0xFFUL << MXC_F_UART_REVC_FIFO_FIFO_POS)) /**< FIFO_FIFO Mask */

 #define MXC_F_UART_REVC_FIFO_PARITY_POS                8 /**< FIFO_PARITY Position */
 #define MXC_F_UART_REVC_FIFO_PARITY                    ((uint32_t)(0x1UL << MXC_F_UART_REVC_FIFO_PARITY_POS)) /**< FIFO_PARITY Mask */

/**@} end of group UART_REVC_FIFO_Register */

/**
 * @ingroup  uart_revc_registers
 * @defgroup UART_REVC_DMA UART_REVC_DMA
 * @brief    DMA Configuration.
 * @{
 */
 #define MXC_F_UART_REVC_DMA_TXDMA_LEVEL_POS            0 /**< DMA_TXDMA_LEVEL Position */
 #define MXC_F_UART_REVC_DMA_TXDMA_LEVEL                ((uint32_t)(0xFUL << MXC_F_UART_REVC_DMA_TXDMA_LEVEL_POS)) /**< DMA_TXDMA_LEVEL Mask */

 #define MXC_F_UART_REVC_DMA_TXDMA_EN_POS               4 /**< DMA_TXDMA_EN Position */
 #define MXC_F_UART_REVC_DMA_TXDMA_EN                   ((uint32_t)(0x1UL << MXC_F_UART_REVC_DMA_TXDMA_EN_POS)) /**< DMA_TXDMA_EN Mask */

 #define MXC_F_UART_REVC_DMA_RXDMA_LEVEL_POS            5 /**< DMA_RXDMA_LEVEL Position */
 #define MXC_F_UART_REVC_DMA_RXDMA_LEVEL                ((uint32_t)(0xFUL << MXC_F_UART_REVC_DMA_RXDMA_LEVEL_POS)) /**< DMA_RXDMA_LEVEL Mask */

 #define MXC_F_UART_REVC_DMA_RXDMA_EN_POS               9 /**< DMA_RXDMA_EN Position */
 #define MXC_F_UART_REVC_DMA_RXDMA_EN                   ((uint32_t)(0x1UL << MXC_F_UART_REVC_DMA_RXDMA_EN_POS)) /**< DMA_RXDMA_EN Mask */

/**@} end of group UART_REVC_DMA_Register */

#ifdef __cplusplus
}
#endif

#endif /* _UART_REVC_REGS_H_ */

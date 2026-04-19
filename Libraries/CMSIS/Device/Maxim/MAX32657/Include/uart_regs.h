/**
 * @file    uart_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the UART Peripheral Module.
 * @note    This file is @generated.
 * @ingroup uart_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_UART_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_UART_REGS_H_

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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
 * @ingroup     uart
 * @defgroup    uart_registers UART_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the UART Peripheral Module.
 * @details     UART Low Power Registers
 */

/**
 * @ingroup uart_registers
 * Structure type to access the UART Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> UART CTRL Register */
    __I  uint32_t status;               /**< <tt>\b 0x0004:</tt> UART STATUS Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0008:</tt> UART INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x000C:</tt> UART INTFL Register */
    __IO uint32_t clkdiv;               /**< <tt>\b 0x0010:</tt> UART CLKDIV Register */
    __IO uint32_t osr;                  /**< <tt>\b 0x0014:</tt> UART OSR Register */
    __IO uint32_t txpeek;               /**< <tt>\b 0x0018:</tt> UART TXPEEK Register */
    __R  uint32_t rsv_0x1c;
    __IO uint32_t fifo;                 /**< <tt>\b 0x0020:</tt> UART FIFO Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t dma;                  /**< <tt>\b 0x0030:</tt> UART DMA Register */
    __IO uint32_t wken;                 /**< <tt>\b 0x0034:</tt> UART WKEN Register */
    __IO uint32_t wkfl;                 /**< <tt>\b 0x0038:</tt> UART WKFL Register */
} mxc_uart_regs_t;

/* Register offsets for module UART */
/**
 * @ingroup    uart_registers
 * @defgroup   UART_Register_Offsets Register Offsets
 * @brief      UART Peripheral Register Offsets from the UART Base Peripheral Address.
 * @{
 */
#define MXC_R_UART_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from UART Base Address: <tt> 0x0000</tt> */
#define MXC_R_UART_STATUS                  ((uint32_t)0x00000004UL) /**< Offset from UART Base Address: <tt> 0x0004</tt> */
#define MXC_R_UART_INTEN                   ((uint32_t)0x00000008UL) /**< Offset from UART Base Address: <tt> 0x0008</tt> */
#define MXC_R_UART_INTFL                   ((uint32_t)0x0000000CUL) /**< Offset from UART Base Address: <tt> 0x000C</tt> */
#define MXC_R_UART_CLKDIV                  ((uint32_t)0x00000010UL) /**< Offset from UART Base Address: <tt> 0x0010</tt> */
#define MXC_R_UART_OSR                     ((uint32_t)0x00000014UL) /**< Offset from UART Base Address: <tt> 0x0014</tt> */
#define MXC_R_UART_TXPEEK                  ((uint32_t)0x00000018UL) /**< Offset from UART Base Address: <tt> 0x0018</tt> */
#define MXC_R_UART_FIFO                    ((uint32_t)0x00000020UL) /**< Offset from UART Base Address: <tt> 0x0020</tt> */
#define MXC_R_UART_DMA                     ((uint32_t)0x00000030UL) /**< Offset from UART Base Address: <tt> 0x0030</tt> */
#define MXC_R_UART_WKEN                    ((uint32_t)0x00000034UL) /**< Offset from UART Base Address: <tt> 0x0034</tt> */
#define MXC_R_UART_WKFL                    ((uint32_t)0x00000038UL) /**< Offset from UART Base Address: <tt> 0x0038</tt> */
/**@} end of group uart_registers */

/**
 * @ingroup  uart_registers
 * @defgroup UART_CTRL UART_CTRL
 * @brief    Control register
 * @{
 */
#define MXC_F_UART_CTRL_RX_THD_VAL_POS                 0 /**< CTRL_RX_THD_VAL Position */
#define MXC_F_UART_CTRL_RX_THD_VAL                     ((uint32_t)(0xFUL << MXC_F_UART_CTRL_RX_THD_VAL_POS)) /**< CTRL_RX_THD_VAL Mask */

#define MXC_F_UART_CTRL_PAR_EN_POS                     4 /**< CTRL_PAR_EN Position */
#define MXC_F_UART_CTRL_PAR_EN                         ((uint32_t)(0x1UL << MXC_F_UART_CTRL_PAR_EN_POS)) /**< CTRL_PAR_EN Mask */

#define MXC_F_UART_CTRL_PAR_EO_POS                     5 /**< CTRL_PAR_EO Position */
#define MXC_F_UART_CTRL_PAR_EO                         ((uint32_t)(0x1UL << MXC_F_UART_CTRL_PAR_EO_POS)) /**< CTRL_PAR_EO Mask */

#define MXC_F_UART_CTRL_PAR_MD_POS                     6 /**< CTRL_PAR_MD Position */
#define MXC_F_UART_CTRL_PAR_MD                         ((uint32_t)(0x1UL << MXC_F_UART_CTRL_PAR_MD_POS)) /**< CTRL_PAR_MD Mask */

#define MXC_F_UART_CTRL_CTS_DIS_POS                    7 /**< CTRL_CTS_DIS Position */
#define MXC_F_UART_CTRL_CTS_DIS                        ((uint32_t)(0x1UL << MXC_F_UART_CTRL_CTS_DIS_POS)) /**< CTRL_CTS_DIS Mask */

#define MXC_F_UART_CTRL_TX_FLUSH_POS                   8 /**< CTRL_TX_FLUSH Position */
#define MXC_F_UART_CTRL_TX_FLUSH                       ((uint32_t)(0x1UL << MXC_F_UART_CTRL_TX_FLUSH_POS)) /**< CTRL_TX_FLUSH Mask */

#define MXC_F_UART_CTRL_RX_FLUSH_POS                   9 /**< CTRL_RX_FLUSH Position */
#define MXC_F_UART_CTRL_RX_FLUSH                       ((uint32_t)(0x1UL << MXC_F_UART_CTRL_RX_FLUSH_POS)) /**< CTRL_RX_FLUSH Mask */

#define MXC_F_UART_CTRL_CHAR_SIZE_POS                  10 /**< CTRL_CHAR_SIZE Position */
#define MXC_F_UART_CTRL_CHAR_SIZE                      ((uint32_t)(0x3UL << MXC_F_UART_CTRL_CHAR_SIZE_POS)) /**< CTRL_CHAR_SIZE Mask */
#define MXC_V_UART_CTRL_CHAR_SIZE_5BITS                ((uint32_t)0x0UL) /**< CTRL_CHAR_SIZE_5BITS Value */
#define MXC_S_UART_CTRL_CHAR_SIZE_5BITS                (MXC_V_UART_CTRL_CHAR_SIZE_5BITS << MXC_F_UART_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_5BITS Setting */
#define MXC_V_UART_CTRL_CHAR_SIZE_6BITS                ((uint32_t)0x1UL) /**< CTRL_CHAR_SIZE_6BITS Value */
#define MXC_S_UART_CTRL_CHAR_SIZE_6BITS                (MXC_V_UART_CTRL_CHAR_SIZE_6BITS << MXC_F_UART_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_6BITS Setting */
#define MXC_V_UART_CTRL_CHAR_SIZE_7BITS                ((uint32_t)0x2UL) /**< CTRL_CHAR_SIZE_7BITS Value */
#define MXC_S_UART_CTRL_CHAR_SIZE_7BITS                (MXC_V_UART_CTRL_CHAR_SIZE_7BITS << MXC_F_UART_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_7BITS Setting */
#define MXC_V_UART_CTRL_CHAR_SIZE_8BITS                ((uint32_t)0x3UL) /**< CTRL_CHAR_SIZE_8BITS Value */
#define MXC_S_UART_CTRL_CHAR_SIZE_8BITS                (MXC_V_UART_CTRL_CHAR_SIZE_8BITS << MXC_F_UART_CTRL_CHAR_SIZE_POS) /**< CTRL_CHAR_SIZE_8BITS Setting */

#define MXC_F_UART_CTRL_STOPBITS_POS                   12 /**< CTRL_STOPBITS Position */
#define MXC_F_UART_CTRL_STOPBITS                       ((uint32_t)(0x1UL << MXC_F_UART_CTRL_STOPBITS_POS)) /**< CTRL_STOPBITS Mask */

#define MXC_F_UART_CTRL_HFC_EN_POS                     13 /**< CTRL_HFC_EN Position */
#define MXC_F_UART_CTRL_HFC_EN                         ((uint32_t)(0x1UL << MXC_F_UART_CTRL_HFC_EN_POS)) /**< CTRL_HFC_EN Mask */

#define MXC_F_UART_CTRL_RTS_NEG_POS                    14 /**< CTRL_RTS_NEG Position */
#define MXC_F_UART_CTRL_RTS_NEG                        ((uint32_t)(0x1UL << MXC_F_UART_CTRL_RTS_NEG_POS)) /**< CTRL_RTS_NEG Mask */

#define MXC_F_UART_CTRL_CLK_EN_POS                     15 /**< CTRL_CLK_EN Position */
#define MXC_F_UART_CTRL_CLK_EN                         ((uint32_t)(0x1UL << MXC_F_UART_CTRL_CLK_EN_POS)) /**< CTRL_CLK_EN Mask */

#define MXC_F_UART_CTRL_CLK_SEL_POS                    16 /**< CTRL_CLK_SEL Position */
#define MXC_F_UART_CTRL_CLK_SEL                        ((uint32_t)(0x3UL << MXC_F_UART_CTRL_CLK_SEL_POS)) /**< CTRL_CLK_SEL Mask */
#define MXC_V_UART_CTRL_CLK_SEL_PERIPHERAL_CLOCK       ((uint32_t)0x0UL) /**< CTRL_CLK_SEL_PERIPHERAL_CLOCK Value */
#define MXC_S_UART_CTRL_CLK_SEL_PERIPHERAL_CLOCK       (MXC_V_UART_CTRL_CLK_SEL_PERIPHERAL_CLOCK << MXC_F_UART_CTRL_CLK_SEL_POS) /**< CTRL_CLK_SEL_PERIPHERAL_CLOCK Setting */
#define MXC_V_UART_CTRL_CLK_SEL_CLK1                   ((uint32_t)0x1UL) /**< CTRL_CLK_SEL_CLK1 Value */
#define MXC_S_UART_CTRL_CLK_SEL_CLK1                   (MXC_V_UART_CTRL_CLK_SEL_CLK1 << MXC_F_UART_CTRL_CLK_SEL_POS) /**< CTRL_CLK_SEL_CLK1 Setting */

#define MXC_F_UART_CTRL_CLK_RDY_POS                    19 /**< CTRL_CLK_RDY Position */
#define MXC_F_UART_CTRL_CLK_RDY                        ((uint32_t)(0x1UL << MXC_F_UART_CTRL_CLK_RDY_POS)) /**< CTRL_CLK_RDY Mask */

#define MXC_F_UART_CTRL_CLK_GATE_POS                   20 /**< CTRL_CLK_GATE Position */
#define MXC_F_UART_CTRL_CLK_GATE                       ((uint32_t)(0x1UL << MXC_F_UART_CTRL_CLK_GATE_POS)) /**< CTRL_CLK_GATE Mask */

/**@} end of group UART_CTRL_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_STATUS UART_STATUS
 * @brief    Status register
 * @{
 */
#define MXC_F_UART_STATUS_TX_BUSY_POS                  0 /**< STATUS_TX_BUSY Position */
#define MXC_F_UART_STATUS_TX_BUSY                      ((uint32_t)(0x1UL << MXC_F_UART_STATUS_TX_BUSY_POS)) /**< STATUS_TX_BUSY Mask */

#define MXC_F_UART_STATUS_RX_BUSY_POS                  1 /**< STATUS_RX_BUSY Position */
#define MXC_F_UART_STATUS_RX_BUSY                      ((uint32_t)(0x1UL << MXC_F_UART_STATUS_RX_BUSY_POS)) /**< STATUS_RX_BUSY Mask */

#define MXC_F_UART_STATUS_RX_EM_POS                    4 /**< STATUS_RX_EM Position */
#define MXC_F_UART_STATUS_RX_EM                        ((uint32_t)(0x1UL << MXC_F_UART_STATUS_RX_EM_POS)) /**< STATUS_RX_EM Mask */

#define MXC_F_UART_STATUS_RX_FULL_POS                  5 /**< STATUS_RX_FULL Position */
#define MXC_F_UART_STATUS_RX_FULL                      ((uint32_t)(0x1UL << MXC_F_UART_STATUS_RX_FULL_POS)) /**< STATUS_RX_FULL Mask */

#define MXC_F_UART_STATUS_TX_EM_POS                    6 /**< STATUS_TX_EM Position */
#define MXC_F_UART_STATUS_TX_EM                        ((uint32_t)(0x1UL << MXC_F_UART_STATUS_TX_EM_POS)) /**< STATUS_TX_EM Mask */

#define MXC_F_UART_STATUS_TX_FULL_POS                  7 /**< STATUS_TX_FULL Position */
#define MXC_F_UART_STATUS_TX_FULL                      ((uint32_t)(0x1UL << MXC_F_UART_STATUS_TX_FULL_POS)) /**< STATUS_TX_FULL Mask */

#define MXC_F_UART_STATUS_RX_LVL_POS                   8 /**< STATUS_RX_LVL Position */
#define MXC_F_UART_STATUS_RX_LVL                       ((uint32_t)(0xFUL << MXC_F_UART_STATUS_RX_LVL_POS)) /**< STATUS_RX_LVL Mask */

#define MXC_F_UART_STATUS_TX_LVL_POS                   12 /**< STATUS_TX_LVL Position */
#define MXC_F_UART_STATUS_TX_LVL                       ((uint32_t)(0xFUL << MXC_F_UART_STATUS_TX_LVL_POS)) /**< STATUS_TX_LVL Mask */

/**@} end of group UART_STATUS_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_INTEN UART_INTEN
 * @brief    Interrupt Enable control register
 * @{
 */
#define MXC_F_UART_INTEN_RX_FERR_POS                   0 /**< INTEN_RX_FERR Position */
#define MXC_F_UART_INTEN_RX_FERR                       ((uint32_t)(0x1UL << MXC_F_UART_INTEN_RX_FERR_POS)) /**< INTEN_RX_FERR Mask */

#define MXC_F_UART_INTEN_RX_PAR_POS                    1 /**< INTEN_RX_PAR Position */
#define MXC_F_UART_INTEN_RX_PAR                        ((uint32_t)(0x1UL << MXC_F_UART_INTEN_RX_PAR_POS)) /**< INTEN_RX_PAR Mask */

#define MXC_F_UART_INTEN_CTS_EV_POS                    2 /**< INTEN_CTS_EV Position */
#define MXC_F_UART_INTEN_CTS_EV                        ((uint32_t)(0x1UL << MXC_F_UART_INTEN_CTS_EV_POS)) /**< INTEN_CTS_EV Mask */

#define MXC_F_UART_INTEN_RX_OV_POS                     3 /**< INTEN_RX_OV Position */
#define MXC_F_UART_INTEN_RX_OV                         ((uint32_t)(0x1UL << MXC_F_UART_INTEN_RX_OV_POS)) /**< INTEN_RX_OV Mask */

#define MXC_F_UART_INTEN_RX_THD_POS                    4 /**< INTEN_RX_THD Position */
#define MXC_F_UART_INTEN_RX_THD                        ((uint32_t)(0x1UL << MXC_F_UART_INTEN_RX_THD_POS)) /**< INTEN_RX_THD Mask */

#define MXC_F_UART_INTEN_TX_OB_POS                     5 /**< INTEN_TX_OB Position */
#define MXC_F_UART_INTEN_TX_OB                         ((uint32_t)(0x1UL << MXC_F_UART_INTEN_TX_OB_POS)) /**< INTEN_TX_OB Mask */

#define MXC_F_UART_INTEN_TX_THD_POS                    6 /**< INTEN_TX_THD Position */
#define MXC_F_UART_INTEN_TX_THD                        ((uint32_t)(0x1UL << MXC_F_UART_INTEN_TX_THD_POS)) /**< INTEN_TX_THD Mask */

#define MXC_F_UART_INTEN_RX_FULL_POS                   7 /**< INTEN_RX_FULL Position */
#define MXC_F_UART_INTEN_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_UART_INTEN_RX_FULL_POS)) /**< INTEN_RX_FULL Mask */

/**@} end of group UART_INTEN_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_INTFL UART_INTFL
 * @brief    Interrupt status flags Control register
 * @{
 */
#define MXC_F_UART_INTFL_RX_FERR_POS                   0 /**< INTFL_RX_FERR Position */
#define MXC_F_UART_INTFL_RX_FERR                       ((uint32_t)(0x1UL << MXC_F_UART_INTFL_RX_FERR_POS)) /**< INTFL_RX_FERR Mask */

#define MXC_F_UART_INTFL_RX_PAR_POS                    1 /**< INTFL_RX_PAR Position */
#define MXC_F_UART_INTFL_RX_PAR                        ((uint32_t)(0x1UL << MXC_F_UART_INTFL_RX_PAR_POS)) /**< INTFL_RX_PAR Mask */

#define MXC_F_UART_INTFL_CTS_EV_POS                    2 /**< INTFL_CTS_EV Position */
#define MXC_F_UART_INTFL_CTS_EV                        ((uint32_t)(0x1UL << MXC_F_UART_INTFL_CTS_EV_POS)) /**< INTFL_CTS_EV Mask */

#define MXC_F_UART_INTFL_RX_OV_POS                     3 /**< INTFL_RX_OV Position */
#define MXC_F_UART_INTFL_RX_OV                         ((uint32_t)(0x1UL << MXC_F_UART_INTFL_RX_OV_POS)) /**< INTFL_RX_OV Mask */

#define MXC_F_UART_INTFL_RX_THD_POS                    4 /**< INTFL_RX_THD Position */
#define MXC_F_UART_INTFL_RX_THD                        ((uint32_t)(0x1UL << MXC_F_UART_INTFL_RX_THD_POS)) /**< INTFL_RX_THD Mask */

#define MXC_F_UART_INTFL_TX_OB_POS                     5 /**< INTFL_TX_OB Position */
#define MXC_F_UART_INTFL_TX_OB                         ((uint32_t)(0x1UL << MXC_F_UART_INTFL_TX_OB_POS)) /**< INTFL_TX_OB Mask */

#define MXC_F_UART_INTFL_TX_THD_POS                    6 /**< INTFL_TX_THD Position */
#define MXC_F_UART_INTFL_TX_THD                        ((uint32_t)(0x1UL << MXC_F_UART_INTFL_TX_THD_POS)) /**< INTFL_TX_THD Mask */

#define MXC_F_UART_INTFL_RX_FULL_POS                   7 /**< INTFL_RX_FULL Position */
#define MXC_F_UART_INTFL_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_UART_INTFL_RX_FULL_POS)) /**< INTFL_RX_FULL Mask */

/**@} end of group UART_INTFL_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_CLKDIV UART_CLKDIV
 * @brief    Clock Divider register
 * @{
 */
#define MXC_F_UART_CLKDIV_CLKDIV_POS                   0 /**< CLKDIV_CLKDIV Position */
#define MXC_F_UART_CLKDIV_CLKDIV                       ((uint32_t)(0xFFFFFUL << MXC_F_UART_CLKDIV_CLKDIV_POS)) /**< CLKDIV_CLKDIV Mask */

/**@} end of group UART_CLKDIV_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_OSR UART_OSR
 * @brief    Over Sampling Rate register
 * @{
 */
#define MXC_F_UART_OSR_OSR_POS                         0 /**< OSR_OSR Position */
#define MXC_F_UART_OSR_OSR                             ((uint32_t)(0x7UL << MXC_F_UART_OSR_OSR_POS)) /**< OSR_OSR Mask */

/**@} end of group UART_OSR_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_TXPEEK UART_TXPEEK
 * @brief    TX FIFO Output Peek register
 * @{
 */
#define MXC_F_UART_TXPEEK_DATA_POS                     0 /**< TXPEEK_DATA Position */
#define MXC_F_UART_TXPEEK_DATA                         ((uint32_t)(0xFFUL << MXC_F_UART_TXPEEK_DATA_POS)) /**< TXPEEK_DATA Mask */

/**@} end of group UART_TXPEEK_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_FIFO UART_FIFO
 * @brief    FIFO Read/Write register
 * @{
 */
#define MXC_F_UART_FIFO_DATA_POS                       0 /**< FIFO_DATA Position */
#define MXC_F_UART_FIFO_DATA                           ((uint32_t)(0xFFUL << MXC_F_UART_FIFO_DATA_POS)) /**< FIFO_DATA Mask */

#define MXC_F_UART_FIFO_RX_PAR_POS                     8 /**< FIFO_RX_PAR Position */
#define MXC_F_UART_FIFO_RX_PAR                         ((uint32_t)(0x1UL << MXC_F_UART_FIFO_RX_PAR_POS)) /**< FIFO_RX_PAR Mask */

/**@} end of group UART_FIFO_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_DMA UART_DMA
 * @brief    DMA Configuration register
 * @{
 */
#define MXC_F_UART_DMA_TX_THD_VAL_POS                  0 /**< DMA_TX_THD_VAL Position */
#define MXC_F_UART_DMA_TX_THD_VAL                      ((uint32_t)(0xFUL << MXC_F_UART_DMA_TX_THD_VAL_POS)) /**< DMA_TX_THD_VAL Mask */

#define MXC_F_UART_DMA_TX_EN_POS                       4 /**< DMA_TX_EN Position */
#define MXC_F_UART_DMA_TX_EN                           ((uint32_t)(0x1UL << MXC_F_UART_DMA_TX_EN_POS)) /**< DMA_TX_EN Mask */

#define MXC_F_UART_DMA_RX_THD_VAL_POS                  5 /**< DMA_RX_THD_VAL Position */
#define MXC_F_UART_DMA_RX_THD_VAL                      ((uint32_t)(0xFUL << MXC_F_UART_DMA_RX_THD_VAL_POS)) /**< DMA_RX_THD_VAL Mask */

#define MXC_F_UART_DMA_RX_EN_POS                       9 /**< DMA_RX_EN Position */
#define MXC_F_UART_DMA_RX_EN                           ((uint32_t)(0x1UL << MXC_F_UART_DMA_RX_EN_POS)) /**< DMA_RX_EN Mask */

/**@} end of group UART_DMA_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_WKEN UART_WKEN
 * @brief    Wake up enable Control register
 * @{
 */
#define MXC_F_UART_WKEN_RX_NE_POS                      0 /**< WKEN_RX_NE Position */
#define MXC_F_UART_WKEN_RX_NE                          ((uint32_t)(0x1UL << MXC_F_UART_WKEN_RX_NE_POS)) /**< WKEN_RX_NE Mask */

#define MXC_F_UART_WKEN_RX_FULL_POS                    1 /**< WKEN_RX_FULL Position */
#define MXC_F_UART_WKEN_RX_FULL                        ((uint32_t)(0x1UL << MXC_F_UART_WKEN_RX_FULL_POS)) /**< WKEN_RX_FULL Mask */

#define MXC_F_UART_WKEN_RX_THD_POS                     2 /**< WKEN_RX_THD Position */
#define MXC_F_UART_WKEN_RX_THD                         ((uint32_t)(0x1UL << MXC_F_UART_WKEN_RX_THD_POS)) /**< WKEN_RX_THD Mask */

/**@} end of group UART_WKEN_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_WKFL UART_WKFL
 * @brief    Wake up Flags register
 * @{
 */
#define MXC_F_UART_WKFL_RX_NE_POS                      0 /**< WKFL_RX_NE Position */
#define MXC_F_UART_WKFL_RX_NE                          ((uint32_t)(0x1UL << MXC_F_UART_WKFL_RX_NE_POS)) /**< WKFL_RX_NE Mask */

#define MXC_F_UART_WKFL_RX_FULL_POS                    1 /**< WKFL_RX_FULL Position */
#define MXC_F_UART_WKFL_RX_FULL                        ((uint32_t)(0x1UL << MXC_F_UART_WKFL_RX_FULL_POS)) /**< WKFL_RX_FULL Mask */

#define MXC_F_UART_WKFL_RX_THD_POS                     2 /**< WKFL_RX_THD Position */
#define MXC_F_UART_WKFL_RX_THD                         ((uint32_t)(0x1UL << MXC_F_UART_WKFL_RX_THD_POS)) /**< WKFL_RX_THD Mask */

/**@} end of group UART_WKFL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_UART_REGS_H_

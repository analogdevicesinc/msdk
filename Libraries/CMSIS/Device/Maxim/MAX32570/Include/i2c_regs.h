/**
 * @file    i2c_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the I2C Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_I2C_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_I2C_REGS_H_

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
 * @ingroup     i2c
 * @defgroup    i2c_registers I2C_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the I2C Peripheral Module.
 * @details     Inter-Integrated Circuit.
 */

/**
 * @ingroup i2c_registers
 * Structure type to access the I2C Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> I2C CTRL Register */
    __IO uint32_t status;               /**< <tt>\b 0x04:</tt> I2C STATUS Register */
    __IO uint32_t intfl0;               /**< <tt>\b 0x08:</tt> I2C INTFL0 Register */
    __IO uint32_t inten0;               /**< <tt>\b 0x0C:</tt> I2C INTEN0 Register */
    __IO uint32_t intfl1;               /**< <tt>\b 0x10:</tt> I2C INTFL1 Register */
    __IO uint32_t inten1;               /**< <tt>\b 0x14:</tt> I2C INTEN1 Register */
    __IO uint32_t fifolen;              /**< <tt>\b 0x18:</tt> I2C FIFOLEN Register */
    __IO uint32_t rxctrl0;              /**< <tt>\b 0x1C:</tt> I2C RXCTRL0 Register */
    __IO uint32_t rxctrl1;              /**< <tt>\b 0x20:</tt> I2C RXCTRL1 Register */
    __IO uint32_t txctrl0;              /**< <tt>\b 0x24:</tt> I2C TXCTRL0 Register */
    __IO uint32_t txctrl1;              /**< <tt>\b 0x28:</tt> I2C TXCTRL1 Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x2C:</tt> I2C FIFO Register */
    __IO uint32_t mstctrl;              /**< <tt>\b 0x30:</tt> I2C MSTCTRL Register */
    __IO uint32_t clklo;                /**< <tt>\b 0x34:</tt> I2C CLKLO Register */
    __IO uint32_t clkhi;                /**< <tt>\b 0x38:</tt> I2C CLKHI Register */
    __R  uint32_t rsv_0x3c;
    __IO uint32_t timeout;              /**< <tt>\b 0x40:</tt> I2C TIMEOUT Register */
    __R  uint32_t rsv_0x44;
    __IO uint32_t dma;                  /**< <tt>\b 0x48:</tt> I2C DMA Register */
    __IO uint32_t slave_addr;           /**< <tt>\b 0x4C:</tt> I2C SLAVE_ADDR Register */
} mxc_i2c_regs_t;

/* Register offsets for module I2C */
/**
 * @ingroup    i2c_registers
 * @defgroup   I2C_Register_Offsets Register Offsets
 * @brief      I2C Peripheral Register Offsets from the I2C Base Peripheral Address.
 * @{
 */
#define MXC_R_I2C_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from I2C Base Address: <tt> 0x0000</tt> */
#define MXC_R_I2C_STATUS                   ((uint32_t)0x00000004UL) /**< Offset from I2C Base Address: <tt> 0x0004</tt> */
#define MXC_R_I2C_INTFL0                   ((uint32_t)0x00000008UL) /**< Offset from I2C Base Address: <tt> 0x0008</tt> */
#define MXC_R_I2C_INTEN0                   ((uint32_t)0x0000000CUL) /**< Offset from I2C Base Address: <tt> 0x000C</tt> */
#define MXC_R_I2C_INTFL1                   ((uint32_t)0x00000010UL) /**< Offset from I2C Base Address: <tt> 0x0010</tt> */
#define MXC_R_I2C_INTEN1                   ((uint32_t)0x00000014UL) /**< Offset from I2C Base Address: <tt> 0x0014</tt> */
#define MXC_R_I2C_FIFOLEN                  ((uint32_t)0x00000018UL) /**< Offset from I2C Base Address: <tt> 0x0018</tt> */
#define MXC_R_I2C_RXCTRL0                  ((uint32_t)0x0000001CUL) /**< Offset from I2C Base Address: <tt> 0x001C</tt> */
#define MXC_R_I2C_RXCTRL1                  ((uint32_t)0x00000020UL) /**< Offset from I2C Base Address: <tt> 0x0020</tt> */
#define MXC_R_I2C_TXCTRL0                  ((uint32_t)0x00000024UL) /**< Offset from I2C Base Address: <tt> 0x0024</tt> */
#define MXC_R_I2C_TXCTRL1                  ((uint32_t)0x00000028UL) /**< Offset from I2C Base Address: <tt> 0x0028</tt> */
#define MXC_R_I2C_FIFO                     ((uint32_t)0x0000002CUL) /**< Offset from I2C Base Address: <tt> 0x002C</tt> */
#define MXC_R_I2C_MSTCTRL                  ((uint32_t)0x00000030UL) /**< Offset from I2C Base Address: <tt> 0x0030</tt> */
#define MXC_R_I2C_CLKLO                    ((uint32_t)0x00000034UL) /**< Offset from I2C Base Address: <tt> 0x0034</tt> */
#define MXC_R_I2C_CLKHI                    ((uint32_t)0x00000038UL) /**< Offset from I2C Base Address: <tt> 0x0038</tt> */
#define MXC_R_I2C_TIMEOUT                  ((uint32_t)0x00000040UL) /**< Offset from I2C Base Address: <tt> 0x0040</tt> */
#define MXC_R_I2C_DMA                      ((uint32_t)0x00000048UL) /**< Offset from I2C Base Address: <tt> 0x0048</tt> */
#define MXC_R_I2C_SLAVE_ADDR               ((uint32_t)0x0000004CUL) /**< Offset from I2C Base Address: <tt> 0x004C</tt> */
/**@} end of group i2c_registers */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CTRL I2C_CTRL
 * @brief    Control Register0.
 * @{
 */
#define MXC_F_I2C_CTRL_I2C_EN_POS                      0 /**< CTRL_I2C_EN Position */
#define MXC_F_I2C_CTRL_I2C_EN                          ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_I2C_EN_POS)) /**< CTRL_I2C_EN Mask */

#define MXC_F_I2C_CTRL_MST_POS                         1 /**< CTRL_MST Position */
#define MXC_F_I2C_CTRL_MST                             ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_MST_POS)) /**< CTRL_MST Mask */

#define MXC_F_I2C_CTRL_GEN_CALL_ADDR_POS               2 /**< CTRL_GEN_CALL_ADDR Position */
#define MXC_F_I2C_CTRL_GEN_CALL_ADDR                   ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_GEN_CALL_ADDR_POS)) /**< CTRL_GEN_CALL_ADDR Mask */

#define MXC_F_I2C_CTRL_RX_MODE_POS                     3 /**< CTRL_RX_MODE Position */
#define MXC_F_I2C_CTRL_RX_MODE                         ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_RX_MODE_POS)) /**< CTRL_RX_MODE Mask */

#define MXC_F_I2C_CTRL_RX_MODE_ACK_POS                 4 /**< CTRL_RX_MODE_ACK Position */
#define MXC_F_I2C_CTRL_RX_MODE_ACK                     ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_RX_MODE_ACK_POS)) /**< CTRL_RX_MODE_ACK Mask */

#define MXC_F_I2C_CTRL_SCL_OUT_POS                     6 /**< CTRL_SCL_OUT Position */
#define MXC_F_I2C_CTRL_SCL_OUT                         ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SCL_OUT_POS)) /**< CTRL_SCL_OUT Mask */

#define MXC_F_I2C_CTRL_SDA_OUT_POS                     7 /**< CTRL_SDA_OUT Position */
#define MXC_F_I2C_CTRL_SDA_OUT                         ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SDA_OUT_POS)) /**< CTRL_SDA_OUT Mask */

#define MXC_F_I2C_CTRL_SCL_POS                         8 /**< CTRL_SCL Position */
#define MXC_F_I2C_CTRL_SCL                             ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SCL_POS)) /**< CTRL_SCL Mask */

#define MXC_F_I2C_CTRL_SDA_POS                         9 /**< CTRL_SDA Position */
#define MXC_F_I2C_CTRL_SDA                             ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SDA_POS)) /**< CTRL_SDA Mask */

#define MXC_F_I2C_CTRL_SW_OUT_EN_POS                   10 /**< CTRL_SW_OUT_EN Position */
#define MXC_F_I2C_CTRL_SW_OUT_EN                       ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SW_OUT_EN_POS)) /**< CTRL_SW_OUT_EN Mask */

#define MXC_F_I2C_CTRL_READ_POS                        11 /**< CTRL_READ Position */
#define MXC_F_I2C_CTRL_READ                            ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_READ_POS)) /**< CTRL_READ Mask */

#define MXC_F_I2C_CTRL_SCL_CLK_STRETCH_DIS_POS         12 /**< CTRL_SCL_CLK_STRETCH_DIS Position */
#define MXC_F_I2C_CTRL_SCL_CLK_STRETCH_DIS             ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SCL_CLK_STRETCH_DIS_POS)) /**< CTRL_SCL_CLK_STRETCH_DIS Mask */

#define MXC_F_I2C_CTRL_SCL_PP_MODE_POS                 13 /**< CTRL_SCL_PP_MODE Position */
#define MXC_F_I2C_CTRL_SCL_PP_MODE                     ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_SCL_PP_MODE_POS)) /**< CTRL_SCL_PP_MODE Mask */

/**@} end of group I2C_CTRL_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_STATUS I2C_STATUS
 * @brief    Status Register.
 * @{
 */
#define MXC_F_I2C_STATUS_BUS_POS                       0 /**< STATUS_BUS Position */
#define MXC_F_I2C_STATUS_BUS                           ((uint32_t)(0x1UL << MXC_F_I2C_STATUS_BUS_POS)) /**< STATUS_BUS Mask */

#define MXC_F_I2C_STATUS_RX_EMPTY_POS                  1 /**< STATUS_RX_EMPTY Position */
#define MXC_F_I2C_STATUS_RX_EMPTY                      ((uint32_t)(0x1UL << MXC_F_I2C_STATUS_RX_EMPTY_POS)) /**< STATUS_RX_EMPTY Mask */

#define MXC_F_I2C_STATUS_RX_FULL_POS                   2 /**< STATUS_RX_FULL Position */
#define MXC_F_I2C_STATUS_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_I2C_STATUS_RX_FULL_POS)) /**< STATUS_RX_FULL Mask */

#define MXC_F_I2C_STATUS_TX_EMPTY_POS                  3 /**< STATUS_TX_EMPTY Position */
#define MXC_F_I2C_STATUS_TX_EMPTY                      ((uint32_t)(0x1UL << MXC_F_I2C_STATUS_TX_EMPTY_POS)) /**< STATUS_TX_EMPTY Mask */

#define MXC_F_I2C_STATUS_TX_FULL_POS                   4 /**< STATUS_TX_FULL Position */
#define MXC_F_I2C_STATUS_TX_FULL                       ((uint32_t)(0x1UL << MXC_F_I2C_STATUS_TX_FULL_POS)) /**< STATUS_TX_FULL Mask */

#define MXC_F_I2C_STATUS_CLK_MODE_POS                  5 /**< STATUS_CLK_MODE Position */
#define MXC_F_I2C_STATUS_CLK_MODE                      ((uint32_t)(0x1UL << MXC_F_I2C_STATUS_CLK_MODE_POS)) /**< STATUS_CLK_MODE Mask */

/**@} end of group I2C_STATUS_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INTFL0 I2C_INTFL0
 * @brief    Interrupt Status Register.
 * @{
 */
#define MXC_F_I2C_INTFL0_DONE_POS                      0 /**< INTFL0_DONE Position */
#define MXC_F_I2C_INTFL0_DONE                          ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_DONE_POS)) /**< INTFL0_DONE Mask */

#define MXC_F_I2C_INTFL0_RX_MODE_POS                   1 /**< INTFL0_RX_MODE Position */
#define MXC_F_I2C_INTFL0_RX_MODE                       ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_RX_MODE_POS)) /**< INTFL0_RX_MODE Mask */

#define MXC_F_I2C_INTFL0_GEN_CALL_ADDR_POS             2 /**< INTFL0_GEN_CALL_ADDR Position */
#define MXC_F_I2C_INTFL0_GEN_CALL_ADDR                 ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_GEN_CALL_ADDR_POS)) /**< INTFL0_GEN_CALL_ADDR Mask */

#define MXC_F_I2C_INTFL0_ADDR_MATCH_POS                3 /**< INTFL0_ADDR_MATCH Position */
#define MXC_F_I2C_INTFL0_ADDR_MATCH                    ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_ADDR_MATCH_POS)) /**< INTFL0_ADDR_MATCH Mask */

#define MXC_F_I2C_INTFL0_RX_THRESH_POS                 4 /**< INTFL0_RX_THRESH Position */
#define MXC_F_I2C_INTFL0_RX_THRESH                     ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_RX_THRESH_POS)) /**< INTFL0_RX_THRESH Mask */

#define MXC_F_I2C_INTFL0_TX_THRESH_POS                 5 /**< INTFL0_TX_THRESH Position */
#define MXC_F_I2C_INTFL0_TX_THRESH                     ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_TX_THRESH_POS)) /**< INTFL0_TX_THRESH Mask */

#define MXC_F_I2C_INTFL0_STOP_POS                      6 /**< INTFL0_STOP Position */
#define MXC_F_I2C_INTFL0_STOP                          ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_STOP_POS)) /**< INTFL0_STOP Mask */

#define MXC_F_I2C_INTFL0_ADDR_ACK_POS                  7 /**< INTFL0_ADDR_ACK Position */
#define MXC_F_I2C_INTFL0_ADDR_ACK                      ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_ADDR_ACK_POS)) /**< INTFL0_ADDR_ACK Mask */

#define MXC_F_I2C_INTFL0_ARB_ER_POS                    8 /**< INTFL0_ARB_ER Position */
#define MXC_F_I2C_INTFL0_ARB_ER                        ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_ARB_ER_POS)) /**< INTFL0_ARB_ER Mask */

#define MXC_F_I2C_INTFL0_TO_ER_POS                     9 /**< INTFL0_TO_ER Position */
#define MXC_F_I2C_INTFL0_TO_ER                         ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_TO_ER_POS)) /**< INTFL0_TO_ER Mask */

#define MXC_F_I2C_INTFL0_ADDR_NACK_ER_POS              10 /**< INTFL0_ADDR_NACK_ER Position */
#define MXC_F_I2C_INTFL0_ADDR_NACK_ER                  ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_ADDR_NACK_ER_POS)) /**< INTFL0_ADDR_NACK_ER Mask */

#define MXC_F_I2C_INTFL0_DATA_ER_POS                   11 /**< INTFL0_DATA_ER Position */
#define MXC_F_I2C_INTFL0_DATA_ER                       ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_DATA_ER_POS)) /**< INTFL0_DATA_ER Mask */

#define MXC_F_I2C_INTFL0_DO_NOT_RESP_ER_POS            12 /**< INTFL0_DO_NOT_RESP_ER Position */
#define MXC_F_I2C_INTFL0_DO_NOT_RESP_ER                ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_DO_NOT_RESP_ER_POS)) /**< INTFL0_DO_NOT_RESP_ER Mask */

#define MXC_F_I2C_INTFL0_START_ER_POS                  13 /**< INTFL0_START_ER Position */
#define MXC_F_I2C_INTFL0_START_ER                      ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_START_ER_POS)) /**< INTFL0_START_ER Mask */

#define MXC_F_I2C_INTFL0_STOP_ER_POS                   14 /**< INTFL0_STOP_ER Position */
#define MXC_F_I2C_INTFL0_STOP_ER                       ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_STOP_ER_POS)) /**< INTFL0_STOP_ER Mask */

#define MXC_F_I2C_INTFL0_TX_LOCK_OUT_POS               15 /**< INTFL0_TX_LOCK_OUT Position */
#define MXC_F_I2C_INTFL0_TX_LOCK_OUT                   ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_TX_LOCK_OUT_POS)) /**< INTFL0_TX_LOCK_OUT Mask */

#define MXC_F_I2C_INTFL0_RD_ADDR_MATCH_POS             22 /**< INTFL0_RD_ADDR_MATCH Position */
#define MXC_F_I2C_INTFL0_RD_ADDR_MATCH                 ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_RD_ADDR_MATCH_POS)) /**< INTFL0_RD_ADDR_MATCH Mask */

#define MXC_F_I2C_INTFL0_WR_ADDR_MATCH_POS             23 /**< INTFL0_WR_ADDR_MATCH Position */
#define MXC_F_I2C_INTFL0_WR_ADDR_MATCH                 ((uint32_t)(0x1UL << MXC_F_I2C_INTFL0_WR_ADDR_MATCH_POS)) /**< INTFL0_WR_ADDR_MATCH Mask */

/**@} end of group I2C_INTFL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INTEN0 I2C_INTEN0
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_I2C_INTEN0_DONE_POS                      0 /**< INTEN0_DONE Position */
#define MXC_F_I2C_INTEN0_DONE                          ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_DONE_POS)) /**< INTEN0_DONE Mask */

#define MXC_F_I2C_INTEN0_RX_MODE_POS                   1 /**< INTEN0_RX_MODE Position */
#define MXC_F_I2C_INTEN0_RX_MODE                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_RX_MODE_POS)) /**< INTEN0_RX_MODE Mask */

#define MXC_F_I2C_INTEN0_GEN_CALL_ADDR_POS             2 /**< INTEN0_GEN_CALL_ADDR Position */
#define MXC_F_I2C_INTEN0_GEN_CALL_ADDR                 ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_GEN_CALL_ADDR_POS)) /**< INTEN0_GEN_CALL_ADDR Mask */

#define MXC_F_I2C_INTEN0_ADDR_MATCH_POS                3 /**< INTEN0_ADDR_MATCH Position */
#define MXC_F_I2C_INTEN0_ADDR_MATCH                    ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ADDR_MATCH_POS)) /**< INTEN0_ADDR_MATCH Mask */

#define MXC_F_I2C_INTEN0_RX_THRESH_POS                 4 /**< INTEN0_RX_THRESH Position */
#define MXC_F_I2C_INTEN0_RX_THRESH                     ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_RX_THRESH_POS)) /**< INTEN0_RX_THRESH Mask */

#define MXC_F_I2C_INTEN0_TX_THRESH_POS                 5 /**< INTEN0_TX_THRESH Position */
#define MXC_F_I2C_INTEN0_TX_THRESH                     ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_TX_THRESH_POS)) /**< INTEN0_TX_THRESH Mask */

#define MXC_F_I2C_INTEN0_STOP_POS                      6 /**< INTEN0_STOP Position */
#define MXC_F_I2C_INTEN0_STOP                          ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_STOP_POS)) /**< INTEN0_STOP Mask */

#define MXC_F_I2C_INTEN0_ADDR_ACK_POS                  7 /**< INTEN0_ADDR_ACK Position */
#define MXC_F_I2C_INTEN0_ADDR_ACK                      ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ADDR_ACK_POS)) /**< INTEN0_ADDR_ACK Mask */

#define MXC_F_I2C_INTEN0_ARB_ER_POS                    8 /**< INTEN0_ARB_ER Position */
#define MXC_F_I2C_INTEN0_ARB_ER                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ARB_ER_POS)) /**< INTEN0_ARB_ER Mask */

#define MXC_F_I2C_INTEN0_TO_ER_POS                     9 /**< INTEN0_TO_ER Position */
#define MXC_F_I2C_INTEN0_TO_ER                         ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_TO_ER_POS)) /**< INTEN0_TO_ER Mask */

#define MXC_F_I2C_INTEN0_ADDR_NACK_ERR_POS             10 /**< INTEN0_ADDR_NACK_ERR Position */
#define MXC_F_I2C_INTEN0_ADDR_NACK_ERR                 ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ADDR_NACK_ERR_POS)) /**< INTEN0_ADDR_NACK_ERR Mask */

#define MXC_F_I2C_INTEN0_DATA_ER_POS                   11 /**< INTEN0_DATA_ER Position */
#define MXC_F_I2C_INTEN0_DATA_ER                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_DATA_ER_POS)) /**< INTEN0_DATA_ER Mask */

#define MXC_F_I2C_INTEN0_DO_NOT_RESP_ER_POS            12 /**< INTEN0_DO_NOT_RESP_ER Position */
#define MXC_F_I2C_INTEN0_DO_NOT_RESP_ER                ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_DO_NOT_RESP_ER_POS)) /**< INTEN0_DO_NOT_RESP_ER Mask */

#define MXC_F_I2C_INTEN0_START_ER_POS                  13 /**< INTEN0_START_ER Position */
#define MXC_F_I2C_INTEN0_START_ER                      ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_START_ER_POS)) /**< INTEN0_START_ER Mask */

#define MXC_F_I2C_INTEN0_STOP_ER_POS                   14 /**< INTEN0_STOP_ER Position */
#define MXC_F_I2C_INTEN0_STOP_ER                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_STOP_ER_POS)) /**< INTEN0_STOP_ER Mask */

#define MXC_F_I2C_INTEN0_TX_LOCK_OUT_POS               15 /**< INTEN0_TX_LOCK_OUT Position */
#define MXC_F_I2C_INTEN0_TX_LOCK_OUT                   ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_TX_LOCK_OUT_POS)) /**< INTEN0_TX_LOCK_OUT Mask */

#define MXC_F_I2C_INTEN0_RD_ADDR_MATCH_POS             22 /**< INTEN0_RD_ADDR_MATCH Position */
#define MXC_F_I2C_INTEN0_RD_ADDR_MATCH                 ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_RD_ADDR_MATCH_POS)) /**< INTEN0_RD_ADDR_MATCH Mask */

#define MXC_F_I2C_INTEN0_WR_ADDR_MATCH_POS             23 /**< INTEN0_WR_ADDR_MATCH Position */
#define MXC_F_I2C_INTEN0_WR_ADDR_MATCH                 ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_WR_ADDR_MATCH_POS)) /**< INTEN0_WR_ADDR_MATCH Mask */

/**@} end of group I2C_INTEN0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INTFL1 I2C_INTFL1
 * @brief    Interrupt Status Register 1.
 * @{
 */
#define MXC_F_I2C_INTFL1_RX_OVERFLOW_POS               0 /**< INTFL1_RX_OVERFLOW Position */
#define MXC_F_I2C_INTFL1_RX_OVERFLOW                   ((uint32_t)(0x1UL << MXC_F_I2C_INTFL1_RX_OVERFLOW_POS)) /**< INTFL1_RX_OVERFLOW Mask */

#define MXC_F_I2C_INTFL1_TX_UNDERFLOW_POS              1 /**< INTFL1_TX_UNDERFLOW Position */
#define MXC_F_I2C_INTFL1_TX_UNDERFLOW                  ((uint32_t)(0x1UL << MXC_F_I2C_INTFL1_TX_UNDERFLOW_POS)) /**< INTFL1_TX_UNDERFLOW Mask */

#define MXC_F_I2C_INTFL1_START_POS                     2 /**< INTFL1_START Position */
#define MXC_F_I2C_INTFL1_START                         ((uint32_t)(0x1UL << MXC_F_I2C_INTFL1_START_POS)) /**< INTFL1_START Mask */

/**@} end of group I2C_INTFL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INTEN1 I2C_INTEN1
 * @brief    Interrupt Staus Register 1.
 * @{
 */
#define MXC_F_I2C_INTEN1_RX_OVERFLOW_POS               0 /**< INTEN1_RX_OVERFLOW Position */
#define MXC_F_I2C_INTEN1_RX_OVERFLOW                   ((uint32_t)(0x1UL << MXC_F_I2C_INTEN1_RX_OVERFLOW_POS)) /**< INTEN1_RX_OVERFLOW Mask */

#define MXC_F_I2C_INTEN1_TX_UNDERFLOW_POS              1 /**< INTEN1_TX_UNDERFLOW Position */
#define MXC_F_I2C_INTEN1_TX_UNDERFLOW                  ((uint32_t)(0x1UL << MXC_F_I2C_INTEN1_TX_UNDERFLOW_POS)) /**< INTEN1_TX_UNDERFLOW Mask */

#define MXC_F_I2C_INTEN1_START_POS                     2 /**< INTEN1_START Position */
#define MXC_F_I2C_INTEN1_START                         ((uint32_t)(0x1UL << MXC_F_I2C_INTEN1_START_POS)) /**< INTEN1_START Mask */

/**@} end of group I2C_INTEN1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_FIFOLEN I2C_FIFOLEN
 * @brief    FIFO Configuration Register.
 * @{
 */
#define MXC_F_I2C_FIFOLEN_RX_LEN_POS                   0 /**< FIFOLEN_RX_LEN Position */
#define MXC_F_I2C_FIFOLEN_RX_LEN                       ((uint32_t)(0xFFUL << MXC_F_I2C_FIFOLEN_RX_LEN_POS)) /**< FIFOLEN_RX_LEN Mask */

#define MXC_F_I2C_FIFOLEN_TX_LEN_POS                   8 /**< FIFOLEN_TX_LEN Position */
#define MXC_F_I2C_FIFOLEN_TX_LEN                       ((uint32_t)(0xFFUL << MXC_F_I2C_FIFOLEN_TX_LEN_POS)) /**< FIFOLEN_TX_LEN Mask */

/**@} end of group I2C_FIFOLEN_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RXCTRL0 I2C_RXCTRL0
 * @brief    Receive Control Register 0.
 * @{
 */
#define MXC_F_I2C_RXCTRL0_DNR_POS                      0 /**< RXCTRL0_DNR Position */
#define MXC_F_I2C_RXCTRL0_DNR                          ((uint32_t)(0x1UL << MXC_F_I2C_RXCTRL0_DNR_POS)) /**< RXCTRL0_DNR Mask */

#define MXC_F_I2C_RXCTRL0_RX_FLUSH_POS                 7 /**< RXCTRL0_RX_FLUSH Position */
#define MXC_F_I2C_RXCTRL0_RX_FLUSH                     ((uint32_t)(0x1UL << MXC_F_I2C_RXCTRL0_RX_FLUSH_POS)) /**< RXCTRL0_RX_FLUSH Mask */

#define MXC_F_I2C_RXCTRL0_RX_THRESH_POS                8 /**< RXCTRL0_RX_THRESH Position */
#define MXC_F_I2C_RXCTRL0_RX_THRESH                    ((uint32_t)(0xFUL << MXC_F_I2C_RXCTRL0_RX_THRESH_POS)) /**< RXCTRL0_RX_THRESH Mask */

/**@} end of group I2C_RXCTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RXCTRL1 I2C_RXCTRL1
 * @brief    Receive Control Register 1.
 * @{
 */
#define MXC_F_I2C_RXCTRL1_RX_CNT_POS                   0 /**< RXCTRL1_RX_CNT Position */
#define MXC_F_I2C_RXCTRL1_RX_CNT                       ((uint32_t)(0xFFUL << MXC_F_I2C_RXCTRL1_RX_CNT_POS)) /**< RXCTRL1_RX_CNT Mask */

#define MXC_F_I2C_RXCTRL1_RX_FIFO_POS                  8 /**< RXCTRL1_RX_FIFO Position */
#define MXC_F_I2C_RXCTRL1_RX_FIFO                      ((uint32_t)(0xFUL << MXC_F_I2C_RXCTRL1_RX_FIFO_POS)) /**< RXCTRL1_RX_FIFO Mask */

/**@} end of group I2C_RXCTRL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TXCTRL0 I2C_TXCTRL0
 * @brief    Transmit Control Register 0.
 * @{
 */
#define MXC_F_I2C_TXCTRL0_TX_PRELOAD_POS               0 /**< TXCTRL0_TX_PRELOAD Position */
#define MXC_F_I2C_TXCTRL0_TX_PRELOAD                   ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_PRELOAD_POS)) /**< TXCTRL0_TX_PRELOAD Mask */

#define MXC_F_I2C_TXCTRL0_TX_READY_MODE_POS            1 /**< TXCTRL0_TX_READY_MODE Position */
#define MXC_F_I2C_TXCTRL0_TX_READY_MODE                ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_READY_MODE_POS)) /**< TXCTRL0_TX_READY_MODE Mask */

#define MXC_F_I2C_TXCTRL0_TX_AMGC_AFD_POS              2 /**< TXCTRL0_TX_AMGC_AFD Position */
#define MXC_F_I2C_TXCTRL0_TX_AMGC_AFD                  ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_AMGC_AFD_POS)) /**< TXCTRL0_TX_AMGC_AFD Mask */

#define MXC_F_I2C_TXCTRL0_TX_AMW_AFD_POS               3 /**< TXCTRL0_TX_AMW_AFD Position */
#define MXC_F_I2C_TXCTRL0_TX_AMW_AFD                   ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_AMW_AFD_POS)) /**< TXCTRL0_TX_AMW_AFD Mask */

#define MXC_F_I2C_TXCTRL0_TX_AMR_AFD_POS               4 /**< TXCTRL0_TX_AMR_AFD Position */
#define MXC_F_I2C_TXCTRL0_TX_AMR_AFD                   ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_AMR_AFD_POS)) /**< TXCTRL0_TX_AMR_AFD Mask */

#define MXC_F_I2C_TXCTRL0_TX_NACK_AFD_POS              5 /**< TXCTRL0_TX_NACK_AFD Position */
#define MXC_F_I2C_TXCTRL0_TX_NACK_AFD                  ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_NACK_AFD_POS)) /**< TXCTRL0_TX_NACK_AFD Mask */

#define MXC_F_I2C_TXCTRL0_TX_FLUSH_POS                 7 /**< TXCTRL0_TX_FLUSH Position */
#define MXC_F_I2C_TXCTRL0_TX_FLUSH                     ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL0_TX_FLUSH_POS)) /**< TXCTRL0_TX_FLUSH Mask */

#define MXC_F_I2C_TXCTRL0_TX_THRESH_POS                8 /**< TXCTRL0_TX_THRESH Position */
#define MXC_F_I2C_TXCTRL0_TX_THRESH                    ((uint32_t)(0xFUL << MXC_F_I2C_TXCTRL0_TX_THRESH_POS)) /**< TXCTRL0_TX_THRESH Mask */

/**@} end of group I2C_TXCTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TXCTRL1 I2C_TXCTRL1
 * @brief    Transmit Control Register 1.
 * @{
 */
#define MXC_F_I2C_TXCTRL1_TX_READY_POS                 0 /**< TXCTRL1_TX_READY Position */
#define MXC_F_I2C_TXCTRL1_TX_READY                     ((uint32_t)(0x1UL << MXC_F_I2C_TXCTRL1_TX_READY_POS)) /**< TXCTRL1_TX_READY Mask */

#define MXC_F_I2C_TXCTRL1_TX_FIFO_POS                  8 /**< TXCTRL1_TX_FIFO Position */
#define MXC_F_I2C_TXCTRL1_TX_FIFO                      ((uint32_t)(0xFUL << MXC_F_I2C_TXCTRL1_TX_FIFO_POS)) /**< TXCTRL1_TX_FIFO Mask */

/**@} end of group I2C_TXCTRL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_FIFO I2C_FIFO
 * @brief    Data Register.
 * @{
 */
#define MXC_F_I2C_FIFO_DATA_POS                        0 /**< FIFO_DATA Position */
#define MXC_F_I2C_FIFO_DATA                            ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_DATA_POS)) /**< FIFO_DATA Mask */

/**@} end of group I2C_FIFO_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_MSTCTRL I2C_MSTCTRL
 * @brief    Master Control Register.
 * @{
 */
#define MXC_F_I2C_MSTCTRL_START_POS                    0 /**< MSTCTRL_START Position */
#define MXC_F_I2C_MSTCTRL_START                        ((uint32_t)(0x1UL << MXC_F_I2C_MSTCTRL_START_POS)) /**< MSTCTRL_START Mask */

#define MXC_F_I2C_MSTCTRL_RESTART_POS                  1 /**< MSTCTRL_RESTART Position */
#define MXC_F_I2C_MSTCTRL_RESTART                      ((uint32_t)(0x1UL << MXC_F_I2C_MSTCTRL_RESTART_POS)) /**< MSTCTRL_RESTART Mask */

#define MXC_F_I2C_MSTCTRL_STOP_POS                     2 /**< MSTCTRL_STOP Position */
#define MXC_F_I2C_MSTCTRL_STOP                         ((uint32_t)(0x1UL << MXC_F_I2C_MSTCTRL_STOP_POS)) /**< MSTCTRL_STOP Mask */

#define MXC_F_I2C_MSTCTRL_SL_EX_ADDR_POS               7 /**< MSTCTRL_SL_EX_ADDR Position */
#define MXC_F_I2C_MSTCTRL_SL_EX_ADDR                   ((uint32_t)(0x1UL << MXC_F_I2C_MSTCTRL_SL_EX_ADDR_POS)) /**< MSTCTRL_SL_EX_ADDR Mask */

/**@} end of group I2C_MSTCTRL_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CLKLO I2C_CLKLO
 * @brief    Clock Low Register.
 * @{
 */
#define MXC_F_I2C_CLKLO_SCL_LO_POS                     0 /**< CLKLO_SCL_LO Position */
#define MXC_F_I2C_CLKLO_SCL_LO                         ((uint32_t)(0x1FFUL << MXC_F_I2C_CLKLO_SCL_LO_POS)) /**< CLKLO_SCL_LO Mask */

/**@} end of group I2C_CLKLO_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CLKHI I2C_CLKHI
 * @brief    Clock high Register.
 * @{
 */
#define MXC_F_I2C_CLKHI_SCL_HI_POS                     0 /**< CLKHI_SCL_HI Position */
#define MXC_F_I2C_CLKHI_SCL_HI                         ((uint32_t)(0x1FFUL << MXC_F_I2C_CLKHI_SCL_HI_POS)) /**< CLKHI_SCL_HI Mask */

/**@} end of group I2C_CLKHI_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TIMEOUT I2C_TIMEOUT
 * @brief    Timeout Register
 * @{
 */
#define MXC_F_I2C_TIMEOUT_TO_POS                       0 /**< TIMEOUT_TO Position */
#define MXC_F_I2C_TIMEOUT_TO                           ((uint32_t)(0xFFFFUL << MXC_F_I2C_TIMEOUT_TO_POS)) /**< TIMEOUT_TO Mask */

/**@} end of group I2C_TIMEOUT_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_DMA I2C_DMA
 * @brief    DMA Register.
 * @{
 */
#define MXC_F_I2C_DMA_TX_EN_POS                        0 /**< DMA_TX_EN Position */
#define MXC_F_I2C_DMA_TX_EN                            ((uint32_t)(0x1UL << MXC_F_I2C_DMA_TX_EN_POS)) /**< DMA_TX_EN Mask */

#define MXC_F_I2C_DMA_RX_EN_POS                        1 /**< DMA_RX_EN Position */
#define MXC_F_I2C_DMA_RX_EN                            ((uint32_t)(0x1UL << MXC_F_I2C_DMA_RX_EN_POS)) /**< DMA_RX_EN Mask */

/**@} end of group I2C_DMA_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_SLAVE_ADDR I2C_SLAVE_ADDR
 * @brief    Slave Address Register.
 * @{
 */
#define MXC_F_I2C_SLAVE_ADDR_SLAVE_ADDR_POS            0 /**< SLAVE_ADDR_SLAVE_ADDR Position */
#define MXC_F_I2C_SLAVE_ADDR_SLAVE_ADDR                ((uint32_t)(0x3FFUL << MXC_F_I2C_SLAVE_ADDR_SLAVE_ADDR_POS)) /**< SLAVE_ADDR_SLAVE_ADDR Mask */

#define MXC_F_I2C_SLAVE_ADDR_EX_ADDR_POS               15 /**< SLAVE_ADDR_EX_ADDR Position */
#define MXC_F_I2C_SLAVE_ADDR_EX_ADDR                   ((uint32_t)(0x1UL << MXC_F_I2C_SLAVE_ADDR_EX_ADDR_POS)) /**< SLAVE_ADDR_EX_ADDR Mask */

/**@} end of group I2C_SLAVE_ADDR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_I2C_REGS_H_

/**
 * @file    i2c_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the I2C Peripheral Module.
 * @note    This file is @generated.
 * @ingroup i2c_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_I2C_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_I2C_REGS_H_

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
    __IO uint32_t int_fl0;              /**< <tt>\b 0x08:</tt> I2C INT_FL0 Register */
    __IO uint32_t int_en0;              /**< <tt>\b 0x0C:</tt> I2C INT_EN0 Register */
    __IO uint32_t int_fl1;              /**< <tt>\b 0x10:</tt> I2C INT_FL1 Register */
    __IO uint32_t int_en1;              /**< <tt>\b 0x14:</tt> I2C INT_EN1 Register */
    __IO uint32_t fifo_len;             /**< <tt>\b 0x18:</tt> I2C FIFO_LEN Register */
    __IO uint32_t rx_ctrl0;             /**< <tt>\b 0x1C:</tt> I2C RX_CTRL0 Register */
    __IO uint32_t rx_ctrl1;             /**< <tt>\b 0x20:</tt> I2C RX_CTRL1 Register */
    __IO uint32_t tx_ctrl0;             /**< <tt>\b 0x24:</tt> I2C TX_CTRL0 Register */
    __IO uint32_t tx_ctrl1;             /**< <tt>\b 0x28:</tt> I2C TX_CTRL1 Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x2C:</tt> I2C FIFO Register */
    __IO uint32_t master_ctrl;          /**< <tt>\b 0x30:</tt> I2C MASTER_CTRL Register */
    __IO uint32_t clk_lo;               /**< <tt>\b 0x34:</tt> I2C CLK_LO Register */
    __IO uint32_t clk_hi;               /**< <tt>\b 0x38:</tt> I2C CLK_HI Register */
    __IO uint32_t hs_clk;               /**< <tt>\b 0x3C:</tt> I2C HS_CLK Register */
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
#define MXC_R_I2C_INT_FL0                  ((uint32_t)0x00000008UL) /**< Offset from I2C Base Address: <tt> 0x0008</tt> */
#define MXC_R_I2C_INT_EN0                  ((uint32_t)0x0000000CUL) /**< Offset from I2C Base Address: <tt> 0x000C</tt> */
#define MXC_R_I2C_INT_FL1                  ((uint32_t)0x00000010UL) /**< Offset from I2C Base Address: <tt> 0x0010</tt> */
#define MXC_R_I2C_INT_EN1                  ((uint32_t)0x00000014UL) /**< Offset from I2C Base Address: <tt> 0x0014</tt> */
#define MXC_R_I2C_FIFO_LEN                 ((uint32_t)0x00000018UL) /**< Offset from I2C Base Address: <tt> 0x0018</tt> */
#define MXC_R_I2C_RX_CTRL0                 ((uint32_t)0x0000001CUL) /**< Offset from I2C Base Address: <tt> 0x001C</tt> */
#define MXC_R_I2C_RX_CTRL1                 ((uint32_t)0x00000020UL) /**< Offset from I2C Base Address: <tt> 0x0020</tt> */
#define MXC_R_I2C_TX_CTRL0                 ((uint32_t)0x00000024UL) /**< Offset from I2C Base Address: <tt> 0x0024</tt> */
#define MXC_R_I2C_TX_CTRL1                 ((uint32_t)0x00000028UL) /**< Offset from I2C Base Address: <tt> 0x0028</tt> */
#define MXC_R_I2C_FIFO                     ((uint32_t)0x0000002CUL) /**< Offset from I2C Base Address: <tt> 0x002C</tt> */
#define MXC_R_I2C_MASTER_CTRL              ((uint32_t)0x00000030UL) /**< Offset from I2C Base Address: <tt> 0x0030</tt> */
#define MXC_R_I2C_CLK_LO                   ((uint32_t)0x00000034UL) /**< Offset from I2C Base Address: <tt> 0x0034</tt> */
#define MXC_R_I2C_CLK_HI                   ((uint32_t)0x00000038UL) /**< Offset from I2C Base Address: <tt> 0x0038</tt> */
#define MXC_R_I2C_HS_CLK                   ((uint32_t)0x0000003CUL) /**< Offset from I2C Base Address: <tt> 0x003C</tt> */
#define MXC_R_I2C_TIMEOUT                  ((uint32_t)0x00000040UL) /**< Offset from I2C Base Address: <tt> 0x0040</tt> */
#define MXC_R_I2C_DMA                      ((uint32_t)0x00000048UL) /**< Offset from I2C Base Address: <tt> 0x0048</tt> */
#define MXC_R_I2C_SLAVE_ADDR               ((uint32_t)0x0000004CUL) /**< Offset from I2C Base Address: <tt> 0x004C</tt> */
/**@} end of group i2c_registers */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CTRL I2C_CTRL
 * @brief    Control Register.
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

#define MXC_F_I2C_CTRL_HS_MODE_POS                     15 /**< CTRL_HS_MODE Position */
#define MXC_F_I2C_CTRL_HS_MODE                         ((uint32_t)(0x1UL << MXC_F_I2C_CTRL_HS_MODE_POS)) /**< CTRL_HS_MODE Mask */

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
 * @defgroup I2C_INT_FL0 I2C_INT_FL0
 * @brief    Interrupt Status Register.
 * @{
 */
#define MXC_F_I2C_INT_FL0_DONE_POS                     0 /**< INT_FL0_DONE Position */
#define MXC_F_I2C_INT_FL0_DONE                         ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_DONE_POS)) /**< INT_FL0_DONE Mask */

#define MXC_F_I2C_INT_FL0_RX_MODE_POS                  1 /**< INT_FL0_RX_MODE Position */
#define MXC_F_I2C_INT_FL0_RX_MODE                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_RX_MODE_POS)) /**< INT_FL0_RX_MODE Mask */

#define MXC_F_I2C_INT_FL0_GEN_CALL_ADDR_POS            2 /**< INT_FL0_GEN_CALL_ADDR Position */
#define MXC_F_I2C_INT_FL0_GEN_CALL_ADDR                ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_GEN_CALL_ADDR_POS)) /**< INT_FL0_GEN_CALL_ADDR Mask */

#define MXC_F_I2C_INT_FL0_ADDR_MATCH_POS               3 /**< INT_FL0_ADDR_MATCH Position */
#define MXC_F_I2C_INT_FL0_ADDR_MATCH                   ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ADDR_MATCH_POS)) /**< INT_FL0_ADDR_MATCH Mask */

#define MXC_F_I2C_INT_FL0_RX_THRESH_POS                4 /**< INT_FL0_RX_THRESH Position */
#define MXC_F_I2C_INT_FL0_RX_THRESH                    ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_RX_THRESH_POS)) /**< INT_FL0_RX_THRESH Mask */

#define MXC_F_I2C_INT_FL0_TX_THRESH_POS                5 /**< INT_FL0_TX_THRESH Position */
#define MXC_F_I2C_INT_FL0_TX_THRESH                    ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_TX_THRESH_POS)) /**< INT_FL0_TX_THRESH Mask */

#define MXC_F_I2C_INT_FL0_STOP_POS                     6 /**< INT_FL0_STOP Position */
#define MXC_F_I2C_INT_FL0_STOP                         ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_STOP_POS)) /**< INT_FL0_STOP Mask */

#define MXC_F_I2C_INT_FL0_ADDR_ACK_POS                 7 /**< INT_FL0_ADDR_ACK Position */
#define MXC_F_I2C_INT_FL0_ADDR_ACK                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ADDR_ACK_POS)) /**< INT_FL0_ADDR_ACK Mask */

#define MXC_F_I2C_INT_FL0_ARB_ER_POS                   8 /**< INT_FL0_ARB_ER Position */
#define MXC_F_I2C_INT_FL0_ARB_ER                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ARB_ER_POS)) /**< INT_FL0_ARB_ER Mask */

#define MXC_F_I2C_INT_FL0_TO_ER_POS                    9 /**< INT_FL0_TO_ER Position */
#define MXC_F_I2C_INT_FL0_TO_ER                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_TO_ER_POS)) /**< INT_FL0_TO_ER Mask */

#define MXC_F_I2C_INT_FL0_ADDR_NACK_ER_POS             10 /**< INT_FL0_ADDR_NACK_ER Position */
#define MXC_F_I2C_INT_FL0_ADDR_NACK_ER                 ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ADDR_NACK_ER_POS)) /**< INT_FL0_ADDR_NACK_ER Mask */

#define MXC_F_I2C_INT_FL0_DATA_ER_POS                  11 /**< INT_FL0_DATA_ER Position */
#define MXC_F_I2C_INT_FL0_DATA_ER                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_DATA_ER_POS)) /**< INT_FL0_DATA_ER Mask */

#define MXC_F_I2C_INT_FL0_DO_NOT_RESP_ER_POS           12 /**< INT_FL0_DO_NOT_RESP_ER Position */
#define MXC_F_I2C_INT_FL0_DO_NOT_RESP_ER               ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_DO_NOT_RESP_ER_POS)) /**< INT_FL0_DO_NOT_RESP_ER Mask */

#define MXC_F_I2C_INT_FL0_START_ER_POS                 13 /**< INT_FL0_START_ER Position */
#define MXC_F_I2C_INT_FL0_START_ER                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_START_ER_POS)) /**< INT_FL0_START_ER Mask */

#define MXC_F_I2C_INT_FL0_STOP_ER_POS                  14 /**< INT_FL0_STOP_ER Position */
#define MXC_F_I2C_INT_FL0_STOP_ER                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_STOP_ER_POS)) /**< INT_FL0_STOP_ER Mask */

#define MXC_F_I2C_INT_FL0_TX_LOCK_OUT_POS              15 /**< INT_FL0_TX_LOCK_OUT Position */
#define MXC_F_I2C_INT_FL0_TX_LOCK_OUT                  ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_TX_LOCK_OUT_POS)) /**< INT_FL0_TX_LOCK_OUT Mask */

#define MXC_F_I2C_INT_FL0_RD_ADDR_MATCH_POS            22 /**< INT_FL0_RD_ADDR_MATCH Position */
#define MXC_F_I2C_INT_FL0_RD_ADDR_MATCH                ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_RD_ADDR_MATCH_POS)) /**< INT_FL0_RD_ADDR_MATCH Mask */

#define MXC_F_I2C_INT_FL0_WR_ADDR_MATCH_POS            23 /**< INT_FL0_WR_ADDR_MATCH Position */
#define MXC_F_I2C_INT_FL0_WR_ADDR_MATCH                ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_WR_ADDR_MATCH_POS)) /**< INT_FL0_WR_ADDR_MATCH Mask */

/**@} end of group I2C_INT_FL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_EN0 I2C_INT_EN0
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_I2C_INT_EN0_DONE_POS                     0 /**< INT_EN0_DONE Position */
#define MXC_F_I2C_INT_EN0_DONE                         ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_DONE_POS)) /**< INT_EN0_DONE Mask */

#define MXC_F_I2C_INT_EN0_RX_MODE_POS                  1 /**< INT_EN0_RX_MODE Position */
#define MXC_F_I2C_INT_EN0_RX_MODE                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_RX_MODE_POS)) /**< INT_EN0_RX_MODE Mask */

#define MXC_F_I2C_INT_EN0_GEN_CALL_ADDR_POS            2 /**< INT_EN0_GEN_CALL_ADDR Position */
#define MXC_F_I2C_INT_EN0_GEN_CALL_ADDR                ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_GEN_CALL_ADDR_POS)) /**< INT_EN0_GEN_CALL_ADDR Mask */

#define MXC_F_I2C_INT_EN0_ADDR_MATCH_POS               3 /**< INT_EN0_ADDR_MATCH Position */
#define MXC_F_I2C_INT_EN0_ADDR_MATCH                   ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ADDR_MATCH_POS)) /**< INT_EN0_ADDR_MATCH Mask */

#define MXC_F_I2C_INT_EN0_RX_THRESH_POS                4 /**< INT_EN0_RX_THRESH Position */
#define MXC_F_I2C_INT_EN0_RX_THRESH                    ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_RX_THRESH_POS)) /**< INT_EN0_RX_THRESH Mask */

#define MXC_F_I2C_INT_EN0_TX_THRESH_POS                5 /**< INT_EN0_TX_THRESH Position */
#define MXC_F_I2C_INT_EN0_TX_THRESH                    ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_TX_THRESH_POS)) /**< INT_EN0_TX_THRESH Mask */

#define MXC_F_I2C_INT_EN0_STOP_POS                     6 /**< INT_EN0_STOP Position */
#define MXC_F_I2C_INT_EN0_STOP                         ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_STOP_POS)) /**< INT_EN0_STOP Mask */

#define MXC_F_I2C_INT_EN0_ADDR_ACK_POS                 7 /**< INT_EN0_ADDR_ACK Position */
#define MXC_F_I2C_INT_EN0_ADDR_ACK                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ADDR_ACK_POS)) /**< INT_EN0_ADDR_ACK Mask */

#define MXC_F_I2C_INT_EN0_ARB_ER_POS                   8 /**< INT_EN0_ARB_ER Position */
#define MXC_F_I2C_INT_EN0_ARB_ER                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ARB_ER_POS)) /**< INT_EN0_ARB_ER Mask */

#define MXC_F_I2C_INT_EN0_TO_ER_POS                    9 /**< INT_EN0_TO_ER Position */
#define MXC_F_I2C_INT_EN0_TO_ER                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_TO_ER_POS)) /**< INT_EN0_TO_ER Mask */

#define MXC_F_I2C_INT_EN0_ADDR_NACK_ER_POS             10 /**< INT_EN0_ADDR_NACK_ER Position */
#define MXC_F_I2C_INT_EN0_ADDR_NACK_ER                 ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ADDR_NACK_ER_POS)) /**< INT_EN0_ADDR_NACK_ER Mask */

#define MXC_F_I2C_INT_EN0_DATA_ER_POS                  11 /**< INT_EN0_DATA_ER Position */
#define MXC_F_I2C_INT_EN0_DATA_ER                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_DATA_ER_POS)) /**< INT_EN0_DATA_ER Mask */

#define MXC_F_I2C_INT_EN0_DO_NOT_RESP_ER_POS           12 /**< INT_EN0_DO_NOT_RESP_ER Position */
#define MXC_F_I2C_INT_EN0_DO_NOT_RESP_ER               ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_DO_NOT_RESP_ER_POS)) /**< INT_EN0_DO_NOT_RESP_ER Mask */

#define MXC_F_I2C_INT_EN0_START_ER_POS                 13 /**< INT_EN0_START_ER Position */
#define MXC_F_I2C_INT_EN0_START_ER                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_START_ER_POS)) /**< INT_EN0_START_ER Mask */

#define MXC_F_I2C_INT_EN0_STOP_ER_POS                  14 /**< INT_EN0_STOP_ER Position */
#define MXC_F_I2C_INT_EN0_STOP_ER                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_STOP_ER_POS)) /**< INT_EN0_STOP_ER Mask */

#define MXC_F_I2C_INT_EN0_TX_LOCK_OUT_POS              15 /**< INT_EN0_TX_LOCK_OUT Position */
#define MXC_F_I2C_INT_EN0_TX_LOCK_OUT                  ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_TX_LOCK_OUT_POS)) /**< INT_EN0_TX_LOCK_OUT Mask */

#define MXC_F_I2C_INT_EN0_RD_ADDR_MATCH_POS            22 /**< INT_EN0_RD_ADDR_MATCH Position */
#define MXC_F_I2C_INT_EN0_RD_ADDR_MATCH                ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_RD_ADDR_MATCH_POS)) /**< INT_EN0_RD_ADDR_MATCH Mask */

#define MXC_F_I2C_INT_EN0_WR_ADDR_MATCH_POS            23 /**< INT_EN0_WR_ADDR_MATCH Position */
#define MXC_F_I2C_INT_EN0_WR_ADDR_MATCH                ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_WR_ADDR_MATCH_POS)) /**< INT_EN0_WR_ADDR_MATCH Mask */

/**@} end of group I2C_INT_EN0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_FL1 I2C_INT_FL1
 * @brief    Interrupt Status Register 1.
 * @{
 */
#define MXC_F_I2C_INT_FL1_RX_OVERFLOW_POS              0 /**< INT_FL1_RX_OVERFLOW Position */
#define MXC_F_I2C_INT_FL1_RX_OVERFLOW                  ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL1_RX_OVERFLOW_POS)) /**< INT_FL1_RX_OVERFLOW Mask */

#define MXC_F_I2C_INT_FL1_TX_UNDERFLOW_POS             1 /**< INT_FL1_TX_UNDERFLOW Position */
#define MXC_F_I2C_INT_FL1_TX_UNDERFLOW                 ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL1_TX_UNDERFLOW_POS)) /**< INT_FL1_TX_UNDERFLOW Mask */

#define MXC_F_I2C_INT_FL1_START_POS                    2 /**< INT_FL1_START Position */
#define MXC_F_I2C_INT_FL1_START                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL1_START_POS)) /**< INT_FL1_START Mask */

/**@} end of group I2C_INT_FL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_EN1 I2C_INT_EN1
 * @brief    Interrupt Staus Register 1.
 * @{
 */
#define MXC_F_I2C_INT_EN1_RX_OVERFLOW_POS              0 /**< INT_EN1_RX_OVERFLOW Position */
#define MXC_F_I2C_INT_EN1_RX_OVERFLOW                  ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN1_RX_OVERFLOW_POS)) /**< INT_EN1_RX_OVERFLOW Mask */

#define MXC_F_I2C_INT_EN1_TX_UNDERFLOW_POS             1 /**< INT_EN1_TX_UNDERFLOW Position */
#define MXC_F_I2C_INT_EN1_TX_UNDERFLOW                 ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN1_TX_UNDERFLOW_POS)) /**< INT_EN1_TX_UNDERFLOW Mask */

#define MXC_F_I2C_INT_EN1_START_POS                    2 /**< INT_EN1_START Position */
#define MXC_F_I2C_INT_EN1_START                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN1_START_POS)) /**< INT_EN1_START Mask */

/**@} end of group I2C_INT_EN1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_FIFO_LEN I2C_FIFO_LEN
 * @brief    FIFO Configuration Register.
 * @{
 */
#define MXC_F_I2C_FIFO_LEN_RX_LEN_POS                  0 /**< FIFO_LEN_RX_LEN Position */
#define MXC_F_I2C_FIFO_LEN_RX_LEN                      ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_LEN_RX_LEN_POS)) /**< FIFO_LEN_RX_LEN Mask */

#define MXC_F_I2C_FIFO_LEN_TX_LEN_POS                  8 /**< FIFO_LEN_TX_LEN Position */
#define MXC_F_I2C_FIFO_LEN_TX_LEN                      ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_LEN_TX_LEN_POS)) /**< FIFO_LEN_TX_LEN Mask */

/**@} end of group I2C_FIFO_LEN_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RX_CTRL0 I2C_RX_CTRL0
 * @brief    Receive Control Register 0.
 * @{
 */
#define MXC_F_I2C_RX_CTRL0_DNR_POS                     0 /**< RX_CTRL0_DNR Position */
#define MXC_F_I2C_RX_CTRL0_DNR                         ((uint32_t)(0x1UL << MXC_F_I2C_RX_CTRL0_DNR_POS)) /**< RX_CTRL0_DNR Mask */

#define MXC_F_I2C_RX_CTRL0_RX_FLUSH_POS                7 /**< RX_CTRL0_RX_FLUSH Position */
#define MXC_F_I2C_RX_CTRL0_RX_FLUSH                    ((uint32_t)(0x1UL << MXC_F_I2C_RX_CTRL0_RX_FLUSH_POS)) /**< RX_CTRL0_RX_FLUSH Mask */

#define MXC_F_I2C_RX_CTRL0_RX_THRESH_POS               8 /**< RX_CTRL0_RX_THRESH Position */
#define MXC_F_I2C_RX_CTRL0_RX_THRESH                   ((uint32_t)(0xFUL << MXC_F_I2C_RX_CTRL0_RX_THRESH_POS)) /**< RX_CTRL0_RX_THRESH Mask */

/**@} end of group I2C_RX_CTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RX_CTRL1 I2C_RX_CTRL1
 * @brief    Receive Control Register 1.
 * @{
 */
#define MXC_F_I2C_RX_CTRL1_RX_CNT_POS                  0 /**< RX_CTRL1_RX_CNT Position */
#define MXC_F_I2C_RX_CTRL1_RX_CNT                      ((uint32_t)(0xFFUL << MXC_F_I2C_RX_CTRL1_RX_CNT_POS)) /**< RX_CTRL1_RX_CNT Mask */

#define MXC_F_I2C_RX_CTRL1_RX_FIFO_POS                 8 /**< RX_CTRL1_RX_FIFO Position */
#define MXC_F_I2C_RX_CTRL1_RX_FIFO                     ((uint32_t)(0xFUL << MXC_F_I2C_RX_CTRL1_RX_FIFO_POS)) /**< RX_CTRL1_RX_FIFO Mask */

/**@} end of group I2C_RX_CTRL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TX_CTRL0 I2C_TX_CTRL0
 * @brief    Transmit Control Register 0.
 * @{
 */
#define MXC_F_I2C_TX_CTRL0_TX_PRELOAD_POS              0 /**< TX_CTRL0_TX_PRELOAD Position */
#define MXC_F_I2C_TX_CTRL0_TX_PRELOAD                  ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_PRELOAD_POS)) /**< TX_CTRL0_TX_PRELOAD Mask */

#define MXC_F_I2C_TX_CTRL0_TX_READY_MODE_POS           1 /**< TX_CTRL0_TX_READY_MODE Position */
#define MXC_F_I2C_TX_CTRL0_TX_READY_MODE               ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_READY_MODE_POS)) /**< TX_CTRL0_TX_READY_MODE Mask */

#define MXC_F_I2C_TX_CTRL0_TX_AMGC_AFD_POS             2 /**< TX_CTRL0_TX_AMGC_AFD Position */
#define MXC_F_I2C_TX_CTRL0_TX_AMGC_AFD                 ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_AMGC_AFD_POS)) /**< TX_CTRL0_TX_AMGC_AFD Mask */

#define MXC_F_I2C_TX_CTRL0_TX_AMW_AFD_POS              3 /**< TX_CTRL0_TX_AMW_AFD Position */
#define MXC_F_I2C_TX_CTRL0_TX_AMW_AFD                  ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_AMW_AFD_POS)) /**< TX_CTRL0_TX_AMW_AFD Mask */

#define MXC_F_I2C_TX_CTRL0_TX_AMR_AFD_POS              4 /**< TX_CTRL0_TX_AMR_AFD Position */
#define MXC_F_I2C_TX_CTRL0_TX_AMR_AFD                  ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_AMR_AFD_POS)) /**< TX_CTRL0_TX_AMR_AFD Mask */

#define MXC_F_I2C_TX_CTRL0_TX_NACK_AFD_POS             5 /**< TX_CTRL0_TX_NACK_AFD Position */
#define MXC_F_I2C_TX_CTRL0_TX_NACK_AFD                 ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_NACK_AFD_POS)) /**< TX_CTRL0_TX_NACK_AFD Mask */

#define MXC_F_I2C_TX_CTRL0_TX_FLUSH_POS                7 /**< TX_CTRL0_TX_FLUSH Position */
#define MXC_F_I2C_TX_CTRL0_TX_FLUSH                    ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TX_FLUSH_POS)) /**< TX_CTRL0_TX_FLUSH Mask */

#define MXC_F_I2C_TX_CTRL0_TX_THRESH_POS               8 /**< TX_CTRL0_TX_THRESH Position */
#define MXC_F_I2C_TX_CTRL0_TX_THRESH                   ((uint32_t)(0xFUL << MXC_F_I2C_TX_CTRL0_TX_THRESH_POS)) /**< TX_CTRL0_TX_THRESH Mask */

/**@} end of group I2C_TX_CTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TX_CTRL1 I2C_TX_CTRL1
 * @brief    Transmit Control Register 1.
 * @{
 */
#define MXC_F_I2C_TX_CTRL1_TX_READY_POS                0 /**< TX_CTRL1_TX_READY Position */
#define MXC_F_I2C_TX_CTRL1_TX_READY                    ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL1_TX_READY_POS)) /**< TX_CTRL1_TX_READY Mask */

#define MXC_F_I2C_TX_CTRL1_TXFIFO_POS                  8 /**< TX_CTRL1_TXFIFO Position */
#define MXC_F_I2C_TX_CTRL1_TXFIFO                      ((uint32_t)(0xFUL << MXC_F_I2C_TX_CTRL1_TXFIFO_POS)) /**< TX_CTRL1_TXFIFO Mask */

/**@} end of group I2C_TX_CTRL1_Register */

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
 * @defgroup I2C_MASTER_CTRL I2C_MASTER_CTRL
 * @brief    Master Control Register.
 * @{
 */
#define MXC_F_I2C_MASTER_CTRL_START_POS                0 /**< MASTER_CTRL_START Position */
#define MXC_F_I2C_MASTER_CTRL_START                    ((uint32_t)(0x1UL << MXC_F_I2C_MASTER_CTRL_START_POS)) /**< MASTER_CTRL_START Mask */

#define MXC_F_I2C_MASTER_CTRL_RESTART_POS              1 /**< MASTER_CTRL_RESTART Position */
#define MXC_F_I2C_MASTER_CTRL_RESTART                  ((uint32_t)(0x1UL << MXC_F_I2C_MASTER_CTRL_RESTART_POS)) /**< MASTER_CTRL_RESTART Mask */

#define MXC_F_I2C_MASTER_CTRL_STOP_POS                 2 /**< MASTER_CTRL_STOP Position */
#define MXC_F_I2C_MASTER_CTRL_STOP                     ((uint32_t)(0x1UL << MXC_F_I2C_MASTER_CTRL_STOP_POS)) /**< MASTER_CTRL_STOP Mask */

#define MXC_F_I2C_MASTER_CTRL_SL_EX_ADDR_POS           7 /**< MASTER_CTRL_SL_EX_ADDR Position */
#define MXC_F_I2C_MASTER_CTRL_SL_EX_ADDR               ((uint32_t)(0x1UL << MXC_F_I2C_MASTER_CTRL_SL_EX_ADDR_POS)) /**< MASTER_CTRL_SL_EX_ADDR Mask */

#define MXC_F_I2C_MASTER_CTRL_MCODE_POS                8 /**< MASTER_CTRL_MCODE Position */
#define MXC_F_I2C_MASTER_CTRL_MCODE                    ((uint32_t)(0x7UL << MXC_F_I2C_MASTER_CTRL_MCODE_POS)) /**< MASTER_CTRL_MCODE Mask */

#define MXC_F_I2C_MASTER_CTRL_SCL_SPEED_UP_POS         11 /**< MASTER_CTRL_SCL_SPEED_UP Position */
#define MXC_F_I2C_MASTER_CTRL_SCL_SPEED_UP             ((uint32_t)(0x1UL << MXC_F_I2C_MASTER_CTRL_SCL_SPEED_UP_POS)) /**< MASTER_CTRL_SCL_SPEED_UP Mask */

/**@} end of group I2C_MASTER_CTRL_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CLK_LO I2C_CLK_LO
 * @brief    Clock Low Register.
 * @{
 */
#define MXC_F_I2C_CLK_LO_SCL_LO_POS                    0 /**< CLK_LO_SCL_LO Position */
#define MXC_F_I2C_CLK_LO_SCL_LO                        ((uint32_t)(0x1FFUL << MXC_F_I2C_CLK_LO_SCL_LO_POS)) /**< CLK_LO_SCL_LO Mask */

/**@} end of group I2C_CLK_LO_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CLK_HI I2C_CLK_HI
 * @brief    Clock high Register.
 * @{
 */
#define MXC_F_I2C_CLK_HI_SCL_HI_POS                    0 /**< CLK_HI_SCL_HI Position */
#define MXC_F_I2C_CLK_HI_SCL_HI                        ((uint32_t)(0x1FFUL << MXC_F_I2C_CLK_HI_SCL_HI_POS)) /**< CLK_HI_SCL_HI Mask */

/**@} end of group I2C_CLK_HI_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_HS_CLK I2C_HS_CLK
 * @brief    HS-Mode Clock Control Register
 * @{
 */
#define MXC_F_I2C_HS_CLK_HS_CLK_LO_POS                 0 /**< HS_CLK_HS_CLK_LO Position */
#define MXC_F_I2C_HS_CLK_HS_CLK_LO                     ((uint32_t)(0xFFUL << MXC_F_I2C_HS_CLK_HS_CLK_LO_POS)) /**< HS_CLK_HS_CLK_LO Mask */

#define MXC_F_I2C_HS_CLK_HS_CLK_HI_POS                 8 /**< HS_CLK_HS_CLK_HI Position */
#define MXC_F_I2C_HS_CLK_HS_CLK_HI                     ((uint32_t)(0xFFUL << MXC_F_I2C_HS_CLK_HS_CLK_HI_POS)) /**< HS_CLK_HS_CLK_HI Mask */

/**@} end of group I2C_HS_CLK_Register */

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
#define MXC_F_I2C_DMA_TXEN_POS                         0 /**< DMA_TXEN Position */
#define MXC_F_I2C_DMA_TXEN                             ((uint32_t)(0x1UL << MXC_F_I2C_DMA_TXEN_POS)) /**< DMA_TXEN Mask */

#define MXC_F_I2C_DMA_RXEN_POS                         1 /**< DMA_RXEN Position */
#define MXC_F_I2C_DMA_RXEN                             ((uint32_t)(0x1UL << MXC_F_I2C_DMA_RXEN_POS)) /**< DMA_RXEN Mask */

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

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_I2C_REGS_H_

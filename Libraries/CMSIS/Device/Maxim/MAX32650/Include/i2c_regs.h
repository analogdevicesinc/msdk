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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_I2C_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_I2C_REGS_H_

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
    __IO uint32_t ctrl0;                /**< <tt>\b 0x00:</tt> I2C CTRL0 Register */
    __IO uint32_t stat;                 /**< <tt>\b 0x04:</tt> I2C STAT Register */
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
    __IO uint32_t mstr_mode;            /**< <tt>\b 0x30:</tt> I2C MSTR_MODE Register */
    __IO uint32_t clk_lo;               /**< <tt>\b 0x34:</tt> I2C CLK_LO Register */
    __IO uint32_t clk_hi;               /**< <tt>\b 0x38:</tt> I2C CLK_HI Register */
    __R  uint32_t rsv_0x3c;
    __IO uint32_t timeout;              /**< <tt>\b 0x40:</tt> I2C TIMEOUT Register */
    __IO uint32_t slv_addr;             /**< <tt>\b 0x44:</tt> I2C SLV_ADDR Register */
    __IO uint32_t dma;                  /**< <tt>\b 0x48:</tt> I2C DMA Register */
} mxc_i2c_regs_t;

/* Register offsets for module I2C */
/**
 * @ingroup    i2c_registers
 * @defgroup   I2C_Register_Offsets Register Offsets
 * @brief      I2C Peripheral Register Offsets from the I2C Base Peripheral Address.
 * @{
 */
#define MXC_R_I2C_CTRL0                    ((uint32_t)0x00000000UL) /**< Offset from I2C Base Address: <tt> 0x0000</tt> */
#define MXC_R_I2C_STAT                     ((uint32_t)0x00000004UL) /**< Offset from I2C Base Address: <tt> 0x0004</tt> */
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
#define MXC_R_I2C_MSTR_MODE                ((uint32_t)0x00000030UL) /**< Offset from I2C Base Address: <tt> 0x0030</tt> */
#define MXC_R_I2C_CLK_LO                   ((uint32_t)0x00000034UL) /**< Offset from I2C Base Address: <tt> 0x0034</tt> */
#define MXC_R_I2C_CLK_HI                   ((uint32_t)0x00000038UL) /**< Offset from I2C Base Address: <tt> 0x0038</tt> */
#define MXC_R_I2C_TIMEOUT                  ((uint32_t)0x00000040UL) /**< Offset from I2C Base Address: <tt> 0x0040</tt> */
#define MXC_R_I2C_SLV_ADDR                 ((uint32_t)0x00000044UL) /**< Offset from I2C Base Address: <tt> 0x0044</tt> */
#define MXC_R_I2C_DMA                      ((uint32_t)0x00000048UL) /**< Offset from I2C Base Address: <tt> 0x0048</tt> */
/**@} end of group i2c_registers */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CTRL0 I2C_CTRL0
 * @brief    Control Register 0.
 * @{
 */
#define MXC_F_I2C_CTRL0_I2CEN_POS                      0 /**< CTRL0_I2CEN Position */
#define MXC_F_I2C_CTRL0_I2CEN                          ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_I2CEN_POS)) /**< CTRL0_I2CEN Mask */
#define MXC_V_I2C_CTRL0_I2CEN_DIS                      ((uint32_t)0x0UL) /**< CTRL0_I2CEN_DIS Value */
#define MXC_S_I2C_CTRL0_I2CEN_DIS                      (MXC_V_I2C_CTRL0_I2CEN_DIS << MXC_F_I2C_CTRL0_I2CEN_POS) /**< CTRL0_I2CEN_DIS Setting */
#define MXC_V_I2C_CTRL0_I2CEN_EN                       ((uint32_t)0x1UL) /**< CTRL0_I2CEN_EN Value */
#define MXC_S_I2C_CTRL0_I2CEN_EN                       (MXC_V_I2C_CTRL0_I2CEN_EN << MXC_F_I2C_CTRL0_I2CEN_POS) /**< CTRL0_I2CEN_EN Setting */

#define MXC_F_I2C_CTRL0_MST_POS                        1 /**< CTRL0_MST Position */
#define MXC_F_I2C_CTRL0_MST                            ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_MST_POS)) /**< CTRL0_MST Mask */
#define MXC_V_I2C_CTRL0_MST_SLAVE_MODE                 ((uint32_t)0x0UL) /**< CTRL0_MST_SLAVE_MODE Value */
#define MXC_S_I2C_CTRL0_MST_SLAVE_MODE                 (MXC_V_I2C_CTRL0_MST_SLAVE_MODE << MXC_F_I2C_CTRL0_MST_POS) /**< CTRL0_MST_SLAVE_MODE Setting */
#define MXC_V_I2C_CTRL0_MST_MASTER_MODE                ((uint32_t)0x1UL) /**< CTRL0_MST_MASTER_MODE Value */
#define MXC_S_I2C_CTRL0_MST_MASTER_MODE                (MXC_V_I2C_CTRL0_MST_MASTER_MODE << MXC_F_I2C_CTRL0_MST_POS) /**< CTRL0_MST_MASTER_MODE Setting */

#define MXC_F_I2C_CTRL0_GCEN_POS                       2 /**< CTRL0_GCEN Position */
#define MXC_F_I2C_CTRL0_GCEN                           ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_GCEN_POS)) /**< CTRL0_GCEN Mask */
#define MXC_V_I2C_CTRL0_GCEN_DIS                       ((uint32_t)0x0UL) /**< CTRL0_GCEN_DIS Value */
#define MXC_S_I2C_CTRL0_GCEN_DIS                       (MXC_V_I2C_CTRL0_GCEN_DIS << MXC_F_I2C_CTRL0_GCEN_POS) /**< CTRL0_GCEN_DIS Setting */
#define MXC_V_I2C_CTRL0_GCEN_EN                        ((uint32_t)0x1UL) /**< CTRL0_GCEN_EN Value */
#define MXC_S_I2C_CTRL0_GCEN_EN                        (MXC_V_I2C_CTRL0_GCEN_EN << MXC_F_I2C_CTRL0_GCEN_POS) /**< CTRL0_GCEN_EN Setting */

#define MXC_F_I2C_CTRL0_IRXM_POS                       3 /**< CTRL0_IRXM Position */
#define MXC_F_I2C_CTRL0_IRXM                           ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_IRXM_POS)) /**< CTRL0_IRXM Mask */
#define MXC_V_I2C_CTRL0_IRXM_DIS                       ((uint32_t)0x0UL) /**< CTRL0_IRXM_DIS Value */
#define MXC_S_I2C_CTRL0_IRXM_DIS                       (MXC_V_I2C_CTRL0_IRXM_DIS << MXC_F_I2C_CTRL0_IRXM_POS) /**< CTRL0_IRXM_DIS Setting */
#define MXC_V_I2C_CTRL0_IRXM_EN                        ((uint32_t)0x1UL) /**< CTRL0_IRXM_EN Value */
#define MXC_S_I2C_CTRL0_IRXM_EN                        (MXC_V_I2C_CTRL0_IRXM_EN << MXC_F_I2C_CTRL0_IRXM_POS) /**< CTRL0_IRXM_EN Setting */

#define MXC_F_I2C_CTRL0_ACK_POS                        4 /**< CTRL0_ACK Position */
#define MXC_F_I2C_CTRL0_ACK                            ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_ACK_POS)) /**< CTRL0_ACK Mask */
#define MXC_V_I2C_CTRL0_ACK_ACK                        ((uint32_t)0x0UL) /**< CTRL0_ACK_ACK Value */
#define MXC_S_I2C_CTRL0_ACK_ACK                        (MXC_V_I2C_CTRL0_ACK_ACK << MXC_F_I2C_CTRL0_ACK_POS) /**< CTRL0_ACK_ACK Setting */
#define MXC_V_I2C_CTRL0_ACK_NACK                       ((uint32_t)0x1UL) /**< CTRL0_ACK_NACK Value */
#define MXC_S_I2C_CTRL0_ACK_NACK                       (MXC_V_I2C_CTRL0_ACK_NACK << MXC_F_I2C_CTRL0_ACK_POS) /**< CTRL0_ACK_NACK Setting */

#define MXC_F_I2C_CTRL0_SCL_OUT_POS                    6 /**< CTRL0_SCL_OUT Position */
#define MXC_F_I2C_CTRL0_SCL_OUT                        ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SCL_OUT_POS)) /**< CTRL0_SCL_OUT Mask */
#define MXC_V_I2C_CTRL0_SCL_OUT_LOW                    ((uint32_t)0x0UL) /**< CTRL0_SCL_OUT_LOW Value */
#define MXC_S_I2C_CTRL0_SCL_OUT_LOW                    (MXC_V_I2C_CTRL0_SCL_OUT_LOW << MXC_F_I2C_CTRL0_SCL_OUT_POS) /**< CTRL0_SCL_OUT_LOW Setting */
#define MXC_V_I2C_CTRL0_SCL_OUT_HIGH                   ((uint32_t)0x1UL) /**< CTRL0_SCL_OUT_HIGH Value */
#define MXC_S_I2C_CTRL0_SCL_OUT_HIGH                   (MXC_V_I2C_CTRL0_SCL_OUT_HIGH << MXC_F_I2C_CTRL0_SCL_OUT_POS) /**< CTRL0_SCL_OUT_HIGH Setting */

#define MXC_F_I2C_CTRL0_SDA_OUT_POS                    7 /**< CTRL0_SDA_OUT Position */
#define MXC_F_I2C_CTRL0_SDA_OUT                        ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SDA_OUT_POS)) /**< CTRL0_SDA_OUT Mask */
#define MXC_V_I2C_CTRL0_SDA_OUT_LOW                    ((uint32_t)0x0UL) /**< CTRL0_SDA_OUT_LOW Value */
#define MXC_S_I2C_CTRL0_SDA_OUT_LOW                    (MXC_V_I2C_CTRL0_SDA_OUT_LOW << MXC_F_I2C_CTRL0_SDA_OUT_POS) /**< CTRL0_SDA_OUT_LOW Setting */
#define MXC_V_I2C_CTRL0_SDA_OUT_HIGH                   ((uint32_t)0x1UL) /**< CTRL0_SDA_OUT_HIGH Value */
#define MXC_S_I2C_CTRL0_SDA_OUT_HIGH                   (MXC_V_I2C_CTRL0_SDA_OUT_HIGH << MXC_F_I2C_CTRL0_SDA_OUT_POS) /**< CTRL0_SDA_OUT_HIGH Setting */

#define MXC_F_I2C_CTRL0_SCL_POS                        8 /**< CTRL0_SCL Position */
#define MXC_F_I2C_CTRL0_SCL                            ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SCL_POS)) /**< CTRL0_SCL Mask */
#define MXC_V_I2C_CTRL0_SCL_LOW                        ((uint32_t)0x0UL) /**< CTRL0_SCL_LOW Value */
#define MXC_S_I2C_CTRL0_SCL_LOW                        (MXC_V_I2C_CTRL0_SCL_LOW << MXC_F_I2C_CTRL0_SCL_POS) /**< CTRL0_SCL_LOW Setting */
#define MXC_V_I2C_CTRL0_SCL_HIGH                       ((uint32_t)0x1UL) /**< CTRL0_SCL_HIGH Value */
#define MXC_S_I2C_CTRL0_SCL_HIGH                       (MXC_V_I2C_CTRL0_SCL_HIGH << MXC_F_I2C_CTRL0_SCL_POS) /**< CTRL0_SCL_HIGH Setting */

#define MXC_F_I2C_CTRL0_SDA_POS                        9 /**< CTRL0_SDA Position */
#define MXC_F_I2C_CTRL0_SDA                            ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SDA_POS)) /**< CTRL0_SDA Mask */
#define MXC_V_I2C_CTRL0_SDA_LOW                        ((uint32_t)0x0UL) /**< CTRL0_SDA_LOW Value */
#define MXC_S_I2C_CTRL0_SDA_LOW                        (MXC_V_I2C_CTRL0_SDA_LOW << MXC_F_I2C_CTRL0_SDA_POS) /**< CTRL0_SDA_LOW Setting */
#define MXC_V_I2C_CTRL0_SDA_HIGH                       ((uint32_t)0x1UL) /**< CTRL0_SDA_HIGH Value */
#define MXC_S_I2C_CTRL0_SDA_HIGH                       (MXC_V_I2C_CTRL0_SDA_HIGH << MXC_F_I2C_CTRL0_SDA_POS) /**< CTRL0_SDA_HIGH Setting */

#define MXC_F_I2C_CTRL0_SWOE_POS                       10 /**< CTRL0_SWOE Position */
#define MXC_F_I2C_CTRL0_SWOE                           ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SWOE_POS)) /**< CTRL0_SWOE Mask */
#define MXC_V_I2C_CTRL0_SWOE_DIS                       ((uint32_t)0x0UL) /**< CTRL0_SWOE_DIS Value */
#define MXC_S_I2C_CTRL0_SWOE_DIS                       (MXC_V_I2C_CTRL0_SWOE_DIS << MXC_F_I2C_CTRL0_SWOE_POS) /**< CTRL0_SWOE_DIS Setting */
#define MXC_V_I2C_CTRL0_SWOE_EN                        ((uint32_t)0x1UL) /**< CTRL0_SWOE_EN Value */
#define MXC_S_I2C_CTRL0_SWOE_EN                        (MXC_V_I2C_CTRL0_SWOE_EN << MXC_F_I2C_CTRL0_SWOE_POS) /**< CTRL0_SWOE_EN Setting */

#define MXC_F_I2C_CTRL0_READ_POS                       11 /**< CTRL0_READ Position */
#define MXC_F_I2C_CTRL0_READ                           ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_READ_POS)) /**< CTRL0_READ Mask */
#define MXC_V_I2C_CTRL0_READ_WRITE                     ((uint32_t)0x0UL) /**< CTRL0_READ_WRITE Value */
#define MXC_S_I2C_CTRL0_READ_WRITE                     (MXC_V_I2C_CTRL0_READ_WRITE << MXC_F_I2C_CTRL0_READ_POS) /**< CTRL0_READ_WRITE Setting */
#define MXC_V_I2C_CTRL0_READ_READ                      ((uint32_t)0x1UL) /**< CTRL0_READ_READ Value */
#define MXC_S_I2C_CTRL0_READ_READ                      (MXC_V_I2C_CTRL0_READ_READ << MXC_F_I2C_CTRL0_READ_POS) /**< CTRL0_READ_READ Setting */

#define MXC_F_I2C_CTRL0_SCL_STRD_POS                   12 /**< CTRL0_SCL_STRD Position */
#define MXC_F_I2C_CTRL0_SCL_STRD                       ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SCL_STRD_POS)) /**< CTRL0_SCL_STRD Mask */
#define MXC_V_I2C_CTRL0_SCL_STRD_EN                    ((uint32_t)0x0UL) /**< CTRL0_SCL_STRD_EN Value */
#define MXC_S_I2C_CTRL0_SCL_STRD_EN                    (MXC_V_I2C_CTRL0_SCL_STRD_EN << MXC_F_I2C_CTRL0_SCL_STRD_POS) /**< CTRL0_SCL_STRD_EN Setting */
#define MXC_V_I2C_CTRL0_SCL_STRD_DIS                   ((uint32_t)0x1UL) /**< CTRL0_SCL_STRD_DIS Value */
#define MXC_S_I2C_CTRL0_SCL_STRD_DIS                   (MXC_V_I2C_CTRL0_SCL_STRD_DIS << MXC_F_I2C_CTRL0_SCL_STRD_POS) /**< CTRL0_SCL_STRD_DIS Setting */

#define MXC_F_I2C_CTRL0_SCL_PPM_POS                    13 /**< CTRL0_SCL_PPM Position */
#define MXC_F_I2C_CTRL0_SCL_PPM                        ((uint32_t)(0x1UL << MXC_F_I2C_CTRL0_SCL_PPM_POS)) /**< CTRL0_SCL_PPM Mask */
#define MXC_V_I2C_CTRL0_SCL_PPM_DIS                    ((uint32_t)0x0UL) /**< CTRL0_SCL_PPM_DIS Value */
#define MXC_S_I2C_CTRL0_SCL_PPM_DIS                    (MXC_V_I2C_CTRL0_SCL_PPM_DIS << MXC_F_I2C_CTRL0_SCL_PPM_POS) /**< CTRL0_SCL_PPM_DIS Setting */
#define MXC_V_I2C_CTRL0_SCL_PPM_EN                     ((uint32_t)0x1UL) /**< CTRL0_SCL_PPM_EN Value */
#define MXC_S_I2C_CTRL0_SCL_PPM_EN                     (MXC_V_I2C_CTRL0_SCL_PPM_EN << MXC_F_I2C_CTRL0_SCL_PPM_POS) /**< CTRL0_SCL_PPM_EN Setting */

/**@} end of group I2C_CTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_STAT I2C_STAT
 * @brief    Status Register.
 * @{
 */
#define MXC_F_I2C_STAT_BUSY_POS                        0 /**< STAT_BUSY Position */
#define MXC_F_I2C_STAT_BUSY                            ((uint32_t)(0x1UL << MXC_F_I2C_STAT_BUSY_POS)) /**< STAT_BUSY Mask */
#define MXC_V_I2C_STAT_BUSY_IDLE                       ((uint32_t)0x0UL) /**< STAT_BUSY_IDLE Value */
#define MXC_S_I2C_STAT_BUSY_IDLE                       (MXC_V_I2C_STAT_BUSY_IDLE << MXC_F_I2C_STAT_BUSY_POS) /**< STAT_BUSY_IDLE Setting */
#define MXC_V_I2C_STAT_BUSY_BUSY                       ((uint32_t)0x1UL) /**< STAT_BUSY_BUSY Value */
#define MXC_S_I2C_STAT_BUSY_BUSY                       (MXC_V_I2C_STAT_BUSY_BUSY << MXC_F_I2C_STAT_BUSY_POS) /**< STAT_BUSY_BUSY Setting */

#define MXC_F_I2C_STAT_RXE_POS                         1 /**< STAT_RXE Position */
#define MXC_F_I2C_STAT_RXE                             ((uint32_t)(0x1UL << MXC_F_I2C_STAT_RXE_POS)) /**< STAT_RXE Mask */
#define MXC_V_I2C_STAT_RXE_NOT_EMPTY                   ((uint32_t)0x0UL) /**< STAT_RXE_NOT_EMPTY Value */
#define MXC_S_I2C_STAT_RXE_NOT_EMPTY                   (MXC_V_I2C_STAT_RXE_NOT_EMPTY << MXC_F_I2C_STAT_RXE_POS) /**< STAT_RXE_NOT_EMPTY Setting */
#define MXC_V_I2C_STAT_RXE_EMPTY                       ((uint32_t)0x1UL) /**< STAT_RXE_EMPTY Value */
#define MXC_S_I2C_STAT_RXE_EMPTY                       (MXC_V_I2C_STAT_RXE_EMPTY << MXC_F_I2C_STAT_RXE_POS) /**< STAT_RXE_EMPTY Setting */

#define MXC_F_I2C_STAT_RXF_POS                         2 /**< STAT_RXF Position */
#define MXC_F_I2C_STAT_RXF                             ((uint32_t)(0x1UL << MXC_F_I2C_STAT_RXF_POS)) /**< STAT_RXF Mask */
#define MXC_V_I2C_STAT_RXF_NOT_FULL                    ((uint32_t)0x0UL) /**< STAT_RXF_NOT_FULL Value */
#define MXC_S_I2C_STAT_RXF_NOT_FULL                    (MXC_V_I2C_STAT_RXF_NOT_FULL << MXC_F_I2C_STAT_RXF_POS) /**< STAT_RXF_NOT_FULL Setting */
#define MXC_V_I2C_STAT_RXF_FULL                        ((uint32_t)0x1UL) /**< STAT_RXF_FULL Value */
#define MXC_S_I2C_STAT_RXF_FULL                        (MXC_V_I2C_STAT_RXF_FULL << MXC_F_I2C_STAT_RXF_POS) /**< STAT_RXF_FULL Setting */

#define MXC_F_I2C_STAT_TXE_POS                         3 /**< STAT_TXE Position */
#define MXC_F_I2C_STAT_TXE                             ((uint32_t)(0x1UL << MXC_F_I2C_STAT_TXE_POS)) /**< STAT_TXE Mask */
#define MXC_V_I2C_STAT_TXE_NOT_EMPTY                   ((uint32_t)0x0UL) /**< STAT_TXE_NOT_EMPTY Value */
#define MXC_S_I2C_STAT_TXE_NOT_EMPTY                   (MXC_V_I2C_STAT_TXE_NOT_EMPTY << MXC_F_I2C_STAT_TXE_POS) /**< STAT_TXE_NOT_EMPTY Setting */
#define MXC_V_I2C_STAT_TXE_EMPTY                       ((uint32_t)0x1UL) /**< STAT_TXE_EMPTY Value */
#define MXC_S_I2C_STAT_TXE_EMPTY                       (MXC_V_I2C_STAT_TXE_EMPTY << MXC_F_I2C_STAT_TXE_POS) /**< STAT_TXE_EMPTY Setting */

#define MXC_F_I2C_STAT_TXF_POS                         4 /**< STAT_TXF Position */
#define MXC_F_I2C_STAT_TXF                             ((uint32_t)(0x1UL << MXC_F_I2C_STAT_TXF_POS)) /**< STAT_TXF Mask */
#define MXC_V_I2C_STAT_TXF_NOT_FULL                    ((uint32_t)0x0UL) /**< STAT_TXF_NOT_FULL Value */
#define MXC_S_I2C_STAT_TXF_NOT_FULL                    (MXC_V_I2C_STAT_TXF_NOT_FULL << MXC_F_I2C_STAT_TXF_POS) /**< STAT_TXF_NOT_FULL Setting */
#define MXC_V_I2C_STAT_TXF_FULL                        ((uint32_t)0x1UL) /**< STAT_TXF_FULL Value */
#define MXC_S_I2C_STAT_TXF_FULL                        (MXC_V_I2C_STAT_TXF_FULL << MXC_F_I2C_STAT_TXF_POS) /**< STAT_TXF_FULL Setting */

#define MXC_F_I2C_STAT_CKMD_POS                        5 /**< STAT_CKMD Position */
#define MXC_F_I2C_STAT_CKMD                            ((uint32_t)(0x1UL << MXC_F_I2C_STAT_CKMD_POS)) /**< STAT_CKMD Mask */
#define MXC_V_I2C_STAT_CKMD_SCL_NOT_ACTIVE             ((uint32_t)0x0UL) /**< STAT_CKMD_SCL_NOT_ACTIVE Value */
#define MXC_S_I2C_STAT_CKMD_SCL_NOT_ACTIVE             (MXC_V_I2C_STAT_CKMD_SCL_NOT_ACTIVE << MXC_F_I2C_STAT_CKMD_POS) /**< STAT_CKMD_SCL_NOT_ACTIVE Setting */
#define MXC_V_I2C_STAT_CKMD_SCL_ACTIVE                 ((uint32_t)0x1UL) /**< STAT_CKMD_SCL_ACTIVE Value */
#define MXC_S_I2C_STAT_CKMD_SCL_ACTIVE                 (MXC_V_I2C_STAT_CKMD_SCL_ACTIVE << MXC_F_I2C_STAT_CKMD_POS) /**< STAT_CKMD_SCL_ACTIVE Setting */

/**@} end of group I2C_STAT_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_FL0 I2C_INT_FL0
 * @brief    Interrupt Status Register.
 * @{
 */
#define MXC_F_I2C_INT_FL0_DONEI_POS                    0 /**< INT_FL0_DONEI Position */
#define MXC_F_I2C_INT_FL0_DONEI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_DONEI_POS)) /**< INT_FL0_DONEI Mask */
#define MXC_V_I2C_INT_FL0_DONEI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_DONEI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_DONEI_INACTIVE               (MXC_V_I2C_INT_FL0_DONEI_INACTIVE << MXC_F_I2C_INT_FL0_DONEI_POS) /**< INT_FL0_DONEI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_DONEI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_DONEI_PENDING Value */
#define MXC_S_I2C_INT_FL0_DONEI_PENDING                (MXC_V_I2C_INT_FL0_DONEI_PENDING << MXC_F_I2C_INT_FL0_DONEI_POS) /**< INT_FL0_DONEI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_IRXMI_POS                    1 /**< INT_FL0_IRXMI Position */
#define MXC_F_I2C_INT_FL0_IRXMI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_IRXMI_POS)) /**< INT_FL0_IRXMI Mask */
#define MXC_V_I2C_INT_FL0_IRXMI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_IRXMI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_IRXMI_INACTIVE               (MXC_V_I2C_INT_FL0_IRXMI_INACTIVE << MXC_F_I2C_INT_FL0_IRXMI_POS) /**< INT_FL0_IRXMI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_IRXMI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_IRXMI_PENDING Value */
#define MXC_S_I2C_INT_FL0_IRXMI_PENDING                (MXC_V_I2C_INT_FL0_IRXMI_PENDING << MXC_F_I2C_INT_FL0_IRXMI_POS) /**< INT_FL0_IRXMI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_GCI_POS                      2 /**< INT_FL0_GCI Position */
#define MXC_F_I2C_INT_FL0_GCI                          ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_GCI_POS)) /**< INT_FL0_GCI Mask */
#define MXC_V_I2C_INT_FL0_GCI_INACTIVE                 ((uint32_t)0x0UL) /**< INT_FL0_GCI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_GCI_INACTIVE                 (MXC_V_I2C_INT_FL0_GCI_INACTIVE << MXC_F_I2C_INT_FL0_GCI_POS) /**< INT_FL0_GCI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_GCI_PENDING                  ((uint32_t)0x1UL) /**< INT_FL0_GCI_PENDING Value */
#define MXC_S_I2C_INT_FL0_GCI_PENDING                  (MXC_V_I2C_INT_FL0_GCI_PENDING << MXC_F_I2C_INT_FL0_GCI_POS) /**< INT_FL0_GCI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_AMI_POS                      3 /**< INT_FL0_AMI Position */
#define MXC_F_I2C_INT_FL0_AMI                          ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_AMI_POS)) /**< INT_FL0_AMI Mask */
#define MXC_V_I2C_INT_FL0_AMI_INACTIVE                 ((uint32_t)0x0UL) /**< INT_FL0_AMI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_AMI_INACTIVE                 (MXC_V_I2C_INT_FL0_AMI_INACTIVE << MXC_F_I2C_INT_FL0_AMI_POS) /**< INT_FL0_AMI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_AMI_PENDING                  ((uint32_t)0x1UL) /**< INT_FL0_AMI_PENDING Value */
#define MXC_S_I2C_INT_FL0_AMI_PENDING                  (MXC_V_I2C_INT_FL0_AMI_PENDING << MXC_F_I2C_INT_FL0_AMI_POS) /**< INT_FL0_AMI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_RXTHI_POS                    4 /**< INT_FL0_RXTHI Position */
#define MXC_F_I2C_INT_FL0_RXTHI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_RXTHI_POS)) /**< INT_FL0_RXTHI Mask */
#define MXC_V_I2C_INT_FL0_RXTHI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_RXTHI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_RXTHI_INACTIVE               (MXC_V_I2C_INT_FL0_RXTHI_INACTIVE << MXC_F_I2C_INT_FL0_RXTHI_POS) /**< INT_FL0_RXTHI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_RXTHI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_RXTHI_PENDING Value */
#define MXC_S_I2C_INT_FL0_RXTHI_PENDING                (MXC_V_I2C_INT_FL0_RXTHI_PENDING << MXC_F_I2C_INT_FL0_RXTHI_POS) /**< INT_FL0_RXTHI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_TXTHI_POS                    5 /**< INT_FL0_TXTHI Position */
#define MXC_F_I2C_INT_FL0_TXTHI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_TXTHI_POS)) /**< INT_FL0_TXTHI Mask */
#define MXC_V_I2C_INT_FL0_TXTHI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_TXTHI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_TXTHI_INACTIVE               (MXC_V_I2C_INT_FL0_TXTHI_INACTIVE << MXC_F_I2C_INT_FL0_TXTHI_POS) /**< INT_FL0_TXTHI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_TXTHI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_TXTHI_PENDING Value */
#define MXC_S_I2C_INT_FL0_TXTHI_PENDING                (MXC_V_I2C_INT_FL0_TXTHI_PENDING << MXC_F_I2C_INT_FL0_TXTHI_POS) /**< INT_FL0_TXTHI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_STOPI_POS                    6 /**< INT_FL0_STOPI Position */
#define MXC_F_I2C_INT_FL0_STOPI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_STOPI_POS)) /**< INT_FL0_STOPI Mask */
#define MXC_V_I2C_INT_FL0_STOPI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_STOPI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_STOPI_INACTIVE               (MXC_V_I2C_INT_FL0_STOPI_INACTIVE << MXC_F_I2C_INT_FL0_STOPI_POS) /**< INT_FL0_STOPI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_STOPI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_STOPI_PENDING Value */
#define MXC_S_I2C_INT_FL0_STOPI_PENDING                (MXC_V_I2C_INT_FL0_STOPI_PENDING << MXC_F_I2C_INT_FL0_STOPI_POS) /**< INT_FL0_STOPI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_ADRACKI_POS                  7 /**< INT_FL0_ADRACKI Position */
#define MXC_F_I2C_INT_FL0_ADRACKI                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ADRACKI_POS)) /**< INT_FL0_ADRACKI Mask */
#define MXC_V_I2C_INT_FL0_ADRACKI_INACTIVE             ((uint32_t)0x0UL) /**< INT_FL0_ADRACKI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_ADRACKI_INACTIVE             (MXC_V_I2C_INT_FL0_ADRACKI_INACTIVE << MXC_F_I2C_INT_FL0_ADRACKI_POS) /**< INT_FL0_ADRACKI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_ADRACKI_PENDING              ((uint32_t)0x1UL) /**< INT_FL0_ADRACKI_PENDING Value */
#define MXC_S_I2C_INT_FL0_ADRACKI_PENDING              (MXC_V_I2C_INT_FL0_ADRACKI_PENDING << MXC_F_I2C_INT_FL0_ADRACKI_POS) /**< INT_FL0_ADRACKI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_ARBERI_POS                   8 /**< INT_FL0_ARBERI Position */
#define MXC_F_I2C_INT_FL0_ARBERI                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ARBERI_POS)) /**< INT_FL0_ARBERI Mask */
#define MXC_V_I2C_INT_FL0_ARBERI_INACTIVE              ((uint32_t)0x0UL) /**< INT_FL0_ARBERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_ARBERI_INACTIVE              (MXC_V_I2C_INT_FL0_ARBERI_INACTIVE << MXC_F_I2C_INT_FL0_ARBERI_POS) /**< INT_FL0_ARBERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_ARBERI_PENDING               ((uint32_t)0x1UL) /**< INT_FL0_ARBERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_ARBERI_PENDING               (MXC_V_I2C_INT_FL0_ARBERI_PENDING << MXC_F_I2C_INT_FL0_ARBERI_POS) /**< INT_FL0_ARBERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_TOERI_POS                    9 /**< INT_FL0_TOERI Position */
#define MXC_F_I2C_INT_FL0_TOERI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_TOERI_POS)) /**< INT_FL0_TOERI Mask */
#define MXC_V_I2C_INT_FL0_TOERI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_TOERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_TOERI_INACTIVE               (MXC_V_I2C_INT_FL0_TOERI_INACTIVE << MXC_F_I2C_INT_FL0_TOERI_POS) /**< INT_FL0_TOERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_TOERI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_TOERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_TOERI_PENDING                (MXC_V_I2C_INT_FL0_TOERI_PENDING << MXC_F_I2C_INT_FL0_TOERI_POS) /**< INT_FL0_TOERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_ADRERI_POS                   10 /**< INT_FL0_ADRERI Position */
#define MXC_F_I2C_INT_FL0_ADRERI                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_ADRERI_POS)) /**< INT_FL0_ADRERI Mask */
#define MXC_V_I2C_INT_FL0_ADRERI_INACTIVE              ((uint32_t)0x0UL) /**< INT_FL0_ADRERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_ADRERI_INACTIVE              (MXC_V_I2C_INT_FL0_ADRERI_INACTIVE << MXC_F_I2C_INT_FL0_ADRERI_POS) /**< INT_FL0_ADRERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_ADRERI_PENDING               ((uint32_t)0x1UL) /**< INT_FL0_ADRERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_ADRERI_PENDING               (MXC_V_I2C_INT_FL0_ADRERI_PENDING << MXC_F_I2C_INT_FL0_ADRERI_POS) /**< INT_FL0_ADRERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_DATAERI_POS                  11 /**< INT_FL0_DATAERI Position */
#define MXC_F_I2C_INT_FL0_DATAERI                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_DATAERI_POS)) /**< INT_FL0_DATAERI Mask */
#define MXC_V_I2C_INT_FL0_DATAERI_INACTIVE             ((uint32_t)0x0UL) /**< INT_FL0_DATAERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_DATAERI_INACTIVE             (MXC_V_I2C_INT_FL0_DATAERI_INACTIVE << MXC_F_I2C_INT_FL0_DATAERI_POS) /**< INT_FL0_DATAERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_DATAERI_PENDING              ((uint32_t)0x1UL) /**< INT_FL0_DATAERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_DATAERI_PENDING              (MXC_V_I2C_INT_FL0_DATAERI_PENDING << MXC_F_I2C_INT_FL0_DATAERI_POS) /**< INT_FL0_DATAERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_DNRERI_POS                   12 /**< INT_FL0_DNRERI Position */
#define MXC_F_I2C_INT_FL0_DNRERI                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_DNRERI_POS)) /**< INT_FL0_DNRERI Mask */
#define MXC_V_I2C_INT_FL0_DNRERI_INACTIVE              ((uint32_t)0x0UL) /**< INT_FL0_DNRERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_DNRERI_INACTIVE              (MXC_V_I2C_INT_FL0_DNRERI_INACTIVE << MXC_F_I2C_INT_FL0_DNRERI_POS) /**< INT_FL0_DNRERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_DNRERI_PENDING               ((uint32_t)0x1UL) /**< INT_FL0_DNRERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_DNRERI_PENDING               (MXC_V_I2C_INT_FL0_DNRERI_PENDING << MXC_F_I2C_INT_FL0_DNRERI_POS) /**< INT_FL0_DNRERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_STRTERI_POS                  13 /**< INT_FL0_STRTERI Position */
#define MXC_F_I2C_INT_FL0_STRTERI                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_STRTERI_POS)) /**< INT_FL0_STRTERI Mask */
#define MXC_V_I2C_INT_FL0_STRTERI_INACTIVE             ((uint32_t)0x0UL) /**< INT_FL0_STRTERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_STRTERI_INACTIVE             (MXC_V_I2C_INT_FL0_STRTERI_INACTIVE << MXC_F_I2C_INT_FL0_STRTERI_POS) /**< INT_FL0_STRTERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_STRTERI_PENDING              ((uint32_t)0x1UL) /**< INT_FL0_STRTERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_STRTERI_PENDING              (MXC_V_I2C_INT_FL0_STRTERI_PENDING << MXC_F_I2C_INT_FL0_STRTERI_POS) /**< INT_FL0_STRTERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_STOPERI_POS                  14 /**< INT_FL0_STOPERI Position */
#define MXC_F_I2C_INT_FL0_STOPERI                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_STOPERI_POS)) /**< INT_FL0_STOPERI Mask */
#define MXC_V_I2C_INT_FL0_STOPERI_INACTIVE             ((uint32_t)0x0UL) /**< INT_FL0_STOPERI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_STOPERI_INACTIVE             (MXC_V_I2C_INT_FL0_STOPERI_INACTIVE << MXC_F_I2C_INT_FL0_STOPERI_POS) /**< INT_FL0_STOPERI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_STOPERI_PENDING              ((uint32_t)0x1UL) /**< INT_FL0_STOPERI_PENDING Value */
#define MXC_S_I2C_INT_FL0_STOPERI_PENDING              (MXC_V_I2C_INT_FL0_STOPERI_PENDING << MXC_F_I2C_INT_FL0_STOPERI_POS) /**< INT_FL0_STOPERI_PENDING Setting */

#define MXC_F_I2C_INT_FL0_TXLOI_POS                    15 /**< INT_FL0_TXLOI Position */
#define MXC_F_I2C_INT_FL0_TXLOI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL0_TXLOI_POS)) /**< INT_FL0_TXLOI Mask */
#define MXC_V_I2C_INT_FL0_TXLOI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL0_TXLOI_INACTIVE Value */
#define MXC_S_I2C_INT_FL0_TXLOI_INACTIVE               (MXC_V_I2C_INT_FL0_TXLOI_INACTIVE << MXC_F_I2C_INT_FL0_TXLOI_POS) /**< INT_FL0_TXLOI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL0_TXLOI_PENDING                ((uint32_t)0x1UL) /**< INT_FL0_TXLOI_PENDING Value */
#define MXC_S_I2C_INT_FL0_TXLOI_PENDING                (MXC_V_I2C_INT_FL0_TXLOI_PENDING << MXC_F_I2C_INT_FL0_TXLOI_POS) /**< INT_FL0_TXLOI_PENDING Setting */

/**@} end of group I2C_INT_FL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_EN0 I2C_INT_EN0
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_I2C_INT_EN0_DONEIE_POS                   0 /**< INT_EN0_DONEIE Position */
#define MXC_F_I2C_INT_EN0_DONEIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_DONEIE_POS)) /**< INT_EN0_DONEIE Mask */
#define MXC_V_I2C_INT_EN0_DONEIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_DONEIE_DIS Value */
#define MXC_S_I2C_INT_EN0_DONEIE_DIS                   (MXC_V_I2C_INT_EN0_DONEIE_DIS << MXC_F_I2C_INT_EN0_DONEIE_POS) /**< INT_EN0_DONEIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_DONEIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_DONEIE_EN Value */
#define MXC_S_I2C_INT_EN0_DONEIE_EN                    (MXC_V_I2C_INT_EN0_DONEIE_EN << MXC_F_I2C_INT_EN0_DONEIE_POS) /**< INT_EN0_DONEIE_EN Setting */

#define MXC_F_I2C_INT_EN0_IRXMIE_POS                   1 /**< INT_EN0_IRXMIE Position */
#define MXC_F_I2C_INT_EN0_IRXMIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_IRXMIE_POS)) /**< INT_EN0_IRXMIE Mask */
#define MXC_V_I2C_INT_EN0_IRXMIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_IRXMIE_DIS Value */
#define MXC_S_I2C_INT_EN0_IRXMIE_DIS                   (MXC_V_I2C_INT_EN0_IRXMIE_DIS << MXC_F_I2C_INT_EN0_IRXMIE_POS) /**< INT_EN0_IRXMIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_IRXMIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_IRXMIE_EN Value */
#define MXC_S_I2C_INT_EN0_IRXMIE_EN                    (MXC_V_I2C_INT_EN0_IRXMIE_EN << MXC_F_I2C_INT_EN0_IRXMIE_POS) /**< INT_EN0_IRXMIE_EN Setting */

#define MXC_F_I2C_INT_EN0_GCIE_POS                     2 /**< INT_EN0_GCIE Position */
#define MXC_F_I2C_INT_EN0_GCIE                         ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_GCIE_POS)) /**< INT_EN0_GCIE Mask */
#define MXC_V_I2C_INT_EN0_GCIE_DIS                     ((uint32_t)0x0UL) /**< INT_EN0_GCIE_DIS Value */
#define MXC_S_I2C_INT_EN0_GCIE_DIS                     (MXC_V_I2C_INT_EN0_GCIE_DIS << MXC_F_I2C_INT_EN0_GCIE_POS) /**< INT_EN0_GCIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_GCIE_EN                      ((uint32_t)0x1UL) /**< INT_EN0_GCIE_EN Value */
#define MXC_S_I2C_INT_EN0_GCIE_EN                      (MXC_V_I2C_INT_EN0_GCIE_EN << MXC_F_I2C_INT_EN0_GCIE_POS) /**< INT_EN0_GCIE_EN Setting */

#define MXC_F_I2C_INT_EN0_AMIE_POS                     3 /**< INT_EN0_AMIE Position */
#define MXC_F_I2C_INT_EN0_AMIE                         ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_AMIE_POS)) /**< INT_EN0_AMIE Mask */
#define MXC_V_I2C_INT_EN0_AMIE_DIS                     ((uint32_t)0x0UL) /**< INT_EN0_AMIE_DIS Value */
#define MXC_S_I2C_INT_EN0_AMIE_DIS                     (MXC_V_I2C_INT_EN0_AMIE_DIS << MXC_F_I2C_INT_EN0_AMIE_POS) /**< INT_EN0_AMIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_AMIE_EN                      ((uint32_t)0x1UL) /**< INT_EN0_AMIE_EN Value */
#define MXC_S_I2C_INT_EN0_AMIE_EN                      (MXC_V_I2C_INT_EN0_AMIE_EN << MXC_F_I2C_INT_EN0_AMIE_POS) /**< INT_EN0_AMIE_EN Setting */

#define MXC_F_I2C_INT_EN0_RXTHIE_POS                   4 /**< INT_EN0_RXTHIE Position */
#define MXC_F_I2C_INT_EN0_RXTHIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_RXTHIE_POS)) /**< INT_EN0_RXTHIE Mask */
#define MXC_V_I2C_INT_EN0_RXTHIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_RXTHIE_DIS Value */
#define MXC_S_I2C_INT_EN0_RXTHIE_DIS                   (MXC_V_I2C_INT_EN0_RXTHIE_DIS << MXC_F_I2C_INT_EN0_RXTHIE_POS) /**< INT_EN0_RXTHIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_RXTHIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_RXTHIE_EN Value */
#define MXC_S_I2C_INT_EN0_RXTHIE_EN                    (MXC_V_I2C_INT_EN0_RXTHIE_EN << MXC_F_I2C_INT_EN0_RXTHIE_POS) /**< INT_EN0_RXTHIE_EN Setting */

#define MXC_F_I2C_INT_EN0_TXTHIE_POS                   5 /**< INT_EN0_TXTHIE Position */
#define MXC_F_I2C_INT_EN0_TXTHIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_TXTHIE_POS)) /**< INT_EN0_TXTHIE Mask */
#define MXC_V_I2C_INT_EN0_TXTHIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_TXTHIE_DIS Value */
#define MXC_S_I2C_INT_EN0_TXTHIE_DIS                   (MXC_V_I2C_INT_EN0_TXTHIE_DIS << MXC_F_I2C_INT_EN0_TXTHIE_POS) /**< INT_EN0_TXTHIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_TXTHIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_TXTHIE_EN Value */
#define MXC_S_I2C_INT_EN0_TXTHIE_EN                    (MXC_V_I2C_INT_EN0_TXTHIE_EN << MXC_F_I2C_INT_EN0_TXTHIE_POS) /**< INT_EN0_TXTHIE_EN Setting */

#define MXC_F_I2C_INT_EN0_STOPIE_POS                   6 /**< INT_EN0_STOPIE Position */
#define MXC_F_I2C_INT_EN0_STOPIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_STOPIE_POS)) /**< INT_EN0_STOPIE Mask */
#define MXC_V_I2C_INT_EN0_STOPIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_STOPIE_DIS Value */
#define MXC_S_I2C_INT_EN0_STOPIE_DIS                   (MXC_V_I2C_INT_EN0_STOPIE_DIS << MXC_F_I2C_INT_EN0_STOPIE_POS) /**< INT_EN0_STOPIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_STOPIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_STOPIE_EN Value */
#define MXC_S_I2C_INT_EN0_STOPIE_EN                    (MXC_V_I2C_INT_EN0_STOPIE_EN << MXC_F_I2C_INT_EN0_STOPIE_POS) /**< INT_EN0_STOPIE_EN Setting */

#define MXC_F_I2C_INT_EN0_ADRACKIE_POS                 7 /**< INT_EN0_ADRACKIE Position */
#define MXC_F_I2C_INT_EN0_ADRACKIE                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ADRACKIE_POS)) /**< INT_EN0_ADRACKIE Mask */
#define MXC_V_I2C_INT_EN0_ADRACKIE_DIS                 ((uint32_t)0x0UL) /**< INT_EN0_ADRACKIE_DIS Value */
#define MXC_S_I2C_INT_EN0_ADRACKIE_DIS                 (MXC_V_I2C_INT_EN0_ADRACKIE_DIS << MXC_F_I2C_INT_EN0_ADRACKIE_POS) /**< INT_EN0_ADRACKIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_ADRACKIE_EN                  ((uint32_t)0x1UL) /**< INT_EN0_ADRACKIE_EN Value */
#define MXC_S_I2C_INT_EN0_ADRACKIE_EN                  (MXC_V_I2C_INT_EN0_ADRACKIE_EN << MXC_F_I2C_INT_EN0_ADRACKIE_POS) /**< INT_EN0_ADRACKIE_EN Setting */

#define MXC_F_I2C_INT_EN0_ARBERIE_POS                  8 /**< INT_EN0_ARBERIE Position */
#define MXC_F_I2C_INT_EN0_ARBERIE                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ARBERIE_POS)) /**< INT_EN0_ARBERIE Mask */
#define MXC_V_I2C_INT_EN0_ARBERIE_DIS                  ((uint32_t)0x0UL) /**< INT_EN0_ARBERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_ARBERIE_DIS                  (MXC_V_I2C_INT_EN0_ARBERIE_DIS << MXC_F_I2C_INT_EN0_ARBERIE_POS) /**< INT_EN0_ARBERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_ARBERIE_EN                   ((uint32_t)0x1UL) /**< INT_EN0_ARBERIE_EN Value */
#define MXC_S_I2C_INT_EN0_ARBERIE_EN                   (MXC_V_I2C_INT_EN0_ARBERIE_EN << MXC_F_I2C_INT_EN0_ARBERIE_POS) /**< INT_EN0_ARBERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_TOERIE_POS                   9 /**< INT_EN0_TOERIE Position */
#define MXC_F_I2C_INT_EN0_TOERIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_TOERIE_POS)) /**< INT_EN0_TOERIE Mask */
#define MXC_V_I2C_INT_EN0_TOERIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_TOERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_TOERIE_DIS                   (MXC_V_I2C_INT_EN0_TOERIE_DIS << MXC_F_I2C_INT_EN0_TOERIE_POS) /**< INT_EN0_TOERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_TOERIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_TOERIE_EN Value */
#define MXC_S_I2C_INT_EN0_TOERIE_EN                    (MXC_V_I2C_INT_EN0_TOERIE_EN << MXC_F_I2C_INT_EN0_TOERIE_POS) /**< INT_EN0_TOERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_ADRERIE_POS                  10 /**< INT_EN0_ADRERIE Position */
#define MXC_F_I2C_INT_EN0_ADRERIE                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_ADRERIE_POS)) /**< INT_EN0_ADRERIE Mask */
#define MXC_V_I2C_INT_EN0_ADRERIE_DIS                  ((uint32_t)0x0UL) /**< INT_EN0_ADRERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_ADRERIE_DIS                  (MXC_V_I2C_INT_EN0_ADRERIE_DIS << MXC_F_I2C_INT_EN0_ADRERIE_POS) /**< INT_EN0_ADRERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_ADRERIE_EN                   ((uint32_t)0x1UL) /**< INT_EN0_ADRERIE_EN Value */
#define MXC_S_I2C_INT_EN0_ADRERIE_EN                   (MXC_V_I2C_INT_EN0_ADRERIE_EN << MXC_F_I2C_INT_EN0_ADRERIE_POS) /**< INT_EN0_ADRERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_DATAERIE_POS                 11 /**< INT_EN0_DATAERIE Position */
#define MXC_F_I2C_INT_EN0_DATAERIE                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_DATAERIE_POS)) /**< INT_EN0_DATAERIE Mask */
#define MXC_V_I2C_INT_EN0_DATAERIE_DIS                 ((uint32_t)0x0UL) /**< INT_EN0_DATAERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_DATAERIE_DIS                 (MXC_V_I2C_INT_EN0_DATAERIE_DIS << MXC_F_I2C_INT_EN0_DATAERIE_POS) /**< INT_EN0_DATAERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_DATAERIE_EN                  ((uint32_t)0x1UL) /**< INT_EN0_DATAERIE_EN Value */
#define MXC_S_I2C_INT_EN0_DATAERIE_EN                  (MXC_V_I2C_INT_EN0_DATAERIE_EN << MXC_F_I2C_INT_EN0_DATAERIE_POS) /**< INT_EN0_DATAERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_DNRERIE_POS                  12 /**< INT_EN0_DNRERIE Position */
#define MXC_F_I2C_INT_EN0_DNRERIE                      ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_DNRERIE_POS)) /**< INT_EN0_DNRERIE Mask */
#define MXC_V_I2C_INT_EN0_DNRERIE_DIS                  ((uint32_t)0x0UL) /**< INT_EN0_DNRERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_DNRERIE_DIS                  (MXC_V_I2C_INT_EN0_DNRERIE_DIS << MXC_F_I2C_INT_EN0_DNRERIE_POS) /**< INT_EN0_DNRERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_DNRERIE_EN                   ((uint32_t)0x1UL) /**< INT_EN0_DNRERIE_EN Value */
#define MXC_S_I2C_INT_EN0_DNRERIE_EN                   (MXC_V_I2C_INT_EN0_DNRERIE_EN << MXC_F_I2C_INT_EN0_DNRERIE_POS) /**< INT_EN0_DNRERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_STRTERIE_POS                 13 /**< INT_EN0_STRTERIE Position */
#define MXC_F_I2C_INT_EN0_STRTERIE                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_STRTERIE_POS)) /**< INT_EN0_STRTERIE Mask */
#define MXC_V_I2C_INT_EN0_STRTERIE_DIS                 ((uint32_t)0x0UL) /**< INT_EN0_STRTERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_STRTERIE_DIS                 (MXC_V_I2C_INT_EN0_STRTERIE_DIS << MXC_F_I2C_INT_EN0_STRTERIE_POS) /**< INT_EN0_STRTERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_STRTERIE_EN                  ((uint32_t)0x1UL) /**< INT_EN0_STRTERIE_EN Value */
#define MXC_S_I2C_INT_EN0_STRTERIE_EN                  (MXC_V_I2C_INT_EN0_STRTERIE_EN << MXC_F_I2C_INT_EN0_STRTERIE_POS) /**< INT_EN0_STRTERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_STOPERIE_POS                 14 /**< INT_EN0_STOPERIE Position */
#define MXC_F_I2C_INT_EN0_STOPERIE                     ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_STOPERIE_POS)) /**< INT_EN0_STOPERIE Mask */
#define MXC_V_I2C_INT_EN0_STOPERIE_DIS                 ((uint32_t)0x0UL) /**< INT_EN0_STOPERIE_DIS Value */
#define MXC_S_I2C_INT_EN0_STOPERIE_DIS                 (MXC_V_I2C_INT_EN0_STOPERIE_DIS << MXC_F_I2C_INT_EN0_STOPERIE_POS) /**< INT_EN0_STOPERIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_STOPERIE_EN                  ((uint32_t)0x1UL) /**< INT_EN0_STOPERIE_EN Value */
#define MXC_S_I2C_INT_EN0_STOPERIE_EN                  (MXC_V_I2C_INT_EN0_STOPERIE_EN << MXC_F_I2C_INT_EN0_STOPERIE_POS) /**< INT_EN0_STOPERIE_EN Setting */

#define MXC_F_I2C_INT_EN0_TXLOIE_POS                   15 /**< INT_EN0_TXLOIE Position */
#define MXC_F_I2C_INT_EN0_TXLOIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN0_TXLOIE_POS)) /**< INT_EN0_TXLOIE Mask */
#define MXC_V_I2C_INT_EN0_TXLOIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN0_TXLOIE_DIS Value */
#define MXC_S_I2C_INT_EN0_TXLOIE_DIS                   (MXC_V_I2C_INT_EN0_TXLOIE_DIS << MXC_F_I2C_INT_EN0_TXLOIE_POS) /**< INT_EN0_TXLOIE_DIS Setting */
#define MXC_V_I2C_INT_EN0_TXLOIE_EN                    ((uint32_t)0x1UL) /**< INT_EN0_TXLOIE_EN Value */
#define MXC_S_I2C_INT_EN0_TXLOIE_EN                    (MXC_V_I2C_INT_EN0_TXLOIE_EN << MXC_F_I2C_INT_EN0_TXLOIE_POS) /**< INT_EN0_TXLOIE_EN Setting */

/**@} end of group I2C_INT_EN0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_FL1 I2C_INT_FL1
 * @brief    Interrupt Status Register 1.
 * @{
 */
#define MXC_F_I2C_INT_FL1_RXOFI_POS                    0 /**< INT_FL1_RXOFI Position */
#define MXC_F_I2C_INT_FL1_RXOFI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL1_RXOFI_POS)) /**< INT_FL1_RXOFI Mask */
#define MXC_V_I2C_INT_FL1_RXOFI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL1_RXOFI_INACTIVE Value */
#define MXC_S_I2C_INT_FL1_RXOFI_INACTIVE               (MXC_V_I2C_INT_FL1_RXOFI_INACTIVE << MXC_F_I2C_INT_FL1_RXOFI_POS) /**< INT_FL1_RXOFI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL1_RXOFI_PENDING                ((uint32_t)0x1UL) /**< INT_FL1_RXOFI_PENDING Value */
#define MXC_S_I2C_INT_FL1_RXOFI_PENDING                (MXC_V_I2C_INT_FL1_RXOFI_PENDING << MXC_F_I2C_INT_FL1_RXOFI_POS) /**< INT_FL1_RXOFI_PENDING Setting */

#define MXC_F_I2C_INT_FL1_TXUFI_POS                    1 /**< INT_FL1_TXUFI Position */
#define MXC_F_I2C_INT_FL1_TXUFI                        ((uint32_t)(0x1UL << MXC_F_I2C_INT_FL1_TXUFI_POS)) /**< INT_FL1_TXUFI Mask */
#define MXC_V_I2C_INT_FL1_TXUFI_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL1_TXUFI_INACTIVE Value */
#define MXC_S_I2C_INT_FL1_TXUFI_INACTIVE               (MXC_V_I2C_INT_FL1_TXUFI_INACTIVE << MXC_F_I2C_INT_FL1_TXUFI_POS) /**< INT_FL1_TXUFI_INACTIVE Setting */
#define MXC_V_I2C_INT_FL1_TXUFI_PENDING                ((uint32_t)0x1UL) /**< INT_FL1_TXUFI_PENDING Value */
#define MXC_S_I2C_INT_FL1_TXUFI_PENDING                (MXC_V_I2C_INT_FL1_TXUFI_PENDING << MXC_F_I2C_INT_FL1_TXUFI_POS) /**< INT_FL1_TXUFI_PENDING Setting */

/**@} end of group I2C_INT_FL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT_EN1 I2C_INT_EN1
 * @brief    Interrupt Staus Register 1.
 * @{
 */
#define MXC_F_I2C_INT_EN1_RXOFIE_POS                   0 /**< INT_EN1_RXOFIE Position */
#define MXC_F_I2C_INT_EN1_RXOFIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN1_RXOFIE_POS)) /**< INT_EN1_RXOFIE Mask */
#define MXC_V_I2C_INT_EN1_RXOFIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN1_RXOFIE_DIS Value */
#define MXC_S_I2C_INT_EN1_RXOFIE_DIS                   (MXC_V_I2C_INT_EN1_RXOFIE_DIS << MXC_F_I2C_INT_EN1_RXOFIE_POS) /**< INT_EN1_RXOFIE_DIS Setting */
#define MXC_V_I2C_INT_EN1_RXOFIE_EN                    ((uint32_t)0x1UL) /**< INT_EN1_RXOFIE_EN Value */
#define MXC_S_I2C_INT_EN1_RXOFIE_EN                    (MXC_V_I2C_INT_EN1_RXOFIE_EN << MXC_F_I2C_INT_EN1_RXOFIE_POS) /**< INT_EN1_RXOFIE_EN Setting */

#define MXC_F_I2C_INT_EN1_TXUFIE_POS                   1 /**< INT_EN1_TXUFIE Position */
#define MXC_F_I2C_INT_EN1_TXUFIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INT_EN1_TXUFIE_POS)) /**< INT_EN1_TXUFIE Mask */
#define MXC_V_I2C_INT_EN1_TXUFIE_DIS                   ((uint32_t)0x0UL) /**< INT_EN1_TXUFIE_DIS Value */
#define MXC_S_I2C_INT_EN1_TXUFIE_DIS                   (MXC_V_I2C_INT_EN1_TXUFIE_DIS << MXC_F_I2C_INT_EN1_TXUFIE_POS) /**< INT_EN1_TXUFIE_DIS Setting */
#define MXC_V_I2C_INT_EN1_TXUFIE_EN                    ((uint32_t)0x1UL) /**< INT_EN1_TXUFIE_EN Value */
#define MXC_S_I2C_INT_EN1_TXUFIE_EN                    (MXC_V_I2C_INT_EN1_TXUFIE_EN << MXC_F_I2C_INT_EN1_TXUFIE_POS) /**< INT_EN1_TXUFIE_EN Setting */

/**@} end of group I2C_INT_EN1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_FIFO_LEN I2C_FIFO_LEN
 * @brief    FIFO Configuration Register.
 * @{
 */
#define MXC_F_I2C_FIFO_LEN_RXLEN_POS                   0 /**< FIFO_LEN_RXLEN Position */
#define MXC_F_I2C_FIFO_LEN_RXLEN                       ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_LEN_RXLEN_POS)) /**< FIFO_LEN_RXLEN Mask */

#define MXC_F_I2C_FIFO_LEN_TXLEN_POS                   8 /**< FIFO_LEN_TXLEN Position */
#define MXC_F_I2C_FIFO_LEN_TXLEN                       ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_LEN_TXLEN_POS)) /**< FIFO_LEN_TXLEN Mask */

/**@} end of group I2C_FIFO_LEN_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RX_CTRL0 I2C_RX_CTRL0
 * @brief    Receive Control Register 0.
 * @{
 */
#define MXC_F_I2C_RX_CTRL0_DNR_POS                     0 /**< RX_CTRL0_DNR Position */
#define MXC_F_I2C_RX_CTRL0_DNR                         ((uint32_t)(0x1UL << MXC_F_I2C_RX_CTRL0_DNR_POS)) /**< RX_CTRL0_DNR Mask */
#define MXC_V_I2C_RX_CTRL0_DNR_RESPOND                 ((uint32_t)0x0UL) /**< RX_CTRL0_DNR_RESPOND Value */
#define MXC_S_I2C_RX_CTRL0_DNR_RESPOND                 (MXC_V_I2C_RX_CTRL0_DNR_RESPOND << MXC_F_I2C_RX_CTRL0_DNR_POS) /**< RX_CTRL0_DNR_RESPOND Setting */
#define MXC_V_I2C_RX_CTRL0_DNR_DONT_RESPOND            ((uint32_t)0x1UL) /**< RX_CTRL0_DNR_DONT_RESPOND Value */
#define MXC_S_I2C_RX_CTRL0_DNR_DONT_RESPOND            (MXC_V_I2C_RX_CTRL0_DNR_DONT_RESPOND << MXC_F_I2C_RX_CTRL0_DNR_POS) /**< RX_CTRL0_DNR_DONT_RESPOND Setting */

#define MXC_F_I2C_RX_CTRL0_RXFSH_POS                   7 /**< RX_CTRL0_RXFSH Position */
#define MXC_F_I2C_RX_CTRL0_RXFSH                       ((uint32_t)(0x1UL << MXC_F_I2C_RX_CTRL0_RXFSH_POS)) /**< RX_CTRL0_RXFSH Mask */
#define MXC_V_I2C_RX_CTRL0_RXFSH_NOT_FLUSHED           ((uint32_t)0x0UL) /**< RX_CTRL0_RXFSH_NOT_FLUSHED Value */
#define MXC_S_I2C_RX_CTRL0_RXFSH_NOT_FLUSHED           (MXC_V_I2C_RX_CTRL0_RXFSH_NOT_FLUSHED << MXC_F_I2C_RX_CTRL0_RXFSH_POS) /**< RX_CTRL0_RXFSH_NOT_FLUSHED Setting */
#define MXC_V_I2C_RX_CTRL0_RXFSH_FLUSH                 ((uint32_t)0x1UL) /**< RX_CTRL0_RXFSH_FLUSH Value */
#define MXC_S_I2C_RX_CTRL0_RXFSH_FLUSH                 (MXC_V_I2C_RX_CTRL0_RXFSH_FLUSH << MXC_F_I2C_RX_CTRL0_RXFSH_POS) /**< RX_CTRL0_RXFSH_FLUSH Setting */

#define MXC_F_I2C_RX_CTRL0_RXTH_POS                    8 /**< RX_CTRL0_RXTH Position */
#define MXC_F_I2C_RX_CTRL0_RXTH                        ((uint32_t)(0xFUL << MXC_F_I2C_RX_CTRL0_RXTH_POS)) /**< RX_CTRL0_RXTH Mask */

/**@} end of group I2C_RX_CTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RX_CTRL1 I2C_RX_CTRL1
 * @brief    Receive Control Register 1.
 * @{
 */
#define MXC_F_I2C_RX_CTRL1_RXCNT_POS                   0 /**< RX_CTRL1_RXCNT Position */
#define MXC_F_I2C_RX_CTRL1_RXCNT                       ((uint32_t)(0xFFUL << MXC_F_I2C_RX_CTRL1_RXCNT_POS)) /**< RX_CTRL1_RXCNT Mask */

#define MXC_F_I2C_RX_CTRL1_RXFIFO_POS                  8 /**< RX_CTRL1_RXFIFO Position */
#define MXC_F_I2C_RX_CTRL1_RXFIFO                      ((uint32_t)(0xFUL << MXC_F_I2C_RX_CTRL1_RXFIFO_POS)) /**< RX_CTRL1_RXFIFO Mask */

/**@} end of group I2C_RX_CTRL1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TX_CTRL0 I2C_TX_CTRL0
 * @brief    Transmit Control Register 0.
 * @{
 */
#define MXC_F_I2C_TX_CTRL0_TXPRELD_POS                 0 /**< TX_CTRL0_TXPRELD Position */
#define MXC_F_I2C_TX_CTRL0_TXPRELD                     ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TXPRELD_POS)) /**< TX_CTRL0_TXPRELD Mask */
#define MXC_V_I2C_TX_CTRL0_TXPRELD_NORMAL              ((uint32_t)0x0UL) /**< TX_CTRL0_TXPRELD_NORMAL Value */
#define MXC_S_I2C_TX_CTRL0_TXPRELD_NORMAL              (MXC_V_I2C_TX_CTRL0_TXPRELD_NORMAL << MXC_F_I2C_TX_CTRL0_TXPRELD_POS) /**< TX_CTRL0_TXPRELD_NORMAL Setting */
#define MXC_V_I2C_TX_CTRL0_TXPRELD_PRELOAD             ((uint32_t)0x1UL) /**< TX_CTRL0_TXPRELD_PRELOAD Value */
#define MXC_S_I2C_TX_CTRL0_TXPRELD_PRELOAD             (MXC_V_I2C_TX_CTRL0_TXPRELD_PRELOAD << MXC_F_I2C_TX_CTRL0_TXPRELD_POS) /**< TX_CTRL0_TXPRELD_PRELOAD Setting */

#define MXC_F_I2C_TX_CTRL0_TXFSH_POS                   7 /**< TX_CTRL0_TXFSH Position */
#define MXC_F_I2C_TX_CTRL0_TXFSH                       ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL0_TXFSH_POS)) /**< TX_CTRL0_TXFSH Mask */
#define MXC_V_I2C_TX_CTRL0_TXFSH_NOT_FLUSHED           ((uint32_t)0x0UL) /**< TX_CTRL0_TXFSH_NOT_FLUSHED Value */
#define MXC_S_I2C_TX_CTRL0_TXFSH_NOT_FLUSHED           (MXC_V_I2C_TX_CTRL0_TXFSH_NOT_FLUSHED << MXC_F_I2C_TX_CTRL0_TXFSH_POS) /**< TX_CTRL0_TXFSH_NOT_FLUSHED Setting */
#define MXC_V_I2C_TX_CTRL0_TXFSH_FLUSH                 ((uint32_t)0x1UL) /**< TX_CTRL0_TXFSH_FLUSH Value */
#define MXC_S_I2C_TX_CTRL0_TXFSH_FLUSH                 (MXC_V_I2C_TX_CTRL0_TXFSH_FLUSH << MXC_F_I2C_TX_CTRL0_TXFSH_POS) /**< TX_CTRL0_TXFSH_FLUSH Setting */

#define MXC_F_I2C_TX_CTRL0_TXTH_POS                    8 /**< TX_CTRL0_TXTH Position */
#define MXC_F_I2C_TX_CTRL0_TXTH                        ((uint32_t)(0xFUL << MXC_F_I2C_TX_CTRL0_TXTH_POS)) /**< TX_CTRL0_TXTH Mask */

/**@} end of group I2C_TX_CTRL0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TX_CTRL1 I2C_TX_CTRL1
 * @brief    Transmit Control Register 1.
 * @{
 */
#define MXC_F_I2C_TX_CTRL1_TXRDY_POS                   0 /**< TX_CTRL1_TXRDY Position */
#define MXC_F_I2C_TX_CTRL1_TXRDY                       ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL1_TXRDY_POS)) /**< TX_CTRL1_TXRDY Mask */
#define MXC_V_I2C_TX_CTRL1_TXRDY_NOT_READY             ((uint32_t)0x0UL) /**< TX_CTRL1_TXRDY_NOT_READY Value */
#define MXC_S_I2C_TX_CTRL1_TXRDY_NOT_READY             (MXC_V_I2C_TX_CTRL1_TXRDY_NOT_READY << MXC_F_I2C_TX_CTRL1_TXRDY_POS) /**< TX_CTRL1_TXRDY_NOT_READY Setting */
#define MXC_V_I2C_TX_CTRL1_TXRDY_READY                 ((uint32_t)0x1UL) /**< TX_CTRL1_TXRDY_READY Value */
#define MXC_S_I2C_TX_CTRL1_TXRDY_READY                 (MXC_V_I2C_TX_CTRL1_TXRDY_READY << MXC_F_I2C_TX_CTRL1_TXRDY_POS) /**< TX_CTRL1_TXRDY_READY Setting */

#define MXC_F_I2C_TX_CTRL1_TXLAST_POS                  1 /**< TX_CTRL1_TXLAST Position */
#define MXC_F_I2C_TX_CTRL1_TXLAST                      ((uint32_t)(0x1UL << MXC_F_I2C_TX_CTRL1_TXLAST_POS)) /**< TX_CTRL1_TXLAST Mask */
#define MXC_V_I2C_TX_CTRL1_TXLAST_PAUSE_ON_LAST        ((uint32_t)0x0UL) /**< TX_CTRL1_TXLAST_PAUSE_ON_LAST Value */
#define MXC_S_I2C_TX_CTRL1_TXLAST_PAUSE_ON_LAST        (MXC_V_I2C_TX_CTRL1_TXLAST_PAUSE_ON_LAST << MXC_F_I2C_TX_CTRL1_TXLAST_POS) /**< TX_CTRL1_TXLAST_PAUSE_ON_LAST Setting */
#define MXC_V_I2C_TX_CTRL1_TXLAST_END_ON_LAST          ((uint32_t)0x1UL) /**< TX_CTRL1_TXLAST_END_ON_LAST Value */
#define MXC_S_I2C_TX_CTRL1_TXLAST_END_ON_LAST          (MXC_V_I2C_TX_CTRL1_TXLAST_END_ON_LAST << MXC_F_I2C_TX_CTRL1_TXLAST_POS) /**< TX_CTRL1_TXLAST_END_ON_LAST Setting */

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
 * @defgroup I2C_MSTR_MODE I2C_MSTR_MODE
 * @brief    Master Control Register.
 * @{
 */
#define MXC_F_I2C_MSTR_MODE_START_POS                  0 /**< MSTR_MODE_START Position */
#define MXC_F_I2C_MSTR_MODE_START                      ((uint32_t)(0x1UL << MXC_F_I2C_MSTR_MODE_START_POS)) /**< MSTR_MODE_START Mask */
#define MXC_V_I2C_MSTR_MODE_START_START                ((uint32_t)0x1UL) /**< MSTR_MODE_START_START Value */
#define MXC_S_I2C_MSTR_MODE_START_START                (MXC_V_I2C_MSTR_MODE_START_START << MXC_F_I2C_MSTR_MODE_START_POS) /**< MSTR_MODE_START_START Setting */

#define MXC_F_I2C_MSTR_MODE_RESTART_POS                1 /**< MSTR_MODE_RESTART Position */
#define MXC_F_I2C_MSTR_MODE_RESTART                    ((uint32_t)(0x1UL << MXC_F_I2C_MSTR_MODE_RESTART_POS)) /**< MSTR_MODE_RESTART Mask */
#define MXC_V_I2C_MSTR_MODE_RESTART_RESTART            ((uint32_t)0x1UL) /**< MSTR_MODE_RESTART_RESTART Value */
#define MXC_S_I2C_MSTR_MODE_RESTART_RESTART            (MXC_V_I2C_MSTR_MODE_RESTART_RESTART << MXC_F_I2C_MSTR_MODE_RESTART_POS) /**< MSTR_MODE_RESTART_RESTART Setting */

#define MXC_F_I2C_MSTR_MODE_STOP_POS                   2 /**< MSTR_MODE_STOP Position */
#define MXC_F_I2C_MSTR_MODE_STOP                       ((uint32_t)(0x1UL << MXC_F_I2C_MSTR_MODE_STOP_POS)) /**< MSTR_MODE_STOP Mask */
#define MXC_V_I2C_MSTR_MODE_STOP_STOP                  ((uint32_t)0x1UL) /**< MSTR_MODE_STOP_STOP Value */
#define MXC_S_I2C_MSTR_MODE_STOP_STOP                  (MXC_V_I2C_MSTR_MODE_STOP_STOP << MXC_F_I2C_MSTR_MODE_STOP_POS) /**< MSTR_MODE_STOP_STOP Setting */

#define MXC_F_I2C_MSTR_MODE_SEA_POS                    7 /**< MSTR_MODE_SEA Position */
#define MXC_F_I2C_MSTR_MODE_SEA                        ((uint32_t)(0x1UL << MXC_F_I2C_MSTR_MODE_SEA_POS)) /**< MSTR_MODE_SEA Mask */
#define MXC_V_I2C_MSTR_MODE_SEA_7BIT_ADDR              ((uint32_t)0x0UL) /**< MSTR_MODE_SEA_7BIT_ADDR Value */
#define MXC_S_I2C_MSTR_MODE_SEA_7BIT_ADDR              (MXC_V_I2C_MSTR_MODE_SEA_7BIT_ADDR << MXC_F_I2C_MSTR_MODE_SEA_POS) /**< MSTR_MODE_SEA_7BIT_ADDR Setting */
#define MXC_V_I2C_MSTR_MODE_SEA_10BIT_ADDR             ((uint32_t)0x1UL) /**< MSTR_MODE_SEA_10BIT_ADDR Value */
#define MXC_S_I2C_MSTR_MODE_SEA_10BIT_ADDR             (MXC_V_I2C_MSTR_MODE_SEA_10BIT_ADDR << MXC_F_I2C_MSTR_MODE_SEA_POS) /**< MSTR_MODE_SEA_10BIT_ADDR Setting */

/**@} end of group I2C_MSTR_MODE_Register */

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
 * @defgroup I2C_TIMEOUT I2C_TIMEOUT
 * @brief    Timeout Register
 * @{
 */
#define MXC_F_I2C_TIMEOUT_TO_POS                       0 /**< TIMEOUT_TO Position */
#define MXC_F_I2C_TIMEOUT_TO                           ((uint32_t)(0xFFFFUL << MXC_F_I2C_TIMEOUT_TO_POS)) /**< TIMEOUT_TO Mask */

/**@} end of group I2C_TIMEOUT_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_SLV_ADDR I2C_SLV_ADDR
 * @brief    Slave Address Register.
 * @{
 */
#define MXC_F_I2C_SLV_ADDR_SLA_POS                     0 /**< SLV_ADDR_SLA Position */
#define MXC_F_I2C_SLV_ADDR_SLA                         ((uint32_t)(0x3FFUL << MXC_F_I2C_SLV_ADDR_SLA_POS)) /**< SLV_ADDR_SLA Mask */

#define MXC_F_I2C_SLV_ADDR_EA_POS                      15 /**< SLV_ADDR_EA Position */
#define MXC_F_I2C_SLV_ADDR_EA                          ((uint32_t)(0x1UL << MXC_F_I2C_SLV_ADDR_EA_POS)) /**< SLV_ADDR_EA Mask */
#define MXC_V_I2C_SLV_ADDR_EA_7BIT_ADDR                ((uint32_t)0x0UL) /**< SLV_ADDR_EA_7BIT_ADDR Value */
#define MXC_S_I2C_SLV_ADDR_EA_7BIT_ADDR                (MXC_V_I2C_SLV_ADDR_EA_7BIT_ADDR << MXC_F_I2C_SLV_ADDR_EA_POS) /**< SLV_ADDR_EA_7BIT_ADDR Setting */
#define MXC_V_I2C_SLV_ADDR_EA_10BIT_ADDR               ((uint32_t)0x1UL) /**< SLV_ADDR_EA_10BIT_ADDR Value */
#define MXC_S_I2C_SLV_ADDR_EA_10BIT_ADDR               (MXC_V_I2C_SLV_ADDR_EA_10BIT_ADDR << MXC_F_I2C_SLV_ADDR_EA_POS) /**< SLV_ADDR_EA_10BIT_ADDR Setting */

/**@} end of group I2C_SLV_ADDR_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_DMA I2C_DMA
 * @brief    DMA Register.
 * @{
 */
#define MXC_F_I2C_DMA_TXEN_POS                         0 /**< DMA_TXEN Position */
#define MXC_F_I2C_DMA_TXEN                             ((uint32_t)(0x1UL << MXC_F_I2C_DMA_TXEN_POS)) /**< DMA_TXEN Mask */
#define MXC_V_I2C_DMA_TXEN_DIS                         ((uint32_t)0x0UL) /**< DMA_TXEN_DIS Value */
#define MXC_S_I2C_DMA_TXEN_DIS                         (MXC_V_I2C_DMA_TXEN_DIS << MXC_F_I2C_DMA_TXEN_POS) /**< DMA_TXEN_DIS Setting */
#define MXC_V_I2C_DMA_TXEN_EN                          ((uint32_t)0x1UL) /**< DMA_TXEN_EN Value */
#define MXC_S_I2C_DMA_TXEN_EN                          (MXC_V_I2C_DMA_TXEN_EN << MXC_F_I2C_DMA_TXEN_POS) /**< DMA_TXEN_EN Setting */

#define MXC_F_I2C_DMA_RXEN_POS                         1 /**< DMA_RXEN Position */
#define MXC_F_I2C_DMA_RXEN                             ((uint32_t)(0x1UL << MXC_F_I2C_DMA_RXEN_POS)) /**< DMA_RXEN Mask */
#define MXC_V_I2C_DMA_RXEN_DIS                         ((uint32_t)0x0UL) /**< DMA_RXEN_DIS Value */
#define MXC_S_I2C_DMA_RXEN_DIS                         (MXC_V_I2C_DMA_RXEN_DIS << MXC_F_I2C_DMA_RXEN_POS) /**< DMA_RXEN_DIS Setting */
#define MXC_V_I2C_DMA_RXEN_EN                          ((uint32_t)0x1UL) /**< DMA_RXEN_EN Value */
#define MXC_S_I2C_DMA_RXEN_EN                          (MXC_V_I2C_DMA_RXEN_EN << MXC_F_I2C_DMA_RXEN_POS) /**< DMA_RXEN_EN Setting */

/**@} end of group I2C_DMA_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_I2C_REGS_H_

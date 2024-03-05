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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_I2C_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_I2C_REGS_H_

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
    __IO uint32_t cn;                   /**< <tt>\b 0x00:</tt> I2C CN Register */
    __IO uint32_t st;                   /**< <tt>\b 0x04:</tt> I2C ST Register */
    __IO uint32_t int0;                 /**< <tt>\b 0x08:</tt> I2C INT0 Register */
    __IO uint32_t inten0;               /**< <tt>\b 0x0C:</tt> I2C INTEN0 Register */
    __IO uint32_t int1;                 /**< <tt>\b 0x10:</tt> I2C INT1 Register */
    __IO uint32_t inten1;               /**< <tt>\b 0x14:</tt> I2C INTEN1 Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x18:</tt> I2C FIFO Register */
    __IO uint32_t rxcfg;                /**< <tt>\b 0x1C:</tt> I2C RXCFG Register */
    __IO uint32_t rx;                   /**< <tt>\b 0x20:</tt> I2C RX Register */
    __IO uint32_t txcfg;                /**< <tt>\b 0x24:</tt> I2C TXCFG Register */
    __IO uint32_t tx;                   /**< <tt>\b 0x28:</tt> I2C TX Register */
    __IO uint32_t data;                 /**< <tt>\b 0x2C:</tt> I2C DATA Register */
    __IO uint32_t mcn;                  /**< <tt>\b 0x30:</tt> I2C MCN Register */
    __IO uint32_t ckl;                  /**< <tt>\b 0x34:</tt> I2C CKL Register */
    __IO uint32_t ckh;                  /**< <tt>\b 0x38:</tt> I2C CKH Register */
    __R  uint32_t rsv_0x3c;
    __IO uint32_t to;                   /**< <tt>\b 0x40:</tt> I2C TO Register */
    __R  uint32_t rsv_0x44;
    __IO uint32_t dma;                  /**< <tt>\b 0x48:</tt> I2C DMA Register */
    __IO uint32_t sla;                  /**< <tt>\b 0x4C:</tt> I2C SLA Register */
} mxc_i2c_regs_t;

/* Register offsets for module I2C */
/**
 * @ingroup    i2c_registers
 * @defgroup   I2C_Register_Offsets Register Offsets
 * @brief      I2C Peripheral Register Offsets from the I2C Base Peripheral Address.
 * @{
 */
#define MXC_R_I2C_CN                       ((uint32_t)0x00000000UL) /**< Offset from I2C Base Address: <tt> 0x0000</tt> */
#define MXC_R_I2C_ST                       ((uint32_t)0x00000004UL) /**< Offset from I2C Base Address: <tt> 0x0004</tt> */
#define MXC_R_I2C_INT0                     ((uint32_t)0x00000008UL) /**< Offset from I2C Base Address: <tt> 0x0008</tt> */
#define MXC_R_I2C_INTEN0                   ((uint32_t)0x0000000CUL) /**< Offset from I2C Base Address: <tt> 0x000C</tt> */
#define MXC_R_I2C_INT1                     ((uint32_t)0x00000010UL) /**< Offset from I2C Base Address: <tt> 0x0010</tt> */
#define MXC_R_I2C_INTEN1                   ((uint32_t)0x00000014UL) /**< Offset from I2C Base Address: <tt> 0x0014</tt> */
#define MXC_R_I2C_FIFO                     ((uint32_t)0x00000018UL) /**< Offset from I2C Base Address: <tt> 0x0018</tt> */
#define MXC_R_I2C_RXCFG                    ((uint32_t)0x0000001CUL) /**< Offset from I2C Base Address: <tt> 0x001C</tt> */
#define MXC_R_I2C_RX                       ((uint32_t)0x00000020UL) /**< Offset from I2C Base Address: <tt> 0x0020</tt> */
#define MXC_R_I2C_TXCFG                    ((uint32_t)0x00000024UL) /**< Offset from I2C Base Address: <tt> 0x0024</tt> */
#define MXC_R_I2C_TX                       ((uint32_t)0x00000028UL) /**< Offset from I2C Base Address: <tt> 0x0028</tt> */
#define MXC_R_I2C_DATA                     ((uint32_t)0x0000002CUL) /**< Offset from I2C Base Address: <tt> 0x002C</tt> */
#define MXC_R_I2C_MCN                      ((uint32_t)0x00000030UL) /**< Offset from I2C Base Address: <tt> 0x0030</tt> */
#define MXC_R_I2C_CKL                      ((uint32_t)0x00000034UL) /**< Offset from I2C Base Address: <tt> 0x0034</tt> */
#define MXC_R_I2C_CKH                      ((uint32_t)0x00000038UL) /**< Offset from I2C Base Address: <tt> 0x0038</tt> */
#define MXC_R_I2C_TO                       ((uint32_t)0x00000040UL) /**< Offset from I2C Base Address: <tt> 0x0040</tt> */
#define MXC_R_I2C_DMA                      ((uint32_t)0x00000048UL) /**< Offset from I2C Base Address: <tt> 0x0048</tt> */
#define MXC_R_I2C_SLA                      ((uint32_t)0x0000004CUL) /**< Offset from I2C Base Address: <tt> 0x004C</tt> */
/**@} end of group i2c_registers */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CN I2C_CN
 * @brief    Control Register0.
 * @{
 */
#define MXC_F_I2C_CN_I2CEN_POS                         0 /**< CN_I2CEN Position */
#define MXC_F_I2C_CN_I2CEN                             ((uint32_t)(0x1UL << MXC_F_I2C_CN_I2CEN_POS)) /**< CN_I2CEN Mask */

#define MXC_F_I2C_CN_MST_POS                           1 /**< CN_MST Position */
#define MXC_F_I2C_CN_MST                               ((uint32_t)(0x1UL << MXC_F_I2C_CN_MST_POS)) /**< CN_MST Mask */

#define MXC_F_I2C_CN_GCEN_POS                          2 /**< CN_GCEN Position */
#define MXC_F_I2C_CN_GCEN                              ((uint32_t)(0x1UL << MXC_F_I2C_CN_GCEN_POS)) /**< CN_GCEN Mask */

#define MXC_F_I2C_CN_IRXM_POS                          3 /**< CN_IRXM Position */
#define MXC_F_I2C_CN_IRXM                              ((uint32_t)(0x1UL << MXC_F_I2C_CN_IRXM_POS)) /**< CN_IRXM Mask */

#define MXC_F_I2C_CN_ACK_POS                           4 /**< CN_ACK Position */
#define MXC_F_I2C_CN_ACK                               ((uint32_t)(0x1UL << MXC_F_I2C_CN_ACK_POS)) /**< CN_ACK Mask */

#define MXC_F_I2C_CN_SCLO_POS                          6 /**< CN_SCLO Position */
#define MXC_F_I2C_CN_SCLO                              ((uint32_t)(0x1UL << MXC_F_I2C_CN_SCLO_POS)) /**< CN_SCLO Mask */

#define MXC_F_I2C_CN_SDAO_POS                          7 /**< CN_SDAO Position */
#define MXC_F_I2C_CN_SDAO                              ((uint32_t)(0x1UL << MXC_F_I2C_CN_SDAO_POS)) /**< CN_SDAO Mask */

#define MXC_F_I2C_CN_SCL_POS                           8 /**< CN_SCL Position */
#define MXC_F_I2C_CN_SCL                               ((uint32_t)(0x1UL << MXC_F_I2C_CN_SCL_POS)) /**< CN_SCL Mask */

#define MXC_F_I2C_CN_SDA_POS                           9 /**< CN_SDA Position */
#define MXC_F_I2C_CN_SDA                               ((uint32_t)(0x1UL << MXC_F_I2C_CN_SDA_POS)) /**< CN_SDA Mask */

#define MXC_F_I2C_CN_SWOE_POS                          10 /**< CN_SWOE Position */
#define MXC_F_I2C_CN_SWOE                              ((uint32_t)(0x1UL << MXC_F_I2C_CN_SWOE_POS)) /**< CN_SWOE Mask */

#define MXC_F_I2C_CN_READ_POS                          11 /**< CN_READ Position */
#define MXC_F_I2C_CN_READ                              ((uint32_t)(0x1UL << MXC_F_I2C_CN_READ_POS)) /**< CN_READ Mask */

#define MXC_F_I2C_CN_SCLSTRD_POS                       12 /**< CN_SCLSTRD Position */
#define MXC_F_I2C_CN_SCLSTRD                           ((uint32_t)(0x1UL << MXC_F_I2C_CN_SCLSTRD_POS)) /**< CN_SCLSTRD Mask */

#define MXC_F_I2C_CN_SCLPPM_POS                        13 /**< CN_SCLPPM Position */
#define MXC_F_I2C_CN_SCLPPM                            ((uint32_t)(0x1UL << MXC_F_I2C_CN_SCLPPM_POS)) /**< CN_SCLPPM Mask */

/**@} end of group I2C_CN_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_ST I2C_ST
 * @brief    Status Register.
 * @{
 */
#define MXC_F_I2C_ST_BUS_POS                           0 /**< ST_BUS Position */
#define MXC_F_I2C_ST_BUS                               ((uint32_t)(0x1UL << MXC_F_I2C_ST_BUS_POS)) /**< ST_BUS Mask */

#define MXC_F_I2C_ST_RXE_POS                           1 /**< ST_RXE Position */
#define MXC_F_I2C_ST_RXE                               ((uint32_t)(0x1UL << MXC_F_I2C_ST_RXE_POS)) /**< ST_RXE Mask */

#define MXC_F_I2C_ST_RXF_POS                           2 /**< ST_RXF Position */
#define MXC_F_I2C_ST_RXF                               ((uint32_t)(0x1UL << MXC_F_I2C_ST_RXF_POS)) /**< ST_RXF Mask */

#define MXC_F_I2C_ST_TXE_POS                           3 /**< ST_TXE Position */
#define MXC_F_I2C_ST_TXE                               ((uint32_t)(0x1UL << MXC_F_I2C_ST_TXE_POS)) /**< ST_TXE Mask */

#define MXC_F_I2C_ST_TXF_POS                           4 /**< ST_TXF Position */
#define MXC_F_I2C_ST_TXF                               ((uint32_t)(0x1UL << MXC_F_I2C_ST_TXF_POS)) /**< ST_TXF Mask */

#define MXC_F_I2C_ST_CKMD_POS                          5 /**< ST_CKMD Position */
#define MXC_F_I2C_ST_CKMD                              ((uint32_t)(0x1UL << MXC_F_I2C_ST_CKMD_POS)) /**< ST_CKMD Mask */

#define MXC_F_I2C_ST_ST_POS                            8 /**< ST_ST Position */
#define MXC_F_I2C_ST_ST                                ((uint32_t)(0xFUL << MXC_F_I2C_ST_ST_POS)) /**< ST_ST Mask */

/**@} end of group I2C_ST_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT0 I2C_INT0
 * @brief    Interrupt Status Register.
 * @{
 */
#define MXC_F_I2C_INT0_DONEI_POS                       0 /**< INT0_DONEI Position */
#define MXC_F_I2C_INT0_DONEI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_DONEI_POS)) /**< INT0_DONEI Mask */

#define MXC_F_I2C_INT0_IRXMI_POS                       1 /**< INT0_IRXMI Position */
#define MXC_F_I2C_INT0_IRXMI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_IRXMI_POS)) /**< INT0_IRXMI Mask */

#define MXC_F_I2C_INT0_GCI_POS                         2 /**< INT0_GCI Position */
#define MXC_F_I2C_INT0_GCI                             ((uint32_t)(0x1UL << MXC_F_I2C_INT0_GCI_POS)) /**< INT0_GCI Mask */

#define MXC_F_I2C_INT0_AMI_POS                         3 /**< INT0_AMI Position */
#define MXC_F_I2C_INT0_AMI                             ((uint32_t)(0x1UL << MXC_F_I2C_INT0_AMI_POS)) /**< INT0_AMI Mask */

#define MXC_F_I2C_INT0_RXTHI_POS                       4 /**< INT0_RXTHI Position */
#define MXC_F_I2C_INT0_RXTHI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_RXTHI_POS)) /**< INT0_RXTHI Mask */

#define MXC_F_I2C_INT0_TXTHI_POS                       5 /**< INT0_TXTHI Position */
#define MXC_F_I2C_INT0_TXTHI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_TXTHI_POS)) /**< INT0_TXTHI Mask */

#define MXC_F_I2C_INT0_STOPI_POS                       6 /**< INT0_STOPI Position */
#define MXC_F_I2C_INT0_STOPI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_STOPI_POS)) /**< INT0_STOPI Mask */

#define MXC_F_I2C_INT0_ADRACKI_POS                     7 /**< INT0_ADRACKI Position */
#define MXC_F_I2C_INT0_ADRACKI                         ((uint32_t)(0x1UL << MXC_F_I2C_INT0_ADRACKI_POS)) /**< INT0_ADRACKI Mask */

#define MXC_F_I2C_INT0_ARBERI_POS                      8 /**< INT0_ARBERI Position */
#define MXC_F_I2C_INT0_ARBERI                          ((uint32_t)(0x1UL << MXC_F_I2C_INT0_ARBERI_POS)) /**< INT0_ARBERI Mask */

#define MXC_F_I2C_INT0_TOERI_POS                       9 /**< INT0_TOERI Position */
#define MXC_F_I2C_INT0_TOERI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_TOERI_POS)) /**< INT0_TOERI Mask */

#define MXC_F_I2C_INT0_ADRERI_POS                      10 /**< INT0_ADRERI Position */
#define MXC_F_I2C_INT0_ADRERI                          ((uint32_t)(0x1UL << MXC_F_I2C_INT0_ADRERI_POS)) /**< INT0_ADRERI Mask */

#define MXC_F_I2C_INT0_DATERI_POS                      11 /**< INT0_DATERI Position */
#define MXC_F_I2C_INT0_DATERI                          ((uint32_t)(0x1UL << MXC_F_I2C_INT0_DATERI_POS)) /**< INT0_DATERI Mask */

#define MXC_F_I2C_INT0_DNRERI_POS                      12 /**< INT0_DNRERI Position */
#define MXC_F_I2C_INT0_DNRERI                          ((uint32_t)(0x1UL << MXC_F_I2C_INT0_DNRERI_POS)) /**< INT0_DNRERI Mask */

#define MXC_F_I2C_INT0_STRTERI_POS                     13 /**< INT0_STRTERI Position */
#define MXC_F_I2C_INT0_STRTERI                         ((uint32_t)(0x1UL << MXC_F_I2C_INT0_STRTERI_POS)) /**< INT0_STRTERI Mask */

#define MXC_F_I2C_INT0_STOPERI_POS                     14 /**< INT0_STOPERI Position */
#define MXC_F_I2C_INT0_STOPERI                         ((uint32_t)(0x1UL << MXC_F_I2C_INT0_STOPERI_POS)) /**< INT0_STOPERI Mask */

#define MXC_F_I2C_INT0_TXLOI_POS                       15 /**< INT0_TXLOI Position */
#define MXC_F_I2C_INT0_TXLOI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_TXLOI_POS)) /**< INT0_TXLOI Mask */

#define MXC_F_I2C_INT0_RDAMI_POS                       22 /**< INT0_RDAMI Position */
#define MXC_F_I2C_INT0_RDAMI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_RDAMI_POS)) /**< INT0_RDAMI Mask */

#define MXC_F_I2C_INT0_WRAMI_POS                       23 /**< INT0_WRAMI Position */
#define MXC_F_I2C_INT0_WRAMI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT0_WRAMI_POS)) /**< INT0_WRAMI Mask */

/**@} end of group I2C_INT0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INTEN0 I2C_INTEN0
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_I2C_INTEN0_DONEIE_POS                    0 /**< INTEN0_DONEIE Position */
#define MXC_F_I2C_INTEN0_DONEIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_DONEIE_POS)) /**< INTEN0_DONEIE Mask */

#define MXC_F_I2C_INTEN0_IRXMIE_POS                    1 /**< INTEN0_IRXMIE Position */
#define MXC_F_I2C_INTEN0_IRXMIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_IRXMIE_POS)) /**< INTEN0_IRXMIE Mask */

#define MXC_F_I2C_INTEN0_GCIE_POS                      2 /**< INTEN0_GCIE Position */
#define MXC_F_I2C_INTEN0_GCIE                          ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_GCIE_POS)) /**< INTEN0_GCIE Mask */

#define MXC_F_I2C_INTEN0_AMIE_POS                      3 /**< INTEN0_AMIE Position */
#define MXC_F_I2C_INTEN0_AMIE                          ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_AMIE_POS)) /**< INTEN0_AMIE Mask */

#define MXC_F_I2C_INTEN0_RXTHIE_POS                    4 /**< INTEN0_RXTHIE Position */
#define MXC_F_I2C_INTEN0_RXTHIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_RXTHIE_POS)) /**< INTEN0_RXTHIE Mask */

#define MXC_F_I2C_INTEN0_TXTHIE_POS                    5 /**< INTEN0_TXTHIE Position */
#define MXC_F_I2C_INTEN0_TXTHIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_TXTHIE_POS)) /**< INTEN0_TXTHIE Mask */

#define MXC_F_I2C_INTEN0_STOPIE_POS                    6 /**< INTEN0_STOPIE Position */
#define MXC_F_I2C_INTEN0_STOPIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_STOPIE_POS)) /**< INTEN0_STOPIE Mask */

#define MXC_F_I2C_INTEN0_ADRACKIE_POS                  7 /**< INTEN0_ADRACKIE Position */
#define MXC_F_I2C_INTEN0_ADRACKIE                      ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ADRACKIE_POS)) /**< INTEN0_ADRACKIE Mask */

#define MXC_F_I2C_INTEN0_ARBERIE_POS                   8 /**< INTEN0_ARBERIE Position */
#define MXC_F_I2C_INTEN0_ARBERIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ARBERIE_POS)) /**< INTEN0_ARBERIE Mask */

#define MXC_F_I2C_INTEN0_TOERIE_POS                    9 /**< INTEN0_TOERIE Position */
#define MXC_F_I2C_INTEN0_TOERIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_TOERIE_POS)) /**< INTEN0_TOERIE Mask */

#define MXC_F_I2C_INTEN0_ADRERIE_POS                   10 /**< INTEN0_ADRERIE Position */
#define MXC_F_I2C_INTEN0_ADRERIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_ADRERIE_POS)) /**< INTEN0_ADRERIE Mask */

#define MXC_F_I2C_INTEN0_DATERIE_POS                   11 /**< INTEN0_DATERIE Position */
#define MXC_F_I2C_INTEN0_DATERIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_DATERIE_POS)) /**< INTEN0_DATERIE Mask */

#define MXC_F_I2C_INTEN0_DNRERIE_POS                   12 /**< INTEN0_DNRERIE Position */
#define MXC_F_I2C_INTEN0_DNRERIE                       ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_DNRERIE_POS)) /**< INTEN0_DNRERIE Mask */

#define MXC_F_I2C_INTEN0_STRTERIE_POS                  13 /**< INTEN0_STRTERIE Position */
#define MXC_F_I2C_INTEN0_STRTERIE                      ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_STRTERIE_POS)) /**< INTEN0_STRTERIE Mask */

#define MXC_F_I2C_INTEN0_STOPERIE_POS                  14 /**< INTEN0_STOPERIE Position */
#define MXC_F_I2C_INTEN0_STOPERIE                      ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_STOPERIE_POS)) /**< INTEN0_STOPERIE Mask */

#define MXC_F_I2C_INTEN0_TXLOIE_POS                    15 /**< INTEN0_TXLOIE Position */
#define MXC_F_I2C_INTEN0_TXLOIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN0_TXLOIE_POS)) /**< INTEN0_TXLOIE Mask */

/**@} end of group I2C_INTEN0_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INT1 I2C_INT1
 * @brief    Interrupt Status Register 1.
 * @{
 */
#define MXC_F_I2C_INT1_RXOFI_POS                       0 /**< INT1_RXOFI Position */
#define MXC_F_I2C_INT1_RXOFI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT1_RXOFI_POS)) /**< INT1_RXOFI Mask */

#define MXC_F_I2C_INT1_TXUFI_POS                       1 /**< INT1_TXUFI Position */
#define MXC_F_I2C_INT1_TXUFI                           ((uint32_t)(0x1UL << MXC_F_I2C_INT1_TXUFI_POS)) /**< INT1_TXUFI Mask */

/**@} end of group I2C_INT1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_INTEN1 I2C_INTEN1
 * @brief    Interrupt Staus Register 1.
 * @{
 */
#define MXC_F_I2C_INTEN1_RXOFIE_POS                    0 /**< INTEN1_RXOFIE Position */
#define MXC_F_I2C_INTEN1_RXOFIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN1_RXOFIE_POS)) /**< INTEN1_RXOFIE Mask */

#define MXC_F_I2C_INTEN1_TXUFIE_POS                    1 /**< INTEN1_TXUFIE Position */
#define MXC_F_I2C_INTEN1_TXUFIE                        ((uint32_t)(0x1UL << MXC_F_I2C_INTEN1_TXUFIE_POS)) /**< INTEN1_TXUFIE Mask */

/**@} end of group I2C_INTEN1_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_FIFO I2C_FIFO
 * @brief    FIFO Configuration Register.
 * @{
 */
#define MXC_F_I2C_FIFO_RXLEN_POS                       0 /**< FIFO_RXLEN Position */
#define MXC_F_I2C_FIFO_RXLEN                           ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_RXLEN_POS)) /**< FIFO_RXLEN Mask */

#define MXC_F_I2C_FIFO_TXLEN_POS                       8 /**< FIFO_TXLEN Position */
#define MXC_F_I2C_FIFO_TXLEN                           ((uint32_t)(0xFFUL << MXC_F_I2C_FIFO_TXLEN_POS)) /**< FIFO_TXLEN Mask */

/**@} end of group I2C_FIFO_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RXCFG I2C_RXCFG
 * @brief    Receive Control Register 0.
 * @{
 */
#define MXC_F_I2C_RXCFG_DNR_POS                        0 /**< RXCFG_DNR Position */
#define MXC_F_I2C_RXCFG_DNR                            ((uint32_t)(0x1UL << MXC_F_I2C_RXCFG_DNR_POS)) /**< RXCFG_DNR Mask */

#define MXC_F_I2C_RXCFG_RXFSH_POS                      7 /**< RXCFG_RXFSH Position */
#define MXC_F_I2C_RXCFG_RXFSH                          ((uint32_t)(0x1UL << MXC_F_I2C_RXCFG_RXFSH_POS)) /**< RXCFG_RXFSH Mask */

#define MXC_F_I2C_RXCFG_RXTH_POS                       8 /**< RXCFG_RXTH Position */
#define MXC_F_I2C_RXCFG_RXTH                           ((uint32_t)(0xFUL << MXC_F_I2C_RXCFG_RXTH_POS)) /**< RXCFG_RXTH Mask */

/**@} end of group I2C_RXCFG_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_RX I2C_RX
 * @brief    Receive Control Register 1.
 * @{
 */
#define MXC_F_I2C_RX_RXCNT_POS                         0 /**< RX_RXCNT Position */
#define MXC_F_I2C_RX_RXCNT                             ((uint32_t)(0xFFUL << MXC_F_I2C_RX_RXCNT_POS)) /**< RX_RXCNT Mask */

#define MXC_F_I2C_RX_RXFIFO_POS                        8 /**< RX_RXFIFO Position */
#define MXC_F_I2C_RX_RXFIFO                            ((uint32_t)(0xFUL << MXC_F_I2C_RX_RXFIFO_POS)) /**< RX_RXFIFO Mask */

/**@} end of group I2C_RX_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TXCFG I2C_TXCFG
 * @brief    Transmit Control Register 0.
 * @{
 */
#define MXC_F_I2C_TXCFG_TXPRELD_POS                    0 /**< TXCFG_TXPRELD Position */
#define MXC_F_I2C_TXCFG_TXPRELD                        ((uint32_t)(0x1UL << MXC_F_I2C_TXCFG_TXPRELD_POS)) /**< TXCFG_TXPRELD Mask */

#define MXC_F_I2C_TXCFG_TXRDYMMODE_POS                 1 /**< TXCFG_TXRDYMMODE Position */
#define MXC_F_I2C_TXCFG_TXRDYMMODE                     ((uint32_t)(0x1UL << MXC_F_I2C_TXCFG_TXRDYMMODE_POS)) /**< TXCFG_TXRDYMMODE Mask */

#define MXC_F_I2C_TXCFG_TXFSH_POS                      7 /**< TXCFG_TXFSH Position */
#define MXC_F_I2C_TXCFG_TXFSH                          ((uint32_t)(0x1UL << MXC_F_I2C_TXCFG_TXFSH_POS)) /**< TXCFG_TXFSH Mask */

#define MXC_F_I2C_TXCFG_TXTH_POS                       8 /**< TXCFG_TXTH Position */
#define MXC_F_I2C_TXCFG_TXTH                           ((uint32_t)(0xFUL << MXC_F_I2C_TXCFG_TXTH_POS)) /**< TXCFG_TXTH Mask */

/**@} end of group I2C_TXCFG_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TX I2C_TX
 * @brief    Transmit Control Register 1.
 * @{
 */
#define MXC_F_I2C_TX_TXRDY_POS                         0 /**< TX_TXRDY Position */
#define MXC_F_I2C_TX_TXRDY                             ((uint32_t)(0x1UL << MXC_F_I2C_TX_TXRDY_POS)) /**< TX_TXRDY Mask */

#define MXC_F_I2C_TX_TXLAST_POS                        1 /**< TX_TXLAST Position */
#define MXC_F_I2C_TX_TXLAST                            ((uint32_t)(0x1UL << MXC_F_I2C_TX_TXLAST_POS)) /**< TX_TXLAST Mask */

#define MXC_F_I2C_TX_TXFIFO_POS                        8 /**< TX_TXFIFO Position */
#define MXC_F_I2C_TX_TXFIFO                            ((uint32_t)(0xFUL << MXC_F_I2C_TX_TXFIFO_POS)) /**< TX_TXFIFO Mask */

/**@} end of group I2C_TX_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_DATA I2C_DATA
 * @brief    Data Register.
 * @{
 */
#define MXC_F_I2C_DATA_DATA_POS                        0 /**< DATA_DATA Position */
#define MXC_F_I2C_DATA_DATA                            ((uint32_t)(0xFFUL << MXC_F_I2C_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group I2C_DATA_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_MCN I2C_MCN
 * @brief    Master Control Register.
 * @{
 */
#define MXC_F_I2C_MCN_START_POS                        0 /**< MCN_START Position */
#define MXC_F_I2C_MCN_START                            ((uint32_t)(0x1UL << MXC_F_I2C_MCN_START_POS)) /**< MCN_START Mask */

#define MXC_F_I2C_MCN_RESTART_POS                      1 /**< MCN_RESTART Position */
#define MXC_F_I2C_MCN_RESTART                          ((uint32_t)(0x1UL << MXC_F_I2C_MCN_RESTART_POS)) /**< MCN_RESTART Mask */

#define MXC_F_I2C_MCN_STOP_POS                         2 /**< MCN_STOP Position */
#define MXC_F_I2C_MCN_STOP                             ((uint32_t)(0x1UL << MXC_F_I2C_MCN_STOP_POS)) /**< MCN_STOP Mask */

#define MXC_F_I2C_MCN_SEA_POS                          7 /**< MCN_SEA Position */
#define MXC_F_I2C_MCN_SEA                              ((uint32_t)(0x1UL << MXC_F_I2C_MCN_SEA_POS)) /**< MCN_SEA Mask */

/**@} end of group I2C_MCN_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CKL I2C_CKL
 * @brief    Clock Low Register.
 * @{
 */
#define MXC_F_I2C_CKL_CKL_POS                          0 /**< CKL_CKL Position */
#define MXC_F_I2C_CKL_CKL                              ((uint32_t)(0x1FFUL << MXC_F_I2C_CKL_CKL_POS)) /**< CKL_CKL Mask */

/**@} end of group I2C_CKL_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_CKH I2C_CKH
 * @brief    Clock high Register.
 * @{
 */
#define MXC_F_I2C_CKH_CKH_POS                          0 /**< CKH_CKH Position */
#define MXC_F_I2C_CKH_CKH                              ((uint32_t)(0x1FFUL << MXC_F_I2C_CKH_CKH_POS)) /**< CKH_CKH Mask */

/**@} end of group I2C_CKH_Register */

/**
 * @ingroup  i2c_registers
 * @defgroup I2C_TO I2C_TO
 * @brief    Timeout Register
 * @{
 */
#define MXC_F_I2C_TO_TO_POS                            0 /**< TO_TO Position */
#define MXC_F_I2C_TO_TO                                ((uint32_t)(0xFFFFUL << MXC_F_I2C_TO_TO_POS)) /**< TO_TO Mask */

/**@} end of group I2C_TO_Register */

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
 * @defgroup I2C_SLA I2C_SLA
 * @brief    Slave Address Register.
 * @{
 */
#define MXC_F_I2C_SLA_SLA_POS                          0 /**< SLA_SLA Position */
#define MXC_F_I2C_SLA_SLA                              ((uint32_t)(0x3FFUL << MXC_F_I2C_SLA_SLA_POS)) /**< SLA_SLA Mask */
#define MXC_V_I2C_SLA_SLA_DIS                          ((uint32_t)0x0UL) /**< SLA_SLA_DIS Value */
#define MXC_S_I2C_SLA_SLA_DIS                          (MXC_V_I2C_SLA_SLA_DIS << MXC_F_I2C_SLA_SLA_POS) /**< SLA_SLA_DIS Setting */
#define MXC_V_I2C_SLA_SLA_EN                           ((uint32_t)0x1UL) /**< SLA_SLA_EN Value */
#define MXC_S_I2C_SLA_SLA_EN                           (MXC_V_I2C_SLA_SLA_EN << MXC_F_I2C_SLA_SLA_POS) /**< SLA_SLA_EN Setting */

#define MXC_F_I2C_SLA_EA_POS                           15 /**< SLA_EA Position */
#define MXC_F_I2C_SLA_EA                               ((uint32_t)(0x1UL << MXC_F_I2C_SLA_EA_POS)) /**< SLA_EA Mask */

/**@} end of group I2C_SLA_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_I2C_REGS_H_

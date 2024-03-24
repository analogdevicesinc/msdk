/**
 * @file    can_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the CAN_REVA Peripheral Module.
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

#ifndef _CAN_REVA_REGS_H_
#define _CAN_REVA_REGS_H_

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
 * @ingroup     can_reva
 * @defgroup    can_reva_registers CAN_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the CAN REVA Peripheral Module.
 * @details Controller Area Network Registers
 */

/**
 * @ingroup can_reva_registers
 * Structure type to access the CAN_REVA Registers.
 */
typedef struct {
    __IO uint8_t  mode;                 /**< <tt>\b 0x0000:</tt> CAN REVA MODE Register */
    __IO uint8_t  cmd;                  /**< <tt>\b 0x0001:</tt> CAN REVA CMD Register */
    __I  uint8_t  stat;                 /**< <tt>\b 0x0002:</tt> CAN REVA STAT Register */
    __IO uint8_t  intfl;                /**< <tt>\b 0x0003:</tt> CAN REVA INTFL Register */
    __IO uint8_t  inten;                /**< <tt>\b 0x0004:</tt> CAN REVA INTEN Register */
    __IO uint8_t  rmc;                  /**< <tt>\b 0x0005:</tt> CAN REVA RMC Register */
    __IO uint8_t  bustim0;              /**< <tt>\b 0x0006:</tt> CAN REVA BUSTIM0 Register */
    __IO uint8_t  bustim1;              /**< <tt>\b 0x0007:</tt> CAN REVA BUSTIM1 Register */
  union{
    __IO uint32_t txfifo32;             /**< <tt>\b 0x0008:</tt> CAN REVA TXFIFO32 Register */
    __IO uint16_t txfifo16[2];          /**< <tt>\b 0x0008:</tt> CAN REVA TXFIFO16 Register */
    __IO uint8_t  txfifo8[4];           /**< <tt>\b 0x0008:</tt> CAN REVA TXFIFO8 Register */
  };
  union{
    __I  uint32_t rxfifo32;             /**< <tt>\b 0x000C:</tt> CAN REVA RXFIFO32 Register */
    __I  uint16_t rxfifo16[2];          /**< <tt>\b 0x000C:</tt> CAN REVA RXFIFO16 Register */
    __I  uint8_t  rxfifo8[4];           /**< <tt>\b 0x000C:</tt> CAN REVA RXFIFO8 Register */
  };
  union{
    __IO uint32_t acr32;                /**< <tt>\b 0x0010:</tt> CAN REVA ACR32 Register */
    __IO uint16_t acr16[2];             /**< <tt>\b 0x0010:</tt> CAN REVA ACR16 Register */
    __IO uint8_t  acr8[4];              /**< <tt>\b 0x0010:</tt> CAN REVA ACR8 Register */
  };
  union{
    __IO uint32_t amr32;                /**< <tt>\b 0x0014:</tt> CAN REVA AMR32 Register */
    __IO uint16_t amr16[2];             /**< <tt>\b 0x0014:</tt> CAN REVA AMR16 Register */
    __IO uint8_t  amr8[4];              /**< <tt>\b 0x0014:</tt> CAN REVA AMR8 Register */
  };
    __I  uint8_t  ecc;                  /**< <tt>\b 0x0018:</tt> CAN REVA ECC Register */
    __IO uint8_t  rxerr;                /**< <tt>\b 0x0019:</tt> CAN REVA RXERR Register */
    __IO uint8_t  txerr;                /**< <tt>\b 0x001A:</tt> CAN REVA TXERR Register */
    __I  uint8_t  alc;                  /**< <tt>\b 0x001B:</tt> CAN REVA ALC Register */
    __IO uint32_t nbt;                  /**< <tt>\b 0x001C:</tt> CAN REVA NBT Register */
    __IO uint32_t dbt_sspp;             /**< <tt>\b 0x0020:</tt> CAN REVA DBT_SSPP Register */
    __IO uint8_t  fdctrl;               /**< <tt>\b 0x0024:</tt> CAN REVA FDCTRL Register */
    __I  uint8_t  fdstat;               /**< <tt>\b 0x0025:</tt> CAN REVA FDSTAT Register */
    __I  uint8_t  dperr;                /**< <tt>\b 0x0026:</tt> CAN REVA DPERR Register */
    __I  uint8_t  aperr;                /**< <tt>\b 0x0027:</tt> CAN REVA APERR Register */
    __IO uint8_t  test;                 /**< <tt>\b 0x0028:</tt> CAN REVA TEST Register */
    __IO uint8_t  wupclkdiv;            /**< <tt>\b 0x0029:</tt> CAN REVA WUPCLKDIV Register */
    __IO uint16_t wupft;                /**< <tt>\b 0x002A:</tt> CAN REVA WUPFT Register */
    __IO uint32_t wupet;                /**< <tt>\b 0x002C:</tt> CAN REVA WUPET Register */
    __IO uint16_t rxdcnt;               /**< <tt>\b 0x0030:</tt> CAN REVA RXDCNT Register */
    __IO uint8_t  txscnt;               /**< <tt>\b 0x0032:</tt> CAN REVA TXSCNT Register */
    __IO uint8_t  txdecmp;              /**< <tt>\b 0x0033:</tt> CAN REVA TXDECMP Register */
    __IO uint8_t  eintfl;               /**< <tt>\b 0x0034:</tt> CAN REVA EINTFL Register */
    __IO uint8_t  einten;               /**< <tt>\b 0x0035:</tt> CAN REVA EINTEN Register */
    __IO uint16_t rxto;                 /**< <tt>\b 0x0036:</tt> CAN REVA RXTO Register */
} mxc_can_reva_regs_t;

/* Register offsets for module CAN REVA */
/**
 * @ingroup    can_reva_registers
 * @defgroup   CAN_REVA_Register_Offsets Register Offsets
 * @brief      CAN REVA Peripheral Register Offsets from the CAN REVA Base Peripheral Address.
 * @{
 */
 #define MXC_R_CAN_REVA_MODE                     ((uint32_t)0x00000000UL) /**< Offset from CAN REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_CAN_REVA_CMD                      ((uint32_t)0x00000001UL) /**< Offset from CAN REVA Base Address: <tt> 0x0001</tt> */ 
 #define MXC_R_CAN_REVA_STAT                     ((uint32_t)0x00000002UL) /**< Offset from CAN REVA Base Address: <tt> 0x0002</tt> */ 
 #define MXC_R_CAN_REVA_INTFL                    ((uint32_t)0x00000003UL) /**< Offset from CAN REVA Base Address: <tt> 0x0003</tt> */ 
 #define MXC_R_CAN_REVA_INTEN                    ((uint32_t)0x00000004UL) /**< Offset from CAN REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_CAN_REVA_RMC                      ((uint32_t)0x00000005UL) /**< Offset from CAN REVA Base Address: <tt> 0x0005</tt> */ 
 #define MXC_R_CAN_REVA_BUSTIM0                  ((uint32_t)0x00000006UL) /**< Offset from CAN REVA Base Address: <tt> 0x0006</tt> */ 
 #define MXC_R_CAN_REVA_BUSTIM1                  ((uint32_t)0x00000007UL) /**< Offset from CAN REVA Base Address: <tt> 0x0007</tt> */ 
 #define MXC_R_CAN_REVA_TXFIFO32                 ((uint32_t)0x00000008UL) /**< Offset from CAN REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_CAN_REVA_TXFIFO16                 ((uint32_t)0x00000008UL) /**< Offset from CAN REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_CAN_REVA_TXFIFO8                  ((uint32_t)0x00000008UL) /**< Offset from CAN REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_CAN_REVA_RXFIFO32                 ((uint32_t)0x0000000CUL) /**< Offset from CAN REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_CAN_REVA_RXFIFO16                 ((uint32_t)0x0000000CUL) /**< Offset from CAN REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_CAN_REVA_RXFIFO8                  ((uint32_t)0x0000000CUL) /**< Offset from CAN REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_CAN_REVA_ACR32                    ((uint32_t)0x00000010UL) /**< Offset from CAN REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_CAN_REVA_ACR16                    ((uint32_t)0x00000010UL) /**< Offset from CAN REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_CAN_REVA_ACR8                     ((uint32_t)0x00000010UL) /**< Offset from CAN REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_CAN_REVA_AMR32                    ((uint32_t)0x00000014UL) /**< Offset from CAN REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_CAN_REVA_AMR16                    ((uint32_t)0x00000014UL) /**< Offset from CAN REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_CAN_REVA_AMR8                     ((uint32_t)0x00000014UL) /**< Offset from CAN REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_CAN_REVA_ECC                      ((uint32_t)0x00000018UL) /**< Offset from CAN REVA Base Address: <tt> 0x0018</tt> */ 
 #define MXC_R_CAN_REVA_RXERR                    ((uint32_t)0x00000019UL) /**< Offset from CAN REVA Base Address: <tt> 0x0019</tt> */ 
 #define MXC_R_CAN_REVA_TXERR                    ((uint32_t)0x0000001AUL) /**< Offset from CAN REVA Base Address: <tt> 0x001A</tt> */ 
 #define MXC_R_CAN_REVA_ALC                      ((uint32_t)0x0000001BUL) /**< Offset from CAN REVA Base Address: <tt> 0x001B</tt> */ 
 #define MXC_R_CAN_REVA_NBT                      ((uint32_t)0x0000001CUL) /**< Offset from CAN REVA Base Address: <tt> 0x001C</tt> */ 
 #define MXC_R_CAN_REVA_DBT_SSPP                 ((uint32_t)0x00000020UL) /**< Offset from CAN REVA Base Address: <tt> 0x0020</tt> */ 
 #define MXC_R_CAN_REVA_FDCTRL                   ((uint32_t)0x00000024UL) /**< Offset from CAN REVA Base Address: <tt> 0x0024</tt> */ 
 #define MXC_R_CAN_REVA_FDSTAT                   ((uint32_t)0x00000025UL) /**< Offset from CAN REVA Base Address: <tt> 0x0025</tt> */ 
 #define MXC_R_CAN_REVA_DPERR                    ((uint32_t)0x00000026UL) /**< Offset from CAN REVA Base Address: <tt> 0x0026</tt> */ 
 #define MXC_R_CAN_REVA_APERR                    ((uint32_t)0x00000027UL) /**< Offset from CAN REVA Base Address: <tt> 0x0027</tt> */ 
 #define MXC_R_CAN_REVA_TEST                     ((uint32_t)0x00000028UL) /**< Offset from CAN REVA Base Address: <tt> 0x0028</tt> */ 
 #define MXC_R_CAN_REVA_WUPCLKDIV                ((uint32_t)0x00000029UL) /**< Offset from CAN REVA Base Address: <tt> 0x0029</tt> */ 
 #define MXC_R_CAN_REVA_WUPFT                    ((uint32_t)0x0000002AUL) /**< Offset from CAN REVA Base Address: <tt> 0x002A</tt> */ 
 #define MXC_R_CAN_REVA_WUPET                    ((uint32_t)0x0000002CUL) /**< Offset from CAN REVA Base Address: <tt> 0x002C</tt> */ 
 #define MXC_R_CAN_REVA_RXDCNT                   ((uint32_t)0x00000030UL) /**< Offset from CAN REVA Base Address: <tt> 0x0030</tt> */ 
 #define MXC_R_CAN_REVA_TXSCNT                   ((uint32_t)0x00000032UL) /**< Offset from CAN REVA Base Address: <tt> 0x0032</tt> */ 
 #define MXC_R_CAN_REVA_TXDECMP                  ((uint32_t)0x00000033UL) /**< Offset from CAN REVA Base Address: <tt> 0x0033</tt> */ 
 #define MXC_R_CAN_REVA_EINTFL                   ((uint32_t)0x00000034UL) /**< Offset from CAN REVA Base Address: <tt> 0x0034</tt> */ 
 #define MXC_R_CAN_REVA_EINTEN                   ((uint32_t)0x00000035UL) /**< Offset from CAN REVA Base Address: <tt> 0x0035</tt> */ 
 #define MXC_R_CAN_REVA_RXTO                     ((uint32_t)0x00000036UL) /**< Offset from CAN REVA Base Address: <tt> 0x0036</tt> */ 
/**@} end of group can_reva_registers */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_MODE CAN_REVA_MODE
 * @brief    Mode Register.
 * @{
 */
 #define MXC_F_CAN_REVA_MODE_AFM_POS                         0 /**< MODE_AFM Position */
 #define MXC_F_CAN_REVA_MODE_AFM                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_MODE_AFM_POS)) /**< MODE_AFM Mask */

 #define MXC_F_CAN_REVA_MODE_LOM_POS                         1 /**< MODE_LOM Position */
 #define MXC_F_CAN_REVA_MODE_LOM                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_MODE_LOM_POS)) /**< MODE_LOM Mask */

 #define MXC_F_CAN_REVA_MODE_RST_POS                         2 /**< MODE_RST Position */
 #define MXC_F_CAN_REVA_MODE_RST                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_MODE_RST_POS)) /**< MODE_RST Mask */

 #define MXC_F_CAN_REVA_MODE_RXTRIG_POS                      3 /**< MODE_RXTRIG Position */
 #define MXC_F_CAN_REVA_MODE_RXTRIG                          ((uint8_t)(0x7UL << MXC_F_CAN_REVA_MODE_RXTRIG_POS)) /**< MODE_RXTRIG Mask */
 #define MXC_V_CAN_REVA_MODE_RXTRIG_1W                       ((uint8_t)0x0UL) /**< MODE_RXTRIG_1W Value */
 #define MXC_S_CAN_REVA_MODE_RXTRIG_1W                       (MXC_V_CAN_REVA_MODE_RXTRIG_1W << MXC_F_CAN_REVA_MODE_RXTRIG_POS) /**< MODE_RXTRIG_1W Setting */
 #define MXC_V_CAN_REVA_MODE_RXTRIG_4W                       ((uint8_t)0x1UL) /**< MODE_RXTRIG_4W Value */
 #define MXC_S_CAN_REVA_MODE_RXTRIG_4W                       (MXC_V_CAN_REVA_MODE_RXTRIG_4W << MXC_F_CAN_REVA_MODE_RXTRIG_POS) /**< MODE_RXTRIG_4W Setting */
 #define MXC_V_CAN_REVA_MODE_RXTRIG_8W                       ((uint8_t)0x2UL) /**< MODE_RXTRIG_8W Value */
 #define MXC_S_CAN_REVA_MODE_RXTRIG_8W                       (MXC_V_CAN_REVA_MODE_RXTRIG_8W << MXC_F_CAN_REVA_MODE_RXTRIG_POS) /**< MODE_RXTRIG_8W Setting */
 #define MXC_V_CAN_REVA_MODE_RXTRIG_16W                      ((uint8_t)0x3UL) /**< MODE_RXTRIG_16W Value */
 #define MXC_S_CAN_REVA_MODE_RXTRIG_16W                      (MXC_V_CAN_REVA_MODE_RXTRIG_16W << MXC_F_CAN_REVA_MODE_RXTRIG_POS) /**< MODE_RXTRIG_16W Setting */
 #define MXC_V_CAN_REVA_MODE_RXTRIG_32W                      ((uint8_t)0x4UL) /**< MODE_RXTRIG_32W Value */
 #define MXC_S_CAN_REVA_MODE_RXTRIG_32W                      (MXC_V_CAN_REVA_MODE_RXTRIG_32W << MXC_F_CAN_REVA_MODE_RXTRIG_POS) /**< MODE_RXTRIG_32W Setting */
 #define MXC_V_CAN_REVA_MODE_RXTRIG_64W                      ((uint8_t)0x5UL) /**< MODE_RXTRIG_64W Value */
 #define MXC_S_CAN_REVA_MODE_RXTRIG_64W                      (MXC_V_CAN_REVA_MODE_RXTRIG_64W << MXC_F_CAN_REVA_MODE_RXTRIG_POS) /**< MODE_RXTRIG_64W Setting */

 #define MXC_F_CAN_REVA_MODE_DMA_POS                         6 /**< MODE_DMA Position */
 #define MXC_F_CAN_REVA_MODE_DMA                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_MODE_DMA_POS)) /**< MODE_DMA Mask */

 #define MXC_F_CAN_REVA_MODE_SLP_POS                         7 /**< MODE_SLP Position */
 #define MXC_F_CAN_REVA_MODE_SLP                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_MODE_SLP_POS)) /**< MODE_SLP Mask */

/**@} end of group CAN_REVA_MODE_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_CMD CAN_REVA_CMD
 * @brief    Command Register.
 * @{
 */
 #define MXC_F_CAN_REVA_CMD_ABORT_POS                        1 /**< CMD_ABORT Position */
 #define MXC_F_CAN_REVA_CMD_ABORT                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_CMD_ABORT_POS)) /**< CMD_ABORT Mask */

 #define MXC_F_CAN_REVA_CMD_TXREQ_POS                        2 /**< CMD_TXREQ Position */
 #define MXC_F_CAN_REVA_CMD_TXREQ                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_CMD_TXREQ_POS)) /**< CMD_TXREQ Mask */

/**@} end of group CAN_REVA_CMD_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_STAT CAN_REVA_STAT
 * @brief    Status Register.
 * @{
 */
 #define MXC_F_CAN_REVA_STAT_BUS_OFF_POS                     0 /**< STAT_BUS_OFF Position */
 #define MXC_F_CAN_REVA_STAT_BUS_OFF                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_BUS_OFF_POS)) /**< STAT_BUS_OFF Mask */

 #define MXC_F_CAN_REVA_STAT_ERR_POS                         1 /**< STAT_ERR Position */
 #define MXC_F_CAN_REVA_STAT_ERR                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_ERR_POS)) /**< STAT_ERR Mask */

 #define MXC_F_CAN_REVA_STAT_TX_POS                          2 /**< STAT_TX Position */
 #define MXC_F_CAN_REVA_STAT_TX                              ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_TX_POS)) /**< STAT_TX Mask */

 #define MXC_F_CAN_REVA_STAT_RX_POS                          3 /**< STAT_RX Position */
 #define MXC_F_CAN_REVA_STAT_RX                              ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_RX_POS)) /**< STAT_RX Mask */

 #define MXC_F_CAN_REVA_STAT_TXBUF_POS                       5 /**< STAT_TXBUF Position */
 #define MXC_F_CAN_REVA_STAT_TXBUF                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_TXBUF_POS)) /**< STAT_TXBUF Mask */

 #define MXC_F_CAN_REVA_STAT_DOR_POS                         6 /**< STAT_DOR Position */
 #define MXC_F_CAN_REVA_STAT_DOR                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_DOR_POS)) /**< STAT_DOR Mask */

 #define MXC_F_CAN_REVA_STAT_RXBUF_POS                       7 /**< STAT_RXBUF Position */
 #define MXC_F_CAN_REVA_STAT_RXBUF                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_STAT_RXBUF_POS)) /**< STAT_RXBUF Mask */

/**@} end of group CAN_REVA_STAT_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_INTFL CAN_REVA_INTFL
 * @brief    Interrupt Status Register.
 * @{
 */
 #define MXC_F_CAN_REVA_INTFL_DOR_POS                        0 /**< INTFL_DOR Position */
 #define MXC_F_CAN_REVA_INTFL_DOR                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_DOR_POS)) /**< INTFL_DOR Mask */

 #define MXC_F_CAN_REVA_INTFL_BERR_POS                       1 /**< INTFL_BERR Position */
 #define MXC_F_CAN_REVA_INTFL_BERR                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_BERR_POS)) /**< INTFL_BERR Mask */

 #define MXC_F_CAN_REVA_INTFL_TX_POS                         2 /**< INTFL_TX Position */
 #define MXC_F_CAN_REVA_INTFL_TX                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_TX_POS)) /**< INTFL_TX Mask */

 #define MXC_F_CAN_REVA_INTFL_RX_POS                         3 /**< INTFL_RX Position */
 #define MXC_F_CAN_REVA_INTFL_RX                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_RX_POS)) /**< INTFL_RX Mask */

 #define MXC_F_CAN_REVA_INTFL_ERPSV_POS                      4 /**< INTFL_ERPSV Position */
 #define MXC_F_CAN_REVA_INTFL_ERPSV                          ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_ERPSV_POS)) /**< INTFL_ERPSV Mask */

 #define MXC_F_CAN_REVA_INTFL_ERWARN_POS                     5 /**< INTFL_ERWARN Position */
 #define MXC_F_CAN_REVA_INTFL_ERWARN                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_ERWARN_POS)) /**< INTFL_ERWARN Mask */

 #define MXC_F_CAN_REVA_INTFL_AL_POS                         6 /**< INTFL_AL Position */
 #define MXC_F_CAN_REVA_INTFL_AL                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_AL_POS)) /**< INTFL_AL Mask */

 #define MXC_F_CAN_REVA_INTFL_WU_POS                         7 /**< INTFL_WU Position */
 #define MXC_F_CAN_REVA_INTFL_WU                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTFL_WU_POS)) /**< INTFL_WU Mask */

/**@} end of group CAN_REVA_INTFL_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_INTEN CAN_REVA_INTEN
 * @brief    Interrupt Enable Register.
 * @{
 */
 #define MXC_F_CAN_REVA_INTEN_DOR_POS                        0 /**< INTEN_DOR Position */
 #define MXC_F_CAN_REVA_INTEN_DOR                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_DOR_POS)) /**< INTEN_DOR Mask */

 #define MXC_F_CAN_REVA_INTEN_BERR_POS                       1 /**< INTEN_BERR Position */
 #define MXC_F_CAN_REVA_INTEN_BERR                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_BERR_POS)) /**< INTEN_BERR Mask */

 #define MXC_F_CAN_REVA_INTEN_TX_POS                         2 /**< INTEN_TX Position */
 #define MXC_F_CAN_REVA_INTEN_TX                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_TX_POS)) /**< INTEN_TX Mask */

 #define MXC_F_CAN_REVA_INTEN_RX_POS                         3 /**< INTEN_RX Position */
 #define MXC_F_CAN_REVA_INTEN_RX                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_RX_POS)) /**< INTEN_RX Mask */

 #define MXC_F_CAN_REVA_INTEN_ERPSV_POS                      4 /**< INTEN_ERPSV Position */
 #define MXC_F_CAN_REVA_INTEN_ERPSV                          ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_ERPSV_POS)) /**< INTEN_ERPSV Mask */

 #define MXC_F_CAN_REVA_INTEN_ERWARN_POS                     5 /**< INTEN_ERWARN Position */
 #define MXC_F_CAN_REVA_INTEN_ERWARN                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_ERWARN_POS)) /**< INTEN_ERWARN Mask */

 #define MXC_F_CAN_REVA_INTEN_AL_POS                         6 /**< INTEN_AL Position */
 #define MXC_F_CAN_REVA_INTEN_AL                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_AL_POS)) /**< INTEN_AL Mask */

 #define MXC_F_CAN_REVA_INTEN_WU_POS                         7 /**< INTEN_WU Position */
 #define MXC_F_CAN_REVA_INTEN_WU                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_INTEN_WU_POS)) /**< INTEN_WU Mask */

/**@} end of group CAN_REVA_INTEN_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RMC CAN_REVA_RMC
 * @brief    Receive Message Counter Register.
 * @{
 */
 #define MXC_F_CAN_REVA_RMC_NUM_MSGS_POS                     0 /**< RMC_NUM_MSGS Position */
 #define MXC_F_CAN_REVA_RMC_NUM_MSGS                         ((uint8_t)(0x1FUL << MXC_F_CAN_REVA_RMC_NUM_MSGS_POS)) /**< RMC_NUM_MSGS Mask */

/**@} end of group CAN_REVA_RMC_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_BUSTIM0 CAN_REVA_BUSTIM0
 * @brief    Bus Timing Register 0.
 * @{
 */
 #define MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV_POS                0 /**< BUSTIM0_BR_CLKDIV Position */
 #define MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV                    ((uint8_t)(0x3FUL << MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV_POS)) /**< BUSTIM0_BR_CLKDIV Mask */

 #define MXC_F_CAN_REVA_BUSTIM0_SJW_POS                      6 /**< BUSTIM0_SJW Position */
 #define MXC_F_CAN_REVA_BUSTIM0_SJW                          ((uint8_t)(0x3UL << MXC_F_CAN_REVA_BUSTIM0_SJW_POS)) /**< BUSTIM0_SJW Mask */

/**@} end of group CAN_REVA_BUSTIM0_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_BUSTIM1 CAN_REVA_BUSTIM1
 * @brief    Bus Timing Register 1.
 * @{
 */
 #define MXC_F_CAN_REVA_BUSTIM1_TSEG1_POS                    0 /**< BUSTIM1_TSEG1 Position */
 #define MXC_F_CAN_REVA_BUSTIM1_TSEG1                        ((uint8_t)(0xFUL << MXC_F_CAN_REVA_BUSTIM1_TSEG1_POS)) /**< BUSTIM1_TSEG1 Mask */

 #define MXC_F_CAN_REVA_BUSTIM1_TSEG2_POS                    4 /**< BUSTIM1_TSEG2 Position */
 #define MXC_F_CAN_REVA_BUSTIM1_TSEG2                        ((uint8_t)(0x7UL << MXC_F_CAN_REVA_BUSTIM1_TSEG2_POS)) /**< BUSTIM1_TSEG2 Mask */

 #define MXC_F_CAN_REVA_BUSTIM1_SAM_POS                      7 /**< BUSTIM1_SAM Position */
 #define MXC_F_CAN_REVA_BUSTIM1_SAM                          ((uint8_t)(0x1UL << MXC_F_CAN_REVA_BUSTIM1_SAM_POS)) /**< BUSTIM1_SAM Mask */

/**@} end of group CAN_REVA_BUSTIM1_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TXFIFO32 CAN_REVA_TXFIFO32
 * @brief    Transmit FIFO Register.
 * @{
 */
 #define MXC_F_CAN_REVA_TXFIFO32_DATA_POS                    0 /**< TXFIFO32_DATA Position */
 #define MXC_F_CAN_REVA_TXFIFO32_DATA                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_CAN_REVA_TXFIFO32_DATA_POS)) /**< TXFIFO32_DATA Mask */

/**@} end of group CAN_REVA_TXFIFO32_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TXFIFO16 CAN_REVA_TXFIFO16
 * @brief    Transmit FIFO Register.
 * @{
 */
 #define MXC_F_CAN_REVA_TXFIFO16_DATA_POS                    0 /**< TXFIFO16_DATA Position */
 #define MXC_F_CAN_REVA_TXFIFO16_DATA                        ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_TXFIFO16_DATA_POS)) /**< TXFIFO16_DATA Mask */

/**@} end of group CAN_REVA_TXFIFO16_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TXFIFO8 CAN_REVA_TXFIFO8
 * @brief    Transmit FIFO Register.
 * @{
 */
 #define MXC_F_CAN_REVA_TXFIFO8_DATA_POS                     0 /**< TXFIFO8_DATA Position */
 #define MXC_F_CAN_REVA_TXFIFO8_DATA                         ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_TXFIFO8_DATA_POS)) /**< TXFIFO8_DATA Mask */

/**@} end of group CAN_REVA_TXFIFO8_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RXFIFO32 CAN_REVA_RXFIFO32
 * @brief    Receive FIFO Register.
 * @{
 */
 #define MXC_F_CAN_REVA_RXFIFO32_DATA_POS                    0 /**< RXFIFO32_DATA Position */
 #define MXC_F_CAN_REVA_RXFIFO32_DATA                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_CAN_REVA_RXFIFO32_DATA_POS)) /**< RXFIFO32_DATA Mask */

/**@} end of group CAN_REVA_RXFIFO32_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RXFIFO16 CAN_REVA_RXFIFO16
 * @brief    Receive FIFO Register.
 * @{
 */
 #define MXC_F_CAN_REVA_RXFIFO16_DATA_POS                    0 /**< RXFIFO16_DATA Position */
 #define MXC_F_CAN_REVA_RXFIFO16_DATA                        ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_RXFIFO16_DATA_POS)) /**< RXFIFO16_DATA Mask */

/**@} end of group CAN_REVA_RXFIFO16_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RXFIFO8 CAN_REVA_RXFIFO8
 * @brief    Receive FIFO Register.
 * @{
 */
 #define MXC_F_CAN_REVA_RXFIFO8_DATA_POS                     0 /**< RXFIFO8_DATA Position */
 #define MXC_F_CAN_REVA_RXFIFO8_DATA                         ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_RXFIFO8_DATA_POS)) /**< RXFIFO8_DATA Mask */

/**@} end of group CAN_REVA_RXFIFO8_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_ACR32 CAN_REVA_ACR32
 * @brief    Acceptance Code Register.
 * @{
 */
 #define MXC_F_CAN_REVA_ACR32_ACR_POS                        0 /**< ACR32_ACR Position */
 #define MXC_F_CAN_REVA_ACR32_ACR                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_CAN_REVA_ACR32_ACR_POS)) /**< ACR32_ACR Mask */

/**@} end of group CAN_REVA_ACR32_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_ACR16 CAN_REVA_ACR16
 * @brief    Acceptance Code Register.
 * @{
 */
 #define MXC_F_CAN_REVA_ACR16_ACR_POS                        0 /**< ACR16_ACR Position */
 #define MXC_F_CAN_REVA_ACR16_ACR                            ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_ACR16_ACR_POS)) /**< ACR16_ACR Mask */

/**@} end of group CAN_REVA_ACR16_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_ACR8 CAN_REVA_ACR8
 * @brief    Acceptance Code Register.
 * @{
 */
 #define MXC_F_CAN_REVA_ACR8_ACR_POS                         0 /**< ACR8_ACR Position */
 #define MXC_F_CAN_REVA_ACR8_ACR                             ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_ACR8_ACR_POS)) /**< ACR8_ACR Mask */

/**@} end of group CAN_REVA_ACR8_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_AMR32 CAN_REVA_AMR32
 * @brief    Acceptance Mask Register.
 * @{
 */
 #define MXC_F_CAN_REVA_AMR32_AMR_POS                        0 /**< AMR32_AMR Position */
 #define MXC_F_CAN_REVA_AMR32_AMR                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_CAN_REVA_AMR32_AMR_POS)) /**< AMR32_AMR Mask */

/**@} end of group CAN_REVA_AMR32_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_AMR16 CAN_REVA_AMR16
 * @brief    Acceptance Mask Register.
 * @{
 */
 #define MXC_F_CAN_REVA_AMR16_AMR_POS                        0 /**< AMR16_AMR Position */
 #define MXC_F_CAN_REVA_AMR16_AMR                            ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_AMR16_AMR_POS)) /**< AMR16_AMR Mask */

/**@} end of group CAN_REVA_AMR16_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_AMR8 CAN_REVA_AMR8
 * @brief    Acceptance Mask Register.
 * @{
 */
 #define MXC_F_CAN_REVA_AMR8_AMR_POS                         0 /**< AMR8_AMR Position */
 #define MXC_F_CAN_REVA_AMR8_AMR                             ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_AMR8_AMR_POS)) /**< AMR8_AMR Mask */

/**@} end of group CAN_REVA_AMR8_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_ECC CAN_REVA_ECC
 * @brief    Error Code Capture Register.
 * @{
 */
 #define MXC_F_CAN_REVA_ECC_BER_POS                          0 /**< ECC_BER Position */
 #define MXC_F_CAN_REVA_ECC_BER                              ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_BER_POS)) /**< ECC_BER Mask */

 #define MXC_F_CAN_REVA_ECC_STFER_POS                        1 /**< ECC_STFER Position */
 #define MXC_F_CAN_REVA_ECC_STFER                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_STFER_POS)) /**< ECC_STFER Mask */

 #define MXC_F_CAN_REVA_ECC_CRCER_POS                        2 /**< ECC_CRCER Position */
 #define MXC_F_CAN_REVA_ECC_CRCER                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_CRCER_POS)) /**< ECC_CRCER Mask */

 #define MXC_F_CAN_REVA_ECC_FRMER_POS                        3 /**< ECC_FRMER Position */
 #define MXC_F_CAN_REVA_ECC_FRMER                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_FRMER_POS)) /**< ECC_FRMER Mask */

 #define MXC_F_CAN_REVA_ECC_ACKER_POS                        4 /**< ECC_ACKER Position */
 #define MXC_F_CAN_REVA_ECC_ACKER                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_ACKER_POS)) /**< ECC_ACKER Mask */

 #define MXC_F_CAN_REVA_ECC_EDIR_POS                         5 /**< ECC_EDIR Position */
 #define MXC_F_CAN_REVA_ECC_EDIR                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_EDIR_POS)) /**< ECC_EDIR Mask */

 #define MXC_F_CAN_REVA_ECC_TXWRN_POS                        6 /**< ECC_TXWRN Position */
 #define MXC_F_CAN_REVA_ECC_TXWRN                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_TXWRN_POS)) /**< ECC_TXWRN Mask */

 #define MXC_F_CAN_REVA_ECC_RXWRN_POS                        7 /**< ECC_RXWRN Position */
 #define MXC_F_CAN_REVA_ECC_RXWRN                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_ECC_RXWRN_POS)) /**< ECC_RXWRN Mask */

/**@} end of group CAN_REVA_ECC_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RXERR CAN_REVA_RXERR
 * @brief    Receive Error Counter.
 * @{
 */
 #define MXC_F_CAN_REVA_RXERR_RXERR_POS                      0 /**< RXERR_RXERR Position */
 #define MXC_F_CAN_REVA_RXERR_RXERR                          ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_RXERR_RXERR_POS)) /**< RXERR_RXERR Mask */

/**@} end of group CAN_REVA_RXERR_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TXERR CAN_REVA_TXERR
 * @brief    Invalidate All Registers.
 * @{
 */
 #define MXC_F_CAN_REVA_TXERR_TXERR_POS                      0 /**< TXERR_TXERR Position */
 #define MXC_F_CAN_REVA_TXERR_TXERR                          ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_TXERR_TXERR_POS)) /**< TXERR_TXERR Mask */

/**@} end of group CAN_REVA_TXERR_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_ALC CAN_REVA_ALC
 * @brief    Arbitration Lost Code Capture Register.
 * @{
 */
 #define MXC_F_CAN_REVA_ALC_ALC_POS                          0 /**< ALC_ALC Position */
 #define MXC_F_CAN_REVA_ALC_ALC                              ((uint8_t)(0x1FUL << MXC_F_CAN_REVA_ALC_ALC_POS)) /**< ALC_ALC Mask */

/**@} end of group CAN_REVA_ALC_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_NBT CAN_REVA_NBT
 * @brief    Nominal Bit Timing Register.
 * @{
 */
 #define MXC_F_CAN_REVA_NBT_NBRP_POS                         0 /**< NBT_NBRP Position */
 #define MXC_F_CAN_REVA_NBT_NBRP                             ((uint32_t)(0x3FFUL << MXC_F_CAN_REVA_NBT_NBRP_POS)) /**< NBT_NBRP Mask */

 #define MXC_F_CAN_REVA_NBT_NSEG1_POS                        10 /**< NBT_NSEG1 Position */
 #define MXC_F_CAN_REVA_NBT_NSEG1                            ((uint32_t)(0xFFUL << MXC_F_CAN_REVA_NBT_NSEG1_POS)) /**< NBT_NSEG1 Mask */

 #define MXC_F_CAN_REVA_NBT_NSEG2_POS                        18 /**< NBT_NSEG2 Position */
 #define MXC_F_CAN_REVA_NBT_NSEG2                            ((uint32_t)(0x7FUL << MXC_F_CAN_REVA_NBT_NSEG2_POS)) /**< NBT_NSEG2 Mask */

 #define MXC_F_CAN_REVA_NBT_NSJW_POS                         25 /**< NBT_NSJW Position */
 #define MXC_F_CAN_REVA_NBT_NSJW                             ((uint32_t)(0x7FUL << MXC_F_CAN_REVA_NBT_NSJW_POS)) /**< NBT_NSJW Mask */

/**@} end of group CAN_REVA_NBT_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_DBT_SSPP CAN_REVA_DBT_SSPP
 * @brief    Data Bit Timing Register.
 * @{
 */
 #define MXC_F_CAN_REVA_DBT_SSPP_DBRP_POS                    0 /**< DBT_SSPP_DBRP Position */
 #define MXC_F_CAN_REVA_DBT_SSPP_DBRP                        ((uint32_t)(0x3FFUL << MXC_F_CAN_REVA_DBT_SSPP_DBRP_POS)) /**< DBT_SSPP_DBRP Mask */

 #define MXC_F_CAN_REVA_DBT_SSPP_DSEG1_POS                   10 /**< DBT_SSPP_DSEG1 Position */
 #define MXC_F_CAN_REVA_DBT_SSPP_DSEG1                       ((uint32_t)(0x3FUL << MXC_F_CAN_REVA_DBT_SSPP_DSEG1_POS)) /**< DBT_SSPP_DSEG1 Mask */

 #define MXC_F_CAN_REVA_DBT_SSPP_DSEG2_POS                   16 /**< DBT_SSPP_DSEG2 Position */
 #define MXC_F_CAN_REVA_DBT_SSPP_DSEG2                       ((uint32_t)(0xFUL << MXC_F_CAN_REVA_DBT_SSPP_DSEG2_POS)) /**< DBT_SSPP_DSEG2 Mask */

 #define MXC_F_CAN_REVA_DBT_SSPP_DSJW_POS                    20 /**< DBT_SSPP_DSJW Position */
 #define MXC_F_CAN_REVA_DBT_SSPP_DSJW                        ((uint32_t)(0xFUL << MXC_F_CAN_REVA_DBT_SSPP_DSJW_POS)) /**< DBT_SSPP_DSJW Mask */

 #define MXC_F_CAN_REVA_DBT_SSPP_SSPP_POS                    24 /**< DBT_SSPP_SSPP Position */
 #define MXC_F_CAN_REVA_DBT_SSPP_SSPP                        ((uint32_t)(0x7FUL << MXC_F_CAN_REVA_DBT_SSPP_SSPP_POS)) /**< DBT_SSPP_SSPP Mask */

/**@} end of group CAN_REVA_DBT_SSPP_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_FDCTRL CAN_REVA_FDCTRL
 * @brief    FD Control Register.
 * @{
 */
 #define MXC_F_CAN_REVA_FDCTRL_FDEN_POS                      0 /**< FDCTRL_FDEN Position */
 #define MXC_F_CAN_REVA_FDCTRL_FDEN                          ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_FDEN_POS)) /**< FDCTRL_FDEN Mask */

 #define MXC_F_CAN_REVA_FDCTRL_BRSEN_POS                     1 /**< FDCTRL_BRSEN Position */
 #define MXC_F_CAN_REVA_FDCTRL_BRSEN                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_BRSEN_POS)) /**< FDCTRL_BRSEN Mask */

 #define MXC_F_CAN_REVA_FDCTRL_EXTBT_POS                     2 /**< FDCTRL_EXTBT Position */
 #define MXC_F_CAN_REVA_FDCTRL_EXTBT                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_EXTBT_POS)) /**< FDCTRL_EXTBT Mask */

 #define MXC_F_CAN_REVA_FDCTRL_ISO_POS                       3 /**< FDCTRL_ISO Position */
 #define MXC_F_CAN_REVA_FDCTRL_ISO                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_ISO_POS)) /**< FDCTRL_ISO Mask */

 #define MXC_F_CAN_REVA_FDCTRL_DAR_POS                       4 /**< FDCTRL_DAR Position */
 #define MXC_F_CAN_REVA_FDCTRL_DAR                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_DAR_POS)) /**< FDCTRL_DAR Mask */

 #define MXC_F_CAN_REVA_FDCTRL_REOM_POS                      5 /**< FDCTRL_REOM Position */
 #define MXC_F_CAN_REVA_FDCTRL_REOM                          ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_REOM_POS)) /**< FDCTRL_REOM Mask */

 #define MXC_F_CAN_REVA_FDCTRL_PED_POS                       6 /**< FDCTRL_PED Position */
 #define MXC_F_CAN_REVA_FDCTRL_PED                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDCTRL_PED_POS)) /**< FDCTRL_PED Mask */

/**@} end of group CAN_REVA_FDCTRL_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_FDSTAT CAN_REVA_FDSTAT
 * @brief    Invalidate All Registers.
 * @{
 */
 #define MXC_F_CAN_REVA_FDSTAT_BITERR_POS                    0 /**< FDSTAT_BITERR Position */
 #define MXC_F_CAN_REVA_FDSTAT_BITERR                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDSTAT_BITERR_POS)) /**< FDSTAT_BITERR Mask */

 #define MXC_F_CAN_REVA_FDSTAT_CRCERR_POS                    1 /**< FDSTAT_CRCERR Position */
 #define MXC_F_CAN_REVA_FDSTAT_CRCERR                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDSTAT_CRCERR_POS)) /**< FDSTAT_CRCERR Mask */

 #define MXC_F_CAN_REVA_FDSTAT_FRMERR_POS                    2 /**< FDSTAT_FRMERR Position */
 #define MXC_F_CAN_REVA_FDSTAT_FRMERR                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDSTAT_FRMERR_POS)) /**< FDSTAT_FRMERR Mask */

 #define MXC_F_CAN_REVA_FDSTAT_STFERR_POS                    3 /**< FDSTAT_STFERR Position */
 #define MXC_F_CAN_REVA_FDSTAT_STFERR                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDSTAT_STFERR_POS)) /**< FDSTAT_STFERR Mask */

 #define MXC_F_CAN_REVA_FDSTAT_PEE_POS                       4 /**< FDSTAT_PEE Position */
 #define MXC_F_CAN_REVA_FDSTAT_PEE                           ((uint8_t)(0x1UL << MXC_F_CAN_REVA_FDSTAT_PEE_POS)) /**< FDSTAT_PEE Mask */

 #define MXC_F_CAN_REVA_FDSTAT_STATE_POS                     6 /**< FDSTAT_STATE Position */
 #define MXC_F_CAN_REVA_FDSTAT_STATE                         ((uint8_t)(0x3UL << MXC_F_CAN_REVA_FDSTAT_STATE_POS)) /**< FDSTAT_STATE Mask */
 #define MXC_V_CAN_REVA_FDSTAT_STATE_INT                     ((uint8_t)0x0UL) /**< FDSTAT_STATE_INT Value */
 #define MXC_S_CAN_REVA_FDSTAT_STATE_INT                     (MXC_V_CAN_REVA_FDSTAT_STATE_INT << MXC_F_CAN_REVA_FDSTAT_STATE_POS) /**< FDSTAT_STATE_INT Setting */
 #define MXC_V_CAN_REVA_FDSTAT_STATE_IDLE                    ((uint8_t)0x1UL) /**< FDSTAT_STATE_IDLE Value */
 #define MXC_S_CAN_REVA_FDSTAT_STATE_IDLE                    (MXC_V_CAN_REVA_FDSTAT_STATE_IDLE << MXC_F_CAN_REVA_FDSTAT_STATE_POS) /**< FDSTAT_STATE_IDLE Setting */
 #define MXC_V_CAN_REVA_FDSTAT_STATE_RX                      ((uint8_t)0x2UL) /**< FDSTAT_STATE_RX Value */
 #define MXC_S_CAN_REVA_FDSTAT_STATE_RX                      (MXC_V_CAN_REVA_FDSTAT_STATE_RX << MXC_F_CAN_REVA_FDSTAT_STATE_POS) /**< FDSTAT_STATE_RX Setting */
 #define MXC_V_CAN_REVA_FDSTAT_STATE_TX                      ((uint8_t)0x3UL) /**< FDSTAT_STATE_TX Value */
 #define MXC_S_CAN_REVA_FDSTAT_STATE_TX                      (MXC_V_CAN_REVA_FDSTAT_STATE_TX << MXC_F_CAN_REVA_FDSTAT_STATE_POS) /**< FDSTAT_STATE_TX Setting */

/**@} end of group CAN_REVA_FDSTAT_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_DPERR CAN_REVA_DPERR
 * @brief    Data Phase Error Counter Register.
 * @{
 */
 #define MXC_F_CAN_REVA_DPERR_DPERR_POS                      0 /**< DPERR_DPERR Position */
 #define MXC_F_CAN_REVA_DPERR_DPERR                          ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_DPERR_DPERR_POS)) /**< DPERR_DPERR Mask */

/**@} end of group CAN_REVA_DPERR_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_APERR CAN_REVA_APERR
 * @brief    Arbitration Phase Error Counter Register.
 * @{
 */
 #define MXC_F_CAN_REVA_APERR_APERR_POS                      0 /**< APERR_APERR Position */
 #define MXC_F_CAN_REVA_APERR_APERR                          ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_APERR_APERR_POS)) /**< APERR_APERR Mask */

/**@} end of group CAN_REVA_APERR_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TEST CAN_REVA_TEST
 * @brief    Invalidate All Registers.
 * @{
 */
 #define MXC_F_CAN_REVA_TEST_LBEN_POS                        0 /**< TEST_LBEN Position */
 #define MXC_F_CAN_REVA_TEST_LBEN                            ((uint8_t)(0x1UL << MXC_F_CAN_REVA_TEST_LBEN_POS)) /**< TEST_LBEN Mask */

 #define MXC_F_CAN_REVA_TEST_TXC_POS                         1 /**< TEST_TXC Position */
 #define MXC_F_CAN_REVA_TEST_TXC                             ((uint8_t)(0x1UL << MXC_F_CAN_REVA_TEST_TXC_POS)) /**< TEST_TXC Mask */

/**@} end of group CAN_REVA_TEST_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_WUPCLKDIV CAN_REVA_WUPCLKDIV
 * @brief    Wake-up timer prescaler.
 * @{
 */
 #define MXC_F_CAN_REVA_WUPCLKDIV_WUPDIV_POS                 0 /**< WUPCLKDIV_WUPDIV Position */
 #define MXC_F_CAN_REVA_WUPCLKDIV_WUPDIV                     ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_WUPCLKDIV_WUPDIV_POS)) /**< WUPCLKDIV_WUPDIV Mask */

/**@} end of group CAN_REVA_WUPCLKDIV_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_WUPFT CAN_REVA_WUPFT
 * @brief    Wake up Filter Time Register.
 * @{
 */
 #define MXC_F_CAN_REVA_WUPFT_WUPFT_POS                      0 /**< WUPFT_WUPFT Position */
 #define MXC_F_CAN_REVA_WUPFT_WUPFT                          ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_WUPFT_WUPFT_POS)) /**< WUPFT_WUPFT Mask */

/**@} end of group CAN_REVA_WUPFT_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_WUPET CAN_REVA_WUPET
 * @brief    Wake-up Expire Time Register.
 * @{
 */
 #define MXC_F_CAN_REVA_WUPET_WUPET_POS                      0 /**< WUPET_WUPET Position */
 #define MXC_F_CAN_REVA_WUPET_WUPET                          ((uint32_t)(0xFFFFFUL << MXC_F_CAN_REVA_WUPET_WUPET_POS)) /**< WUPET_WUPET Mask */

/**@} end of group CAN_REVA_WUPET_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RXDCNT CAN_REVA_RXDCNT
 * @brief    RX FIFO Data Counter Register.
 * @{
 */
 #define MXC_F_CAN_REVA_RXDCNT_RXDCNT_POS                    0 /**< RXDCNT_RXDCNT Position */
 #define MXC_F_CAN_REVA_RXDCNT_RXDCNT                        ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_RXDCNT_RXDCNT_POS)) /**< RXDCNT_RXDCNT Mask */

/**@} end of group CAN_REVA_RXDCNT_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TXSCNT CAN_REVA_TXSCNT
 * @brief    TX FIFO Space Counter.
 * @{
 */
 #define MXC_F_CAN_REVA_TXSCNT_TXSCNT_POS                    0 /**< TXSCNT_TXSCNT Position */
 #define MXC_F_CAN_REVA_TXSCNT_TXSCNT                        ((uint8_t)(0xFFUL << MXC_F_CAN_REVA_TXSCNT_TXSCNT_POS)) /**< TXSCNT_TXSCNT Mask */

/**@} end of group CAN_REVA_TXSCNT_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_TXDECMP CAN_REVA_TXDECMP
 * @brief    Invalidate All Registers.
 * @{
 */
 #define MXC_F_CAN_REVA_TXDECMP_TDCO_POS                     0 /**< TXDECMP_TDCO Position */
 #define MXC_F_CAN_REVA_TXDECMP_TDCO                         ((uint8_t)(0x7FUL << MXC_F_CAN_REVA_TXDECMP_TDCO_POS)) /**< TXDECMP_TDCO Mask */

 #define MXC_F_CAN_REVA_TXDECMP_TDCEN_POS                    7 /**< TXDECMP_TDCEN Position */
 #define MXC_F_CAN_REVA_TXDECMP_TDCEN                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_TXDECMP_TDCEN_POS)) /**< TXDECMP_TDCEN Mask */

/**@} end of group CAN_REVA_TXDECMP_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_EINTFL CAN_REVA_EINTFL
 * @brief    Extended Interrupt Flag Register.
 * @{
 */
 #define MXC_F_CAN_REVA_EINTFL_RX_THD_POS                    0 /**< EINTFL_RX_THD Position */
 #define MXC_F_CAN_REVA_EINTFL_RX_THD                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_EINTFL_RX_THD_POS)) /**< EINTFL_RX_THD Mask */

 #define MXC_F_CAN_REVA_EINTFL_RX_TO_POS                     1 /**< EINTFL_RX_TO Position */
 #define MXC_F_CAN_REVA_EINTFL_RX_TO                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_EINTFL_RX_TO_POS)) /**< EINTFL_RX_TO Mask */

/**@} end of group CAN_REVA_EINTFL_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_EINTEN CAN_REVA_EINTEN
 * @brief    Extended Interrupt Enable Register.
 * @{
 */
 #define MXC_F_CAN_REVA_EINTEN_RX_THD_POS                    0 /**< EINTEN_RX_THD Position */
 #define MXC_F_CAN_REVA_EINTEN_RX_THD                        ((uint8_t)(0x1UL << MXC_F_CAN_REVA_EINTEN_RX_THD_POS)) /**< EINTEN_RX_THD Mask */

 #define MXC_F_CAN_REVA_EINTEN_RX_TO_POS                     1 /**< EINTEN_RX_TO Position */
 #define MXC_F_CAN_REVA_EINTEN_RX_TO                         ((uint8_t)(0x1UL << MXC_F_CAN_REVA_EINTEN_RX_TO_POS)) /**< EINTEN_RX_TO Mask */

/**@} end of group CAN_REVA_EINTEN_Register */

/**
 * @ingroup  can_reva_registers
 * @defgroup CAN_REVA_RXTO CAN_REVA_RXTO
 * @brief    RX FIFO Timeout Register.
 * @{
 */
 #define MXC_F_CAN_REVA_RXTO_RX_TO_POS                       0 /**< RXTO_RX_TO Position */
 #define MXC_F_CAN_REVA_RXTO_RX_TO                           ((uint16_t)(0xFFFFUL << MXC_F_CAN_REVA_RXTO_RX_TO_POS)) /**< RXTO_RX_TO Mask */

/**@} end of group CAN_REVA_RXTO_Register */

#ifdef __cplusplus
}
#endif

#endif /* _CAN_REVA_REGS_H_ */

/**
 * @file    emac_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the EMAC_REVA Peripheral Module.
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

#ifndef _EMAC_REVA_REGS_H_
#define _EMAC_REVA_REGS_H_

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
 * @ingroup     emac_reva
 * @defgroup    emac_reva_registers EMAC_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the EMAC_REVA Peripheral Module.
 * @details 10/100 Ethernet MAC.
 */

/**
 * @ingroup emac_reva_registers
 * Structure type to access the EMAC_REVA Registers.
 */
typedef struct {
    __IO uint32_t cn;                   /**< <tt>\b 0x00:</tt> EMAC_REVA CN Register */
    __IO uint32_t cfg;                  /**< <tt>\b 0x04:</tt> EMAC_REVA CFG Register */
    __I  uint32_t status;               /**< <tt>\b 0x08:</tt> EMAC_REVA STATUS Register */
    __R  uint32_t rsv_0xc_0x13[2];
    __IO uint32_t tx_st;                /**< <tt>\b 0x14:</tt> EMAC_REVA TX_ST Register */
    __IO uint32_t rxbuf_ptr;            /**< <tt>\b 0x18:</tt> EMAC_REVA RXBUF_PTR Register */
    __IO uint32_t txbuf_ptr;            /**< <tt>\b 0x1C:</tt> EMAC_REVA TXBUF_PTR Register */
    __IO uint32_t rx_st;                /**< <tt>\b 0x20:</tt> EMAC_REVA RX_ST Register */
    __IO uint32_t int_st;               /**< <tt>\b 0x24:</tt> EMAC_REVA INT_ST Register */
    __O  uint32_t int_en;               /**< <tt>\b 0x28:</tt> EMAC_REVA INT_EN Register */
    __O  uint32_t int_dis;              /**< <tt>\b 0x2C:</tt> EMAC_REVA INT_DIS Register */
    __I  uint32_t int_mask;             /**< <tt>\b 0x30:</tt> EMAC_REVA INT_MASK Register */
    __IO uint32_t phy_mt;               /**< <tt>\b 0x34:</tt> EMAC_REVA PHY_MT Register */
    __I  uint32_t pt;                   /**< <tt>\b 0x38:</tt> EMAC_REVA PT Register */
    __IO uint32_t pfr;                  /**< <tt>\b 0x3C:</tt> EMAC_REVA PFR Register */
    __IO uint32_t ftok;                 /**< <tt>\b 0x40:</tt> EMAC_REVA FTOK Register */
    __IO uint32_t scf;                  /**< <tt>\b 0x44:</tt> EMAC_REVA SCF Register */
    __IO uint32_t mcf;                  /**< <tt>\b 0x48:</tt> EMAC_REVA MCF Register */
    __IO uint32_t frok;                 /**< <tt>\b 0x4C:</tt> EMAC_REVA FROK Register */
    __IO uint32_t fcs_err;              /**< <tt>\b 0x50:</tt> EMAC_REVA FCS_ERR Register */
    __IO uint32_t algn_err;             /**< <tt>\b 0x54:</tt> EMAC_REVA ALGN_ERR Register */
    __IO uint32_t dftxf;                /**< <tt>\b 0x58:</tt> EMAC_REVA DFTXF Register */
    __IO uint32_t lc;                   /**< <tt>\b 0x5C:</tt> EMAC_REVA LC Register */
    __IO uint32_t ec;                   /**< <tt>\b 0x60:</tt> EMAC_REVA EC Register */
    __IO uint32_t tur_err;              /**< <tt>\b 0x64:</tt> EMAC_REVA TUR_ERR Register */
    __IO uint32_t cs_err;               /**< <tt>\b 0x68:</tt> EMAC_REVA CS_ERR Register */
    __IO uint32_t rr_err;               /**< <tt>\b 0x6C:</tt> EMAC_REVA RR_ERR Register */
    __IO uint32_t ror_err;              /**< <tt>\b 0x70:</tt> EMAC_REVA ROR_ERR Register */
    __IO uint32_t rs_err;               /**< <tt>\b 0x74:</tt> EMAC_REVA RS_ERR Register */
    __IO uint32_t el_err;               /**< <tt>\b 0x78:</tt> EMAC_REVA EL_ERR Register */
    __IO uint32_t rj;                   /**< <tt>\b 0x7C:</tt> EMAC_REVA RJ Register */
    __IO uint32_t usf;                  /**< <tt>\b 0x80:</tt> EMAC_REVA USF Register */
    __IO uint32_t sqe_err;              /**< <tt>\b 0x84:</tt> EMAC_REVA SQE_ERR Register */
    __IO uint32_t rlfm;                 /**< <tt>\b 0x88:</tt> EMAC_REVA RLFM Register */
    __IO uint32_t tpf;                  /**< <tt>\b 0x8C:</tt> EMAC_REVA TPF Register */
    __IO uint32_t hashl;                /**< <tt>\b 0x90:</tt> EMAC_REVA HASHL Register */
    __IO uint32_t hashh;                /**< <tt>\b 0x94:</tt> EMAC_REVA HASHH Register */
    __IO uint32_t sa1l;                 /**< <tt>\b 0x98:</tt> EMAC_REVA SA1L Register */
    __IO uint32_t sa1h;                 /**< <tt>\b 0x9C:</tt> EMAC_REVA SA1H Register */
    __IO uint32_t sa2l;                 /**< <tt>\b 0xA0:</tt> EMAC_REVA SA2L Register */
    __IO uint32_t sa2h;                 /**< <tt>\b 0xA4:</tt> EMAC_REVA SA2H Register */
    __IO uint32_t sa3l;                 /**< <tt>\b 0xA8:</tt> EMAC_REVA SA3L Register */
    __IO uint32_t sa3h;                 /**< <tt>\b 0xAC:</tt> EMAC_REVA SA3H Register */
    __IO uint32_t sa4l;                 /**< <tt>\b 0xB0:</tt> EMAC_REVA SA4L Register */
    __IO uint32_t sa4h;                 /**< <tt>\b 0xB4:</tt> EMAC_REVA SA4H Register */
    __IO uint32_t tid_ck;               /**< <tt>\b 0xB8:</tt> EMAC_REVA TID_CK Register */
    __IO uint32_t tpq;                  /**< <tt>\b 0xBC:</tt> EMAC_REVA TPQ Register */
    __IO uint32_t usrio;                /**< <tt>\b 0xC0:</tt> EMAC_REVA USRIO Register */
    __IO uint32_t wol;                  /**< <tt>\b 0xC4:</tt> EMAC_REVA WOL Register */
    __R  uint32_t rsv_0xc8_0xfb[13];
    __I  uint32_t rev;                  /**< <tt>\b 0xFC:</tt> EMAC_REVA REV Register */
} mxc_emac_reva_regs_t;

/* Register offsets for module EMAC_REVA */
/**
 * @ingroup    emac_reva_registers
 * @defgroup   EMAC_REVA_Register_Offsets Register Offsets
 * @brief      EMAC_REVA Peripheral Register Offsets from the EMAC_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_EMAC_REVA_CN                 ((uint32_t)0x00000000UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_EMAC_REVA_CFG                ((uint32_t)0x00000004UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_EMAC_REVA_STATUS             ((uint32_t)0x00000008UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_EMAC_REVA_TX_ST              ((uint32_t)0x00000014UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_EMAC_REVA_RXBUF_PTR          ((uint32_t)0x00000018UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0018</tt> */ 
 #define MXC_R_EMAC_REVA_TXBUF_PTR          ((uint32_t)0x0000001CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x001C</tt> */ 
 #define MXC_R_EMAC_REVA_RX_ST              ((uint32_t)0x00000020UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0020</tt> */ 
 #define MXC_R_EMAC_REVA_INT_ST             ((uint32_t)0x00000024UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0024</tt> */ 
 #define MXC_R_EMAC_REVA_INT_EN             ((uint32_t)0x00000028UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0028</tt> */ 
 #define MXC_R_EMAC_REVA_INT_DIS            ((uint32_t)0x0000002CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x002C</tt> */ 
 #define MXC_R_EMAC_REVA_INT_MASK           ((uint32_t)0x00000030UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0030</tt> */ 
 #define MXC_R_EMAC_REVA_PHY_MT             ((uint32_t)0x00000034UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0034</tt> */ 
 #define MXC_R_EMAC_REVA_PT                 ((uint32_t)0x00000038UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0038</tt> */ 
 #define MXC_R_EMAC_REVA_PFR                ((uint32_t)0x0000003CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x003C</tt> */ 
 #define MXC_R_EMAC_REVA_FTOK               ((uint32_t)0x00000040UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0040</tt> */ 
 #define MXC_R_EMAC_REVA_SCF                ((uint32_t)0x00000044UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0044</tt> */ 
 #define MXC_R_EMAC_REVA_MCF                ((uint32_t)0x00000048UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0048</tt> */ 
 #define MXC_R_EMAC_REVA_FROK               ((uint32_t)0x0000004CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x004C</tt> */ 
 #define MXC_R_EMAC_REVA_FCS_ERR            ((uint32_t)0x00000050UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0050</tt> */ 
 #define MXC_R_EMAC_REVA_ALGN_ERR           ((uint32_t)0x00000054UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0054</tt> */ 
 #define MXC_R_EMAC_REVA_DFTXF              ((uint32_t)0x00000058UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0058</tt> */ 
 #define MXC_R_EMAC_REVA_LC                 ((uint32_t)0x0000005CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x005C</tt> */ 
 #define MXC_R_EMAC_REVA_EC                 ((uint32_t)0x00000060UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0060</tt> */ 
 #define MXC_R_EMAC_REVA_TUR_ERR            ((uint32_t)0x00000064UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0064</tt> */ 
 #define MXC_R_EMAC_REVA_CS_ERR             ((uint32_t)0x00000068UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0068</tt> */ 
 #define MXC_R_EMAC_REVA_RR_ERR             ((uint32_t)0x0000006CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x006C</tt> */ 
 #define MXC_R_EMAC_REVA_ROR_ERR            ((uint32_t)0x00000070UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0070</tt> */ 
 #define MXC_R_EMAC_REVA_RS_ERR             ((uint32_t)0x00000074UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0074</tt> */ 
 #define MXC_R_EMAC_REVA_EL_ERR             ((uint32_t)0x00000078UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0078</tt> */ 
 #define MXC_R_EMAC_REVA_RJ                 ((uint32_t)0x0000007CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x007C</tt> */ 
 #define MXC_R_EMAC_REVA_USF                ((uint32_t)0x00000080UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0080</tt> */ 
 #define MXC_R_EMAC_REVA_SQE_ERR            ((uint32_t)0x00000084UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0084</tt> */ 
 #define MXC_R_EMAC_REVA_RLFM               ((uint32_t)0x00000088UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0088</tt> */ 
 #define MXC_R_EMAC_REVA_TPF                ((uint32_t)0x0000008CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x008C</tt> */ 
 #define MXC_R_EMAC_REVA_HASHL              ((uint32_t)0x00000090UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0090</tt> */ 
 #define MXC_R_EMAC_REVA_HASHH              ((uint32_t)0x00000094UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0094</tt> */ 
 #define MXC_R_EMAC_REVA_SA1L               ((uint32_t)0x00000098UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x0098</tt> */ 
 #define MXC_R_EMAC_REVA_SA1H               ((uint32_t)0x0000009CUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x009C</tt> */ 
 #define MXC_R_EMAC_REVA_SA2L               ((uint32_t)0x000000A0UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00A0</tt> */ 
 #define MXC_R_EMAC_REVA_SA2H               ((uint32_t)0x000000A4UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00A4</tt> */ 
 #define MXC_R_EMAC_REVA_SA3L               ((uint32_t)0x000000A8UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00A8</tt> */ 
 #define MXC_R_EMAC_REVA_SA3H               ((uint32_t)0x000000ACUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00AC</tt> */ 
 #define MXC_R_EMAC_REVA_SA4L               ((uint32_t)0x000000B0UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00B0</tt> */ 
 #define MXC_R_EMAC_REVA_SA4H               ((uint32_t)0x000000B4UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00B4</tt> */ 
 #define MXC_R_EMAC_REVA_TID_CK             ((uint32_t)0x000000B8UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00B8</tt> */ 
 #define MXC_R_EMAC_REVA_TPQ                ((uint32_t)0x000000BCUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00BC</tt> */ 
 #define MXC_R_EMAC_REVA_USRIO              ((uint32_t)0x000000C0UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00C0</tt> */ 
 #define MXC_R_EMAC_REVA_WOL                ((uint32_t)0x000000C4UL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00C4</tt> */ 
 #define MXC_R_EMAC_REVA_REV                ((uint32_t)0x000000FCUL) /**< Offset from EMAC_REVA Base Address: <tt> 0x00FC</tt> */ 
/**@} end of group emac_reva_registers */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_CN EMAC_REVA_CN
 * @brief    Network Control Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_CN_LB_POS                      0 /**< CN_LB Position */
 #define MXC_F_EMAC_REVA_CN_LB                          ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_LB_POS)) /**< CN_LB Mask */

 #define MXC_F_EMAC_REVA_CN_LBL_POS                     1 /**< CN_LBL Position */
 #define MXC_F_EMAC_REVA_CN_LBL                         ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_LBL_POS)) /**< CN_LBL Mask */

 #define MXC_F_EMAC_REVA_CN_RXEN_POS                    2 /**< CN_RXEN Position */
 #define MXC_F_EMAC_REVA_CN_RXEN                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_RXEN_POS)) /**< CN_RXEN Mask */

 #define MXC_F_EMAC_REVA_CN_TXEN_POS                    3 /**< CN_TXEN Position */
 #define MXC_F_EMAC_REVA_CN_TXEN                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_TXEN_POS)) /**< CN_TXEN Mask */

 #define MXC_F_EMAC_REVA_CN_MPEN_POS                    4 /**< CN_MPEN Position */
 #define MXC_F_EMAC_REVA_CN_MPEN                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_MPEN_POS)) /**< CN_MPEN Mask */

 #define MXC_F_EMAC_REVA_CN_CLST_POS                    5 /**< CN_CLST Position */
 #define MXC_F_EMAC_REVA_CN_CLST                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_CLST_POS)) /**< CN_CLST Mask */

 #define MXC_F_EMAC_REVA_CN_INCST_POS                   6 /**< CN_INCST Position */
 #define MXC_F_EMAC_REVA_CN_INCST                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_INCST_POS)) /**< CN_INCST Mask */

 #define MXC_F_EMAC_REVA_CN_WREN_POS                    7 /**< CN_WREN Position */
 #define MXC_F_EMAC_REVA_CN_WREN                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_WREN_POS)) /**< CN_WREN Mask */

 #define MXC_F_EMAC_REVA_CN_BP_POS                      8 /**< CN_BP Position */
 #define MXC_F_EMAC_REVA_CN_BP                          ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_BP_POS)) /**< CN_BP Mask */

 #define MXC_F_EMAC_REVA_CN_TXSTART_POS                 9 /**< CN_TXSTART Position */
 #define MXC_F_EMAC_REVA_CN_TXSTART                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_TXSTART_POS)) /**< CN_TXSTART Mask */

 #define MXC_F_EMAC_REVA_CN_TXHALT_POS                  10 /**< CN_TXHALT Position */
 #define MXC_F_EMAC_REVA_CN_TXHALT                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_TXHALT_POS)) /**< CN_TXHALT Mask */

 #define MXC_F_EMAC_REVA_CN_TXPF_POS                    11 /**< CN_TXPF Position */
 #define MXC_F_EMAC_REVA_CN_TXPF                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_TXPF_POS)) /**< CN_TXPF Mask */

 #define MXC_F_EMAC_REVA_CN_TXZQPF_POS                  12 /**< CN_TXZQPF Position */
 #define MXC_F_EMAC_REVA_CN_TXZQPF                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CN_TXZQPF_POS)) /**< CN_TXZQPF Mask */

/**@} end of group EMAC_REVA_CN_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_CFG EMAC_REVA_CFG
 * @brief    Network Configuration Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_CFG_SPD_POS                    0 /**< CFG_SPD Position */
 #define MXC_F_EMAC_REVA_CFG_SPD                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_SPD_POS)) /**< CFG_SPD Mask */

 #define MXC_F_EMAC_REVA_CFG_FULLDPLX_POS               1 /**< CFG_FULLDPLX Position */
 #define MXC_F_EMAC_REVA_CFG_FULLDPLX                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_FULLDPLX_POS)) /**< CFG_FULLDPLX Mask */

 #define MXC_F_EMAC_REVA_CFG_BITRATE_POS                2 /**< CFG_BITRATE Position */
 #define MXC_F_EMAC_REVA_CFG_BITRATE                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_BITRATE_POS)) /**< CFG_BITRATE Mask */

 #define MXC_F_EMAC_REVA_CFG_JUMBOFR_POS                3 /**< CFG_JUMBOFR Position */
 #define MXC_F_EMAC_REVA_CFG_JUMBOFR                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_JUMBOFR_POS)) /**< CFG_JUMBOFR Mask */

 #define MXC_F_EMAC_REVA_CFG_COPYAF_POS                 4 /**< CFG_COPYAF Position */
 #define MXC_F_EMAC_REVA_CFG_COPYAF                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_COPYAF_POS)) /**< CFG_COPYAF Mask */

 #define MXC_F_EMAC_REVA_CFG_NOBC_POS                   5 /**< CFG_NOBC Position */
 #define MXC_F_EMAC_REVA_CFG_NOBC                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_NOBC_POS)) /**< CFG_NOBC Mask */

 #define MXC_F_EMAC_REVA_CFG_MHEN_POS                   6 /**< CFG_MHEN Position */
 #define MXC_F_EMAC_REVA_CFG_MHEN                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_MHEN_POS)) /**< CFG_MHEN Mask */

 #define MXC_F_EMAC_REVA_CFG_UHEN_POS                   7 /**< CFG_UHEN Position */
 #define MXC_F_EMAC_REVA_CFG_UHEN                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_UHEN_POS)) /**< CFG_UHEN Mask */

 #define MXC_F_EMAC_REVA_CFG_RXFR_POS                   8 /**< CFG_RXFR Position */
 #define MXC_F_EMAC_REVA_CFG_RXFR                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_RXFR_POS)) /**< CFG_RXFR Mask */

 #define MXC_F_EMAC_REVA_CFG_EAE_POS                    9 /**< CFG_EAE Position */
 #define MXC_F_EMAC_REVA_CFG_EAE                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_EAE_POS)) /**< CFG_EAE Mask */

 #define MXC_F_EMAC_REVA_CFG_MDCCLK_POS                 10 /**< CFG_MDCCLK Position */
 #define MXC_F_EMAC_REVA_CFG_MDCCLK                     ((uint32_t)(0x3UL << MXC_F_EMAC_REVA_CFG_MDCCLK_POS)) /**< CFG_MDCCLK Mask */
 #define MXC_V_EMAC_REVA_CFG_MDCCLK_DIV8                ((uint32_t)0x0UL) /**< CFG_MDCCLK_DIV8 Value */
 #define MXC_S_EMAC_REVA_CFG_MDCCLK_DIV8                (MXC_V_EMAC_REVA_CFG_MDCCLK_DIV8 << MXC_F_EMAC_REVA_CFG_MDCCLK_POS) /**< CFG_MDCCLK_DIV8 Setting */
 #define MXC_V_EMAC_REVA_CFG_MDCCLK_DIV16               ((uint32_t)0x1UL) /**< CFG_MDCCLK_DIV16 Value */
 #define MXC_S_EMAC_REVA_CFG_MDCCLK_DIV16               (MXC_V_EMAC_REVA_CFG_MDCCLK_DIV16 << MXC_F_EMAC_REVA_CFG_MDCCLK_POS) /**< CFG_MDCCLK_DIV16 Setting */
 #define MXC_V_EMAC_REVA_CFG_MDCCLK_DIV32               ((uint32_t)0x2UL) /**< CFG_MDCCLK_DIV32 Value */
 #define MXC_S_EMAC_REVA_CFG_MDCCLK_DIV32               (MXC_V_EMAC_REVA_CFG_MDCCLK_DIV32 << MXC_F_EMAC_REVA_CFG_MDCCLK_POS) /**< CFG_MDCCLK_DIV32 Setting */
 #define MXC_V_EMAC_REVA_CFG_MDCCLK_DIV64               ((uint32_t)0x3UL) /**< CFG_MDCCLK_DIV64 Value */
 #define MXC_S_EMAC_REVA_CFG_MDCCLK_DIV64               (MXC_V_EMAC_REVA_CFG_MDCCLK_DIV64 << MXC_F_EMAC_REVA_CFG_MDCCLK_POS) /**< CFG_MDCCLK_DIV64 Setting */

 #define MXC_F_EMAC_REVA_CFG_TXPF_POS                   11 /**< CFG_TXPF Position */
 #define MXC_F_EMAC_REVA_CFG_TXPF                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_TXPF_POS)) /**< CFG_TXPF Mask */

 #define MXC_F_EMAC_REVA_CFG_RTTST_POS                  12 /**< CFG_RTTST Position */
 #define MXC_F_EMAC_REVA_CFG_RTTST                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_RTTST_POS)) /**< CFG_RTTST Mask */

 #define MXC_F_EMAC_REVA_CFG_PAUSEEN_POS                13 /**< CFG_PAUSEEN Position */
 #define MXC_F_EMAC_REVA_CFG_PAUSEEN                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_PAUSEEN_POS)) /**< CFG_PAUSEEN Mask */

 #define MXC_F_EMAC_REVA_CFG_RXBUFFOFS_POS              14 /**< CFG_RXBUFFOFS Position */
 #define MXC_F_EMAC_REVA_CFG_RXBUFFOFS                  ((uint32_t)(0x3UL << MXC_F_EMAC_REVA_CFG_RXBUFFOFS_POS)) /**< CFG_RXBUFFOFS Mask */

 #define MXC_F_EMAC_REVA_CFG_RXLFCEN_POS                16 /**< CFG_RXLFCEN Position */
 #define MXC_F_EMAC_REVA_CFG_RXLFCEN                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_RXLFCEN_POS)) /**< CFG_RXLFCEN Mask */

 #define MXC_F_EMAC_REVA_CFG_DCRXFCS_POS                17 /**< CFG_DCRXFCS Position */
 #define MXC_F_EMAC_REVA_CFG_DCRXFCS                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_DCRXFCS_POS)) /**< CFG_DCRXFCS Mask */

 #define MXC_F_EMAC_REVA_CFG_HDPLXRXEN_POS              18 /**< CFG_HDPLXRXEN Position */
 #define MXC_F_EMAC_REVA_CFG_HDPLXRXEN                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_HDPLXRXEN_POS)) /**< CFG_HDPLXRXEN Mask */

 #define MXC_F_EMAC_REVA_CFG_IGNRXFCS_POS               19 /**< CFG_IGNRXFCS Position */
 #define MXC_F_EMAC_REVA_CFG_IGNRXFCS                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_CFG_IGNRXFCS_POS)) /**< CFG_IGNRXFCS Mask */

/**@} end of group EMAC_REVA_CFG_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_STATUS EMAC_REVA_STATUS
 * @brief    Network Status Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_STATUS_LINK_POS                0 /**< STATUS_LINK Position */
 #define MXC_F_EMAC_REVA_STATUS_LINK                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_STATUS_LINK_POS)) /**< STATUS_LINK Mask */

 #define MXC_F_EMAC_REVA_STATUS_MDIO_POS                1 /**< STATUS_MDIO Position */
 #define MXC_F_EMAC_REVA_STATUS_MDIO                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_STATUS_MDIO_POS)) /**< STATUS_MDIO Mask */

 #define MXC_F_EMAC_REVA_STATUS_IDLE_POS                2 /**< STATUS_IDLE Position */
 #define MXC_F_EMAC_REVA_STATUS_IDLE                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_STATUS_IDLE_POS)) /**< STATUS_IDLE Mask */

/**@} end of group EMAC_REVA_STATUS_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_TX_ST EMAC_REVA_TX_ST
 * @brief    Transmit Status Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_TX_ST_UBR_POS                  0 /**< TX_ST_UBR Position */
 #define MXC_F_EMAC_REVA_TX_ST_UBR                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_UBR_POS)) /**< TX_ST_UBR Mask */

 #define MXC_F_EMAC_REVA_TX_ST_COLS_POS                 1 /**< TX_ST_COLS Position */
 #define MXC_F_EMAC_REVA_TX_ST_COLS                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_COLS_POS)) /**< TX_ST_COLS Mask */

 #define MXC_F_EMAC_REVA_TX_ST_RTYLIM_POS               2 /**< TX_ST_RTYLIM Position */
 #define MXC_F_EMAC_REVA_TX_ST_RTYLIM                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_RTYLIM_POS)) /**< TX_ST_RTYLIM Mask */

 #define MXC_F_EMAC_REVA_TX_ST_TXGO_POS                 3 /**< TX_ST_TXGO Position */
 #define MXC_F_EMAC_REVA_TX_ST_TXGO                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_TXGO_POS)) /**< TX_ST_TXGO Mask */

 #define MXC_F_EMAC_REVA_TX_ST_BEMF_POS                 4 /**< TX_ST_BEMF Position */
 #define MXC_F_EMAC_REVA_TX_ST_BEMF                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_BEMF_POS)) /**< TX_ST_BEMF Mask */

 #define MXC_F_EMAC_REVA_TX_ST_TXCMPL_POS               5 /**< TX_ST_TXCMPL Position */
 #define MXC_F_EMAC_REVA_TX_ST_TXCMPL                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_TXCMPL_POS)) /**< TX_ST_TXCMPL Mask */

 #define MXC_F_EMAC_REVA_TX_ST_TXUR_POS                 6 /**< TX_ST_TXUR Position */
 #define MXC_F_EMAC_REVA_TX_ST_TXUR                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_TX_ST_TXUR_POS)) /**< TX_ST_TXUR Mask */

/**@} end of group EMAC_REVA_TX_ST_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_RXBUF_PTR EMAC_REVA_RXBUF_PTR
 * @brief    Receive Buffer Queue Pointer Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_RXBUF_PTR_RXBUF_POS            2 /**< RXBUF_PTR_RXBUF Position */
 #define MXC_F_EMAC_REVA_RXBUF_PTR_RXBUF                ((uint32_t)(0x3FFFFFFFUL << MXC_F_EMAC_REVA_RXBUF_PTR_RXBUF_POS)) /**< RXBUF_PTR_RXBUF Mask */

/**@} end of group EMAC_REVA_RXBUF_PTR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_TXBUF_PTR EMAC_REVA_TXBUF_PTR
 * @brief    Transmit Buffer Queue Pointer Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_TXBUF_PTR_TXBUF_POS            2 /**< TXBUF_PTR_TXBUF Position */
 #define MXC_F_EMAC_REVA_TXBUF_PTR_TXBUF                ((uint32_t)(0x3FFFFFFFUL << MXC_F_EMAC_REVA_TXBUF_PTR_TXBUF_POS)) /**< TXBUF_PTR_TXBUF Mask */

/**@} end of group EMAC_REVA_TXBUF_PTR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_RX_ST EMAC_REVA_RX_ST
 * @brief    Receive Status Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_RX_ST_BNA_POS                  0 /**< RX_ST_BNA Position */
 #define MXC_F_EMAC_REVA_RX_ST_BNA                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_RX_ST_BNA_POS)) /**< RX_ST_BNA Mask */

 #define MXC_F_EMAC_REVA_RX_ST_FR_POS                   1 /**< RX_ST_FR Position */
 #define MXC_F_EMAC_REVA_RX_ST_FR                       ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_RX_ST_FR_POS)) /**< RX_ST_FR Mask */

 #define MXC_F_EMAC_REVA_RX_ST_RXOR_POS                 2 /**< RX_ST_RXOR Position */
 #define MXC_F_EMAC_REVA_RX_ST_RXOR                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_RX_ST_RXOR_POS)) /**< RX_ST_RXOR Mask */

/**@} end of group EMAC_REVA_RX_ST_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_INT_ST EMAC_REVA_INT_ST
 * @brief    Interrupt Status Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_INT_ST_MPS_POS                 0 /**< INT_ST_MPS Position */
 #define MXC_F_EMAC_REVA_INT_ST_MPS                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_MPS_POS)) /**< INT_ST_MPS Mask */

 #define MXC_F_EMAC_REVA_INT_ST_RXCMPL_POS              1 /**< INT_ST_RXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_ST_RXCMPL                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_RXCMPL_POS)) /**< INT_ST_RXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_ST_RXUBR_POS               2 /**< INT_ST_RXUBR Position */
 #define MXC_F_EMAC_REVA_INT_ST_RXUBR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_RXUBR_POS)) /**< INT_ST_RXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_ST_TXUBR_POS               3 /**< INT_ST_TXUBR Position */
 #define MXC_F_EMAC_REVA_INT_ST_TXUBR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_TXUBR_POS)) /**< INT_ST_TXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_ST_TXUR_POS                4 /**< INT_ST_TXUR Position */
 #define MXC_F_EMAC_REVA_INT_ST_TXUR                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_TXUR_POS)) /**< INT_ST_TXUR Mask */

 #define MXC_F_EMAC_REVA_INT_ST_RLE_POS                 5 /**< INT_ST_RLE Position */
 #define MXC_F_EMAC_REVA_INT_ST_RLE                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_RLE_POS)) /**< INT_ST_RLE Mask */

 #define MXC_F_EMAC_REVA_INT_ST_TXERR_POS               6 /**< INT_ST_TXERR Position */
 #define MXC_F_EMAC_REVA_INT_ST_TXERR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_TXERR_POS)) /**< INT_ST_TXERR Mask */

 #define MXC_F_EMAC_REVA_INT_ST_TXCMPL_POS              7 /**< INT_ST_TXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_ST_TXCMPL                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_TXCMPL_POS)) /**< INT_ST_TXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_ST_LC_POS                  9 /**< INT_ST_LC Position */
 #define MXC_F_EMAC_REVA_INT_ST_LC                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_LC_POS)) /**< INT_ST_LC Mask */

 #define MXC_F_EMAC_REVA_INT_ST_RXOR_POS                10 /**< INT_ST_RXOR Position */
 #define MXC_F_EMAC_REVA_INT_ST_RXOR                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_RXOR_POS)) /**< INT_ST_RXOR Mask */

 #define MXC_F_EMAC_REVA_INT_ST_HRESPNO_POS             11 /**< INT_ST_HRESPNO Position */
 #define MXC_F_EMAC_REVA_INT_ST_HRESPNO                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_HRESPNO_POS)) /**< INT_ST_HRESPNO Mask */

 #define MXC_F_EMAC_REVA_INT_ST_PPR_POS                 12 /**< INT_ST_PPR Position */
 #define MXC_F_EMAC_REVA_INT_ST_PPR                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_PPR_POS)) /**< INT_ST_PPR Mask */

 #define MXC_F_EMAC_REVA_INT_ST_PTZ_POS                 13 /**< INT_ST_PTZ Position */
 #define MXC_F_EMAC_REVA_INT_ST_PTZ                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_ST_PTZ_POS)) /**< INT_ST_PTZ Mask */

/**@} end of group EMAC_REVA_INT_ST_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_INT_EN EMAC_REVA_INT_EN
 * @brief    Interrupt Enable Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_INT_EN_MPS_POS                 0 /**< INT_EN_MPS Position */
 #define MXC_F_EMAC_REVA_INT_EN_MPS                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_MPS_POS)) /**< INT_EN_MPS Mask */

 #define MXC_F_EMAC_REVA_INT_EN_RXCMPL_POS              1 /**< INT_EN_RXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_EN_RXCMPL                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_RXCMPL_POS)) /**< INT_EN_RXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_EN_RXUBR_POS               2 /**< INT_EN_RXUBR Position */
 #define MXC_F_EMAC_REVA_INT_EN_RXUBR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_RXUBR_POS)) /**< INT_EN_RXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_EN_TXUBR_POS               3 /**< INT_EN_TXUBR Position */
 #define MXC_F_EMAC_REVA_INT_EN_TXUBR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_TXUBR_POS)) /**< INT_EN_TXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_EN_TXUR_POS                4 /**< INT_EN_TXUR Position */
 #define MXC_F_EMAC_REVA_INT_EN_TXUR                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_TXUR_POS)) /**< INT_EN_TXUR Mask */

 #define MXC_F_EMAC_REVA_INT_EN_RLE_POS                 5 /**< INT_EN_RLE Position */
 #define MXC_F_EMAC_REVA_INT_EN_RLE                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_RLE_POS)) /**< INT_EN_RLE Mask */

 #define MXC_F_EMAC_REVA_INT_EN_TXERR_POS               6 /**< INT_EN_TXERR Position */
 #define MXC_F_EMAC_REVA_INT_EN_TXERR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_TXERR_POS)) /**< INT_EN_TXERR Mask */

 #define MXC_F_EMAC_REVA_INT_EN_TXCMPL_POS              7 /**< INT_EN_TXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_EN_TXCMPL                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_TXCMPL_POS)) /**< INT_EN_TXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_EN_LC_POS                  9 /**< INT_EN_LC Position */
 #define MXC_F_EMAC_REVA_INT_EN_LC                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_LC_POS)) /**< INT_EN_LC Mask */

 #define MXC_F_EMAC_REVA_INT_EN_RXOR_POS                10 /**< INT_EN_RXOR Position */
 #define MXC_F_EMAC_REVA_INT_EN_RXOR                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_RXOR_POS)) /**< INT_EN_RXOR Mask */

 #define MXC_F_EMAC_REVA_INT_EN_HRESPNO_POS             11 /**< INT_EN_HRESPNO Position */
 #define MXC_F_EMAC_REVA_INT_EN_HRESPNO                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_HRESPNO_POS)) /**< INT_EN_HRESPNO Mask */

 #define MXC_F_EMAC_REVA_INT_EN_PPR_POS                 12 /**< INT_EN_PPR Position */
 #define MXC_F_EMAC_REVA_INT_EN_PPR                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_PPR_POS)) /**< INT_EN_PPR Mask */

 #define MXC_F_EMAC_REVA_INT_EN_PTZ_POS                 13 /**< INT_EN_PTZ Position */
 #define MXC_F_EMAC_REVA_INT_EN_PTZ                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_EN_PTZ_POS)) /**< INT_EN_PTZ Mask */

/**@} end of group EMAC_REVA_INT_EN_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_INT_DIS EMAC_REVA_INT_DIS
 * @brief    Interrupt Disable Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_INT_DIS_MPS_POS                0 /**< INT_DIS_MPS Position */
 #define MXC_F_EMAC_REVA_INT_DIS_MPS                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_MPS_POS)) /**< INT_DIS_MPS Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_RXCMPL_POS             1 /**< INT_DIS_RXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_DIS_RXCMPL                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_RXCMPL_POS)) /**< INT_DIS_RXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_RXUBR_POS              2 /**< INT_DIS_RXUBR Position */
 #define MXC_F_EMAC_REVA_INT_DIS_RXUBR                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_RXUBR_POS)) /**< INT_DIS_RXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_TXUBR_POS              3 /**< INT_DIS_TXUBR Position */
 #define MXC_F_EMAC_REVA_INT_DIS_TXUBR                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_TXUBR_POS)) /**< INT_DIS_TXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_TXUR_POS               4 /**< INT_DIS_TXUR Position */
 #define MXC_F_EMAC_REVA_INT_DIS_TXUR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_TXUR_POS)) /**< INT_DIS_TXUR Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_RLE_POS                5 /**< INT_DIS_RLE Position */
 #define MXC_F_EMAC_REVA_INT_DIS_RLE                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_RLE_POS)) /**< INT_DIS_RLE Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_TXERR_POS              6 /**< INT_DIS_TXERR Position */
 #define MXC_F_EMAC_REVA_INT_DIS_TXERR                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_TXERR_POS)) /**< INT_DIS_TXERR Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_TXCMPL_POS             7 /**< INT_DIS_TXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_DIS_TXCMPL                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_TXCMPL_POS)) /**< INT_DIS_TXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_LC_POS                 9 /**< INT_DIS_LC Position */
 #define MXC_F_EMAC_REVA_INT_DIS_LC                     ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_LC_POS)) /**< INT_DIS_LC Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_RXOR_POS               10 /**< INT_DIS_RXOR Position */
 #define MXC_F_EMAC_REVA_INT_DIS_RXOR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_RXOR_POS)) /**< INT_DIS_RXOR Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_HRESPNO_POS            11 /**< INT_DIS_HRESPNO Position */
 #define MXC_F_EMAC_REVA_INT_DIS_HRESPNO                ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_HRESPNO_POS)) /**< INT_DIS_HRESPNO Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_PPR_POS                12 /**< INT_DIS_PPR Position */
 #define MXC_F_EMAC_REVA_INT_DIS_PPR                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_PPR_POS)) /**< INT_DIS_PPR Mask */

 #define MXC_F_EMAC_REVA_INT_DIS_PTZ_POS                13 /**< INT_DIS_PTZ Position */
 #define MXC_F_EMAC_REVA_INT_DIS_PTZ                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_DIS_PTZ_POS)) /**< INT_DIS_PTZ Mask */

/**@} end of group EMAC_REVA_INT_DIS_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_INT_MASK EMAC_REVA_INT_MASK
 * @brief    Interrupt Mask Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_INT_MASK_MPS_POS               0 /**< INT_MASK_MPS Position */
 #define MXC_F_EMAC_REVA_INT_MASK_MPS                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_MPS_POS)) /**< INT_MASK_MPS Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_RXCMPL_POS            1 /**< INT_MASK_RXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_MASK_RXCMPL                ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_RXCMPL_POS)) /**< INT_MASK_RXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_RXUBR_POS             2 /**< INT_MASK_RXUBR Position */
 #define MXC_F_EMAC_REVA_INT_MASK_RXUBR                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_RXUBR_POS)) /**< INT_MASK_RXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_TXUBR_POS             3 /**< INT_MASK_TXUBR Position */
 #define MXC_F_EMAC_REVA_INT_MASK_TXUBR                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_TXUBR_POS)) /**< INT_MASK_TXUBR Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_TXUR_POS              4 /**< INT_MASK_TXUR Position */
 #define MXC_F_EMAC_REVA_INT_MASK_TXUR                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_TXUR_POS)) /**< INT_MASK_TXUR Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_RLE_POS               5 /**< INT_MASK_RLE Position */
 #define MXC_F_EMAC_REVA_INT_MASK_RLE                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_RLE_POS)) /**< INT_MASK_RLE Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_TXERR_POS             6 /**< INT_MASK_TXERR Position */
 #define MXC_F_EMAC_REVA_INT_MASK_TXERR                 ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_TXERR_POS)) /**< INT_MASK_TXERR Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_TXCMPL_POS            7 /**< INT_MASK_TXCMPL Position */
 #define MXC_F_EMAC_REVA_INT_MASK_TXCMPL                ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_TXCMPL_POS)) /**< INT_MASK_TXCMPL Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_LC_POS                9 /**< INT_MASK_LC Position */
 #define MXC_F_EMAC_REVA_INT_MASK_LC                    ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_LC_POS)) /**< INT_MASK_LC Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_RXOR_POS              10 /**< INT_MASK_RXOR Position */
 #define MXC_F_EMAC_REVA_INT_MASK_RXOR                  ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_RXOR_POS)) /**< INT_MASK_RXOR Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_HRESPNO_POS           11 /**< INT_MASK_HRESPNO Position */
 #define MXC_F_EMAC_REVA_INT_MASK_HRESPNO               ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_HRESPNO_POS)) /**< INT_MASK_HRESPNO Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_PPR_POS               12 /**< INT_MASK_PPR Position */
 #define MXC_F_EMAC_REVA_INT_MASK_PPR                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_PPR_POS)) /**< INT_MASK_PPR Mask */

 #define MXC_F_EMAC_REVA_INT_MASK_PTZ_POS               13 /**< INT_MASK_PTZ Position */
 #define MXC_F_EMAC_REVA_INT_MASK_PTZ                   ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_INT_MASK_PTZ_POS)) /**< INT_MASK_PTZ Mask */

/**@} end of group EMAC_REVA_INT_MASK_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_PHY_MT EMAC_REVA_PHY_MT
 * @brief    PHY Maintenance Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_PHY_MT_DATA_POS                0 /**< PHY_MT_DATA Position */
 #define MXC_F_EMAC_REVA_PHY_MT_DATA                    ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_PHY_MT_DATA_POS)) /**< PHY_MT_DATA Mask */

 #define MXC_F_EMAC_REVA_PHY_MT_CODE_POS                16 /**< PHY_MT_CODE Position */
 #define MXC_F_EMAC_REVA_PHY_MT_CODE                    ((uint32_t)(0x3UL << MXC_F_EMAC_REVA_PHY_MT_CODE_POS)) /**< PHY_MT_CODE Mask */

 #define MXC_F_EMAC_REVA_PHY_MT_REGADDR_POS             18 /**< PHY_MT_REGADDR Position */
 #define MXC_F_EMAC_REVA_PHY_MT_REGADDR                 ((uint32_t)(0x1FUL << MXC_F_EMAC_REVA_PHY_MT_REGADDR_POS)) /**< PHY_MT_REGADDR Mask */

 #define MXC_F_EMAC_REVA_PHY_MT_PHYADDR_POS             23 /**< PHY_MT_PHYADDR Position */
 #define MXC_F_EMAC_REVA_PHY_MT_PHYADDR                 ((uint32_t)(0x1FUL << MXC_F_EMAC_REVA_PHY_MT_PHYADDR_POS)) /**< PHY_MT_PHYADDR Mask */

 #define MXC_F_EMAC_REVA_PHY_MT_OP_POS                  28 /**< PHY_MT_OP Position */
 #define MXC_F_EMAC_REVA_PHY_MT_OP                      ((uint32_t)(0x3UL << MXC_F_EMAC_REVA_PHY_MT_OP_POS)) /**< PHY_MT_OP Mask */
 #define MXC_V_EMAC_REVA_PHY_MT_OP_WRITE                ((uint32_t)0x1UL) /**< PHY_MT_OP_WRITE Value */
 #define MXC_S_EMAC_REVA_PHY_MT_OP_WRITE                (MXC_V_EMAC_REVA_PHY_MT_OP_WRITE << MXC_F_EMAC_REVA_PHY_MT_OP_POS) /**< PHY_MT_OP_WRITE Setting */
 #define MXC_V_EMAC_REVA_PHY_MT_OP_READ                 ((uint32_t)0x2UL) /**< PHY_MT_OP_READ Value */
 #define MXC_S_EMAC_REVA_PHY_MT_OP_READ                 (MXC_V_EMAC_REVA_PHY_MT_OP_READ << MXC_F_EMAC_REVA_PHY_MT_OP_POS) /**< PHY_MT_OP_READ Setting */

 #define MXC_F_EMAC_REVA_PHY_MT_SOP_POS                 30 /**< PHY_MT_SOP Position */
 #define MXC_F_EMAC_REVA_PHY_MT_SOP                     ((uint32_t)(0x3UL << MXC_F_EMAC_REVA_PHY_MT_SOP_POS)) /**< PHY_MT_SOP Mask */

/**@} end of group EMAC_REVA_PHY_MT_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_PT EMAC_REVA_PT
 * @brief    Pause Time Register.
 * @{
 */
 #define MXC_F_EMAC_REVA_PT_TIME_POS                    0 /**< PT_TIME Position */
 #define MXC_F_EMAC_REVA_PT_TIME                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_PT_TIME_POS)) /**< PT_TIME Mask */

/**@} end of group EMAC_REVA_PT_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_PFR EMAC_REVA_PFR
 * @brief    Pause Frame Received OK.
 * @{
 */
 #define MXC_F_EMAC_REVA_PFR_PFR_POS                    0 /**< PFR_PFR Position */
 #define MXC_F_EMAC_REVA_PFR_PFR                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_PFR_PFR_POS)) /**< PFR_PFR Mask */

/**@} end of group EMAC_REVA_PFR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_FTOK EMAC_REVA_FTOK
 * @brief    Frames Transmitted OK.
 * @{
 */
 #define MXC_F_EMAC_REVA_FTOK_FTOK_POS                  0 /**< FTOK_FTOK Position */
 #define MXC_F_EMAC_REVA_FTOK_FTOK                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_FTOK_FTOK_POS)) /**< FTOK_FTOK Mask */

/**@} end of group EMAC_REVA_FTOK_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SCF EMAC_REVA_SCF
 * @brief    Single Collision Frames.
 * @{
 */
 #define MXC_F_EMAC_REVA_SCF_SCF_POS                    0 /**< SCF_SCF Position */
 #define MXC_F_EMAC_REVA_SCF_SCF                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_SCF_SCF_POS)) /**< SCF_SCF Mask */

/**@} end of group EMAC_REVA_SCF_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_MCF EMAC_REVA_MCF
 * @brief    Multiple Collision Frames.
 * @{
 */
 #define MXC_F_EMAC_REVA_MCF_MCF_POS                    0 /**< MCF_MCF Position */
 #define MXC_F_EMAC_REVA_MCF_MCF                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_MCF_MCF_POS)) /**< MCF_MCF Mask */

/**@} end of group EMAC_REVA_MCF_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_FROK EMAC_REVA_FROK
 * @brief    Fames Received OK.
 * @{
 */
 #define MXC_F_EMAC_REVA_FROK_FROK_POS                  0 /**< FROK_FROK Position */
 #define MXC_F_EMAC_REVA_FROK_FROK                      ((uint32_t)(0xFFFFFFUL << MXC_F_EMAC_REVA_FROK_FROK_POS)) /**< FROK_FROK Mask */

/**@} end of group EMAC_REVA_FROK_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_FCS_ERR EMAC_REVA_FCS_ERR
 * @brief    Frame Check Sequence Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_FCS_ERR_FCSERR_POS             0 /**< FCS_ERR_FCSERR Position */
 #define MXC_F_EMAC_REVA_FCS_ERR_FCSERR                 ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_FCS_ERR_FCSERR_POS)) /**< FCS_ERR_FCSERR Mask */

/**@} end of group EMAC_REVA_FCS_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_ALGN_ERR EMAC_REVA_ALGN_ERR
 * @brief    Alignment Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_ALGN_ERR_ALGNERR_POS           0 /**< ALGN_ERR_ALGNERR Position */
 #define MXC_F_EMAC_REVA_ALGN_ERR_ALGNERR               ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_ALGN_ERR_ALGNERR_POS)) /**< ALGN_ERR_ALGNERR Mask */

/**@} end of group EMAC_REVA_ALGN_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_DFTXF EMAC_REVA_DFTXF
 * @brief    Deferred Transmission Frames.
 * @{
 */
 #define MXC_F_EMAC_REVA_DFTXF_DFTXF_POS                0 /**< DFTXF_DFTXF Position */
 #define MXC_F_EMAC_REVA_DFTXF_DFTXF                    ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_DFTXF_DFTXF_POS)) /**< DFTXF_DFTXF Mask */

/**@} end of group EMAC_REVA_DFTXF_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_LC EMAC_REVA_LC
 * @brief    Late Collisions.
 * @{
 */
 #define MXC_F_EMAC_REVA_LC_LC_POS                      0 /**< LC_LC Position */
 #define MXC_F_EMAC_REVA_LC_LC                          ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_LC_LC_POS)) /**< LC_LC Mask */

/**@} end of group EMAC_REVA_LC_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_EC EMAC_REVA_EC
 * @brief    Excessive Collisions.
 * @{
 */
 #define MXC_F_EMAC_REVA_EC_EC_POS                      0 /**< EC_EC Position */
 #define MXC_F_EMAC_REVA_EC_EC                          ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_EC_EC_POS)) /**< EC_EC Mask */

/**@} end of group EMAC_REVA_EC_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_TUR_ERR EMAC_REVA_TUR_ERR
 * @brief    Transmit Underrun Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_TUR_ERR_TURERR_POS             0 /**< TUR_ERR_TURERR Position */
 #define MXC_F_EMAC_REVA_TUR_ERR_TURERR                 ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_TUR_ERR_TURERR_POS)) /**< TUR_ERR_TURERR Mask */

/**@} end of group EMAC_REVA_TUR_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_CS_ERR EMAC_REVA_CS_ERR
 * @brief    Carrier Sense Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_CS_ERR_CSERR_POS               0 /**< CS_ERR_CSERR Position */
 #define MXC_F_EMAC_REVA_CS_ERR_CSERR                   ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_CS_ERR_CSERR_POS)) /**< CS_ERR_CSERR Mask */

/**@} end of group EMAC_REVA_CS_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_RR_ERR EMAC_REVA_RR_ERR
 * @brief    Receive Resource Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_RR_ERR_RRERR_POS               0 /**< RR_ERR_RRERR Position */
 #define MXC_F_EMAC_REVA_RR_ERR_RRERR                   ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_RR_ERR_RRERR_POS)) /**< RR_ERR_RRERR Mask */

/**@} end of group EMAC_REVA_RR_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_ROR_ERR EMAC_REVA_ROR_ERR
 * @brief    Receive Overrun Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_ROR_ERR_RORERR_POS             0 /**< ROR_ERR_RORERR Position */
 #define MXC_F_EMAC_REVA_ROR_ERR_RORERR                 ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_ROR_ERR_RORERR_POS)) /**< ROR_ERR_RORERR Mask */

/**@} end of group EMAC_REVA_ROR_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_RS_ERR EMAC_REVA_RS_ERR
 * @brief    Receive Symbol Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_RS_ERR_RSERR_POS               0 /**< RS_ERR_RSERR Position */
 #define MXC_F_EMAC_REVA_RS_ERR_RSERR                   ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_RS_ERR_RSERR_POS)) /**< RS_ERR_RSERR Mask */

/**@} end of group EMAC_REVA_RS_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_EL_ERR EMAC_REVA_EL_ERR
 * @brief    Excessive Length Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_EL_ERR_ELERR_POS               0 /**< EL_ERR_ELERR Position */
 #define MXC_F_EMAC_REVA_EL_ERR_ELERR                   ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_EL_ERR_ELERR_POS)) /**< EL_ERR_ELERR Mask */

/**@} end of group EMAC_REVA_EL_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_RJ EMAC_REVA_RJ
 * @brief    Receive Jabber.
 * @{
 */
 #define MXC_F_EMAC_REVA_RJ_RJERR_POS                   0 /**< RJ_RJERR Position */
 #define MXC_F_EMAC_REVA_RJ_RJERR                       ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_RJ_RJERR_POS)) /**< RJ_RJERR Mask */

/**@} end of group EMAC_REVA_RJ_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_USF EMAC_REVA_USF
 * @brief    Undersize Frames.
 * @{
 */
 #define MXC_F_EMAC_REVA_USF_USF_POS                    0 /**< USF_USF Position */
 #define MXC_F_EMAC_REVA_USF_USF                        ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_USF_USF_POS)) /**< USF_USF Mask */

/**@} end of group EMAC_REVA_USF_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SQE_ERR EMAC_REVA_SQE_ERR
 * @brief    SQE Test Errors.
 * @{
 */
 #define MXC_F_EMAC_REVA_SQE_ERR_SQEERR_POS             0 /**< SQE_ERR_SQEERR Position */
 #define MXC_F_EMAC_REVA_SQE_ERR_SQEERR                 ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_SQE_ERR_SQEERR_POS)) /**< SQE_ERR_SQEERR Mask */

/**@} end of group EMAC_REVA_SQE_ERR_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_RLFM EMAC_REVA_RLFM
 * @brief    Received Length Field Mismatch.
 * @{
 */
 #define MXC_F_EMAC_REVA_RLFM_RLFM_POS                  0 /**< RLFM_RLFM Position */
 #define MXC_F_EMAC_REVA_RLFM_RLFM                      ((uint32_t)(0xFFUL << MXC_F_EMAC_REVA_RLFM_RLFM_POS)) /**< RLFM_RLFM Mask */

/**@} end of group EMAC_REVA_RLFM_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_TPF EMAC_REVA_TPF
 * @brief    Transmitted Pause Frames.
 * @{
 */
 #define MXC_F_EMAC_REVA_TPF_TPF_POS                    0 /**< TPF_TPF Position */
 #define MXC_F_EMAC_REVA_TPF_TPF                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_TPF_TPF_POS)) /**< TPF_TPF Mask */

/**@} end of group EMAC_REVA_TPF_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_HASHL EMAC_REVA_HASHL
 * @brief    Hash Register Bottom [31:0].
 * @{
 */
 #define MXC_F_EMAC_REVA_HASHL_HASH_POS                 0 /**< HASHL_HASH Position */
 #define MXC_F_EMAC_REVA_HASHL_HASH                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_HASHL_HASH_POS)) /**< HASHL_HASH Mask */

/**@} end of group EMAC_REVA_HASHL_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_HASHH EMAC_REVA_HASHH
 * @brief    Hash Register top [63:32].
 * @{
 */
 #define MXC_F_EMAC_REVA_HASHH_HASH_POS                 0 /**< HASHH_HASH Position */
 #define MXC_F_EMAC_REVA_HASHH_HASH                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_HASHH_HASH_POS)) /**< HASHH_HASH Mask */

/**@} end of group EMAC_REVA_HASHH_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA1L EMAC_REVA_SA1L
 * @brief    Specific Address 1 Bottom.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA1L_ADDR_POS                  0 /**< SA1L_ADDR Position */
 #define MXC_F_EMAC_REVA_SA1L_ADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_SA1L_ADDR_POS)) /**< SA1L_ADDR Mask */

/**@} end of group EMAC_REVA_SA1L_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA1H EMAC_REVA_SA1H
 * @brief    Specific Address 1 Top.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA1H_ADDR_POS                  0 /**< SA1H_ADDR Position */
 #define MXC_F_EMAC_REVA_SA1H_ADDR                      ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_SA1H_ADDR_POS)) /**< SA1H_ADDR Mask */

/**@} end of group EMAC_REVA_SA1H_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA2L EMAC_REVA_SA2L
 * @brief    Specific Address 2 Bottom.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA2L_ADDR_POS                  0 /**< SA2L_ADDR Position */
 #define MXC_F_EMAC_REVA_SA2L_ADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_SA2L_ADDR_POS)) /**< SA2L_ADDR Mask */

/**@} end of group EMAC_REVA_SA2L_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA2H EMAC_REVA_SA2H
 * @brief    Specific Address 2 Top.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA2H_ADDR_POS                  0 /**< SA2H_ADDR Position */
 #define MXC_F_EMAC_REVA_SA2H_ADDR                      ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_SA2H_ADDR_POS)) /**< SA2H_ADDR Mask */

/**@} end of group EMAC_REVA_SA2H_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA3L EMAC_REVA_SA3L
 * @brief    Specific Address 3 Bottom.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA3L_ADDR_POS                  0 /**< SA3L_ADDR Position */
 #define MXC_F_EMAC_REVA_SA3L_ADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_SA3L_ADDR_POS)) /**< SA3L_ADDR Mask */

/**@} end of group EMAC_REVA_SA3L_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA3H EMAC_REVA_SA3H
 * @brief    Specific Address 3 Top.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA3H_ADDR_POS                  0 /**< SA3H_ADDR Position */
 #define MXC_F_EMAC_REVA_SA3H_ADDR                      ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_SA3H_ADDR_POS)) /**< SA3H_ADDR Mask */

/**@} end of group EMAC_REVA_SA3H_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA4L EMAC_REVA_SA4L
 * @brief    Specific Address 4 Bottom.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA4L_ADDR_POS                  0 /**< SA4L_ADDR Position */
 #define MXC_F_EMAC_REVA_SA4L_ADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMAC_REVA_SA4L_ADDR_POS)) /**< SA4L_ADDR Mask */

/**@} end of group EMAC_REVA_SA4L_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_SA4H EMAC_REVA_SA4H
 * @brief    Specific Address 4 Top.
 * @{
 */
 #define MXC_F_EMAC_REVA_SA4H_ADDR_POS                  0 /**< SA4H_ADDR Position */
 #define MXC_F_EMAC_REVA_SA4H_ADDR                      ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_SA4H_ADDR_POS)) /**< SA4H_ADDR Mask */

/**@} end of group EMAC_REVA_SA4H_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_TID_CK EMAC_REVA_TID_CK
 * @brief    Type ID Checking.
 * @{
 */
 #define MXC_F_EMAC_REVA_TID_CK_TID_POS                 0 /**< TID_CK_TID Position */
 #define MXC_F_EMAC_REVA_TID_CK_TID                     ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_TID_CK_TID_POS)) /**< TID_CK_TID Mask */

/**@} end of group EMAC_REVA_TID_CK_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_TPQ EMAC_REVA_TPQ
 * @brief    Transmit Pause Quantum.
 * @{
 */
 #define MXC_F_EMAC_REVA_TPQ_TPQ_POS                    0 /**< TPQ_TPQ Position */
 #define MXC_F_EMAC_REVA_TPQ_TPQ                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_TPQ_TPQ_POS)) /**< TPQ_TPQ Mask */

/**@} end of group EMAC_REVA_TPQ_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_USRIO EMAC_REVA_USRIO
 * @brief    User Input Output Register
 * @{
 */
 #define MXC_F_EMAC_REVA_USRIO_MII_POS                  0 /**< USRIO_MII Position */
 #define MXC_F_EMAC_REVA_USRIO_MII                      ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_USRIO_MII_POS)) /**< USRIO_MII Mask */

/**@} end of group EMAC_REVA_USRIO_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_WOL EMAC_REVA_WOL
 * @brief    Wake On LAN Register
 * @{
 */
 #define MXC_F_EMAC_REVA_WOL_IP_POS                     0 /**< WOL_IP Position */
 #define MXC_F_EMAC_REVA_WOL_IP                         ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_WOL_IP_POS)) /**< WOL_IP Mask */

 #define MXC_F_EMAC_REVA_WOL_MAG_POS                    16 /**< WOL_MAG Position */
 #define MXC_F_EMAC_REVA_WOL_MAG                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_WOL_MAG_POS)) /**< WOL_MAG Mask */

 #define MXC_F_EMAC_REVA_WOL_ARP_POS                    17 /**< WOL_ARP Position */
 #define MXC_F_EMAC_REVA_WOL_ARP                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_WOL_ARP_POS)) /**< WOL_ARP Mask */

 #define MXC_F_EMAC_REVA_WOL_SA1_POS                    18 /**< WOL_SA1 Position */
 #define MXC_F_EMAC_REVA_WOL_SA1                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_WOL_SA1_POS)) /**< WOL_SA1 Mask */

 #define MXC_F_EMAC_REVA_WOL_MTI_POS                    19 /**< WOL_MTI Position */
 #define MXC_F_EMAC_REVA_WOL_MTI                        ((uint32_t)(0x1UL << MXC_F_EMAC_REVA_WOL_MTI_POS)) /**< WOL_MTI Mask */

/**@} end of group EMAC_REVA_WOL_Register */

/**
 * @ingroup  emac_reva_registers
 * @defgroup EMAC_REVA_REV EMAC_REVA_REV
 * @brief    Revision register.
 * @{
 */
 #define MXC_F_EMAC_REVA_REV_REV_POS                    0 /**< REV_REV Position */
 #define MXC_F_EMAC_REVA_REV_REV                        ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_REV_REV_POS)) /**< REV_REV Mask */

 #define MXC_F_EMAC_REVA_REV_PART_POS                   16 /**< REV_PART Position */
 #define MXC_F_EMAC_REVA_REV_PART                       ((uint32_t)(0xFFFFUL << MXC_F_EMAC_REVA_REV_PART_POS)) /**< REV_PART Mask */

/**@} end of group EMAC_REVA_REV_Register */

#ifdef __cplusplus
}
#endif

#endif /* _EMAC_REVA_REGS_H_ */

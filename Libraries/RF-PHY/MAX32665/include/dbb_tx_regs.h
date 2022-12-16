/* *****************************************************************************
 * Copyright (C) Analog Devices, All rights Reserved.
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
 **************************************************************************** */

/**
 * @file    dbb_tx_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the DBB_TX Peripheral Module.
 */

#ifndef MAX32665_INCLUDE_DBB_TX_REGS_H_
#define MAX32665_INCLUDE_DBB_TX_REGS_H_

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

#if defined (__ICCARM__)
#pragma system_include
#endif

#if defined (__CC_ARM)
#pragma anon_unions
#endif
/*/ @cond */
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
/*/ @endcond */

/* **** Definitions **** */

/**
 * @ingroup     dbb_tx
 * @defgroup    dbb_tx_registers DBB_TX_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the DBB_TX Peripheral Module.
 * @details PAN2G TX Config
 */

/**
 * @ingroup dbb_tx_registers
 * Structure type to access the DBB_TX Registers.
 */
typedef struct {
    __IO uint32_t tx_info_version;                /**< <tt>\b 0x0000:</tt> DBB_TX TX_INFO_VERSION Register */
    __IO uint16_t tx_general_standard;            /**< <tt>\b 0x0004:</tt> DBB_TX TX_GENERAL_STANDARD Register */
    __IO uint16_t tx_general_control;             /**< <tt>\b 0x0006:</tt> DBB_TX TX_GENERAL_CONTROL Register */
    __IO uint32_t tx_general_send_multi;          /**< <tt>\b 0x0008:</tt> DBB_TX TX_GENERAL_SEND_MULTI Register */
    __IO uint16_t tx_phy_high_freq_coef;          /**< <tt>\b 0x000c:</tt> DBB_TX TX_PHY_HIGH_FREQ_COEF Register */
    __IO uint16_t tx_phy_low_freq_coef;           /**< <tt>\b 0x000e:</tt> DBB_TX TX_PHY_LOW_FREQ_COEF Register */
    __IO uint32_t tx_phy_freq_carrier;            /**< <tt>\b 0x0010:</tt> DBB_TX TX_PHY_FREQ_CARRIER Register */
    __IO uint16_t tx_phy_amp_coef;                /**< <tt>\b 0x0014:</tt> DBB_TX TX_PHY_AMP_COEF Register */
    __IO uint16_t tx_phy_filt;                    /**< <tt>\b 0x0016:</tt> DBB_TX TX_PHY_FILT Register */
    __IO uint16_t tx_phy_filt_coefs[16];          /**< <tt>\b 0x0018:</tt> DBB_TX TX_PHY_FILT_COEFS Register */
    __IO uint16_t tx_phy_freq_offs;               /**< <tt>\b 0x0038:</tt> DBB_TX TX_PHY_FREQ_OFFS Register */
    __IO uint16_t tx_phy_ovrsamp_ratio;           /**< <tt>\b 0x003a:</tt> DBB_TX TX_PHY_OVRSAMP_RATIO Register */
    __IO uint16_t tx_phy_pattern_gen;             /**< <tt>\b 0x003c:</tt> DBB_TX TX_PHY_PATTERN_GEN Register */
    __IO uint16_t tx_phy_pattern;                 /**< <tt>\b 0x003e:</tt> DBB_TX TX_PHY_PATTERN Register */
    __IO uint16_t tx_phy_const_output_enable;     /**< <tt>\b 0x0040:</tt> DBB_TX TX_PHY_CONST_OUTPUT_ENABLE Register */
    __IO uint16_t tx_phy_const_output_amp;        /**< <tt>\b 0x0042:</tt> DBB_TX TX_PHY_CONST_OUTPUT_AMP Register */
    __IO uint32_t tx_phy_const_output_freq_high;  /**< <tt>\b 0x0044:</tt> DBB_TX TX_PHY_CONST_OUTPUT_FREQ_HIGH Register */
    __IO uint32_t tx_phy_const_output_freq_low;   /**< <tt>\b 0x0048:</tt> DBB_TX TX_PHY_CONST_OUTPUT_FREQ_LOW Register */
    __IO uint32_t tx_dbg_dl_lb;                   /**< <tt>\b 0x004c:</tt> DBB_TX TX_DBG_DL_LB Register */
    __IO uint16_t tx_dl_bypass;                   /**< <tt>\b 0x0050:</tt> DBB_TX TX_DL_BYPASS Register */
    __IO uint16_t tx_dl_crc_mode;                 /**< <tt>\b 0x0052:</tt> DBB_TX TX_DL_CRC_MODE Register */
    __IO uint32_t tx_dl_crc_init_phr;             /**< <tt>\b 0x0054:</tt> DBB_TX TX_DL_CRC_INIT_PHR Register */
    __IO uint32_t tx_dl_crc_init_pld;             /**< <tt>\b 0x0058:</tt> DBB_TX TX_DL_CRC_INIT_PLD Register */
    __IO uint16_t tx_dl_bch;                      /**< <tt>\b 0x005c:</tt> DBB_TX TX_DL_BCH Register */
    __IO uint16_t tx_dl_ban_settings;             /**< <tt>\b 0x005e:</tt> DBB_TX TX_DL_BAN_SETTINGS Register */
    __IO uint32_t tx_dl_btle_settings_adv_acc_addr; /**< <tt>\b 0x0060:</tt> DBB_TX TX_DL_BTLE_SETTINGS_ADV_ACC_ADDR Register */
    __IO uint32_t tx_dl_btle_settings_acc_addr;   /**< <tt>\b 0x0064:</tt> DBB_TX TX_DL_BTLE_SETTINGS_ACC_ADDR Register */
    __IO uint16_t tx_dl_btle_settings_whit;       /**< <tt>\b 0x0068:</tt> DBB_TX TX_DL_BTLE_SETTINGS_WHIT Register */
    __IO uint16_t tx_dl_btle_longrange;           /**< <tt>\b 0x006a:</tt> DBB_TX TX_DL_BTLE_LONGRANGE Register */
    __IO uint16_t tx_dl_phr_modulation;           /**< <tt>\b 0x006c:</tt> DBB_TX TX_DL_PHR_MODULATION Register */
    __IO uint16_t tx_dl_phr;                      /**< <tt>\b 0x006e:</tt> DBB_TX TX_DL_PHR Register */
    __IO uint8_t  tx_pld_mem[264];                /**< <tt>\b 0x0070:</tt> DBB_TX TX_PLD_MEM Register */
    __IO uint16_t tx_dl2_btle_speed_mode;         /**< <tt>\b 0x0178:</tt> DBB_TX TX_DL2_BTLE_SPEED_MODE Register */
} mxc_dbb_tx_regs_t;

/* Register offsets for module DBB_TX */
/**
 * @ingroup    dbb_tx_registers
 * @defgroup   DBB_TX_Register_Offsets Register Offsets
 * @brief      DBB_TX Peripheral Register Offsets from the DBB_TX Base Peripheral Address.
 * @{
 */
#define MXC_R_DBB_TX_TX_INFO_VERSION                 ((uint32_t)0x00000000UL) /**< Offset from DBB_TX Base Address: <tt> 0x0000</tt> */
#define MXC_R_DBB_TX_TX_GENERAL_STANDARD             ((uint32_t)0x00000004UL) /**< Offset from DBB_TX Base Address: <tt> 0x0004</tt> */
#define MXC_R_DBB_TX_TX_GENERAL_CONTROL              ((uint32_t)0x00000006UL) /**< Offset from DBB_TX Base Address: <tt> 0x0006</tt> */
#define MXC_R_DBB_TX_TX_GENERAL_SEND_MULTI           ((uint32_t)0x00000008UL) /**< Offset from DBB_TX Base Address: <tt> 0x0008</tt> */
#define MXC_R_DBB_TX_TX_PHY_HIGH_FREQ_COEF           ((uint32_t)0x0000000CUL) /**< Offset from DBB_TX Base Address: <tt> 0x000C</tt> */
#define MXC_R_DBB_TX_TX_PHY_LOW_FREQ_COEF            ((uint32_t)0x0000000EUL) /**< Offset from DBB_TX Base Address: <tt> 0x000E</tt> */
#define MXC_R_DBB_TX_TX_PHY_FREQ_CARRIER             ((uint32_t)0x00000010UL) /**< Offset from DBB_TX Base Address: <tt> 0x0010</tt> */
#define MXC_R_DBB_TX_TX_PHY_AMP_COEF                 ((uint32_t)0x00000014UL) /**< Offset from DBB_TX Base Address: <tt> 0x0014</tt> */
#define MXC_R_DBB_TX_TX_PHY_FILT                     ((uint32_t)0x00000016UL) /**< Offset from DBB_TX Base Address: <tt> 0x0016</tt> */
#define MXC_R_DBB_TX_TX_PHY_FILT_COEFS               ((uint32_t)0x00000018UL) /**< Offset from DBB_TX Base Address: <tt> 0x0018</tt> */
#define MXC_R_DBB_TX_TX_PHY_FREQ_OFFS                ((uint32_t)0x00000038UL) /**< Offset from DBB_TX Base Address: <tt> 0x0038</tt> */
#define MXC_R_DBB_TX_TX_PHY_OVRSAMP_RATIO            ((uint32_t)0x0000003AUL) /**< Offset from DBB_TX Base Address: <tt> 0x003A</tt> */
#define MXC_R_DBB_TX_TX_PHY_PATTERN_GEN              ((uint32_t)0x0000003CUL) /**< Offset from DBB_TX Base Address: <tt> 0x003C</tt> */
#define MXC_R_DBB_TX_TX_PHY_PATTERN                  ((uint32_t)0x0000003EUL) /**< Offset from DBB_TX Base Address: <tt> 0x003E</tt> */
#define MXC_R_DBB_TX_TX_PHY_CONST_OUTPUT_ENABLE      ((uint32_t)0x00000040UL) /**< Offset from DBB_TX Base Address: <tt> 0x0040</tt> */
#define MXC_R_DBB_TX_TX_PHY_CONST_OUTPUT_AMP         ((uint32_t)0x00000042UL) /**< Offset from DBB_TX Base Address: <tt> 0x0042</tt> */
#define MXC_R_DBB_TX_TX_PHY_CONST_OUTPUT_FREQ_HIGH   ((uint32_t)0x00000044UL) /**< Offset from DBB_TX Base Address: <tt> 0x0044</tt> */
#define MXC_R_DBB_TX_TX_PHY_CONST_OUTPUT_FREQ_LOW    ((uint32_t)0x00000048UL) /**< Offset from DBB_TX Base Address: <tt> 0x0048</tt> */
#define MXC_R_DBB_TX_TX_DBG_DL_LB                    ((uint32_t)0x0000004CUL) /**< Offset from DBB_TX Base Address: <tt> 0x004C</tt> */
#define MXC_R_DBB_TX_TX_DL_BYPASS                    ((uint32_t)0x00000050UL) /**< Offset from DBB_TX Base Address: <tt> 0x0050</tt> */
#define MXC_R_DBB_TX_TX_DL_CRC_MODE                  ((uint32_t)0x00000052UL) /**< Offset from DBB_TX Base Address: <tt> 0x0052</tt> */
#define MXC_R_DBB_TX_TX_DL_CRC_INIT_PHR              ((uint32_t)0x00000054UL) /**< Offset from DBB_TX Base Address: <tt> 0x0054</tt> */
#define MXC_R_DBB_TX_TX_DL_CRC_INIT_PLD              ((uint32_t)0x00000058UL) /**< Offset from DBB_TX Base Address: <tt> 0x0058</tt> */
#define MXC_R_DBB_TX_TX_DL_BCH                       ((uint32_t)0x0000005CUL) /**< Offset from DBB_TX Base Address: <tt> 0x005C</tt> */
#define MXC_R_DBB_TX_TX_DL_BAN_SETTINGS              ((uint32_t)0x0000005EUL) /**< Offset from DBB_TX Base Address: <tt> 0x005E</tt> */
#define MXC_R_DBB_TX_TX_DL_BTLE_SETTINGS_ADV_ACC_ADDR ((uint32_t)0x00000060UL) /**< Offset from DBB_TX Base Address: <tt> 0x0060</tt> */
#define MXC_R_DBB_TX_TX_DL_BTLE_SETTINGS_ACC_ADDR    ((uint32_t)0x00000064UL) /**< Offset from DBB_TX Base Address: <tt> 0x0064</tt> */
#define MXC_R_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT        ((uint32_t)0x00000068UL) /**< Offset from DBB_TX Base Address: <tt> 0x0068</tt> */
#define MXC_R_DBB_TX_TX_DL_BTLE_LONGRANGE            ((uint32_t)0x0000006AUL) /**< Offset from DBB_TX Base Address: <tt> 0x006A</tt> */
#define MXC_R_DBB_TX_TX_DL_PHR_MODULATION            ((uint32_t)0x0000006CUL) /**< Offset from DBB_TX Base Address: <tt> 0x006C</tt> */
#define MXC_R_DBB_TX_TX_DL_PHR                       ((uint32_t)0x0000006EUL) /**< Offset from DBB_TX Base Address: <tt> 0x006E</tt> */
#define MXC_R_DBB_TX_TX_PLD_MEM                      ((uint32_t)0x00000070UL) /**< Offset from DBB_TX Base Address: <tt> 0x0070</tt> */
#define MXC_R_DBB_TX_TX_DL2_BTLE_SPEED_MODE          ((uint32_t)0x00000178UL) /**< Offset from DBB_TX Base Address: <tt> 0x0178</tt> */
/**@} end of group dbb_tx_registers */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_INFO_VERSION DBB_TX_TX_INFO_VERSION
 * @brief    version
 * @{
 */
#define MXC_F_DBB_TX_TX_INFO_VERSION_MINOR_POS         0 /**< TX_INFO_VERSION_MINOR Position */
#define MXC_F_DBB_TX_TX_INFO_VERSION_MINOR             ((uint32_t)(0xFFUL << MXC_F_DBB_TX_TX_INFO_VERSION_MINOR_POS)) /**< TX_INFO_VERSION_MINOR Mask */

#define MXC_F_DBB_TX_TX_INFO_VERSION_MAJOR_POS         8 /**< TX_INFO_VERSION_MAJOR Position */
#define MXC_F_DBB_TX_TX_INFO_VERSION_MAJOR             ((uint32_t)(0xFFUL << MXC_F_DBB_TX_TX_INFO_VERSION_MAJOR_POS)) /**< TX_INFO_VERSION_MAJOR Mask */

/**@} end of group DBB_TX_TX_INFO_VERSION_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_GENERAL_STANDARD DBB_TX_TX_GENERAL_STANDARD
 * @brief    Standard: 1 = Zigbee (15.4), 2 = BAN (15.6),  3 = Bluetooth Low Energy (BTLE)
 * @{
 */
#define MXC_F_DBB_TX_TX_GENERAL_STANDARD_STANDARD_POS  0 /**< TX_GENERAL_STANDARD_STANDARD Position */
#define MXC_F_DBB_TX_TX_GENERAL_STANDARD_STANDARD      ((uint16_t)(0x3UL << MXC_F_DBB_TX_TX_GENERAL_STANDARD_STANDARD_POS)) /**< TX_GENERAL_STANDARD_STANDARD Mask */
#define MXC_V_DBB_TX_TX_GENERAL_STANDARD_STANDARD_ZIGBEE ((uint16_t)0x1UL) /**< TX_GENERAL_STANDARD_STANDARD_ZIGBEE Value */
#define MXC_S_DBB_TX_TX_GENERAL_STANDARD_STANDARD_ZIGBEE (MXC_V_DBB_TX_TX_GENERAL_STANDARD_STANDARD_ZIGBEE << MXC_F_DBB_TX_TX_GENERAL_STANDARD_STANDARD_POS) /**< TX_GENERAL_STANDARD_STANDARD_ZIGBEE Setting */
#define MXC_V_DBB_TX_TX_GENERAL_STANDARD_STANDARD_BAN  ((uint16_t)0x2UL) /**< TX_GENERAL_STANDARD_STANDARD_BAN Value */
#define MXC_S_DBB_TX_TX_GENERAL_STANDARD_STANDARD_BAN  (MXC_V_DBB_TX_TX_GENERAL_STANDARD_STANDARD_BAN << MXC_F_DBB_TX_TX_GENERAL_STANDARD_STANDARD_POS) /**< TX_GENERAL_STANDARD_STANDARD_BAN Setting */
#define MXC_V_DBB_TX_TX_GENERAL_STANDARD_STANDARD_BTLE ((uint16_t)0x3UL) /**< TX_GENERAL_STANDARD_STANDARD_BTLE Value */
#define MXC_S_DBB_TX_TX_GENERAL_STANDARD_STANDARD_BTLE (MXC_V_DBB_TX_TX_GENERAL_STANDARD_STANDARD_BTLE << MXC_F_DBB_TX_TX_GENERAL_STANDARD_STANDARD_POS) /**< TX_GENERAL_STANDARD_STANDARD_BTLE Setting */

/**@} end of group DBB_TX_TX_GENERAL_STANDARD_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_GENERAL_CONTROL DBB_TX_TX_GENERAL_CONTROL
 * @brief    control
 * @{
 */
#define MXC_F_DBB_TX_TX_GENERAL_CONTROL_PHY_ENABLE_POS 0 /**< TX_GENERAL_CONTROL_PHY_ENABLE Position */
#define MXC_F_DBB_TX_TX_GENERAL_CONTROL_PHY_ENABLE     ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_GENERAL_CONTROL_PHY_ENABLE_POS)) /**< TX_GENERAL_CONTROL_PHY_ENABLE Mask */

#define MXC_F_DBB_TX_TX_GENERAL_CONTROL_DL_ENABLE_POS  1 /**< TX_GENERAL_CONTROL_DL_ENABLE Position */
#define MXC_F_DBB_TX_TX_GENERAL_CONTROL_DL_ENABLE      ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_GENERAL_CONTROL_DL_ENABLE_POS)) /**< TX_GENERAL_CONTROL_DL_ENABLE Mask */

#define MXC_F_DBB_TX_TX_GENERAL_CONTROL_ENCRYPTION_ENABLE_POS 4 /**< TX_GENERAL_CONTROL_ENCRYPTION_ENABLE Position */
#define MXC_F_DBB_TX_TX_GENERAL_CONTROL_ENCRYPTION_ENABLE ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_GENERAL_CONTROL_ENCRYPTION_ENABLE_POS)) /**< TX_GENERAL_CONTROL_ENCRYPTION_ENABLE Mask */

/**@} end of group DBB_TX_TX_GENERAL_CONTROL_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_PHY_FILT DBB_TX_TX_PHY_FILT
 * @brief    filt
 * @{
 */
#define MXC_F_DBB_TX_TX_PHY_FILT_SKIP_MUX_SEL_POS      0 /**< TX_PHY_FILT_SKIP_MUX_SEL Position */
#define MXC_F_DBB_TX_TX_PHY_FILT_SKIP_MUX_SEL          ((uint16_t)(0x3FUL << MXC_F_DBB_TX_TX_PHY_FILT_SKIP_MUX_SEL_POS)) /**< TX_PHY_FILT_SKIP_MUX_SEL Mask */

#define MXC_F_DBB_TX_TX_PHY_FILT_SCALING_FACTOR_POS    6 /**< TX_PHY_FILT_SCALING_FACTOR Position */
#define MXC_F_DBB_TX_TX_PHY_FILT_SCALING_FACTOR        ((uint16_t)(0xFUL << MXC_F_DBB_TX_TX_PHY_FILT_SCALING_FACTOR_POS)) /**< TX_PHY_FILT_SCALING_FACTOR Mask */

/**@} end of group DBB_TX_TX_PHY_FILT_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_PHY_FREQ_OFFS DBB_TX_TX_PHY_FREQ_OFFS
 * @brief    freq_offs
 * @{
 */
#define MXC_F_DBB_TX_TX_PHY_FREQ_OFFS_START_POS        0 /**< TX_PHY_FREQ_OFFS_START Position */
#define MXC_F_DBB_TX_TX_PHY_FREQ_OFFS_START            ((uint16_t)(0x1FUL << MXC_F_DBB_TX_TX_PHY_FREQ_OFFS_START_POS)) /**< TX_PHY_FREQ_OFFS_START Mask */

#define MXC_F_DBB_TX_TX_PHY_FREQ_OFFS_END_POS          5 /**< TX_PHY_FREQ_OFFS_END Position */
#define MXC_F_DBB_TX_TX_PHY_FREQ_OFFS_END              ((uint16_t)(0x1FUL << MXC_F_DBB_TX_TX_PHY_FREQ_OFFS_END_POS)) /**< TX_PHY_FREQ_OFFS_END Mask */

/**@} end of group DBB_TX_TX_PHY_FREQ_OFFS_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_PHY_PATTERN_GEN DBB_TX_TX_PHY_PATTERN_GEN
 * @brief    pattern_gen
 * @{
 */
#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_EN_POS         0 /**< TX_PHY_PATTERN_GEN_EN Position */
#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_EN             ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_EN_POS)) /**< TX_PHY_PATTERN_GEN_EN Mask */

#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_RST_POS        1 /**< TX_PHY_PATTERN_GEN_RST Position */
#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_RST            ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_RST_POS)) /**< TX_PHY_PATTERN_GEN_RST Mask */

#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_MODE_POS       2 /**< TX_PHY_PATTERN_GEN_MODE Position */
#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_MODE           ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_MODE_POS)) /**< TX_PHY_PATTERN_GEN_MODE Mask */
#define MXC_V_DBB_TX_TX_PHY_PATTERN_GEN_MODE_PRBS      ((uint16_t)0x0UL) /**< TX_PHY_PATTERN_GEN_MODE_PRBS Value */
#define MXC_S_DBB_TX_TX_PHY_PATTERN_GEN_MODE_PRBS      (MXC_V_DBB_TX_TX_PHY_PATTERN_GEN_MODE_PRBS << MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_MODE_POS) /**< TX_PHY_PATTERN_GEN_MODE_PRBS Setting */
#define MXC_V_DBB_TX_TX_PHY_PATTERN_GEN_MODE_USER      ((uint16_t)0x1UL) /**< TX_PHY_PATTERN_GEN_MODE_USER Value */
#define MXC_S_DBB_TX_TX_PHY_PATTERN_GEN_MODE_USER      (MXC_V_DBB_TX_TX_PHY_PATTERN_GEN_MODE_USER << MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_MODE_POS) /**< TX_PHY_PATTERN_GEN_MODE_USER Setting */

#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_LEN_POS        3 /**< TX_PHY_PATTERN_GEN_LEN Position */
#define MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_LEN            ((uint16_t)(0x1FUL << MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_LEN_POS)) /**< TX_PHY_PATTERN_GEN_LEN Mask */

/**@} end of group DBB_TX_TX_PHY_PATTERN_GEN_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL_CRC_MODE DBB_TX_TX_DL_CRC_MODE
 * @brief    crc_mode
 * @{
 */
#define MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR_POS            0 /**< TX_DL_CRC_MODE_PHR Position */
#define MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR                ((uint16_t)(0x3UL << MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR_POS)) /**< TX_DL_CRC_MODE_PHR Mask */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_BYP            ((uint16_t)0x0UL) /**< TX_DL_CRC_MODE_PHR_BYP Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PHR_BYP            (MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_BYP << MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR_POS) /**< TX_DL_CRC_MODE_PHR_BYP Setting */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_CRC4           ((uint16_t)0x1UL) /**< TX_DL_CRC_MODE_PHR_CRC4 Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PHR_CRC4           (MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_CRC4 << MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR_POS) /**< TX_DL_CRC_MODE_PHR_CRC4 Setting */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_CRC16          ((uint16_t)0x2UL) /**< TX_DL_CRC_MODE_PHR_CRC16 Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PHR_CRC16          (MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_CRC16 << MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR_POS) /**< TX_DL_CRC_MODE_PHR_CRC16 Setting */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_CRC24          ((uint16_t)0x3UL) /**< TX_DL_CRC_MODE_PHR_CRC24 Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PHR_CRC24          (MXC_V_DBB_TX_TX_DL_CRC_MODE_PHR_CRC24 << MXC_F_DBB_TX_TX_DL_CRC_MODE_PHR_POS) /**< TX_DL_CRC_MODE_PHR_CRC24 Setting */

#define MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD_POS            2 /**< TX_DL_CRC_MODE_PLD Position */
#define MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD                ((uint16_t)(0x3UL << MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD_POS)) /**< TX_DL_CRC_MODE_PLD Mask */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_BYP            ((uint16_t)0x0UL) /**< TX_DL_CRC_MODE_PLD_BYP Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PLD_BYP            (MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_BYP << MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD_POS) /**< TX_DL_CRC_MODE_PLD_BYP Setting */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_CRC4           ((uint16_t)0x1UL) /**< TX_DL_CRC_MODE_PLD_CRC4 Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PLD_CRC4           (MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_CRC4 << MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD_POS) /**< TX_DL_CRC_MODE_PLD_CRC4 Setting */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_CRC16          ((uint16_t)0x2UL) /**< TX_DL_CRC_MODE_PLD_CRC16 Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PLD_CRC16          (MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_CRC16 << MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD_POS) /**< TX_DL_CRC_MODE_PLD_CRC16 Setting */
#define MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_CRC24          ((uint16_t)0x3UL) /**< TX_DL_CRC_MODE_PLD_CRC24 Value */
#define MXC_S_DBB_TX_TX_DL_CRC_MODE_PLD_CRC24          (MXC_V_DBB_TX_TX_DL_CRC_MODE_PLD_CRC24 << MXC_F_DBB_TX_TX_DL_CRC_MODE_PLD_POS) /**< TX_DL_CRC_MODE_PLD_CRC24 Setting */

/**@} end of group DBB_TX_TX_DL_CRC_MODE_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL_BCH DBB_TX_TX_DL_BCH
 * @brief    bch
 * @{
 */
#define MXC_F_DBB_TX_TX_DL_BCH_PHR_ENABLE_POS          0 /**< TX_DL_BCH_PHR_ENABLE Position */
#define MXC_F_DBB_TX_TX_DL_BCH_PHR_ENABLE              ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BCH_PHR_ENABLE_POS)) /**< TX_DL_BCH_PHR_ENABLE Mask */

#define MXC_F_DBB_TX_TX_DL_BCH_PLD_ENABLE_POS          1 /**< TX_DL_BCH_PLD_ENABLE Position */
#define MXC_F_DBB_TX_TX_DL_BCH_PLD_ENABLE              ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BCH_PLD_ENABLE_POS)) /**< TX_DL_BCH_PLD_ENABLE Mask */

/**@} end of group DBB_TX_TX_DL_BCH_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL_BAN_SETTINGS DBB_TX_TX_DL_BAN_SETTINGS
 * @brief    ban_settings
 * @{
 */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SCRMBL_SD_POS 0 /**< TX_DL_BAN_SETTINGS_PHR_SCRMBL_SD Position */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SCRMBL_SD  ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SCRMBL_SD_POS)) /**< TX_DL_BAN_SETTINGS_PHR_SCRMBL_SD Mask */

#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS 1 /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT Position */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT  ((uint16_t)(0x7UL << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS)) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT Mask */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 ((uint16_t)0x1UL) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 ((uint16_t)0x2UL) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 ((uint16_t)0x3UL) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 ((uint16_t)0x4UL) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 ((uint16_t)0x5UL) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 Setting */

#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_MDR_LENGTH_POS 4 /**< TX_DL_BAN_SETTINGS_MDR_LENGTH Position */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_MDR_LENGTH     ((uint16_t)(0xFUL << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_MDR_LENGTH_POS)) /**< TX_DL_BAN_SETTINGS_MDR_LENGTH Mask */

#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PREAMBLE_NBR_POS 8 /**< TX_DL_BAN_SETTINGS_PREAMBLE_NBR Position */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PREAMBLE_NBR   ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PREAMBLE_NBR_POS)) /**< TX_DL_BAN_SETTINGS_PREAMBLE_NBR Mask */

#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS 9 /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD Position */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD   ((uint16_t)(0x7UL << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS)) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD Mask */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK ((uint16_t)0x0UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK ((uint16_t)0x1UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK ((uint16_t)0x2UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK ((uint16_t)0x3UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK ((uint16_t)0x4UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK ((uint16_t)0x5UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK Setting */

#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS 12 /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT Position */
#define MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT ((uint16_t)(0x7UL << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS)) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT Mask */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 ((uint16_t)0x1UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 ((uint16_t)0x2UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 ((uint16_t)0x3UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 ((uint16_t)0x4UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 Setting */
#define MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 ((uint16_t)0x5UL) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 Value */
#define MXC_S_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 (MXC_V_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 << MXC_F_DBB_TX_TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< TX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 Setting */

/**@} end of group DBB_TX_TX_DL_BAN_SETTINGS_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL_BTLE_SETTINGS_WHIT DBB_TX_TX_DL_BTLE_SETTINGS_WHIT
 * @brief    btle_settings_whit
 * @{
 */
#define MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS 0 /**< TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR Position */
#define MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR  ((uint16_t)(0x3FUL << MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS)) /**< TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR Mask */

#define MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS 6 /**< TX_DL_BTLE_SETTINGS_WHIT_BYPASS Position */
#define MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_BYPASS   ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS)) /**< TX_DL_BTLE_SETTINGS_WHIT_BYPASS Mask */

/**@} end of group DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL_BTLE_LONGRANGE DBB_TX_TX_DL_BTLE_LONGRANGE
 * @brief    btle_longrange
 * @{
 */
#define MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_EN_POS 0 /**< TX_DL_BTLE_LONGRANGE_FECENC_EN Position */
#define MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_EN    ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_EN_POS)) /**< TX_DL_BTLE_LONGRANGE_FECENC_EN Mask */

#define MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_CLR_POS 1 /**< TX_DL_BTLE_LONGRANGE_FECENC_CLR Position */
#define MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_CLR   ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_CLR_POS)) /**< TX_DL_BTLE_LONGRANGE_FECENC_CLR Mask */

#define MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_POS   2 /**< TX_DL_BTLE_LONGRANGE_S_EQ_8 Position */
#define MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8       ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_POS)) /**< TX_DL_BTLE_LONGRANGE_S_EQ_8 Mask */
#define MXC_V_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_S8    ((uint16_t)0x1UL) /**< TX_DL_BTLE_LONGRANGE_S_EQ_8_S8 Value */
#define MXC_S_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_S8    (MXC_V_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_S8 << MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_POS) /**< TX_DL_BTLE_LONGRANGE_S_EQ_8_S8 Setting */
#define MXC_V_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_S2    ((uint16_t)0x0UL) /**< TX_DL_BTLE_LONGRANGE_S_EQ_8_S2 Value */
#define MXC_S_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_S2    (MXC_V_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_S2 << MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8_POS) /**< TX_DL_BTLE_LONGRANGE_S_EQ_8_S2 Setting */

/**@} end of group DBB_TX_TX_DL_BTLE_LONGRANGE_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL_PHR_MODULATION DBB_TX_TX_DL_PHR_MODULATION
 * @brief    Header modulation: OQPSK=0,  1 = DBPSK 2 = DQPSK, 3 = D8PSK, 4 = GMSK, 5 = GFSK
 * @{
 */
#define MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS      0 /**< TX_DL_PHR_MODULATION_MOD Position */
#define MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD          ((uint16_t)(0x7UL << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS)) /**< TX_DL_PHR_MODULATION_MOD Mask */
#define MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_OQPSK    ((uint16_t)0x0UL) /**< TX_DL_PHR_MODULATION_MOD_OQPSK Value */
#define MXC_S_DBB_TX_TX_DL_PHR_MODULATION_MOD_OQPSK    (MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_OQPSK << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS) /**< TX_DL_PHR_MODULATION_MOD_OQPSK Setting */
#define MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_DBPSK    ((uint16_t)0x1UL) /**< TX_DL_PHR_MODULATION_MOD_DBPSK Value */
#define MXC_S_DBB_TX_TX_DL_PHR_MODULATION_MOD_DBPSK    (MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_DBPSK << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS) /**< TX_DL_PHR_MODULATION_MOD_DBPSK Setting */
#define MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_DQPSK    ((uint16_t)0x2UL) /**< TX_DL_PHR_MODULATION_MOD_DQPSK Value */
#define MXC_S_DBB_TX_TX_DL_PHR_MODULATION_MOD_DQPSK    (MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_DQPSK << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS) /**< TX_DL_PHR_MODULATION_MOD_DQPSK Setting */
#define MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_D8PSK    ((uint16_t)0x3UL) /**< TX_DL_PHR_MODULATION_MOD_D8PSK Value */
#define MXC_S_DBB_TX_TX_DL_PHR_MODULATION_MOD_D8PSK    (MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_D8PSK << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS) /**< TX_DL_PHR_MODULATION_MOD_D8PSK Setting */
#define MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_GMSK     ((uint16_t)0x4UL) /**< TX_DL_PHR_MODULATION_MOD_GMSK Value */
#define MXC_S_DBB_TX_TX_DL_PHR_MODULATION_MOD_GMSK     (MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_GMSK << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS) /**< TX_DL_PHR_MODULATION_MOD_GMSK Setting */
#define MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_GFSK     ((uint16_t)0x5UL) /**< TX_DL_PHR_MODULATION_MOD_GFSK Value */
#define MXC_S_DBB_TX_TX_DL_PHR_MODULATION_MOD_GFSK     (MXC_V_DBB_TX_TX_DL_PHR_MODULATION_MOD_GFSK << MXC_F_DBB_TX_TX_DL_PHR_MODULATION_MOD_POS) /**< TX_DL_PHR_MODULATION_MOD_GFSK Setting */

/**@} end of group DBB_TX_TX_DL_PHR_MODULATION_Register */

/**
 * @ingroup  dbb_tx_registers
 * @defgroup DBB_TX_TX_DL2_BTLE_SPEED_MODE DBB_TX_TX_DL2_BTLE_SPEED_MODE
 * @brief    Set BTLE speed mode 0 = 1 Mb/s, 1 = 2 Mb/s
 * @{
 */
#define MXC_F_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_POS  0 /**< TX_DL2_BTLE_SPEED_MODE_SPEED Position */
#define MXC_F_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED      ((uint16_t)(0x1UL << MXC_F_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_POS)) /**< TX_DL2_BTLE_SPEED_MODE_SPEED Mask */
#define MXC_V_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_1M   ((uint16_t)0x0UL) /**< TX_DL2_BTLE_SPEED_MODE_SPEED_1M Value */
#define MXC_S_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_1M   (MXC_V_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_1M << MXC_F_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_POS) /**< TX_DL2_BTLE_SPEED_MODE_SPEED_1M Setting */
#define MXC_V_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_2M   ((uint16_t)0x1UL) /**< TX_DL2_BTLE_SPEED_MODE_SPEED_2M Value */
#define MXC_S_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_2M   (MXC_V_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_2M << MXC_F_DBB_TX_TX_DL2_BTLE_SPEED_MODE_SPEED_POS) /**< TX_DL2_BTLE_SPEED_MODE_SPEED_2M Setting */

/**@} end of group DBB_TX_TX_DL2_BTLE_SPEED_MODE_Register */

#ifdef __cplusplus
}
#endif

#endif // MAX32665_INCLUDE_DBB_TX_REGS_H_

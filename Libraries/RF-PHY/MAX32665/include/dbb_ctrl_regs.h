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
 * @file    dbb_ctrl_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the DBB_CTRL Peripheral Module.
 */

#ifndef MAX32665_INCLUDE_DBB_CTRL_REGS_H_
#define MAX32665_INCLUDE_DBB_CTRL_REGS_H_

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
 * @ingroup     dbb_ctrl
 * @defgroup    dbb_ctrl_registers DBB_CTRL_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the DBB_CTRL Peripheral Module.
 * @details DIBS Control Config
 */

/**
 * @ingroup dbb_ctrl_registers
 * Structure type to access the DBB_CTRL Registers.
 */
typedef struct {
    __IO uint16_t dbb_ctrl_version;               /**< <tt>\b 0x0000:</tt> DBB_CTRL DBB_CTRL_VERSION Register */
    __IO uint16_t dbb_ctrl_rst;                   /**< <tt>\b 0x0002:</tt> DBB_CTRL DBB_CTRL_RST Register */
    __IO uint32_t tx_ctrl_dbb_en_dly;             /**< <tt>\b 0x0004:</tt> DBB_CTRL TX_CTRL_DBB_EN_DLY Register */
    __IO uint32_t tx_ctrl_rffe_en_dly[2];         /**< <tt>\b 0x0008:</tt> DBB_CTRL TX_CTRL_RFFE_EN_DLY Register */
    __IO uint32_t tx_ctrl_dbb_dis_dly;            /**< <tt>\b 0x0010:</tt> DBB_CTRL TX_CTRL_DBB_DIS_DLY Register */
    __IO uint32_t tx_ctrl_rffe_dis_dly[2];        /**< <tt>\b 0x0014:</tt> DBB_CTRL TX_CTRL_RFFE_DIS_DLY Register */
    __IO uint16_t tx_ctrl_cmd;                    /**< <tt>\b 0x001c:</tt> DBB_CTRL TX_CTRL_CMD Register */
    __IO uint16_t tx_ctrl_debug_en_tx_on_sfd_to;  /**< <tt>\b 0x001e:</tt> DBB_CTRL TX_CTRL_DEBUG_EN_TX_ON_SFD_TO Register */
    __IO uint32_t rx_ctrl_dbb_en_dly;             /**< <tt>\b 0x0020:</tt> DBB_CTRL RX_CTRL_DBB_EN_DLY Register */
    __IO uint32_t rx_ctrl_rffe_en_dly[2];         /**< <tt>\b 0x0024:</tt> DBB_CTRL RX_CTRL_RFFE_EN_DLY Register */
    __IO uint32_t rx_ctrl_dbb_dis_dly;            /**< <tt>\b 0x002c:</tt> DBB_CTRL RX_CTRL_DBB_DIS_DLY Register */
    __IO uint32_t rx_ctrl_rffe_dis_dly[2];        /**< <tt>\b 0x0030:</tt> DBB_CTRL RX_CTRL_RFFE_DIS_DLY Register */
    __IO uint32_t rx_ctrl_cmd;                    /**< <tt>\b 0x0038:</tt> DBB_CTRL RX_CTRL_CMD Register */
    __IO uint16_t tx_pmu_wake_up_dly;             /**< <tt>\b 0x003c:</tt> DBB_CTRL TX_PMU_WAKE_UP_DLY Register */
    __IO uint16_t tx_pmu_ctrl;                    /**< <tt>\b 0x003e:</tt> DBB_CTRL TX_PMU_CTRL Register */
    __IO uint16_t rffe_pmu_wake_up_dly;           /**< <tt>\b 0x0040:</tt> DBB_CTRL RFFE_PMU_WAKE_UP_DLY Register */
    __IO uint16_t rffe_pmu_ctrl;                  /**< <tt>\b 0x0042:</tt> DBB_CTRL RFFE_PMU_CTRL Register */
    __IO uint16_t rx_pmu_wake_up_dly;             /**< <tt>\b 0x0044:</tt> DBB_CTRL RX_PMU_WAKE_UP_DLY Register */
    __IO uint16_t rx_pmu_ctrl;                    /**< <tt>\b 0x0046:</tt> DBB_CTRL RX_PMU_CTRL Register */
    __IO uint32_t gen_pmu_status;                 /**< <tt>\b 0x0048:</tt> DBB_CTRL GEN_PMU_STATUS Register */
    __IO uint16_t event_timing_cntr_clk_mult_p;   /**< <tt>\b 0x004c:</tt> DBB_CTRL EVENT_TIMING_CNTR_CLK_MULT_P Register */
    __IO uint16_t event_timing_cntr_clk_div_q;    /**< <tt>\b 0x004e:</tt> DBB_CTRL EVENT_TIMING_CNTR_CLK_DIV_Q Register */
    __IO uint32_t event_timing_cntr_val;          /**< <tt>\b 0x0050:</tt> DBB_CTRL EVENT_TIMING_CNTR_VAL Register */
    __IO uint32_t event_timing_tx_enable_time;    /**< <tt>\b 0x0054:</tt> DBB_CTRL EVENT_TIMING_TX_ENABLE_TIME Register */
    __IO uint32_t event_timing_rx_enable_time;    /**< <tt>\b 0x0058:</tt> DBB_CTRL EVENT_TIMING_RX_ENABLE_TIME Register */
    __IO uint32_t event_timing_gp_event_time;     /**< <tt>\b 0x005c:</tt> DBB_CTRL EVENT_TIMING_GP_EVENT_TIME Register */
    __IO uint32_t event_timing_tx_enable_delta_time; /**< <tt>\b 0x0060:</tt> DBB_CTRL EVENT_TIMING_TX_ENABLE_DELTA_TIME Register */
    __IO uint32_t event_timing_rx_enable_delta_time; /**< <tt>\b 0x0064:</tt> DBB_CTRL EVENT_TIMING_RX_ENABLE_DELTA_TIME Register */
    __IO uint32_t event_timing_gp_event_delta_time; /**< <tt>\b 0x0068:</tt> DBB_CTRL EVENT_TIMING_GP_EVENT_DELTA_TIME Register */
    __IO uint32_t event_timing_ctrl;              /**< <tt>\b 0x006c:</tt> DBB_CTRL EVENT_TIMING_CTRL Register */
    __IO uint32_t event_timing_timestamp_tx_done; /**< <tt>\b 0x0070:</tt> DBB_CTRL EVENT_TIMING_TIMESTAMP_TX_DONE Register */
    __IO uint32_t event_timing_timestamp_rx_received; /**< <tt>\b 0x0074:</tt> DBB_CTRL EVENT_TIMING_TIMESTAMP_RX_RECEIVED Register */
    __IO uint32_t event_timing_timestamp_rx_sfd_det; /**< <tt>\b 0x0078:</tt> DBB_CTRL EVENT_TIMING_TIMESTAMP_RX_SFD_DET Register */
    __IO uint32_t event_timing_timestamp_rx_sfd_to; /**< <tt>\b 0x007c:</tt> DBB_CTRL EVENT_TIMING_TIMESTAMP_RX_SFD_TO Register */
    __IO uint32_t event_timing_timestamp_rx_energy_det; /**< <tt>\b 0x0080:</tt> DBB_CTRL EVENT_TIMING_TIMESTAMP_RX_ENERGY_DET Register */
    __IO uint16_t events_status;                  /**< <tt>\b 0x0084:</tt> DBB_CTRL EVENTS_STATUS Register */
    __IO uint16_t events_irq_test;                /**< <tt>\b 0x0086:</tt> DBB_CTRL EVENTS_IRQ_TEST Register */
    __IO uint16_t cmu_gating_on;                  /**< <tt>\b 0x0088:</tt> DBB_CTRL CMU_GATING_ON Register */
    __IO uint16_t cmu_main_mult_p;                /**< <tt>\b 0x008a:</tt> DBB_CTRL CMU_MAIN_MULT_P Register */
    __IO uint16_t cmu_main_div_q;                 /**< <tt>\b 0x008c:</tt> DBB_CTRL CMU_MAIN_DIV_Q Register */
    __IO uint16_t cmu_phy_mult_p;                 /**< <tt>\b 0x008e:</tt> DBB_CTRL CMU_PHY_MULT_P Register */
    __IO uint16_t cmu_phy_div_q;                  /**< <tt>\b 0x0090:</tt> DBB_CTRL CMU_PHY_DIV_Q Register */
    __IO uint16_t cmu_dl_mult_p;                  /**< <tt>\b 0x0092:</tt> DBB_CTRL CMU_DL_MULT_P Register */
    __IO uint16_t cmu_dl_div_q;                   /**< <tt>\b 0x0094:</tt> DBB_CTRL CMU_DL_DIV_Q Register */
    __R  uint16_t rsv_0x96_0xff[53];
    __IO uint32_t aes_st;                         /**< <tt>\b 0x0100:</tt> DBB_CTRL AES_ST Register */
    __IO uint32_t aes_aad;                        /**< <tt>\b 0x0104:</tt> DBB_CTRL AES_AAD Register */
    __R  uint32_t rsv_0x108;
    __IO uint32_t aes_ctrl;                       /**< <tt>\b 0x010C:</tt> DBB_CTRL AES_CTRL Register */
    __IO uint8_t  aes_key[16];                    /**< <tt>\b 0x0110:</tt> DBB_CTRL AES_KEY Register */
    __IO uint8_t  aes_ctr_blk[16];                /**< <tt>\b 0x0120:</tt> DBB_CTRL AES_CTR_BLK Register */
} mxc_dbb_ctrl_regs_t;

/* Register offsets for module DBB_CTRL */
/**
 * @ingroup    dbb_ctrl_registers
 * @defgroup   DBB_CTRL_Register_Offsets Register Offsets
 * @brief      DBB_CTRL Peripheral Register Offsets from the DBB_CTRL Base Peripheral Address.
 * @{
 */
#define MXC_R_DBB_CTRL_DBB_CTRL_VERSION              ((uint32_t)0x00000000UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0000</tt> */
#define MXC_R_DBB_CTRL_DBB_CTRL_RST                  ((uint32_t)0x00000002UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0002</tt> */
#define MXC_R_DBB_CTRL_TX_CTRL_DBB_EN_DLY            ((uint32_t)0x00000004UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0004</tt> */
#define MXC_R_DBB_CTRL_TX_CTRL_RFFE_EN_DLY           ((uint32_t)0x00000008UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0008</tt> */
#define MXC_R_DBB_CTRL_TX_CTRL_DBB_DIS_DLY           ((uint32_t)0x00000010UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0010</tt> */
#define MXC_R_DBB_CTRL_TX_CTRL_RFFE_DIS_DLY          ((uint32_t)0x00000014UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0014</tt> */
#define MXC_R_DBB_CTRL_TX_CTRL_CMD                   ((uint32_t)0x0000001CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x001C</tt> */
#define MXC_R_DBB_CTRL_TX_CTRL_DEBUG_EN_TX_ON_SFD_TO ((uint32_t)0x0000001EUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x001E</tt> */
#define MXC_R_DBB_CTRL_RX_CTRL_DBB_EN_DLY            ((uint32_t)0x00000020UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0020</tt> */
#define MXC_R_DBB_CTRL_RX_CTRL_RFFE_EN_DLY           ((uint32_t)0x00000024UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0024</tt> */
#define MXC_R_DBB_CTRL_RX_CTRL_DBB_DIS_DLY           ((uint32_t)0x0000002CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x002C</tt> */
#define MXC_R_DBB_CTRL_RX_CTRL_RFFE_DIS_DLY          ((uint32_t)0x00000030UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0030</tt> */
#define MXC_R_DBB_CTRL_RX_CTRL_CMD                   ((uint32_t)0x00000038UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0038</tt> */
#define MXC_R_DBB_CTRL_TX_PMU_WAKE_UP_DLY            ((uint32_t)0x0000003CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x003C</tt> */
#define MXC_R_DBB_CTRL_TX_PMU_CTRL                   ((uint32_t)0x0000003EUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x003E</tt> */
#define MXC_R_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY          ((uint32_t)0x00000040UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0040</tt> */
#define MXC_R_DBB_CTRL_RFFE_PMU_CTRL                 ((uint32_t)0x00000042UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0042</tt> */
#define MXC_R_DBB_CTRL_RX_PMU_WAKE_UP_DLY            ((uint32_t)0x00000044UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0044</tt> */
#define MXC_R_DBB_CTRL_RX_PMU_CTRL                   ((uint32_t)0x00000046UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0046</tt> */
#define MXC_R_DBB_CTRL_GEN_PMU_STATUS                ((uint32_t)0x00000048UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0048</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_CNTR_CLK_MULT_P  ((uint32_t)0x0000004CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x004C</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_CNTR_CLK_DIV_Q   ((uint32_t)0x0000004EUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x004E</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_CNTR_VAL         ((uint32_t)0x00000050UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0050</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TX_ENABLE_TIME   ((uint32_t)0x00000054UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0054</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_RX_ENABLE_TIME   ((uint32_t)0x00000058UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0058</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_GP_EVENT_TIME    ((uint32_t)0x0000005CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x005C</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TX_ENABLE_DELTA_TIME ((uint32_t)0x00000060UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0060</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_RX_ENABLE_DELTA_TIME ((uint32_t)0x00000064UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0064</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_GP_EVENT_DELTA_TIME ((uint32_t)0x00000068UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0068</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_CTRL             ((uint32_t)0x0000006CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x006C</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TIMESTAMP_TX_DONE ((uint32_t)0x00000070UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0070</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TIMESTAMP_RX_RECEIVED ((uint32_t)0x00000074UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0074</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TIMESTAMP_RX_SFD_DET ((uint32_t)0x00000078UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0078</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TIMESTAMP_RX_SFD_TO ((uint32_t)0x0000007CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x007C</tt> */
#define MXC_R_DBB_CTRL_EVENT_TIMING_TIMESTAMP_RX_ENERGY_DET ((uint32_t)0x00000080UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0080</tt> */
#define MXC_R_DBB_CTRL_EVENTS_STATUS                 ((uint32_t)0x00000084UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0084</tt> */
#define MXC_R_DBB_CTRL_EVENTS_IRQ_TEST               ((uint32_t)0x00000086UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0086</tt> */
#define MXC_R_DBB_CTRL_CMU_GATING_ON                 ((uint32_t)0x00000088UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0088</tt> */
#define MXC_R_DBB_CTRL_CMU_MAIN_MULT_P               ((uint32_t)0x0000008AUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x008A</tt> */
#define MXC_R_DBB_CTRL_CMU_MAIN_DIV_Q                ((uint32_t)0x0000008CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x008C</tt> */
#define MXC_R_DBB_CTRL_CMU_PHY_MULT_P                ((uint32_t)0x0000008EUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x008E</tt> */
#define MXC_R_DBB_CTRL_CMU_PHY_DIV_Q                 ((uint32_t)0x00000090UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0090</tt> */
#define MXC_R_DBB_CTRL_CMU_DL_MULT_P                 ((uint32_t)0x00000092UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0092</tt> */
#define MXC_R_DBB_CTRL_CMU_DL_DIV_Q                  ((uint32_t)0x00000094UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0094</tt> */
#define MXC_R_DBB_CTRL_AES_ST                        ((uint32_t)0x00000100UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0100</tt> */
#define MXC_R_DBB_CTRL_AES_AAD                       ((uint32_t)0x00000104UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0104</tt> */
#define MXC_R_DBB_CTRL_AES_CTRL                      ((uint32_t)0x0000010CUL) /**< Offset from DBB_CTRL Base Address: <tt> 0x010C</tt> */
#define MXC_R_DBB_CTRL_AES_KEY                       ((uint32_t)0x00000110UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0110</tt> */
#define MXC_R_DBB_CTRL_AES_CTR_BLK                   ((uint32_t)0x00000120UL) /**< Offset from DBB_CTRL Base Address: <tt> 0x0120</tt> */
/**@} end of group dbb_ctrl_registers */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_DBB_CTRL_VERSION DBB_CTRL_DBB_CTRL_VERSION
 * @brief    version
 * @{
 */
#define MXC_F_DBB_CTRL_DBB_CTRL_VERSION_MINOR_POS      0 /**< DBB_CTRL_VERSION_MINOR Position */
#define MXC_F_DBB_CTRL_DBB_CTRL_VERSION_MINOR          ((uint16_t)(0xFFUL << MXC_F_DBB_CTRL_DBB_CTRL_VERSION_MINOR_POS)) /**< DBB_CTRL_VERSION_MINOR Mask */

#define MXC_F_DBB_CTRL_DBB_CTRL_VERSION_MAJOR_POS      8 /**< DBB_CTRL_VERSION_MAJOR Position */
#define MXC_F_DBB_CTRL_DBB_CTRL_VERSION_MAJOR          ((uint16_t)(0xFFUL << MXC_F_DBB_CTRL_DBB_CTRL_VERSION_MAJOR_POS)) /**< DBB_CTRL_VERSION_MAJOR Mask */

/**@} end of group DBB_CTRL_DBB_CTRL_VERSION_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_DBB_CTRL_RST DBB_CTRL_DBB_CTRL_RST
 * @brief    Digital baseband reset
 * @{
 */
#define MXC_F_DBB_CTRL_DBB_CTRL_RST_RST_POS            0 /**< DBB_CTRL_RST_RST Position */
#define MXC_F_DBB_CTRL_DBB_CTRL_RST_RST                ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_DBB_CTRL_RST_RST_POS)) /**< DBB_CTRL_RST_RST Mask */

#define MXC_F_DBB_CTRL_DBB_CTRL_RST_ANT_SEL_POS        1 /**< DBB_CTRL_RST_ANT_SEL Position */
#define MXC_F_DBB_CTRL_DBB_CTRL_RST_ANT_SEL            ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_DBB_CTRL_RST_ANT_SEL_POS)) /**< DBB_CTRL_RST_ANT_SEL Mask */

/**@} end of group DBB_CTRL_DBB_CTRL_RST_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_TX_CTRL_CMD DBB_CTRL_TX_CTRL_CMD
 * @brief    cmd
 * @{
 */
#define MXC_F_DBB_CTRL_TX_CTRL_CMD_ENABLE_POS          0 /**< TX_CTRL_CMD_ENABLE Position */
#define MXC_F_DBB_CTRL_TX_CTRL_CMD_ENABLE              ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_TX_CTRL_CMD_ENABLE_POS)) /**< TX_CTRL_CMD_ENABLE Mask */

#define MXC_F_DBB_CTRL_TX_CTRL_CMD_DISABLE_POS         1 /**< TX_CTRL_CMD_DISABLE Position */
#define MXC_F_DBB_CTRL_TX_CTRL_CMD_DISABLE             ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_TX_CTRL_CMD_DISABLE_POS)) /**< TX_CTRL_CMD_DISABLE Mask */

#define MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP_POS            2 /**< TX_CTRL_CMD_STOP Position */
#define MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP                ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP_POS)) /**< TX_CTRL_CMD_STOP Mask */

/**@} end of group DBB_CTRL_TX_CTRL_CMD_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_RX_CTRL_CMD DBB_CTRL_RX_CTRL_CMD
 * @brief    cmd
 * @{
 */
#define MXC_F_DBB_CTRL_RX_CTRL_CMD_ENABLE_POS          0 /**< RX_CTRL_CMD_ENABLE Position */
#define MXC_F_DBB_CTRL_RX_CTRL_CMD_ENABLE              ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_RX_CTRL_CMD_ENABLE_POS)) /**< RX_CTRL_CMD_ENABLE Mask */

#define MXC_F_DBB_CTRL_RX_CTRL_CMD_DISABLE_POS         1 /**< RX_CTRL_CMD_DISABLE Position */
#define MXC_F_DBB_CTRL_RX_CTRL_CMD_DISABLE             ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_RX_CTRL_CMD_DISABLE_POS)) /**< RX_CTRL_CMD_DISABLE Mask */

#define MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP_POS            2 /**< RX_CTRL_CMD_STOP Position */
#define MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP                ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP_POS)) /**< RX_CTRL_CMD_STOP Mask */

/**@} end of group DBB_CTRL_RX_CTRL_CMD_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_TX_PMU_WAKE_UP_DLY DBB_CTRL_TX_PMU_WAKE_UP_DLY
 * @brief    wake_up_dly
 * @{
 */
#define MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_STAB_POS     0 /**< TX_PMU_WAKE_UP_DLY_STAB Position */
#define MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_STAB         ((uint16_t)(0x3FUL << MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_STAB_POS)) /**< TX_PMU_WAKE_UP_DLY_STAB Mask */

#define MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_UNUSED_POS   6 /**< TX_PMU_WAKE_UP_DLY_UNUSED Position */
#define MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_UNUSED       ((uint16_t)(0x3UL << MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_UNUSED_POS)) /**< TX_PMU_WAKE_UP_DLY_UNUSED Mask */

#define MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_ISO_POS      8 /**< TX_PMU_WAKE_UP_DLY_ISO Position */
#define MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_ISO          ((uint16_t)(0x3FUL << MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_ISO_POS)) /**< TX_PMU_WAKE_UP_DLY_ISO Mask */

/**@} end of group DBB_CTRL_TX_PMU_WAKE_UP_DLY_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_TX_PMU_CTRL DBB_CTRL_TX_PMU_CTRL
 * @brief    ctrl
 * @{
 */
#define MXC_F_DBB_CTRL_TX_PMU_CTRL_PWR_GATING_ON_POS   0 /**< TX_PMU_CTRL_PWR_GATING_ON Position */
#define MXC_F_DBB_CTRL_TX_PMU_CTRL_PWR_GATING_ON       ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_TX_PMU_CTRL_PWR_GATING_ON_POS)) /**< TX_PMU_CTRL_PWR_GATING_ON Mask */

#define MXC_F_DBB_CTRL_TX_PMU_CTRL_WAKEUP_POS          1 /**< TX_PMU_CTRL_WAKEUP Position */
#define MXC_F_DBB_CTRL_TX_PMU_CTRL_WAKEUP              ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_TX_PMU_CTRL_WAKEUP_POS)) /**< TX_PMU_CTRL_WAKEUP Mask */

#define MXC_F_DBB_CTRL_TX_PMU_CTRL_SLEEP_POS           2 /**< TX_PMU_CTRL_SLEEP Position */
#define MXC_F_DBB_CTRL_TX_PMU_CTRL_SLEEP               ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_TX_PMU_CTRL_SLEEP_POS)) /**< TX_PMU_CTRL_SLEEP Mask */

/**@} end of group DBB_CTRL_TX_PMU_CTRL_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_RFFE_PMU_WAKE_UP_DLY DBB_CTRL_RFFE_PMU_WAKE_UP_DLY
 * @brief    wake_up_dly
 * @{
 */
#define MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_STAB_POS   0 /**< RFFE_PMU_WAKE_UP_DLY_STAB Position */
#define MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_STAB       ((uint16_t)(0x3FUL << MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_STAB_POS)) /**< RFFE_PMU_WAKE_UP_DLY_STAB Mask */

#define MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_UNUSED_POS 6 /**< RFFE_PMU_WAKE_UP_DLY_UNUSED Position */
#define MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_UNUSED     ((uint16_t)(0x3UL << MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_UNUSED_POS)) /**< RFFE_PMU_WAKE_UP_DLY_UNUSED Mask */

#define MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_ISO_POS    8 /**< RFFE_PMU_WAKE_UP_DLY_ISO Position */
#define MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_ISO        ((uint16_t)(0x3FUL << MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_ISO_POS)) /**< RFFE_PMU_WAKE_UP_DLY_ISO Mask */

/**@} end of group DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_RFFE_PMU_CTRL DBB_CTRL_RFFE_PMU_CTRL
 * @brief    ctrl
 * @{
 */
#define MXC_F_DBB_CTRL_RFFE_PMU_CTRL_PWR_GATING_ON_POS 0 /**< RFFE_PMU_CTRL_PWR_GATING_ON Position */
#define MXC_F_DBB_CTRL_RFFE_PMU_CTRL_PWR_GATING_ON     ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_RFFE_PMU_CTRL_PWR_GATING_ON_POS)) /**< RFFE_PMU_CTRL_PWR_GATING_ON Mask */

#define MXC_F_DBB_CTRL_RFFE_PMU_CTRL_WAKEUP_POS        1 /**< RFFE_PMU_CTRL_WAKEUP Position */
#define MXC_F_DBB_CTRL_RFFE_PMU_CTRL_WAKEUP            ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_RFFE_PMU_CTRL_WAKEUP_POS)) /**< RFFE_PMU_CTRL_WAKEUP Mask */

#define MXC_F_DBB_CTRL_RFFE_PMU_CTRL_SLEEP_POS         2 /**< RFFE_PMU_CTRL_SLEEP Position */
#define MXC_F_DBB_CTRL_RFFE_PMU_CTRL_SLEEP             ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_RFFE_PMU_CTRL_SLEEP_POS)) /**< RFFE_PMU_CTRL_SLEEP Mask */

/**@} end of group DBB_CTRL_RFFE_PMU_CTRL_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_RX_PMU_WAKE_UP_DLY DBB_CTRL_RX_PMU_WAKE_UP_DLY
 * @brief    wake_up_dly
 * @{
 */
#define MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_STAB_POS     0 /**< RX_PMU_WAKE_UP_DLY_STAB Position */
#define MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_STAB         ((uint16_t)(0x3FUL << MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_STAB_POS)) /**< RX_PMU_WAKE_UP_DLY_STAB Mask */

#define MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_UNUSED_POS   6 /**< RX_PMU_WAKE_UP_DLY_UNUSED Position */
#define MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_UNUSED       ((uint16_t)(0x3UL << MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_UNUSED_POS)) /**< RX_PMU_WAKE_UP_DLY_UNUSED Mask */

#define MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_ISO_POS      8 /**< RX_PMU_WAKE_UP_DLY_ISO Position */
#define MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_ISO          ((uint16_t)(0x3FUL << MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_ISO_POS)) /**< RX_PMU_WAKE_UP_DLY_ISO Mask */

/**@} end of group DBB_CTRL_RX_PMU_WAKE_UP_DLY_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_RX_PMU_CTRL DBB_CTRL_RX_PMU_CTRL
 * @brief    ctrl
 * @{
 */
#define MXC_F_DBB_CTRL_RX_PMU_CTRL_PWR_GATING_ON_POS   0 /**< RX_PMU_CTRL_PWR_GATING_ON Position */
#define MXC_F_DBB_CTRL_RX_PMU_CTRL_PWR_GATING_ON       ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_RX_PMU_CTRL_PWR_GATING_ON_POS)) /**< RX_PMU_CTRL_PWR_GATING_ON Mask */

#define MXC_F_DBB_CTRL_RX_PMU_CTRL_WAKEUP_POS          1 /**< RX_PMU_CTRL_WAKEUP Position */
#define MXC_F_DBB_CTRL_RX_PMU_CTRL_WAKEUP              ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_RX_PMU_CTRL_WAKEUP_POS)) /**< RX_PMU_CTRL_WAKEUP Mask */

#define MXC_F_DBB_CTRL_RX_PMU_CTRL_SLEEP_POS           2 /**< RX_PMU_CTRL_SLEEP Position */
#define MXC_F_DBB_CTRL_RX_PMU_CTRL_SLEEP               ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_RX_PMU_CTRL_SLEEP_POS)) /**< RX_PMU_CTRL_SLEEP Mask */

/**@} end of group DBB_CTRL_RX_PMU_CTRL_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_GEN_PMU_STATUS DBB_CTRL_GEN_PMU_STATUS
 * @brief    status
 * @{
 */
#define MXC_F_DBB_CTRL_GEN_PMU_STATUS_TX_POS           0 /**< GEN_PMU_STATUS_TX Position */
#define MXC_F_DBB_CTRL_GEN_PMU_STATUS_TX               ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_GEN_PMU_STATUS_TX_POS)) /**< GEN_PMU_STATUS_TX Mask */

#define MXC_F_DBB_CTRL_GEN_PMU_STATUS_RFE_POS          1 /**< GEN_PMU_STATUS_RFE Position */
#define MXC_F_DBB_CTRL_GEN_PMU_STATUS_RFE              ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_GEN_PMU_STATUS_RFE_POS)) /**< GEN_PMU_STATUS_RFE Mask */

#define MXC_F_DBB_CTRL_GEN_PMU_STATUS_RX_POS           2 /**< GEN_PMU_STATUS_RX Position */
#define MXC_F_DBB_CTRL_GEN_PMU_STATUS_RX               ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_GEN_PMU_STATUS_RX_POS)) /**< GEN_PMU_STATUS_RX Mask */

/**@} end of group DBB_CTRL_GEN_PMU_STATUS_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_EVENT_TIMING_CTRL DBB_CTRL_EVENT_TIMING_CTRL
 * @brief    ctrl
 * @{
 */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE_POS    0 /**< EVENT_TIMING_CTRL_ENABLE Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE        ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE_POS)) /**< EVENT_TIMING_CTRL_ENABLE Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_RST_POS       1 /**< EVENT_TIMING_CTRL_RST Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_RST           ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_RST_POS)) /**< EVENT_TIMING_CTRL_RST Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_TX_ENABLE_TIME_POS 2 /**< EVENT_TIMING_CTRL_SET_TX_ENABLE_TIME Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_TX_ENABLE_TIME ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_TX_ENABLE_TIME_POS)) /**< EVENT_TIMING_CTRL_SET_TX_ENABLE_TIME Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_RX_ENABLE_TIME_POS 3 /**< EVENT_TIMING_CTRL_SET_RX_ENABLE_TIME Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_RX_ENABLE_TIME ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_RX_ENABLE_TIME_POS)) /**< EVENT_TIMING_CTRL_SET_RX_ENABLE_TIME Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_GP_EVENT_TIME_POS 4 /**< EVENT_TIMING_CTRL_SET_GP_EVENT_TIME Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_GP_EVENT_TIME ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_GP_EVENT_TIME_POS)) /**< EVENT_TIMING_CTRL_SET_GP_EVENT_TIME Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_TX_ENABLE_EVENT_POS 5 /**< EVENT_TIMING_CTRL_STOP_TX_ENABLE_EVENT Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_TX_ENABLE_EVENT ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_TX_ENABLE_EVENT_POS)) /**< EVENT_TIMING_CTRL_STOP_TX_ENABLE_EVENT Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_RX_ENABLE_EVENT_POS 6 /**< EVENT_TIMING_CTRL_STOP_RX_ENABLE_EVENT Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_RX_ENABLE_EVENT ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_RX_ENABLE_EVENT_POS)) /**< EVENT_TIMING_CTRL_STOP_RX_ENABLE_EVENT Mask */

#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_GP_EVENT_POS 7 /**< EVENT_TIMING_CTRL_STOP_GP_EVENT Position */
#define MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_GP_EVENT ((uint32_t)(0x1UL << MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_GP_EVENT_POS)) /**< EVENT_TIMING_CTRL_STOP_GP_EVENT Mask */

/**@} end of group DBB_CTRL_EVENT_TIMING_CTRL_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_EVENTS_STATUS DBB_CTRL_EVENTS_STATUS
 * @brief    status
 * @{
 */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_TX_DONE_POS       0 /**< EVENTS_STATUS_TX_DONE Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_TX_DONE           ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_TX_DONE_POS)) /**< EVENTS_STATUS_TX_DONE Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_RECEIVED_POS   1 /**< EVENTS_STATUS_RX_RECEIVED Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_RECEIVED       ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RX_RECEIVED_POS)) /**< EVENTS_STATUS_RX_RECEIVED Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_ENERGY_DET_POS 2 /**< EVENTS_STATUS_RX_ENERGY_DET Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_ENERGY_DET     ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RX_ENERGY_DET_POS)) /**< EVENTS_STATUS_RX_ENERGY_DET Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD_DET_POS    3 /**< EVENTS_STATUS_RX_SFD_DET Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD_DET        ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD_DET_POS)) /**< EVENTS_STATUS_RX_SFD_DET Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD_TO_POS     4 /**< EVENTS_STATUS_RX_SFD_TO Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD_TO         ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD_TO_POS)) /**< EVENTS_STATUS_RX_SFD_TO Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY_POS  5 /**< EVENTS_STATUS_RFE_SPIM_RDY Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY      ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY_POS)) /**< EVENTS_STATUS_RFE_SPIM_RDY Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_INVALID_ADDR_POS  6 /**< EVENTS_STATUS_INVALID_ADDR Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_INVALID_ADDR      ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_INVALID_ADDR_POS)) /**< EVENTS_STATUS_INVALID_ADDR Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_GP_EVENT_POS      7 /**< EVENTS_STATUS_GP_EVENT Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_GP_EVENT          ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_GP_EVENT_POS)) /**< EVENTS_STATUS_GP_EVENT Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_TX_AES_DONE_POS   8 /**< EVENTS_STATUS_TX_AES_DONE Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_TX_AES_DONE       ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_TX_AES_DONE_POS)) /**< EVENTS_STATUS_TX_AES_DONE Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_AES_DONE_POS   9 /**< EVENTS_STATUS_RX_AES_DONE Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_AES_DONE       ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RX_AES_DONE_POS)) /**< EVENTS_STATUS_RX_AES_DONE Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_DECRYPT_FAIL_POS  10 /**< EVENTS_STATUS_DECRYPT_FAIL Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_DECRYPT_FAIL      ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_DECRYPT_FAIL_POS)) /**< EVENTS_STATUS_DECRYPT_FAIL Mask */

#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD2_DE_POS    11 /**< EVENTS_STATUS_RX_SFD2_DE Position */
#define MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD2_DE        ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_STATUS_RX_SFD2_DE_POS)) /**< EVENTS_STATUS_RX_SFD2_DE Mask */

/**@} end of group DBB_CTRL_EVENTS_STATUS_Register */

/**
 * @ingroup  dbb_ctrl_registers
 * @defgroup DBB_CTRL_EVENTS_IRQ_TEST DBB_CTRL_EVENTS_IRQ_TEST
 * @brief    irq_test
 * @{
 */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_TX_DONE_POS     0 /**< EVENTS_IRQ_TEST_TX_DONE Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_TX_DONE         ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_TX_DONE_POS)) /**< EVENTS_IRQ_TEST_TX_DONE Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_RECEIVED_POS 1 /**< EVENTS_IRQ_TEST_RX_RECEIVED Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_RECEIVED     ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_RECEIVED_POS)) /**< EVENTS_IRQ_TEST_RX_RECEIVED Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_ENERGY_DET_POS 2 /**< EVENTS_IRQ_TEST_RX_ENERGY_DET Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_ENERGY_DET   ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_ENERGY_DET_POS)) /**< EVENTS_IRQ_TEST_RX_ENERGY_DET Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD_DET_POS  3 /**< EVENTS_IRQ_TEST_RX_SFD_DET Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD_DET      ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD_DET_POS)) /**< EVENTS_IRQ_TEST_RX_SFD_DET Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD_TO_POS   4 /**< EVENTS_IRQ_TEST_RX_SFD_TO Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD_TO       ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD_TO_POS)) /**< EVENTS_IRQ_TEST_RX_SFD_TO Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RFE_SPIM_RDY_POS 5 /**< EVENTS_IRQ_TEST_RFE_SPIM_RDY Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RFE_SPIM_RDY    ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RFE_SPIM_RDY_POS)) /**< EVENTS_IRQ_TEST_RFE_SPIM_RDY Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_INVALID_ADDR_POS 6 /**< EVENTS_IRQ_TEST_INVALID_ADDR Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_INVALID_ADDR    ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_INVALID_ADDR_POS)) /**< EVENTS_IRQ_TEST_INVALID_ADDR Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_GP_EVENT_POS    7 /**< EVENTS_IRQ_TEST_GP_EVENT Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_GP_EVENT        ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_GP_EVENT_POS)) /**< EVENTS_IRQ_TEST_GP_EVENT Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_TX_AES_DONE_POS 8 /**< EVENTS_IRQ_TEST_TX_AES_DONE Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_TX_AES_DONE     ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_TX_AES_DONE_POS)) /**< EVENTS_IRQ_TEST_TX_AES_DONE Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_AES_DONE_POS 9 /**< EVENTS_IRQ_TEST_RX_AES_DONE Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_AES_DONE     ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_AES_DONE_POS)) /**< EVENTS_IRQ_TEST_RX_AES_DONE Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_DECRYPT_FAIL_POS 10 /**< EVENTS_IRQ_TEST_DECRYPT_FAIL Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_DECRYPT_FAIL    ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_DECRYPT_FAIL_POS)) /**< EVENTS_IRQ_TEST_DECRYPT_FAIL Mask */

#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD2_DE_POS  11 /**< EVENTS_IRQ_TEST_RX_SFD2_DE Position */
#define MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD2_DE      ((uint16_t)(0x1UL << MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_RX_SFD2_DE_POS)) /**< EVENTS_IRQ_TEST_RX_SFD2_DE Mask */

/**@} end of group DBB_CTRL_EVENTS_IRQ_TEST_Register */

#ifdef __cplusplus
}
#endif

#endif // MAX32665_INCLUDE_DBB_CTRL_REGS_H_

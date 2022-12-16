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
 * @file    dbb_rffe_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the DBB_RFFE Peripheral Module.
 */

#ifndef MAX32665_INCLUDE_DBB_RFFE_REGS_H_
#define MAX32665_INCLUDE_DBB_RFFE_REGS_H_

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
 * @ingroup     dbb_rffe
 * @defgroup    dbb_rffe_registers DBB_RFFE_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the DBB_RFFE Peripheral Module.
 * @details DIBS RFFE IC Config
 */

/**
 * @ingroup dbb_rffe_registers
 * Structure type to access the DBB_RFFE Registers.
 */
typedef struct {
    __IO uint16_t rffe_ifc_version;               /**< <tt>\b 0x0000:</tt> DBB_RFFE RFFE_IFC_VERSION Register */
    __IO uint16_t rffe_ifc_rffe_version;          /**< <tt>\b 0x0002:</tt> DBB_RFFE RFFE_IFC_RFFE_VERSION Register */
    __IO uint16_t general_param;                  /**< <tt>\b 0x0004:</tt> DBB_RFFE GENERAL_PARAM Register */
    __IO uint16_t general_delay_freq;             /**< <tt>\b 0x0006:</tt> DBB_RFFE GENERAL_DELAY_FREQ Register */
    __IO uint16_t general_delay_amp;              /**< <tt>\b 0x0008:</tt> DBB_RFFE GENERAL_DELAY_AMP Register */
    __IO uint16_t general_spi_invert_csn;         /**< <tt>\b 0x000a:</tt> DBB_RFFE GENERAL_SPI_INVERT_CSN Register */
    __IO uint32_t general_skip_sdmod;             /**< <tt>\b 0x000c:</tt> DBB_RFFE GENERAL_SKIP_SDMOD Register */
    __IO uint16_t tx_seq_ena_seq_lngth_total;     /**< <tt>\b 0x0010:</tt> DBB_RFFE TX_SEQ_ENA_SEQ_LNGTH_TOTAL Register */
    __IO uint16_t tx_seq_ena_seq_lngth_part;      /**< <tt>\b 0x0012:</tt> DBB_RFFE TX_SEQ_ENA_SEQ_LNGTH_PART Register */
    __IO uint16_t tx_seq_ena_cmd[16];             /**< <tt>\b 0x0014:</tt> DBB_RFFE TX_SEQ_ENA_CMD Register */
    __IO uint16_t tx_seq_dis_seq_lngth_total;     /**< <tt>\b 0x0034:</tt> DBB_RFFE TX_SEQ_DIS_SEQ_LNGTH_TOTAL Register */
    __IO uint16_t tx_seq_dis_seq_lngth_part;      /**< <tt>\b 0x0036:</tt> DBB_RFFE TX_SEQ_DIS_SEQ_LNGTH_PART Register */
    __IO uint16_t tx_seq_dis_cmd[16];             /**< <tt>\b 0x0038:</tt> DBB_RFFE TX_SEQ_DIS_CMD Register */
    __IO uint32_t tx_seq_spi;                     /**< <tt>\b 0x0058:</tt> DBB_RFFE TX_SEQ_SPI Register */
    __IO uint16_t rx_seq_ena_seq_lngth_total;     /**< <tt>\b 0x005c:</tt> DBB_RFFE RX_SEQ_ENA_SEQ_LNGTH_TOTAL Register */
    __IO uint16_t rx_seq_ena_seq_lngth_part;      /**< <tt>\b 0x005e:</tt> DBB_RFFE RX_SEQ_ENA_SEQ_LNGTH_PART Register */
    __IO uint16_t rx_seq_ena_cmd[16];             /**< <tt>\b 0x0060:</tt> DBB_RFFE RX_SEQ_ENA_CMD Register */
    __IO uint16_t rx_seq_dis_seq_lngth_total;     /**< <tt>\b 0x0080:</tt> DBB_RFFE RX_SEQ_DIS_SEQ_LNGTH_TOTAL Register */
    __IO uint16_t rx_seq_dis_seq_lngth_part;      /**< <tt>\b 0x0082:</tt> DBB_RFFE RX_SEQ_DIS_SEQ_LNGTH_PART Register */
    __IO uint16_t rx_seq_dis_cmd[16];             /**< <tt>\b 0x0084:</tt> DBB_RFFE RX_SEQ_DIS_CMD Register */
    __IO uint32_t rx_seq_spi;                     /**< <tt>\b 0x00a4:</tt> DBB_RFFE RX_SEQ_SPI Register */
    __IO uint16_t rffe_spim_cfg;                  /**< <tt>\b 0x00a8:</tt> DBB_RFFE RFFE_SPIM_CFG Register */
    __IO uint16_t rffe_spim_data_out;             /**< <tt>\b 0x00aa:</tt> DBB_RFFE RFFE_SPIM_DATA_OUT Register */
    __IO uint16_t rffe_spim_start_transaction;    /**< <tt>\b 0x00ac:</tt> DBB_RFFE RFFE_SPIM_START_TRANSACTION Register */
    __IO uint16_t rffe_spim_data_in;              /**< <tt>\b 0x00ae:</tt> DBB_RFFE RFFE_SPIM_DATA_IN Register */
    __IO uint32_t agc_spi_cfg;                    /**< <tt>\b 0x00b0:</tt> DBB_RFFE AGC_SPI_CFG Register */
    __IO uint16_t agc_enc5_gain_addr;             /**< <tt>\b 0x00b4:</tt> DBB_RFFE AGC_ENC5_GAIN_ADDR Register */
    __IO uint16_t agc_enc5_gain_table[8];         /**< <tt>\b 0x00b6:</tt> DBB_RFFE AGC_ENC5_GAIN_TABLE Register */
    __IO uint16_t agc_enc5_gain_table_db[8];      /**< <tt>\b 0x00c6:</tt> DBB_RFFE AGC_ENC5_GAIN_TABLE_DB Register */
    __IO uint16_t agc_enc5_ready_tmr;             /**< <tt>\b 0x00d6:</tt> DBB_RFFE AGC_ENC5_READY_TMR Register */
    __IO uint16_t agc_enc5_offs_i_addr;           /**< <tt>\b 0x00d8:</tt> DBB_RFFE AGC_ENC5_OFFS_I_ADDR Register */
    __IO uint16_t agc_enc5_offs_q_addr;           /**< <tt>\b 0x00da:</tt> DBB_RFFE AGC_ENC5_OFFS_Q_ADDR Register */
    __IO uint16_t agc_enc5_dc_offs;               /**< <tt>\b 0x00dc:</tt> DBB_RFFE AGC_ENC5_DC_OFFS Register */
    __IO uint16_t agc_enc5_offs_i_bypass[8];      /**< <tt>\b 0x00de:</tt> DBB_RFFE AGC_ENC5_OFFS_I_BYPASS Register */
    __IO uint16_t agc_enc5_offs_q_bypass[8];      /**< <tt>\b 0x00ee:</tt> DBB_RFFE AGC_ENC5_OFFS_Q_BYPASS Register */
    __IO uint16_t agc_enc5_curr_dc_offs_i[8];     /**< <tt>\b 0x00fe:</tt> DBB_RFFE AGC_ENC5_CURR_DC_OFFS_I Register */
    __IO uint16_t agc_enc5_curr_dc_offs_q[8];     /**< <tt>\b 0x010e:</tt> DBB_RFFE AGC_ENC5_CURR_DC_OFFS_Q Register */
    __R  uint16_t rsv_0x11e;
    __IO uint16_t general2_skip_fir_amp;          /**< <tt>\b 0x0120:</tt> DBB_RFFE GENERAL2_SKIP_FIR_AMP Register */
    __IO int16_t  general2_fir_coefficients[4];   /**< <tt>\b 0x0122:</tt> DBB_RFFE GENERAL2_FIR_COEFFICIENTS Register */
    __IO uint16_t general2_fir_scalingfactor;     /**< <tt>\b 0x012a:</tt> DBB_RFFE GENERAL2_FIR_SCALINGFACTOR Register */
    __IO uint16_t anti_alias_am_skip_fir;         /**< <tt>\b 0x012c:</tt> DBB_RFFE ANTI_ALIAS_AM_SKIP_FIR Register */
    __IO uint16_t anti_alias_am_coeff[4];         /**< <tt>\b 0x012e:</tt> DBB_RFFE ANTI_ALIAS_AM_COEFF Register */
    __IO uint16_t anti_alias_am_scalingfactor;    /**< <tt>\b 0x0136:</tt> DBB_RFFE ANTI_ALIAS_AM_SCALINGFACTOR Register */
    __IO uint16_t anti_alias_fm_skip_fir;         /**< <tt>\b 0x0138:</tt> DBB_RFFE ANTI_ALIAS_FM_SKIP_FIR Register */
    __IO uint16_t anti_alias_fm_coeff[4];         /**< <tt>\b 0x013a:</tt> DBB_RFFE ANTI_ALIAS_FM_COEFF Register */
    __IO uint16_t anti_alias_fm_scalingfactor;    /**< <tt>\b 0x0142:</tt> DBB_RFFE ANTI_ALIAS_FM_SCALINGFACTOR Register */
    __IO uint16_t cdc_clk_div_en;                 /**< <tt>\b 0x0144:</tt> DBB_RFFE CDC_CLK_DIV_EN Register */
    __R  uint16_t rsv_0x146_0x163[15];
    __IO uint32_t iq_ctrl;                        /**< <tt>\b 0x0164:</tt> DBB_RFFE IQ_CTRL Register */
    __IO uint16_t iq_data;                        /**< <tt>\b 0x0168:</tt> DBB_RFFE IQ_DATA Register */
} mxc_dbb_rffe_regs_t;

/* Register offsets for module DBB_RFFE */
/**
 * @ingroup    dbb_rffe_registers
 * @defgroup   DBB_RFFE_Register_Offsets Register Offsets
 * @brief      DBB_RFFE Peripheral Register Offsets from the DBB_RFFE Base Peripheral Address.
 * @{
 */
#define MXC_R_DBB_RFFE_RFFE_IFC_VERSION              ((uint32_t)0x00000000UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0000</tt> */
#define MXC_R_DBB_RFFE_RFFE_IFC_RFFE_VERSION         ((uint32_t)0x00000002UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0002</tt> */
#define MXC_R_DBB_RFFE_GENERAL_PARAM                 ((uint32_t)0x00000004UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0004</tt> */
#define MXC_R_DBB_RFFE_GENERAL_DELAY_FREQ            ((uint32_t)0x00000006UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0006</tt> */
#define MXC_R_DBB_RFFE_GENERAL_DELAY_AMP             ((uint32_t)0x00000008UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0008</tt> */
#define MXC_R_DBB_RFFE_GENERAL_SPI_INVERT_CSN        ((uint32_t)0x0000000AUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x000A</tt> */
#define MXC_R_DBB_RFFE_GENERAL_SKIP_SDMOD            ((uint32_t)0x0000000CUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x000C</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_ENA_SEQ_LNGTH_TOTAL    ((uint32_t)0x00000010UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0010</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_ENA_SEQ_LNGTH_PART     ((uint32_t)0x00000012UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0012</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_ENA_CMD                ((uint32_t)0x00000014UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0014</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_DIS_SEQ_LNGTH_TOTAL    ((uint32_t)0x00000034UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0034</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_DIS_SEQ_LNGTH_PART     ((uint32_t)0x00000036UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0036</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_DIS_CMD                ((uint32_t)0x00000038UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0038</tt> */
#define MXC_R_DBB_RFFE_TX_SEQ_SPI                    ((uint32_t)0x00000058UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0058</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_ENA_SEQ_LNGTH_TOTAL    ((uint32_t)0x0000005CUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x005C</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_ENA_SEQ_LNGTH_PART     ((uint32_t)0x0000005EUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x005E</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_ENA_CMD                ((uint32_t)0x00000060UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0060</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_DIS_SEQ_LNGTH_TOTAL    ((uint32_t)0x00000080UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0080</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_DIS_SEQ_LNGTH_PART     ((uint32_t)0x00000082UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0082</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_DIS_CMD                ((uint32_t)0x00000084UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0084</tt> */
#define MXC_R_DBB_RFFE_RX_SEQ_SPI                    ((uint32_t)0x000000A4UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00A4</tt> */
#define MXC_R_DBB_RFFE_RFFE_SPIM_CFG                 ((uint32_t)0x000000A8UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00A8</tt> */
#define MXC_R_DBB_RFFE_RFFE_SPIM_DATA_OUT            ((uint32_t)0x000000AAUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00AA</tt> */
#define MXC_R_DBB_RFFE_RFFE_SPIM_START_TRANSACTION   ((uint32_t)0x000000ACUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00AC</tt> */
#define MXC_R_DBB_RFFE_RFFE_SPIM_DATA_IN             ((uint32_t)0x000000AEUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00AE</tt> */
#define MXC_R_DBB_RFFE_AGC_SPI_CFG                   ((uint32_t)0x000000B0UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00B0</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_GAIN_ADDR            ((uint32_t)0x000000B4UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00B4</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_GAIN_TABLE           ((uint32_t)0x000000B6UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00B6</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_GAIN_TABLE_DB        ((uint32_t)0x000000C6UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00C6</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_READY_TMR            ((uint32_t)0x000000D6UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00D6</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_OFFS_I_ADDR          ((uint32_t)0x000000D8UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00D8</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_OFFS_Q_ADDR          ((uint32_t)0x000000DAUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00DA</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_DC_OFFS              ((uint32_t)0x000000DCUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00DC</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_OFFS_I_BYPASS        ((uint32_t)0x000000DEUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00DE</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_OFFS_Q_BYPASS        ((uint32_t)0x000000EEUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00EE</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_CURR_DC_OFFS_I       ((uint32_t)0x000000FEUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x00FE</tt> */
#define MXC_R_DBB_RFFE_AGC_ENC5_CURR_DC_OFFS_Q       ((uint32_t)0x0000010EUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x010E</tt> */
#define MXC_R_DBB_RFFE_GENERAL2_SKIP_FIR_AMP         ((uint32_t)0x00000120UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0120</tt> */
#define MXC_R_DBB_RFFE_GENERAL2_FIR_COEFFICIENTS     ((uint32_t)0x00000122UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0122</tt> */
#define MXC_R_DBB_RFFE_GENERAL2_FIR_SCALINGFACTOR    ((uint32_t)0x0000012AUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x012A</tt> */
#define MXC_R_DBB_RFFE_ANTI_ALIAS_AM_SKIP_FIR        ((uint32_t)0x0000012CUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x012C</tt> */
#define MXC_R_DBB_RFFE_ANTI_ALIAS_AM_COEFF           ((uint32_t)0x0000012EUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x012E</tt> */
#define MXC_R_DBB_RFFE_ANTI_ALIAS_AM_SCALINGFACTOR   ((uint32_t)0x00000136UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0136</tt> */
#define MXC_R_DBB_RFFE_ANTI_ALIAS_FM_SKIP_FIR        ((uint32_t)0x00000138UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0138</tt> */
#define MXC_R_DBB_RFFE_ANTI_ALIAS_FM_COEFF           ((uint32_t)0x0000013AUL) /**< Offset from DBB_RFFE Base Address: <tt> 0x013A</tt> */
#define MXC_R_DBB_RFFE_ANTI_ALIAS_FM_SCALINGFACTOR   ((uint32_t)0x00000142UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0142</tt> */
#define MXC_R_DBB_RFFE_CDC_CLK_DIV_EN                ((uint32_t)0x00000144UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0144</tt> */
#define MXC_R_DBB_RFFE_IQ_CTRL                       ((uint32_t)0x00000164UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0164</tt> */
#define MXC_R_DBB_RFFE_IQ_DATA                       ((uint32_t)0x00000168UL) /**< Offset from DBB_RFFE Base Address: <tt> 0x0168</tt> */
/**@} end of group dbb_rffe_registers */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_RFFE_IFC_VERSION DBB_RFFE_RFFE_IFC_VERSION
 * @brief    version
 * @{
 */
#define MXC_F_DBB_RFFE_RFFE_IFC_VERSION_MINOR_POS      0 /**< RFFE_IFC_VERSION_MINOR Position */
#define MXC_F_DBB_RFFE_RFFE_IFC_VERSION_MINOR          ((uint16_t)(0xFFUL << MXC_F_DBB_RFFE_RFFE_IFC_VERSION_MINOR_POS)) /**< RFFE_IFC_VERSION_MINOR Mask */

#define MXC_F_DBB_RFFE_RFFE_IFC_VERSION_MAJOR_POS      8 /**< RFFE_IFC_VERSION_MAJOR Position */
#define MXC_F_DBB_RFFE_RFFE_IFC_VERSION_MAJOR          ((uint16_t)(0xFFUL << MXC_F_DBB_RFFE_RFFE_IFC_VERSION_MAJOR_POS)) /**< RFFE_IFC_VERSION_MAJOR Mask */

/**@} end of group DBB_RFFE_RFFE_IFC_VERSION_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_RFFE_IFC_RFFE_VERSION DBB_RFFE_RFFE_IFC_RFFE_VERSION
 * @brief    Select the output mode to the RFFE: '00' = PR400 1G RFFE mode, '01' =  the PR400
 *           2G RFFE mode, '10' = PAN 1G RFFE mode, '11' = PAN 2G mode
 * @{
 */
#define MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_POS 0 /**< RFFE_IFC_RFFE_VERSION_VERSION Position */
#define MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION   ((uint16_t)(0x3UL << MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_POS)) /**< RFFE_IFC_RFFE_VERSION_VERSION Mask */
#define MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PR400_1G ((uint16_t)0x0UL) /**< RFFE_IFC_RFFE_VERSION_VERSION_PR400_1G Value */
#define MXC_S_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PR400_1G (MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PR400_1G << MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_POS) /**< RFFE_IFC_RFFE_VERSION_VERSION_PR400_1G Setting */
#define MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PR400_2G ((uint16_t)0x1UL) /**< RFFE_IFC_RFFE_VERSION_VERSION_PR400_2G Value */
#define MXC_S_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PR400_2G (MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PR400_2G << MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_POS) /**< RFFE_IFC_RFFE_VERSION_VERSION_PR400_2G Setting */
#define MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_1G ((uint16_t)0x2UL) /**< RFFE_IFC_RFFE_VERSION_VERSION_PAN_1G Value */
#define MXC_S_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_1G (MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_1G << MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_POS) /**< RFFE_IFC_RFFE_VERSION_VERSION_PAN_1G Setting */
#define MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G ((uint16_t)0x3UL) /**< RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G Value */
#define MXC_S_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G (MXC_V_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G << MXC_F_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_POS) /**< RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G Setting */

/**@} end of group DBB_RFFE_RFFE_IFC_RFFE_VERSION_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_GENERAL_PARAM DBB_RFFE_GENERAL_PARAM
 * @brief    param
 * @{
 */
#define MXC_F_DBB_RFFE_GENERAL_PARAM_BYP_CDC_POS       0 /**< GENERAL_PARAM_BYP_CDC Position */
#define MXC_F_DBB_RFFE_GENERAL_PARAM_BYP_CDC           ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_GENERAL_PARAM_BYP_CDC_POS)) /**< GENERAL_PARAM_BYP_CDC Mask */

#define MXC_F_DBB_RFFE_GENERAL_PARAM_SWAP_IQ_POS       1 /**< GENERAL_PARAM_SWAP_IQ Position */
#define MXC_F_DBB_RFFE_GENERAL_PARAM_SWAP_IQ           ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_GENERAL_PARAM_SWAP_IQ_POS)) /**< GENERAL_PARAM_SWAP_IQ Mask */

#define MXC_F_DBB_RFFE_GENERAL_PARAM_SWAP_AM_FM_POS    2 /**< GENERAL_PARAM_SWAP_AM_FM Position */
#define MXC_F_DBB_RFFE_GENERAL_PARAM_SWAP_AM_FM        ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_GENERAL_PARAM_SWAP_AM_FM_POS)) /**< GENERAL_PARAM_SWAP_AM_FM Mask */

/**@} end of group DBB_RFFE_GENERAL_PARAM_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_GENERAL_DELAY_FREQ DBB_RFFE_GENERAL_DELAY_FREQ
 * @brief    delay_freq
 * @{
 */
#define MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_LOW_POS      0 /**< GENERAL_DELAY_FREQ_LOW Position */
#define MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_LOW          ((uint16_t)(0x3FUL << MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_LOW_POS)) /**< GENERAL_DELAY_FREQ_LOW Mask */

#define MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_HIGH_POS     6 /**< GENERAL_DELAY_FREQ_HIGH Position */
#define MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_HIGH         ((uint16_t)(0x3FUL << MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_HIGH_POS)) /**< GENERAL_DELAY_FREQ_HIGH Mask */

/**@} end of group DBB_RFFE_GENERAL_DELAY_FREQ_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_GENERAL_SKIP_SDMOD DBB_RFFE_GENERAL_SKIP_SDMOD
 * @brief    skip_sdmod
 * @{
 */
#define MXC_F_DBB_RFFE_GENERAL_SKIP_SDMOD_DIV_POS      0 /**< GENERAL_SKIP_SDMOD_DIV Position */
#define MXC_F_DBB_RFFE_GENERAL_SKIP_SDMOD_DIV          ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_GENERAL_SKIP_SDMOD_DIV_POS)) /**< GENERAL_SKIP_SDMOD_DIV Mask */

#define MXC_F_DBB_RFFE_GENERAL_SKIP_SDMOD_AMP_POS      1 /**< GENERAL_SKIP_SDMOD_AMP Position */
#define MXC_F_DBB_RFFE_GENERAL_SKIP_SDMOD_AMP          ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_GENERAL_SKIP_SDMOD_AMP_POS)) /**< GENERAL_SKIP_SDMOD_AMP Mask */

/**@} end of group DBB_RFFE_GENERAL_SKIP_SDMOD_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_TX_SEQ_SPI DBB_RFFE_TX_SEQ_SPI
 * @brief    spi
 * @{
 */
#define MXC_F_DBB_RFFE_TX_SEQ_SPI_CLK_RATE_POS         0 /**< TX_SEQ_SPI_CLK_RATE Position */
#define MXC_F_DBB_RFFE_TX_SEQ_SPI_CLK_RATE             ((uint32_t)(0xFUL << MXC_F_DBB_RFFE_TX_SEQ_SPI_CLK_RATE_POS)) /**< TX_SEQ_SPI_CLK_RATE Mask */

#define MXC_F_DBB_RFFE_TX_SEQ_SPI_ENDIANESS_POS        4 /**< TX_SEQ_SPI_ENDIANESS Position */
#define MXC_F_DBB_RFFE_TX_SEQ_SPI_ENDIANESS            ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_TX_SEQ_SPI_ENDIANESS_POS)) /**< TX_SEQ_SPI_ENDIANESS Mask */

#define MXC_F_DBB_RFFE_TX_SEQ_SPI_CPOL_POS             5 /**< TX_SEQ_SPI_CPOL Position */
#define MXC_F_DBB_RFFE_TX_SEQ_SPI_CPOL                 ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_TX_SEQ_SPI_CPOL_POS)) /**< TX_SEQ_SPI_CPOL Mask */

#define MXC_F_DBB_RFFE_TX_SEQ_SPI_CPHA_POS             6 /**< TX_SEQ_SPI_CPHA Position */
#define MXC_F_DBB_RFFE_TX_SEQ_SPI_CPHA                 ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_TX_SEQ_SPI_CPHA_POS)) /**< TX_SEQ_SPI_CPHA Mask */

#define MXC_F_DBB_RFFE_TX_SEQ_SPI_ENABLE_POS           7 /**< TX_SEQ_SPI_ENABLE Position */
#define MXC_F_DBB_RFFE_TX_SEQ_SPI_ENABLE               ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_TX_SEQ_SPI_ENABLE_POS)) /**< TX_SEQ_SPI_ENABLE Mask */

/**@} end of group DBB_RFFE_TX_SEQ_SPI_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_RX_SEQ_SPI DBB_RFFE_RX_SEQ_SPI
 * @brief    spi
 * @{
 */
#define MXC_F_DBB_RFFE_RX_SEQ_SPI_CLK_RATE_POS         0 /**< RX_SEQ_SPI_CLK_RATE Position */
#define MXC_F_DBB_RFFE_RX_SEQ_SPI_CLK_RATE             ((uint32_t)(0xFUL << MXC_F_DBB_RFFE_RX_SEQ_SPI_CLK_RATE_POS)) /**< RX_SEQ_SPI_CLK_RATE Mask */

#define MXC_F_DBB_RFFE_RX_SEQ_SPI_ENDIANESS_POS        4 /**< RX_SEQ_SPI_ENDIANESS Position */
#define MXC_F_DBB_RFFE_RX_SEQ_SPI_ENDIANESS            ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_RX_SEQ_SPI_ENDIANESS_POS)) /**< RX_SEQ_SPI_ENDIANESS Mask */

#define MXC_F_DBB_RFFE_RX_SEQ_SPI_CPOL_POS             5 /**< RX_SEQ_SPI_CPOL Position */
#define MXC_F_DBB_RFFE_RX_SEQ_SPI_CPOL                 ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_RX_SEQ_SPI_CPOL_POS)) /**< RX_SEQ_SPI_CPOL Mask */

#define MXC_F_DBB_RFFE_RX_SEQ_SPI_CPHA_POS             6 /**< RX_SEQ_SPI_CPHA Position */
#define MXC_F_DBB_RFFE_RX_SEQ_SPI_CPHA                 ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_RX_SEQ_SPI_CPHA_POS)) /**< RX_SEQ_SPI_CPHA Mask */

#define MXC_F_DBB_RFFE_RX_SEQ_SPI_ENABLE_POS           7 /**< RX_SEQ_SPI_ENABLE Position */
#define MXC_F_DBB_RFFE_RX_SEQ_SPI_ENABLE               ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_RX_SEQ_SPI_ENABLE_POS)) /**< RX_SEQ_SPI_ENABLE Mask */

/**@} end of group DBB_RFFE_RX_SEQ_SPI_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_RFFE_SPIM_CFG DBB_RFFE_RFFE_SPIM_CFG
 * @brief    cfg
 * @{
 */
#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CLK_RATE_POS      0 /**< RFFE_SPIM_CFG_CLK_RATE Position */
#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CLK_RATE          ((uint16_t)(0xFUL << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CLK_RATE_POS)) /**< RFFE_SPIM_CFG_CLK_RATE Mask */

#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENDIANESS_POS     4 /**< RFFE_SPIM_CFG_ENDIANESS Position */
#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENDIANESS         ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENDIANESS_POS)) /**< RFFE_SPIM_CFG_ENDIANESS Mask */

#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPOL_POS          5 /**< RFFE_SPIM_CFG_CPOL Position */
#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPOL              ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPOL_POS)) /**< RFFE_SPIM_CFG_CPOL Mask */

#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPHA_POS          6 /**< RFFE_SPIM_CFG_CPHA Position */
#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPHA              ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPHA_POS)) /**< RFFE_SPIM_CFG_CPHA Mask */

#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENABLE_POS        7 /**< RFFE_SPIM_CFG_ENABLE Position */
#define MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENABLE            ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENABLE_POS)) /**< RFFE_SPIM_CFG_ENABLE Mask */

/**@} end of group DBB_RFFE_RFFE_SPIM_CFG_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_AGC_SPI_CFG DBB_RFFE_AGC_SPI_CFG
 * @brief    cfg
 * @{
 */
#define MXC_F_DBB_RFFE_AGC_SPI_CFG_CLK_RATE_POS        0 /**< AGC_SPI_CFG_CLK_RATE Position */
#define MXC_F_DBB_RFFE_AGC_SPI_CFG_CLK_RATE            ((uint32_t)(0xFUL << MXC_F_DBB_RFFE_AGC_SPI_CFG_CLK_RATE_POS)) /**< AGC_SPI_CFG_CLK_RATE Mask */

#define MXC_F_DBB_RFFE_AGC_SPI_CFG_ENDIANESS_POS       4 /**< AGC_SPI_CFG_ENDIANESS Position */
#define MXC_F_DBB_RFFE_AGC_SPI_CFG_ENDIANESS           ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_AGC_SPI_CFG_ENDIANESS_POS)) /**< AGC_SPI_CFG_ENDIANESS Mask */

#define MXC_F_DBB_RFFE_AGC_SPI_CFG_CPOL_POS            5 /**< AGC_SPI_CFG_CPOL Position */
#define MXC_F_DBB_RFFE_AGC_SPI_CFG_CPOL                ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_AGC_SPI_CFG_CPOL_POS)) /**< AGC_SPI_CFG_CPOL Mask */

#define MXC_F_DBB_RFFE_AGC_SPI_CFG_CPHA_POS            6 /**< AGC_SPI_CFG_CPHA Position */
#define MXC_F_DBB_RFFE_AGC_SPI_CFG_CPHA                ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_AGC_SPI_CFG_CPHA_POS)) /**< AGC_SPI_CFG_CPHA Mask */

#define MXC_F_DBB_RFFE_AGC_SPI_CFG_ENABLE_POS          7 /**< AGC_SPI_CFG_ENABLE Position */
#define MXC_F_DBB_RFFE_AGC_SPI_CFG_ENABLE              ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_AGC_SPI_CFG_ENABLE_POS)) /**< AGC_SPI_CFG_ENABLE Mask */

/**@} end of group DBB_RFFE_AGC_SPI_CFG_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_AGC_ENC5_DC_OFFS DBB_RFFE_AGC_ENC5_DC_OFFS
 * @brief    dc_offs
 * @{
 */
#define MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_ENABLE_POS     0 /**< AGC_ENC5_DC_OFFS_ENABLE Position */
#define MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_ENABLE         ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_ENABLE_POS)) /**< AGC_ENC5_DC_OFFS_ENABLE Mask */

#define MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_BYPASS_EN_POS  1 /**< AGC_ENC5_DC_OFFS_BYPASS_EN Position */
#define MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_BYPASS_EN      ((uint16_t)(0x1UL << MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_BYPASS_EN_POS)) /**< AGC_ENC5_DC_OFFS_BYPASS_EN Mask */

/**@} end of group DBB_RFFE_AGC_ENC5_DC_OFFS_Register */

/**
 * @ingroup  dbb_rffe_registers
 * @defgroup DBB_RFFE_IQ_CTRL DBB_RFFE_IQ_CTRL
 * @brief    iq_ctrl
 * @{
 */
#define MXC_F_DBB_RFFE_IQ_CTRL_ENABLE_POS              0 /**< IQ_CTRL_ENABLE Position */
#define MXC_F_DBB_RFFE_IQ_CTRL_ENABLE                  ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_IQ_CTRL_ENABLE_POS)) /**< IQ_CTRL_ENABLE Mask */

#define MXC_F_DBB_RFFE_IQ_CTRL_DONE_POS                1 /**< IQ_CTRL_DONE Position */
#define MXC_F_DBB_RFFE_IQ_CTRL_DONE                    ((uint32_t)(0x1UL << MXC_F_DBB_RFFE_IQ_CTRL_DONE_POS)) /**< IQ_CTRL_DONE Mask */

/**@} end of group DBB_RFFE_IQ_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // MAX32665_INCLUDE_DBB_RFFE_REGS_H_

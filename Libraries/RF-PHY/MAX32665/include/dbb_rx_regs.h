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
 * @file    dbb_rx_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the DBB_RX Peripheral Module.
 */

#ifndef MAX32665_INCLUDE_DBB_RX_REGS_H_
#define MAX32665_INCLUDE_DBB_RX_REGS_H_

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
 * @ingroup     dbb_rx
 * @defgroup    dbb_rx_registers DBB_RX_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the DBB_RX Peripheral Module.
 * @details PAN2G RX Config
 */

/**
 * @ingroup dbb_rx_registers
 * Structure type to access the DBB_RX Registers.
 */
typedef struct {
    __IO uint32_t rx_info_version;                /**< <tt>\b 0x0000:</tt> DBB_RX RX_INFO_VERSION Register */
    __IO uint16_t rx_general_standard;            /**< <tt>\b 0x0004:</tt> DBB_RX RX_GENERAL_STANDARD Register */
    __IO uint16_t rx_general_control;             /**< <tt>\b 0x0006:</tt> DBB_RX RX_GENERAL_CONTROL Register */
    __IO uint16_t rx_phy_general_control;         /**< <tt>\b 0x0008:</tt> DBB_RX RX_PHY_GENERAL_CONTROL Register */
    __IO uint16_t rx_phy_general_agc_freeze_timer; /**< <tt>\b 0x000a:</tt> DBB_RX RX_PHY_GENERAL_AGC_FREEZE_TIMER Register */
    __IO uint16_t rx_phy_general_sig_detect_init_timer; /**< <tt>\b 0x000c:</tt> DBB_RX RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER Register */
    __IO uint16_t rx_phy_general_sig_detect_rst_timer; /**< <tt>\b 0x000e:</tt> DBB_RX RX_PHY_GENERAL_SIG_DETECT_RST_TIMER Register */
    __IO uint32_t rx_phy_general_ddc_rst_timer;   /**< <tt>\b 0x0010:</tt> DBB_RX RX_PHY_GENERAL_DDC_RST_TIMER Register */
    __IO uint16_t rx_phy_ddc_alpha_start[8];      /**< <tt>\b 0x0014:</tt> DBB_RX RX_PHY_DDC_ALPHA_START Register */
    __IO uint16_t rx_phy_ddc_alpha_stop[8];       /**< <tt>\b 0x0024:</tt> DBB_RX RX_PHY_DDC_ALPHA_STOP Register */
    __IO uint16_t rx_phy_ddc_alpha_step[8];       /**< <tt>\b 0x0034:</tt> DBB_RX RX_PHY_DDC_ALPHA_STEP Register */
    __IO uint16_t rx_phy_ddc_beta[8];             /**< <tt>\b 0x0044:</tt> DBB_RX RX_PHY_DDC_BETA Register */
    __IO uint16_t rx_phy_ddc_rst_tmr;             /**< <tt>\b 0x0054:</tt> DBB_RX RX_PHY_DDC_RST_TMR Register */
    __IO uint16_t rx_phy_ddc_offs_i[8];           /**< <tt>\b 0x0056:</tt> DBB_RX RX_PHY_DDC_OFFS_I Register */
    __IO uint16_t rx_phy_ddc_offs_q[8];           /**< <tt>\b 0x0066:</tt> DBB_RX RX_PHY_DDC_OFFS_Q Register */
    __R  uint16_t rsv_0x76;
    __IO uint16_t rx_phy_agc5_overwr;             /**< <tt>\b 0x0078:</tt> DBB_RX RX_PHY_AGC5_OVERWR Register */
    __IO uint16_t rx_phy_agc5_gain;               /**< <tt>\b 0x007a:</tt> DBB_RX RX_PHY_AGC5_GAIN Register */
    __IO uint16_t rx_phy_agc5_thresh7upper;       /**< <tt>\b 0x007c:</tt> DBB_RX RX_PHY_AGC5_THRESH7UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh7lower;       /**< <tt>\b 0x007e:</tt> DBB_RX RX_PHY_AGC5_THRESH7LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh6upper;       /**< <tt>\b 0x0080:</tt> DBB_RX RX_PHY_AGC5_THRESH6UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh6lower;       /**< <tt>\b 0x0082:</tt> DBB_RX RX_PHY_AGC5_THRESH6LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh5upper;       /**< <tt>\b 0x0084:</tt> DBB_RX RX_PHY_AGC5_THRESH5UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh5lower;       /**< <tt>\b 0x0086:</tt> DBB_RX RX_PHY_AGC5_THRESH5LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh4upper;       /**< <tt>\b 0x0088:</tt> DBB_RX RX_PHY_AGC5_THRESH4UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh4lower;       /**< <tt>\b 0x008a:</tt> DBB_RX RX_PHY_AGC5_THRESH4LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh3upper;       /**< <tt>\b 0x008c:</tt> DBB_RX RX_PHY_AGC5_THRESH3UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh3lower;       /**< <tt>\b 0x008e:</tt> DBB_RX RX_PHY_AGC5_THRESH3LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh2upper;       /**< <tt>\b 0x0090:</tt> DBB_RX RX_PHY_AGC5_THRESH2UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh2lower;       /**< <tt>\b 0x0092:</tt> DBB_RX RX_PHY_AGC5_THRESH2LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh1upper;       /**< <tt>\b 0x0094:</tt> DBB_RX RX_PHY_AGC5_THRESH1UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh1lower;       /**< <tt>\b 0x0096:</tt> DBB_RX RX_PHY_AGC5_THRESH1LOWER Register */
    __IO uint16_t rx_phy_agc5_thresh0upper;       /**< <tt>\b 0x0098:</tt> DBB_RX RX_PHY_AGC5_THRESH0UPPER Register */
    __IO uint16_t rx_phy_agc5_thresh0lower;       /**< <tt>\b 0x009a:</tt> DBB_RX RX_PHY_AGC5_THRESH0LOWER Register */
    __IO uint16_t rx_phy_agc5_row_upper;          /**< <tt>\b 0x009c:</tt> DBB_RX RX_PHY_AGC5_ROW_UPPER Register */
    __IO uint16_t rx_phy_agc5_row_lower;          /**< <tt>\b 0x009e:</tt> DBB_RX RX_PHY_AGC5_ROW_LOWER Register */
    __IO uint16_t rx_phy_agc5_cntr;               /**< <tt>\b 0x00a0:</tt> DBB_RX RX_PHY_AGC5_CNTR Register */
    __IO uint16_t rx_phy_agc5_offs_init;          /**< <tt>\b 0x00a2:</tt> DBB_RX RX_PHY_AGC5_OFFS_INIT Register */
    __IO int16_t  rx_phy_phsmagest_filter_coeff[10]; /**< <tt>\b 0x00a4:</tt> DBB_RX RX_PHY_PHSMAGEST_FILTER_COEFF Register */
    __IO uint32_t rx_phy_phsmagest_scaling_factor; /**< <tt>\b 0x00b8:</tt> DBB_RX RX_PHY_PHSMAGEST_SCALING_FACTOR Register */
    __IO int16_t  rx_phy_filters_coefficients[16]; /**< <tt>\b 0x00bc:</tt> DBB_RX RX_PHY_FILTERS_COEFFICIENTS Register */
    __IO uint32_t rx_phy_filters_settings;        /**< <tt>\b 0x00dc:</tt> DBB_RX RX_PHY_FILTERS_SETTINGS Register */
    __IO uint16_t rx_phy_rssi_settings;           /**< <tt>\b 0x00e0:</tt> DBB_RX RX_PHY_RSSI_SETTINGS Register */
    __IO uint16_t rx_phy_rssi_ed_threshold;       /**< <tt>\b 0x00e2:</tt> DBB_RX RX_PHY_RSSI_ED_THRESHOLD Register */
    __IO uint16_t rx_phy_signal_det_settings;     /**< <tt>\b 0x00e4:</tt> DBB_RX RX_PHY_SIGNAL_DET_SETTINGS Register */
    __IO uint16_t rx_phy_signal_det_thresholds;   /**< <tt>\b 0x00e6:</tt> DBB_RX RX_PHY_SIGNAL_DET_THRESHOLDS Register */
    __IO uint16_t rx_phy_signal_det_select_v2;    /**< <tt>\b 0x00e8:</tt> DBB_RX RX_PHY_SIGNAL_DET_SELECT_V2 Register */
    __IO uint16_t rx_phy_signal_det_settings_v2_div_factor; /**< <tt>\b 0x00ea:</tt> DBB_RX RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR Register */
    __IO uint16_t rx_phy_signal_det_gear_v2;      /**< <tt>\b 0x00ec:</tt> DBB_RX RX_PHY_SIGNAL_DET_GEAR_V2 Register */
    __IO uint16_t rx_phy_signal_det_thresholds_v2_noise_factor; /**< <tt>\b 0x00ee:</tt> DBB_RX RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR Register */
    __IO uint32_t rx_phy_frame_sync_pd_comp;      /**< <tt>\b 0x00f0:</tt> DBB_RX RX_PHY_FRAME_SYNC_PD_COMP Register */
    __IO uint16_t rx_phy_frame_sync_sfd[4];       /**< <tt>\b 0x00f4:</tt> DBB_RX RX_PHY_FRAME_SYNC_SFD Register */
    __IO uint16_t rx_phy_frame_sync_settings;     /**< <tt>\b 0x00fc:</tt> DBB_RX RX_PHY_FRAME_SYNC_SETTINGS Register */
    __IO uint16_t rx_phy_frame_sync_settings_min_corr_val; /**< <tt>\b 0x00fe:</tt> DBB_RX RX_PHY_FRAME_SYNC_SETTINGS_MIN_CORR_VAL Register */
    __IO uint32_t rx_phy_frame_sync_timeout_tmr_periode; /**< <tt>\b 0x0100:</tt> DBB_RX RX_PHY_FRAME_SYNC_TIMEOUT_TMR_PERIODE Register */
    __IO uint16_t rx_phy_timing_sync_cfg;         /**< <tt>\b 0x0104:</tt> DBB_RX RX_PHY_TIMING_SYNC_CFG Register */
    __IO uint16_t rx_phy_timing_sync_log_sum;     /**< <tt>\b 0x0106:</tt> DBB_RX RX_PHY_TIMING_SYNC_LOG_SUM Register */
    __IO uint16_t rx_phy_tim_sync_old_cfg;        /**< <tt>\b 0x0108:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_CFG Register */
    __IO uint16_t rx_phy_tim_sync_old_te_thresh;  /**< <tt>\b 0x010a:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_TE_THRESH Register */
    __IO uint16_t rx_phy_tim_sync_old_te_thresh_scaling; /**< <tt>\b 0x010c:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_TE_THRESH_SCALING Register */
    __IO uint16_t rx_phy_tim_sync_old_init;       /**< <tt>\b 0x010e:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_INIT Register */
    __IO uint16_t rx_phy_tim_sync_old_te_cnt_thresh; /**< <tt>\b 0x0110:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH Register */
    __IO uint16_t rx_phy_tim_sync_old_te_cnt;     /**< <tt>\b 0x0112:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_TE_CNT Register */
    __IO uint16_t rx_phy_tim_sync_old_te_fifo_max_ptr; /**< <tt>\b 0x0114:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_TE_FIFO_MAX_PTR Register */
    __IO uint16_t rx_phy_tim_sync_old_adjust_cnt; /**< <tt>\b 0x0116:</tt> DBB_RX RX_PHY_TIM_SYNC_OLD_ADJUST_CNT Register */
    __IO uint32_t rx_phy_cfo_cfg;                 /**< <tt>\b 0x0118:</tt> DBB_RX RX_PHY_CFO_CFG Register */
    __IO uint16_t rx_phy_cfo_filter_coefficients[9]; /**< <tt>\b 0x011c:</tt> DBB_RX RX_PHY_CFO_FILTER_COEFFICIENTS Register */
    __IO uint16_t rx_phy_cfo_filter_scaling;      /**< <tt>\b 0x012e:</tt> DBB_RX RX_PHY_CFO_FILTER_SCALING Register */
    __IO uint16_t rx_phy_cfo_pa_precnt_num;       /**< <tt>\b 0x0130:</tt> DBB_RX RX_PHY_CFO_PA_PRECNT_NUM Register */
    __IO uint16_t rx_phy_cfo_ph_accum_part_num;   /**< <tt>\b 0x0132:</tt> DBB_RX RX_PHY_CFO_PH_ACCUM_PART_NUM Register */
    __IO uint16_t rx_phy_cfo_ed_thresh_factor;    /**< <tt>\b 0x0134:</tt> DBB_RX RX_PHY_CFO_ED_THRESH_FACTOR Register */
    __IO uint16_t rx_phy_cfo_ed_slow_factor;      /**< <tt>\b 0x0136:</tt> DBB_RX RX_PHY_CFO_ED_SLOW_FACTOR Register */
    __IO uint16_t rx_phy_cfo_ed_fast_factor;      /**< <tt>\b 0x0138:</tt> DBB_RX RX_PHY_CFO_ED_FAST_FACTOR Register */
    __R  uint16_t rsv_0x13a;
    __IO uint32_t rx_phy_cfo_pa_max_limit_val;    /**< <tt>\b 0x013c:</tt> DBB_RX RX_PHY_CFO_PA_MAX_LIMIT_VAL Register */
    __IO uint32_t rx_phy_cfo_cfo_est;             /**< <tt>\b 0x0140:</tt> DBB_RX RX_PHY_CFO_CFO_EST Register */
    __IO uint32_t rx_phy_cfo_cfo_est_khz;         /**< <tt>\b 0x0144:</tt> DBB_RX RX_PHY_CFO_CFO_EST_KHZ Register */
    __IO uint32_t rx_debug_dl_lb;                 /**< <tt>\b 0x0148:</tt> DBB_RX RX_DEBUG_DL_LB Register */
    __IO uint32_t rx_phy_dbg_cfg;                 /**< <tt>\b 0x014c:</tt> DBB_RX RX_PHY_DBG_CFG Register */
    __IO uint32_t rx_phy_dbg_data;                /**< <tt>\b 0x0150:</tt> DBB_RX RX_PHY_DBG_DATA Register */
    __IO uint16_t rx_dl_bypass;                   /**< <tt>\b 0x0154:</tt> DBB_RX RX_DL_BYPASS Register */
    __IO uint16_t rx_dl_crc_mode;                 /**< <tt>\b 0x0156:</tt> DBB_RX RX_DL_CRC_MODE Register */
    __IO uint32_t rx_dl_crc_init_phr;             /**< <tt>\b 0x0158:</tt> DBB_RX RX_DL_CRC_INIT_PHR Register */
    __IO uint32_t rx_dl_crc_init_pld;             /**< <tt>\b 0x015c:</tt> DBB_RX RX_DL_CRC_INIT_PLD Register */
    __IO uint32_t rx_dl_crc_residu;               /**< <tt>\b 0x0160:</tt> DBB_RX RX_DL_CRC_RESIDU Register */
    __IO uint16_t rx_dl_bch;                      /**< <tt>\b 0x0164:</tt> DBB_RX RX_DL_BCH Register */
    __IO uint16_t rx_dl_sprd_seq_err_nr;          /**< <tt>\b 0x0166:</tt> DBB_RX RX_DL_SPRD_SEQ_ERR_NR Register */
    __IO uint32_t rx_dl_ban_settings;             /**< <tt>\b 0x0168:</tt> DBB_RX RX_DL_BAN_SETTINGS Register */
    __IO uint32_t rx_dl_btle_settings_adv_acc_addr; /**< <tt>\b 0x016c:</tt> DBB_RX RX_DL_BTLE_SETTINGS_ADV_ACC_ADDR Register */
    __IO uint16_t rx_dl_btle_settings_whit;       /**< <tt>\b 0x0170:</tt> DBB_RX RX_DL_BTLE_SETTINGS_WHIT Register */
    __IO uint16_t rx_dl_phr_modulation;           /**< <tt>\b 0x0172:</tt> DBB_RX RX_DL_PHR_MODULATION Register */
    __IO uint32_t rx_phy_rssi_out_pwr;            /**< <tt>\b 0x0174:</tt> DBB_RX RX_PHY_RSSI_OUT_PWR Register */
    __IO uint16_t rx_dl_out_crc_stat;             /**< <tt>\b 0x0178:</tt> DBB_RX RX_DL_OUT_CRC_STAT Register */
    __IO uint16_t rx_dl_out_phr;                  /**< <tt>\b 0x017a:</tt> DBB_RX RX_DL_OUT_PHR Register */
    __IO uint16_t rx_dl_out_acc_add_nok;          /**< <tt>\b 0x017c:</tt> DBB_RX RX_DL_OUT_ACC_ADD_NOK Register */
    __IO uint16_t rx_dl_out_ci;                   /**< <tt>\b 0x017e:</tt> DBB_RX RX_DL_OUT_CI Register */
    __IO uint32_t rx_pld_mem[66];                 /**< <tt>\b 0x0180:</tt> DBB_RX RX_PLD_MEM Register */
    __IO uint32_t rx_phy_signal_det2_timeout_tmr_periode; /**< <tt>\b 0x0288:</tt> DBB_RX RX_PHY_SIGNAL_DET2_TIMEOUT_TMR_PERIODE Register */
    __IO uint16_t rx_phy_signal_det2_ed_slow_fact; /**< <tt>\b 0x028c:</tt> DBB_RX RX_PHY_SIGNAL_DET2_ED_SLOW_FACT Register */
    __IO uint16_t rx_phy_signal_det2_ed_fast_fact; /**< <tt>\b 0x028e:</tt> DBB_RX RX_PHY_SIGNAL_DET2_ED_FAST_FACT Register */
    __IO uint32_t rx_phy_signal_det2_ed_threshold_fact; /**< <tt>\b 0x0290:</tt> DBB_RX RX_PHY_SIGNAL_DET2_ED_THRESHOLD_FACT Register */
    __IO uint32_t rx_phy_signal_det2_ed_timeout_tmr_periode; /**< <tt>\b 0x0294:</tt> DBB_RX RX_PHY_SIGNAL_DET2_ED_TIMEOUT_TMR_PERIODE Register */
    __IO uint16_t rx_phy_signal_det2_stop_high_filt_corner; /**< <tt>\b 0x0298:</tt> DBB_RX RX_PHY_SIGNAL_DET2_STOP_HIGH_FILT_CORNER Register */
    __IO uint16_t rx_phy_signal_det2_beta_mux_en_stage; /**< <tt>\b 0x029a:</tt> DBB_RX RX_PHY_SIGNAL_DET2_BETA_MUX_EN_STAGE Register */
    __IO uint32_t rx_phy_signal_det2_beta_mux_threshold; /**< <tt>\b 0x029c:</tt> DBB_RX RX_PHY_SIGNAL_DET2_BETA_MUX_THRESHOLD Register */
    __IO uint32_t rx_phy_signal_det2_gamma_mux_en_stage; /**< <tt>\b 0x02a0:</tt> DBB_RX RX_PHY_SIGNAL_DET2_GAMMA_MUX_EN_STAGE Register */
    __IO uint32_t rx_phy_signal_det2_gamma_mux_threshold; /**< <tt>\b 0x02a4:</tt> DBB_RX RX_PHY_SIGNAL_DET2_GAMMA_MUX_THRESHOLD Register */
    __IO uint16_t rx_phy_signal_det2_alpha[4];    /**< <tt>\b 0x02a8:</tt> DBB_RX RX_PHY_SIGNAL_DET2_ALPHA Register */
    __IO uint16_t rx_phy_signal_det2_beta[4];     /**< <tt>\b 0x02b0:</tt> DBB_RX RX_PHY_SIGNAL_DET2_BETA Register */
    __IO uint16_t rx_phy_signal_det2_gamma[4];    /**< <tt>\b 0x02b8:</tt> DBB_RX RX_PHY_SIGNAL_DET2_GAMMA Register */
    __IO uint32_t rx_phy_cfo2_ed_timeout_tmr_periode; /**< <tt>\b 0x02c0:</tt> DBB_RX RX_PHY_CFO2_ED_TIMEOUT_TMR_PERIODE Register */
    __IO uint16_t rx_phy_cfo2_cfg_ph_accum_upscl_fact; /**< <tt>\b 0x02c4:</tt> DBB_RX RX_PHY_CFO2_CFG_PH_ACCUM_UPSCL_FACT Register */
    __IO uint16_t rx_phy_cfo2_cfg_ph_accum_dwnscl_fact; /**< <tt>\b 0x02c6:</tt> DBB_RX RX_PHY_CFO2_CFG_PH_ACCUM_DWNSCL_FACT Register */
    __IO uint32_t rx_phy_cfo2_cfg_ph_accum_comp;  /**< <tt>\b 0x02c8:</tt> DBB_RX RX_PHY_CFO2_CFG_PH_ACCUM_COMP Register */
    __IO uint16_t rx_phy_btlespeed_anti_alias_enable; /**< <tt>\b 0x02cc:</tt> DBB_RX RX_PHY_BTLESPEED_ANTI_ALIAS_ENABLE Register */
    __IO uint16_t rx_phy_btlespeed_filter_coeff[4]; /**< <tt>\b 0x02ce:</tt> DBB_RX RX_PHY_BTLESPEED_FILTER_COEFF Register */
    __IO uint16_t rx_phy_btlespeed_scaling_factor; /**< <tt>\b 0x02d6:</tt> DBB_RX RX_PHY_BTLESPEED_SCALING_FACTOR Register */
    __IO uint16_t rx_phy_frame_sync_lr_corr_length; /**< <tt>\b 0x02d8:</tt> DBB_RX RX_PHY_FRAME_SYNC_LR_CORR_LENGTH Register */
    __IO uint16_t rx_phy_frame_sync_lr_thresh_factor; /**< <tt>\b 0x02da:</tt> DBB_RX RX_PHY_FRAME_SYNC_LR_THRESH_FACTOR Register */
} mxc_dbb_rx_regs_t;

/* Register offsets for module DBB_RX */
/**
 * @ingroup    dbb_rx_registers
 * @defgroup   DBB_RX_Register_Offsets Register Offsets
 * @brief      DBB_RX Peripheral Register Offsets from the DBB_RX Base Peripheral Address.
 * @{
 */
#define MXC_R_DBB_RX_RX_INFO_VERSION                 ((uint32_t)0x00000000UL) /**< Offset from DBB_RX Base Address: <tt> 0x0000</tt> */
#define MXC_R_DBB_RX_RX_GENERAL_STANDARD             ((uint32_t)0x00000004UL) /**< Offset from DBB_RX Base Address: <tt> 0x0004</tt> */
#define MXC_R_DBB_RX_RX_GENERAL_CONTROL              ((uint32_t)0x00000006UL) /**< Offset from DBB_RX Base Address: <tt> 0x0006</tt> */
#define MXC_R_DBB_RX_RX_PHY_GENERAL_CONTROL          ((uint32_t)0x00000008UL) /**< Offset from DBB_RX Base Address: <tt> 0x0008</tt> */
#define MXC_R_DBB_RX_RX_PHY_GENERAL_AGC_FREEZE_TIMER ((uint32_t)0x0000000AUL) /**< Offset from DBB_RX Base Address: <tt> 0x000A</tt> */
#define MXC_R_DBB_RX_RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER ((uint32_t)0x0000000CUL) /**< Offset from DBB_RX Base Address: <tt> 0x000C</tt> */
#define MXC_R_DBB_RX_RX_PHY_GENERAL_SIG_DETECT_RST_TIMER ((uint32_t)0x0000000EUL) /**< Offset from DBB_RX Base Address: <tt> 0x000E</tt> */
#define MXC_R_DBB_RX_RX_PHY_GENERAL_DDC_RST_TIMER    ((uint32_t)0x00000010UL) /**< Offset from DBB_RX Base Address: <tt> 0x0010</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_ALPHA_START          ((uint32_t)0x00000014UL) /**< Offset from DBB_RX Base Address: <tt> 0x0014</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_ALPHA_STOP           ((uint32_t)0x00000024UL) /**< Offset from DBB_RX Base Address: <tt> 0x0024</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_ALPHA_STEP           ((uint32_t)0x00000034UL) /**< Offset from DBB_RX Base Address: <tt> 0x0034</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_BETA                 ((uint32_t)0x00000044UL) /**< Offset from DBB_RX Base Address: <tt> 0x0044</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_RST_TMR              ((uint32_t)0x00000054UL) /**< Offset from DBB_RX Base Address: <tt> 0x0054</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_OFFS_I               ((uint32_t)0x00000056UL) /**< Offset from DBB_RX Base Address: <tt> 0x0056</tt> */
#define MXC_R_DBB_RX_RX_PHY_DDC_OFFS_Q               ((uint32_t)0x00000066UL) /**< Offset from DBB_RX Base Address: <tt> 0x0066</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_OVERWR              ((uint32_t)0x00000078UL) /**< Offset from DBB_RX Base Address: <tt> 0x0078</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_GAIN                ((uint32_t)0x0000007AUL) /**< Offset from DBB_RX Base Address: <tt> 0x007A</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH7UPPER        ((uint32_t)0x0000007CUL) /**< Offset from DBB_RX Base Address: <tt> 0x007C</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH7LOWER        ((uint32_t)0x0000007EUL) /**< Offset from DBB_RX Base Address: <tt> 0x007E</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH6UPPER        ((uint32_t)0x00000080UL) /**< Offset from DBB_RX Base Address: <tt> 0x0080</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH6LOWER        ((uint32_t)0x00000082UL) /**< Offset from DBB_RX Base Address: <tt> 0x0082</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH5UPPER        ((uint32_t)0x00000084UL) /**< Offset from DBB_RX Base Address: <tt> 0x0084</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH5LOWER        ((uint32_t)0x00000086UL) /**< Offset from DBB_RX Base Address: <tt> 0x0086</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH4UPPER        ((uint32_t)0x00000088UL) /**< Offset from DBB_RX Base Address: <tt> 0x0088</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH4LOWER        ((uint32_t)0x0000008AUL) /**< Offset from DBB_RX Base Address: <tt> 0x008A</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH3UPPER        ((uint32_t)0x0000008CUL) /**< Offset from DBB_RX Base Address: <tt> 0x008C</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH3LOWER        ((uint32_t)0x0000008EUL) /**< Offset from DBB_RX Base Address: <tt> 0x008E</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH2UPPER        ((uint32_t)0x00000090UL) /**< Offset from DBB_RX Base Address: <tt> 0x0090</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH2LOWER        ((uint32_t)0x00000092UL) /**< Offset from DBB_RX Base Address: <tt> 0x0092</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH1UPPER        ((uint32_t)0x00000094UL) /**< Offset from DBB_RX Base Address: <tt> 0x0094</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH1LOWER        ((uint32_t)0x00000096UL) /**< Offset from DBB_RX Base Address: <tt> 0x0096</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH0UPPER        ((uint32_t)0x00000098UL) /**< Offset from DBB_RX Base Address: <tt> 0x0098</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_THRESH0LOWER        ((uint32_t)0x0000009AUL) /**< Offset from DBB_RX Base Address: <tt> 0x009A</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_ROW_UPPER           ((uint32_t)0x0000009CUL) /**< Offset from DBB_RX Base Address: <tt> 0x009C</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_ROW_LOWER           ((uint32_t)0x0000009EUL) /**< Offset from DBB_RX Base Address: <tt> 0x009E</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_CNTR                ((uint32_t)0x000000A0UL) /**< Offset from DBB_RX Base Address: <tt> 0x00A0</tt> */
#define MXC_R_DBB_RX_RX_PHY_AGC5_OFFS_INIT           ((uint32_t)0x000000A2UL) /**< Offset from DBB_RX Base Address: <tt> 0x00A2</tt> */
#define MXC_R_DBB_RX_RX_PHY_PHSMAGEST_FILTER_COEFF   ((uint32_t)0x000000A4UL) /**< Offset from DBB_RX Base Address: <tt> 0x00A4</tt> */
#define MXC_R_DBB_RX_RX_PHY_PHSMAGEST_SCALING_FACTOR ((uint32_t)0x000000B8UL) /**< Offset from DBB_RX Base Address: <tt> 0x00B8</tt> */
#define MXC_R_DBB_RX_RX_PHY_FILTERS_COEFFICIENTS     ((uint32_t)0x000000BCUL) /**< Offset from DBB_RX Base Address: <tt> 0x00BC</tt> */
#define MXC_R_DBB_RX_RX_PHY_FILTERS_SETTINGS         ((uint32_t)0x000000DCUL) /**< Offset from DBB_RX Base Address: <tt> 0x00DC</tt> */
#define MXC_R_DBB_RX_RX_PHY_RSSI_SETTINGS            ((uint32_t)0x000000E0UL) /**< Offset from DBB_RX Base Address: <tt> 0x00E0</tt> */
#define MXC_R_DBB_RX_RX_PHY_RSSI_ED_THRESHOLD        ((uint32_t)0x000000E2UL) /**< Offset from DBB_RX Base Address: <tt> 0x00E2</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS      ((uint32_t)0x000000E4UL) /**< Offset from DBB_RX Base Address: <tt> 0x00E4</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS    ((uint32_t)0x000000E6UL) /**< Offset from DBB_RX Base Address: <tt> 0x00E6</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET_SELECT_V2     ((uint32_t)0x000000E8UL) /**< Offset from DBB_RX Base Address: <tt> 0x00E8</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR ((uint32_t)0x000000EAUL) /**< Offset from DBB_RX Base Address: <tt> 0x00EA</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2       ((uint32_t)0x000000ECUL) /**< Offset from DBB_RX Base Address: <tt> 0x00EC</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR ((uint32_t)0x000000EEUL) /**< Offset from DBB_RX Base Address: <tt> 0x00EE</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP       ((uint32_t)0x000000F0UL) /**< Offset from DBB_RX Base Address: <tt> 0x00F0</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_SFD           ((uint32_t)0x000000F4UL) /**< Offset from DBB_RX Base Address: <tt> 0x00F4</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS      ((uint32_t)0x000000FCUL) /**< Offset from DBB_RX Base Address: <tt> 0x00FC</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_MIN_CORR_VAL ((uint32_t)0x000000FEUL) /**< Offset from DBB_RX Base Address: <tt> 0x00FE</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_TIMEOUT_TMR_PERIODE ((uint32_t)0x00000100UL) /**< Offset from DBB_RX Base Address: <tt> 0x0100</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIMING_SYNC_CFG          ((uint32_t)0x00000104UL) /**< Offset from DBB_RX Base Address: <tt> 0x0104</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIMING_SYNC_LOG_SUM      ((uint32_t)0x00000106UL) /**< Offset from DBB_RX Base Address: <tt> 0x0106</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG         ((uint32_t)0x00000108UL) /**< Offset from DBB_RX Base Address: <tt> 0x0108</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_THRESH   ((uint32_t)0x0000010AUL) /**< Offset from DBB_RX Base Address: <tt> 0x010A</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_THRESH_SCALING ((uint32_t)0x0000010CUL) /**< Offset from DBB_RX Base Address: <tt> 0x010C</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT        ((uint32_t)0x0000010EUL) /**< Offset from DBB_RX Base Address: <tt> 0x010E</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH ((uint32_t)0x00000110UL) /**< Offset from DBB_RX Base Address: <tt> 0x0110</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT      ((uint32_t)0x00000112UL) /**< Offset from DBB_RX Base Address: <tt> 0x0112</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_FIFO_MAX_PTR ((uint32_t)0x00000114UL) /**< Offset from DBB_RX Base Address: <tt> 0x0114</tt> */
#define MXC_R_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT  ((uint32_t)0x00000116UL) /**< Offset from DBB_RX Base Address: <tt> 0x0116</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_CFG                  ((uint32_t)0x00000118UL) /**< Offset from DBB_RX Base Address: <tt> 0x0118</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_FILTER_COEFFICIENTS  ((uint32_t)0x0000011CUL) /**< Offset from DBB_RX Base Address: <tt> 0x011C</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_FILTER_SCALING       ((uint32_t)0x0000012EUL) /**< Offset from DBB_RX Base Address: <tt> 0x012E</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_PA_PRECNT_NUM        ((uint32_t)0x00000130UL) /**< Offset from DBB_RX Base Address: <tt> 0x0130</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_PH_ACCUM_PART_NUM    ((uint32_t)0x00000132UL) /**< Offset from DBB_RX Base Address: <tt> 0x0132</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_ED_THRESH_FACTOR     ((uint32_t)0x00000134UL) /**< Offset from DBB_RX Base Address: <tt> 0x0134</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_ED_SLOW_FACTOR       ((uint32_t)0x00000136UL) /**< Offset from DBB_RX Base Address: <tt> 0x0136</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_ED_FAST_FACTOR       ((uint32_t)0x00000138UL) /**< Offset from DBB_RX Base Address: <tt> 0x0138</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_PA_MAX_LIMIT_VAL     ((uint32_t)0x0000013CUL) /**< Offset from DBB_RX Base Address: <tt> 0x013C</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_CFO_EST              ((uint32_t)0x00000140UL) /**< Offset from DBB_RX Base Address: <tt> 0x0140</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO_CFO_EST_KHZ          ((uint32_t)0x00000144UL) /**< Offset from DBB_RX Base Address: <tt> 0x0144</tt> */
#define MXC_R_DBB_RX_RX_DEBUG_DL_LB                  ((uint32_t)0x00000148UL) /**< Offset from DBB_RX Base Address: <tt> 0x0148</tt> */
#define MXC_R_DBB_RX_RX_PHY_DBG_CFG                  ((uint32_t)0x0000014CUL) /**< Offset from DBB_RX Base Address: <tt> 0x014C</tt> */
#define MXC_R_DBB_RX_RX_PHY_DBG_DATA                 ((uint32_t)0x00000150UL) /**< Offset from DBB_RX Base Address: <tt> 0x0150</tt> */
#define MXC_R_DBB_RX_RX_DL_BYPASS                    ((uint32_t)0x00000154UL) /**< Offset from DBB_RX Base Address: <tt> 0x0154</tt> */
#define MXC_R_DBB_RX_RX_DL_CRC_MODE                  ((uint32_t)0x00000156UL) /**< Offset from DBB_RX Base Address: <tt> 0x0156</tt> */
#define MXC_R_DBB_RX_RX_DL_CRC_INIT_PHR              ((uint32_t)0x00000158UL) /**< Offset from DBB_RX Base Address: <tt> 0x0158</tt> */
#define MXC_R_DBB_RX_RX_DL_CRC_INIT_PLD              ((uint32_t)0x0000015CUL) /**< Offset from DBB_RX Base Address: <tt> 0x015C</tt> */
#define MXC_R_DBB_RX_RX_DL_CRC_RESIDU                ((uint32_t)0x00000160UL) /**< Offset from DBB_RX Base Address: <tt> 0x0160</tt> */
#define MXC_R_DBB_RX_RX_DL_BCH                       ((uint32_t)0x00000164UL) /**< Offset from DBB_RX Base Address: <tt> 0x0164</tt> */
#define MXC_R_DBB_RX_RX_DL_SPRD_SEQ_ERR_NR           ((uint32_t)0x00000166UL) /**< Offset from DBB_RX Base Address: <tt> 0x0166</tt> */
#define MXC_R_DBB_RX_RX_DL_BAN_SETTINGS              ((uint32_t)0x00000168UL) /**< Offset from DBB_RX Base Address: <tt> 0x0168</tt> */
#define MXC_R_DBB_RX_RX_DL_BTLE_SETTINGS_ADV_ACC_ADDR ((uint32_t)0x0000016CUL) /**< Offset from DBB_RX Base Address: <tt> 0x016C</tt> */
#define MXC_R_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT        ((uint32_t)0x00000170UL) /**< Offset from DBB_RX Base Address: <tt> 0x0170</tt> */
#define MXC_R_DBB_RX_RX_DL_PHR_MODULATION            ((uint32_t)0x00000172UL) /**< Offset from DBB_RX Base Address: <tt> 0x0172</tt> */
#define MXC_R_DBB_RX_RX_PHY_RSSI_OUT_PWR             ((uint32_t)0x00000174UL) /**< Offset from DBB_RX Base Address: <tt> 0x0174</tt> */
#define MXC_R_DBB_RX_RX_DL_OUT_CRC_STAT              ((uint32_t)0x00000178UL) /**< Offset from DBB_RX Base Address: <tt> 0x0178</tt> */
#define MXC_R_DBB_RX_RX_DL_OUT_PHR                   ((uint32_t)0x0000017AUL) /**< Offset from DBB_RX Base Address: <tt> 0x017A</tt> */
#define MXC_R_DBB_RX_RX_DL_OUT_ACC_ADD_NOK           ((uint32_t)0x0000017CUL) /**< Offset from DBB_RX Base Address: <tt> 0x017C</tt> */
#define MXC_R_DBB_RX_RX_DL_OUT_CI                    ((uint32_t)0x0000017EUL) /**< Offset from DBB_RX Base Address: <tt> 0x017E</tt> */
#define MXC_R_DBB_RX_RX_PLD_MEM                      ((uint32_t)0x00000180UL) /**< Offset from DBB_RX Base Address: <tt> 0x0180</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_TIMEOUT_TMR_PERIODE ((uint32_t)0x00000288UL) /**< Offset from DBB_RX Base Address: <tt> 0x0288</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_ED_SLOW_FACT ((uint32_t)0x0000028CUL) /**< Offset from DBB_RX Base Address: <tt> 0x028C</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_ED_FAST_FACT ((uint32_t)0x0000028EUL) /**< Offset from DBB_RX Base Address: <tt> 0x028E</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_ED_THRESHOLD_FACT ((uint32_t)0x00000290UL) /**< Offset from DBB_RX Base Address: <tt> 0x0290</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_ED_TIMEOUT_TMR_PERIODE ((uint32_t)0x00000294UL) /**< Offset from DBB_RX Base Address: <tt> 0x0294</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_STOP_HIGH_FILT_CORNER ((uint32_t)0x00000298UL) /**< Offset from DBB_RX Base Address: <tt> 0x0298</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_BETA_MUX_EN_STAGE ((uint32_t)0x0000029AUL) /**< Offset from DBB_RX Base Address: <tt> 0x029A</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_BETA_MUX_THRESHOLD ((uint32_t)0x0000029CUL) /**< Offset from DBB_RX Base Address: <tt> 0x029C</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_GAMMA_MUX_EN_STAGE ((uint32_t)0x000002A0UL) /**< Offset from DBB_RX Base Address: <tt> 0x02A0</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_GAMMA_MUX_THRESHOLD ((uint32_t)0x000002A4UL) /**< Offset from DBB_RX Base Address: <tt> 0x02A4</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_ALPHA        ((uint32_t)0x000002A8UL) /**< Offset from DBB_RX Base Address: <tt> 0x02A8</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_BETA         ((uint32_t)0x000002B0UL) /**< Offset from DBB_RX Base Address: <tt> 0x02B0</tt> */
#define MXC_R_DBB_RX_RX_PHY_SIGNAL_DET2_GAMMA        ((uint32_t)0x000002B8UL) /**< Offset from DBB_RX Base Address: <tt> 0x02B8</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO2_ED_TIMEOUT_TMR_PERIODE ((uint32_t)0x000002C0UL) /**< Offset from DBB_RX Base Address: <tt> 0x02C0</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO2_CFG_PH_ACCUM_UPSCL_FACT ((uint32_t)0x000002C4UL) /**< Offset from DBB_RX Base Address: <tt> 0x02C4</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO2_CFG_PH_ACCUM_DWNSCL_FACT ((uint32_t)0x000002C6UL) /**< Offset from DBB_RX Base Address: <tt> 0x02C6</tt> */
#define MXC_R_DBB_RX_RX_PHY_CFO2_CFG_PH_ACCUM_COMP   ((uint32_t)0x000002C8UL) /**< Offset from DBB_RX Base Address: <tt> 0x02C8</tt> */
#define MXC_R_DBB_RX_RX_PHY_BTLESPEED_ANTI_ALIAS_ENABLE ((uint32_t)0x000002CCUL) /**< Offset from DBB_RX Base Address: <tt> 0x02CC</tt> */
#define MXC_R_DBB_RX_RX_PHY_BTLESPEED_FILTER_COEFF   ((uint32_t)0x000002CEUL) /**< Offset from DBB_RX Base Address: <tt> 0x02CE</tt> */
#define MXC_R_DBB_RX_RX_PHY_BTLESPEED_SCALING_FACTOR ((uint32_t)0x000002D6UL) /**< Offset from DBB_RX Base Address: <tt> 0x02D6</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_LR_CORR_LENGTH ((uint32_t)0x000002D8UL) /**< Offset from DBB_RX Base Address: <tt> 0x02D8</tt> */
#define MXC_R_DBB_RX_RX_PHY_FRAME_SYNC_LR_THRESH_FACTOR ((uint32_t)0x000002DAUL) /**< Offset from DBB_RX Base Address: <tt> 0x02DA</tt> */
/**@} end of group dbb_rx_registers */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_INFO_VERSION DBB_RX_RX_INFO_VERSION
 * @brief    version
 * @{
 */
#define MXC_F_DBB_RX_RX_INFO_VERSION_MINOR_POS         0 /**< RX_INFO_VERSION_MINOR Position */
#define MXC_F_DBB_RX_RX_INFO_VERSION_MINOR             ((uint32_t)(0xFFUL << MXC_F_DBB_RX_RX_INFO_VERSION_MINOR_POS)) /**< RX_INFO_VERSION_MINOR Mask */

#define MXC_F_DBB_RX_RX_INFO_VERSION_MAJOR_POS         8 /**< RX_INFO_VERSION_MAJOR Position */
#define MXC_F_DBB_RX_RX_INFO_VERSION_MAJOR             ((uint32_t)(0xFFUL << MXC_F_DBB_RX_RX_INFO_VERSION_MAJOR_POS)) /**< RX_INFO_VERSION_MAJOR Mask */

/**@} end of group DBB_RX_RX_INFO_VERSION_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_GENERAL_STANDARD DBB_RX_RX_GENERAL_STANDARD
 * @brief    Standard: 1 = Zigbee (15.4), 2 = BAN (15.6),  3 = Bluetooth Low Energy (BTLE)
 * @{
 */
#define MXC_F_DBB_RX_RX_GENERAL_STANDARD_STANDARD_POS  0 /**< RX_GENERAL_STANDARD_STANDARD Position */
#define MXC_F_DBB_RX_RX_GENERAL_STANDARD_STANDARD      ((uint16_t)(0x3UL << MXC_F_DBB_RX_RX_GENERAL_STANDARD_STANDARD_POS)) /**< RX_GENERAL_STANDARD_STANDARD Mask */
#define MXC_V_DBB_RX_RX_GENERAL_STANDARD_STANDARD_ZIGBEE ((uint16_t)0x1UL) /**< RX_GENERAL_STANDARD_STANDARD_ZIGBEE Value */
#define MXC_S_DBB_RX_RX_GENERAL_STANDARD_STANDARD_ZIGBEE (MXC_V_DBB_RX_RX_GENERAL_STANDARD_STANDARD_ZIGBEE << MXC_F_DBB_RX_RX_GENERAL_STANDARD_STANDARD_POS) /**< RX_GENERAL_STANDARD_STANDARD_ZIGBEE Setting */
#define MXC_V_DBB_RX_RX_GENERAL_STANDARD_STANDARD_BAN  ((uint16_t)0x2UL) /**< RX_GENERAL_STANDARD_STANDARD_BAN Value */
#define MXC_S_DBB_RX_RX_GENERAL_STANDARD_STANDARD_BAN  (MXC_V_DBB_RX_RX_GENERAL_STANDARD_STANDARD_BAN << MXC_F_DBB_RX_RX_GENERAL_STANDARD_STANDARD_POS) /**< RX_GENERAL_STANDARD_STANDARD_BAN Setting */
#define MXC_V_DBB_RX_RX_GENERAL_STANDARD_STANDARD_BTLE ((uint16_t)0x3UL) /**< RX_GENERAL_STANDARD_STANDARD_BTLE Value */
#define MXC_S_DBB_RX_RX_GENERAL_STANDARD_STANDARD_BTLE (MXC_V_DBB_RX_RX_GENERAL_STANDARD_STANDARD_BTLE << MXC_F_DBB_RX_RX_GENERAL_STANDARD_STANDARD_POS) /**< RX_GENERAL_STANDARD_STANDARD_BTLE Setting */

/**@} end of group DBB_RX_RX_GENERAL_STANDARD_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_GENERAL_CONTROL DBB_RX_RX_GENERAL_CONTROL
 * @brief    control
 * @{
 */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_AGC_ENABLE_POS 0 /**< RX_GENERAL_CONTROL_AGC_ENABLE Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_AGC_ENABLE     ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_AGC_ENABLE_POS)) /**< RX_GENERAL_CONTROL_AGC_ENABLE Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_RESERVED_POS   1 /**< RX_GENERAL_CONTROL_RESERVED Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_RESERVED       ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_RESERVED_POS)) /**< RX_GENERAL_CONTROL_RESERVED Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_PHY_ENABLE_POS 2 /**< RX_GENERAL_CONTROL_PHY_ENABLE Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_PHY_ENABLE     ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_PHY_ENABLE_POS)) /**< RX_GENERAL_CONTROL_PHY_ENABLE Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_DL_ENABLE_POS  3 /**< RX_GENERAL_CONTROL_DL_ENABLE Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_DL_ENABLE      ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_DL_ENABLE_POS)) /**< RX_GENERAL_CONTROL_DL_ENABLE Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_PH_MG_EST_ENABLE_POS 4 /**< RX_GENERAL_CONTROL_PH_MG_EST_ENABLE Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_PH_MG_EST_ENABLE ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_PH_MG_EST_ENABLE_POS)) /**< RX_GENERAL_CONTROL_PH_MG_EST_ENABLE Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_CFO_HPF_ENABLE_POS 5 /**< RX_GENERAL_CONTROL_CFO_HPF_ENABLE Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_CFO_HPF_ENABLE ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_CFO_HPF_ENABLE_POS)) /**< RX_GENERAL_CONTROL_CFO_HPF_ENABLE Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN_POS 6 /**< RX_GENERAL_CONTROL_BTLE_LNGRNG_EN Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN_POS)) /**< RX_GENERAL_CONTROL_BTLE_LNGRNG_EN Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_DISABLE_TIMING_SYNC_POS 7 /**< RX_GENERAL_CONTROL_DISABLE_TIMING_SYNC Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_DISABLE_TIMING_SYNC ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_DISABLE_TIMING_SYNC_POS)) /**< RX_GENERAL_CONTROL_DISABLE_TIMING_SYNC Mask */

#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE_POS 8 /**< RX_GENERAL_CONTROL_ENCRYPTION_ENABLE Position */
#define MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE_POS)) /**< RX_GENERAL_CONTROL_ENCRYPTION_ENABLE Mask */

/**@} end of group DBB_RX_RX_GENERAL_CONTROL_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_GENERAL_CONTROL DBB_RX_RX_PHY_GENERAL_CONTROL
 * @brief    control
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT_POS 0 /**< RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT Position */
#define MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT_POS)) /**< RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT Mask */

#define MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_START_CCA_POS 1 /**< RX_PHY_GENERAL_CONTROL_START_CCA Position */
#define MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_START_CCA  ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_START_CCA_POS)) /**< RX_PHY_GENERAL_CONTROL_START_CCA Mask */

#define MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_POS 2 /**< RX_PHY_GENERAL_CONTROL_CONT_SYNC Position */
#define MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC  ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_POS)) /**< RX_PHY_GENERAL_CONTROL_CONT_SYNC Mask */
#define MXC_V_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_UNTIL_PLD ((uint16_t)0x0UL) /**< RX_PHY_GENERAL_CONTROL_CONT_SYNC_UNTIL_PLD Value */
#define MXC_S_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_UNTIL_PLD (MXC_V_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_UNTIL_PLD << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_POS) /**< RX_PHY_GENERAL_CONTROL_CONT_SYNC_UNTIL_PLD Setting */
#define MXC_V_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_DURING_PLD ((uint16_t)0x1UL) /**< RX_PHY_GENERAL_CONTROL_CONT_SYNC_DURING_PLD Value */
#define MXC_S_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_DURING_PLD (MXC_V_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_DURING_PLD << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_POS) /**< RX_PHY_GENERAL_CONTROL_CONT_SYNC_DURING_PLD Setting */

/**@} end of group DBB_RX_RX_PHY_GENERAL_CONTROL_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_AGC5_OVERWR DBB_RX_RX_PHY_AGC5_OVERWR
 * @brief    overwr
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_AGC5_OVERWR_WR_GAIN_POS    0 /**< RX_PHY_AGC5_OVERWR_WR_GAIN Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_OVERWR_WR_GAIN        ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_AGC5_OVERWR_WR_GAIN_POS)) /**< RX_PHY_AGC5_OVERWR_WR_GAIN Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_OVERWR_NEW_GAIN_POS   1 /**< RX_PHY_AGC5_OVERWR_NEW_GAIN Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_OVERWR_NEW_GAIN       ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_AGC5_OVERWR_NEW_GAIN_POS)) /**< RX_PHY_AGC5_OVERWR_NEW_GAIN Mask */

/**@} end of group DBB_RX_RX_PHY_AGC5_OVERWR_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_AGC5_GAIN DBB_RX_RX_PHY_AGC5_GAIN
 * @brief    gain
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INIT_POS         0 /**< RX_PHY_AGC5_GAIN_INIT Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INIT             ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INIT_POS)) /**< RX_PHY_AGC5_GAIN_INIT Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MIN_POS          3 /**< RX_PHY_AGC5_GAIN_MIN Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MIN              ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MIN_POS)) /**< RX_PHY_AGC5_GAIN_MIN Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MAX_POS          6 /**< RX_PHY_AGC5_GAIN_MAX Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MAX              ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MAX_POS)) /**< RX_PHY_AGC5_GAIN_MAX Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INDEX_POS        9 /**< RX_PHY_AGC5_GAIN_INDEX Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INDEX            ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INDEX_POS)) /**< RX_PHY_AGC5_GAIN_INDEX Mask */

/**@} end of group DBB_RX_RX_PHY_AGC5_GAIN_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_AGC5_ROW_UPPER DBB_RX_RX_PHY_AGC5_ROW_UPPER
 * @brief    row_upper
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_LE_POS      0 /**< RX_PHY_AGC5_ROW_UPPER_LE Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_LE          ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_LE_POS)) /**< RX_PHY_AGC5_ROW_UPPER_LE Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_INCR_POS    1 /**< RX_PHY_AGC5_ROW_UPPER_INCR Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_INCR        ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_INCR_POS)) /**< RX_PHY_AGC5_ROW_UPPER_INCR Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_BLK_POS     5 /**< RX_PHY_AGC5_ROW_UPPER_BLK Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_BLK         ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_BLK_POS)) /**< RX_PHY_AGC5_ROW_UPPER_BLK Mask */

/**@} end of group DBB_RX_RX_PHY_AGC5_ROW_UPPER_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_AGC5_ROW_LOWER DBB_RX_RX_PHY_AGC5_ROW_LOWER
 * @brief    row_lower
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_LE_POS      0 /**< RX_PHY_AGC5_ROW_LOWER_LE Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_LE          ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_LE_POS)) /**< RX_PHY_AGC5_ROW_LOWER_LE Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_INCR_POS    1 /**< RX_PHY_AGC5_ROW_LOWER_INCR Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_INCR        ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_INCR_POS)) /**< RX_PHY_AGC5_ROW_LOWER_INCR Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_BLK_POS     5 /**< RX_PHY_AGC5_ROW_LOWER_BLK Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_BLK         ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_BLK_POS)) /**< RX_PHY_AGC5_ROW_LOWER_BLK Mask */

/**@} end of group DBB_RX_RX_PHY_AGC5_ROW_LOWER_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_AGC5_CNTR DBB_RX_RX_PHY_AGC5_CNTR
 * @brief    cntr
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE_POS       0 /**< RX_PHY_AGC5_CNTR_STABLE Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE           ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE_POS)) /**< RX_PHY_AGC5_CNTR_STABLE Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS        8 /**< RX_PHY_AGC5_CNTR_VALID Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID            ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS)) /**< RX_PHY_AGC5_CNTR_VALID Mask */

/**@} end of group DBB_RX_RX_PHY_AGC5_CNTR_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_AGC5_OFFS_INIT DBB_RX_RX_PHY_AGC5_OFFS_INIT
 * @brief    offs_init
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_START_POS   0 /**< RX_PHY_AGC5_OFFS_INIT_START Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_START       ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_START_POS)) /**< RX_PHY_AGC5_OFFS_INIT_START Mask */

#define MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_DONE_POS    1 /**< RX_PHY_AGC5_OFFS_INIT_DONE Position */
#define MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_DONE        ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_DONE_POS)) /**< RX_PHY_AGC5_OFFS_INIT_DONE Mask */

/**@} end of group DBB_RX_RX_PHY_AGC5_OFFS_INIT_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_FILTERS_SETTINGS DBB_RX_RX_PHY_FILTERS_SETTINGS
 * @brief    settings
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SKIP_MUX_SEL_POS 0 /**< RX_PHY_FILTERS_SETTINGS_SKIP_MUX_SEL Position */
#define MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SKIP_MUX_SEL ((uint32_t)(0x3FUL << MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SKIP_MUX_SEL_POS)) /**< RX_PHY_FILTERS_SETTINGS_SKIP_MUX_SEL Mask */

#define MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SCALING_FACTOR_POS 6 /**< RX_PHY_FILTERS_SETTINGS_SCALING_FACTOR Position */
#define MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SCALING_FACTOR ((uint32_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SCALING_FACTOR_POS)) /**< RX_PHY_FILTERS_SETTINGS_SCALING_FACTOR Mask */

/**@} end of group DBB_RX_RX_PHY_FILTERS_SETTINGS_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_RSSI_SETTINGS DBB_RX_RX_PHY_RSSI_SETTINGS
 * @brief    settings
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_NUM_CALC_POS 0 /**< RX_PHY_RSSI_SETTINGS_NUM_CALC Position */
#define MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_NUM_CALC     ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_NUM_CALC_POS)) /**< RX_PHY_RSSI_SETTINGS_NUM_CALC Mask */

#define MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_POW_TWO_SAMPLES_POS 4 /**< RX_PHY_RSSI_SETTINGS_POW_TWO_SAMPLES Position */
#define MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_POW_TWO_SAMPLES ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_POW_TWO_SAMPLES_POS)) /**< RX_PHY_RSSI_SETTINGS_POW_TWO_SAMPLES Mask */

#define MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_COMP_FACT_POS 8 /**< RX_PHY_RSSI_SETTINGS_COMP_FACT Position */
#define MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_COMP_FACT    ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_COMP_FACT_POS)) /**< RX_PHY_RSSI_SETTINGS_COMP_FACT Mask */

/**@} end of group DBB_RX_RX_PHY_RSSI_SETTINGS_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS
 * @brief    settings
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR_POS 0 /**< RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR_POS)) /**< RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR Mask */

#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES_POS 4 /**< RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES_POS)) /**< RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES Mask */

/**@} end of group DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS
 * @brief    thresholds
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP_POS 0 /**< RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP_POS)) /**< RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP Mask */

#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR_POS 8 /**< RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR_POS)) /**< RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR Mask */

/**@} end of group DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR
 * @brief    settings_v2
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR_DIV_FACTOR_POS 0 /**< RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR_DIV_FACTOR Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR_DIV_FACTOR ((uint16_t)(0xFUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR_DIV_FACTOR_POS)) /**< RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR_DIV_FACTOR Mask */

/**@} end of group DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_V2_DIV_FACTOR_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2 DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2
 * @brief    gear_v2
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA_POS 0 /**< RX_PHY_SIGNAL_DET_GEAR_V2_DELTA Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA   ((uint16_t)(0x3UL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA_POS)) /**< RX_PHY_SIGNAL_DET_GEAR_V2_DELTA Mask */

#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START_POS 2 /**< RX_PHY_SIGNAL_DET_GEAR_V2_START Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START   ((uint16_t)(0x3UL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START_POS)) /**< RX_PHY_SIGNAL_DET_GEAR_V2_START Mask */

#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX_POS 4 /**< RX_PHY_SIGNAL_DET_GEAR_V2_MAX Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX     ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX_POS)) /**< RX_PHY_SIGNAL_DET_GEAR_V2_MAX Mask */

#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS 7 /**< RX_PHY_SIGNAL_DET_GEAR_V2_STEP Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP    ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS)) /**< RX_PHY_SIGNAL_DET_GEAR_V2_STEP Mask */

/**@} end of group DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR
 * @brief    thresholds_v2
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR_NOISE_FACTOR_POS 0 /**< RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR_NOISE_FACTOR Position */
#define MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR_NOISE_FACTOR ((uint16_t)(0x1FUL << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR_NOISE_FACTOR_POS)) /**< RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR_NOISE_FACTOR Mask */

/**@} end of group DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_V2_NOISE_FACTOR_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP
 * @brief    pd_comp
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_POS 0 /**< RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR Position */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_POS)) /**< RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR Mask */
#define MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_ALL ((uint32_t)0x1UL) /**< RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_ALL Value */
#define MXC_S_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_ALL (MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_ALL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_POS) /**< RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_ALL Setting */
#define MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_FIRST ((uint32_t)0x0UL) /**< RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_FIRST Value */
#define MXC_S_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_FIRST (MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_FIRST << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_POS) /**< RX_PHY_FRAME_SYNC_PD_COMP_COMP_ALL_GAIN_CORR_FIRST Setting */

#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_NUM_SAMPLES_CORR_POS 1 /**< RX_PHY_FRAME_SYNC_PD_COMP_NUM_SAMPLES_CORR Position */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_NUM_SAMPLES_CORR ((uint32_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_NUM_SAMPLES_CORR_POS)) /**< RX_PHY_FRAME_SYNC_PD_COMP_NUM_SAMPLES_CORR Mask */

/**@} end of group DBB_RX_RX_PHY_FRAME_SYNC_PD_COMP_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS
 * @brief    settings
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH_POS 0 /**< RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH Position */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH ((uint16_t)(0x7FUL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH_POS)) /**< RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH Mask */

#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR_POS 7 /**< RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR Position */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR ((uint16_t)(0x7FUL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR_POS)) /**< RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR Mask */

#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_POS 14 /**< RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT Position */
#define MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_POS)) /**< RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT Mask */
#define MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_ALL ((uint16_t)0x1UL) /**< RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_ALL Value */
#define MXC_S_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_ALL (MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_ALL << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_POS) /**< RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_ALL Setting */
#define MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_SAMPLE ((uint16_t)0x0UL) /**< RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_SAMPLE Value */
#define MXC_S_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_SAMPLE (MXC_V_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_SAMPLE << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_POS) /**< RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_SAMPLE Setting */

/**@} end of group DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_TIMING_SYNC_CFG DBB_RX_RX_PHY_TIMING_SYNC_CFG
 * @brief    cfg
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT_POS  0 /**< RX_PHY_TIMING_SYNC_CFG_LIMIT Position */
#define MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT      ((uint16_t)(0x1FFUL << MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT_POS)) /**< RX_PHY_TIMING_SYNC_CFG_LIMIT Mask */

#define MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_BITSHIFT_POS 9 /**< RX_PHY_TIMING_SYNC_CFG_BITSHIFT Position */
#define MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_BITSHIFT   ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_BITSHIFT_POS)) /**< RX_PHY_TIMING_SYNC_CFG_BITSHIFT Mask */

/**@} end of group DBB_RX_RX_PHY_TIMING_SYNC_CFG_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG
 * @brief    Timing sync general config: 0 = Dynamic thresholding mode, 1 = Static
 *           thresholding mode
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_POS   0 /**< RX_PHY_TIM_SYNC_OLD_CFG_CFG Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG       ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_POS)) /**< RX_PHY_TIM_SYNC_OLD_CFG_CFG Mask */
#define MXC_V_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_STATIC ((uint16_t)0x1UL) /**< RX_PHY_TIM_SYNC_OLD_CFG_CFG_STATIC Value */
#define MXC_S_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_STATIC (MXC_V_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_STATIC << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_POS) /**< RX_PHY_TIM_SYNC_OLD_CFG_CFG_STATIC Setting */
#define MXC_V_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_DYNAMIC ((uint16_t)0x0UL) /**< RX_PHY_TIM_SYNC_OLD_CFG_CFG_DYNAMIC Value */
#define MXC_S_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_DYNAMIC (MXC_V_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_DYNAMIC << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_CFG_POS) /**< RX_PHY_TIM_SYNC_OLD_CFG_CFG_DYNAMIC Setting */

/**@} end of group DBB_RX_RX_PHY_TIM_SYNC_OLD_CFG_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT
 * @brief    init
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_SEL_CNT_MAX_POS 0 /**< RX_PHY_TIM_SYNC_OLD_INIT_SEL_CNT_MAX Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_SEL_CNT_MAX ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_SEL_CNT_MAX_POS)) /**< RX_PHY_TIM_SYNC_OLD_INIT_SEL_CNT_MAX Mask */

#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_MIN_NUM_CHIP_POS 8 /**< RX_PHY_TIM_SYNC_OLD_INIT_MIN_NUM_CHIP Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_MIN_NUM_CHIP ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_MIN_NUM_CHIP_POS)) /**< RX_PHY_TIM_SYNC_OLD_INIT_MIN_NUM_CHIP Mask */

/**@} end of group DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH
 * @brief    te_cnt_thresh
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_VALUE_POS 0 /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_VALUE Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_VALUE ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_VALUE_POS)) /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_VALUE Mask */

#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_INCR_VALUE_POS 8 /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_INCR_VALUE Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_INCR_VALUE ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_INCR_VALUE_POS)) /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_INCR_VALUE Mask */

/**@} end of group DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT
 * @brief    te_cnt
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_INCR_VALUE_POS 0 /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_INCR_VALUE Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_INCR_VALUE ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_INCR_VALUE_POS)) /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_INCR_VALUE Mask */

#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_DECR_VALUE_POS 8 /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_DECR_VALUE Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_DECR_VALUE ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_DECR_VALUE_POS)) /**< RX_PHY_TIM_SYNC_OLD_TE_CNT_DECR_VALUE Mask */

/**@} end of group DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT
 * @brief    adjust_cnt
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_FAST_POS 0 /**< RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_FAST Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_FAST ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_FAST_POS)) /**< RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_FAST Mask */

#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_SLOW_POS 8 /**< RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_SLOW Position */
#define MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_SLOW ((uint16_t)(0xFFUL << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_SLOW_POS)) /**< RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_IS_SLOW Mask */

/**@} end of group DBB_RX_RX_PHY_TIM_SYNC_OLD_ADJUST_CNT_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_CFO_CFG DBB_RX_RX_PHY_CFO_CFG
 * @brief    CFO estimation and compensation config general (cfg(0)=enable, cfg(1)=comp.
 *           enable, cfg(2)=comp. data sign invert, cfg(3)=comp. data SW override enable,
 *           cfg(31) and cfg(24:16)=CFO comp. data used in override mode)
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_ENABLE_POS         0 /**< RX_PHY_CFO_CFG_ENABLE Position */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_ENABLE             ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_CFO_CFG_ENABLE_POS)) /**< RX_PHY_CFO_CFG_ENABLE Mask */

#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_ENABLE_POS    1 /**< RX_PHY_CFO_CFG_COMP_ENABLE Position */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_ENABLE        ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_ENABLE_POS)) /**< RX_PHY_CFO_CFG_COMP_ENABLE Mask */

#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_INV_POS       2 /**< RX_PHY_CFO_CFG_COMP_INV Position */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_INV           ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_INV_POS)) /**< RX_PHY_CFO_CFG_COMP_INV Mask */

#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_SW_OVER_POS        3 /**< RX_PHY_CFO_CFG_SW_OVER Position */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_SW_OVER            ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_CFO_CFG_SW_OVER_POS)) /**< RX_PHY_CFO_CFG_SW_OVER Mask */

#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_DATA0_POS     16 /**< RX_PHY_CFO_CFG_COMP_DATA0 Position */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_DATA0         ((uint32_t)(0x1FFUL << MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_DATA0_POS)) /**< RX_PHY_CFO_CFG_COMP_DATA0 Mask */

#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_DATA1_POS     31 /**< RX_PHY_CFO_CFG_COMP_DATA1 Position */
#define MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_DATA1         ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_CFO_CFG_COMP_DATA1_POS)) /**< RX_PHY_CFO_CFG_COMP_DATA1 Mask */

/**@} end of group DBB_RX_RX_PHY_CFO_CFG_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DEBUG_DL_LB DBB_RX_RX_DEBUG_DL_LB
 * @brief    loop back datalink: 0 = normal mode, 1 = loopback mode
 * @{
 */
#define MXC_F_DBB_RX_RX_DEBUG_DL_LB_LOOP_POS           0 /**< RX_DEBUG_DL_LB_LOOP Position */
#define MXC_F_DBB_RX_RX_DEBUG_DL_LB_LOOP               ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_DEBUG_DL_LB_LOOP_POS)) /**< RX_DEBUG_DL_LB_LOOP Mask */
#define MXC_V_DBB_RX_RX_DEBUG_DL_LB_LOOP_LOOPBACK      ((uint32_t)0x1UL) /**< RX_DEBUG_DL_LB_LOOP_LOOPBACK Value */
#define MXC_S_DBB_RX_RX_DEBUG_DL_LB_LOOP_LOOPBACK      (MXC_V_DBB_RX_RX_DEBUG_DL_LB_LOOP_LOOPBACK << MXC_F_DBB_RX_RX_DEBUG_DL_LB_LOOP_POS) /**< RX_DEBUG_DL_LB_LOOP_LOOPBACK Setting */
#define MXC_V_DBB_RX_RX_DEBUG_DL_LB_LOOP_NORMAL        ((uint32_t)0x0UL) /**< RX_DEBUG_DL_LB_LOOP_NORMAL Value */
#define MXC_S_DBB_RX_RX_DEBUG_DL_LB_LOOP_NORMAL        (MXC_V_DBB_RX_RX_DEBUG_DL_LB_LOOP_NORMAL << MXC_F_DBB_RX_RX_DEBUG_DL_LB_LOOP_POS) /**< RX_DEBUG_DL_LB_LOOP_NORMAL Setting */

/**@} end of group DBB_RX_RX_DEBUG_DL_LB_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_PHY_DBG_CFG DBB_RX_RX_PHY_DBG_CFG
 * @brief    RX debug config (cfg(0)=enable, cfg(1)=log to mem enable, cfg(4:2)=data source
 *           (0=IQ (after AGC), 1=IQ (after filter), 2=IQ (before AGC), 3=phase diff, 4=dbg
 *           counter))
 * @{
 */
#define MXC_F_DBB_RX_RX_PHY_DBG_CFG_ENABLE_POS         0 /**< RX_PHY_DBG_CFG_ENABLE Position */
#define MXC_F_DBB_RX_RX_PHY_DBG_CFG_ENABLE             ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_DBG_CFG_ENABLE_POS)) /**< RX_PHY_DBG_CFG_ENABLE Mask */

#define MXC_F_DBB_RX_RX_PHY_DBG_CFG_LOG_POS            1 /**< RX_PHY_DBG_CFG_LOG Position */
#define MXC_F_DBB_RX_RX_PHY_DBG_CFG_LOG                ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_PHY_DBG_CFG_LOG_POS)) /**< RX_PHY_DBG_CFG_LOG Mask */

#define MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS            2 /**< RX_PHY_DBG_CFG_SRC Position */
#define MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC                ((uint32_t)(0x7UL << MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS)) /**< RX_PHY_DBG_CFG_SRC Mask */
#define MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_AFTER_AGC   ((uint32_t)0x0UL) /**< RX_PHY_DBG_CFG_SRC_IQ_AFTER_AGC Value */
#define MXC_S_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_AFTER_AGC   (MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_AFTER_AGC << MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS) /**< RX_PHY_DBG_CFG_SRC_IQ_AFTER_AGC Setting */
#define MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_AFTER_FILTER ((uint32_t)0x1UL) /**< RX_PHY_DBG_CFG_SRC_IQ_AFTER_FILTER Value */
#define MXC_S_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_AFTER_FILTER (MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_AFTER_FILTER << MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS) /**< RX_PHY_DBG_CFG_SRC_IQ_AFTER_FILTER Setting */
#define MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_BEFORE_AGC  ((uint32_t)0x2UL) /**< RX_PHY_DBG_CFG_SRC_IQ_BEFORE_AGC Value */
#define MXC_S_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_BEFORE_AGC  (MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_IQ_BEFORE_AGC << MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS) /**< RX_PHY_DBG_CFG_SRC_IQ_BEFORE_AGC Setting */
#define MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_PHASE          ((uint32_t)0x3UL) /**< RX_PHY_DBG_CFG_SRC_PHASE Value */
#define MXC_S_DBB_RX_RX_PHY_DBG_CFG_SRC_PHASE          (MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_PHASE << MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS) /**< RX_PHY_DBG_CFG_SRC_PHASE Setting */
#define MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_DBG_COUNTER    ((uint32_t)0x4UL) /**< RX_PHY_DBG_CFG_SRC_DBG_COUNTER Value */
#define MXC_S_DBB_RX_RX_PHY_DBG_CFG_SRC_DBG_COUNTER    (MXC_V_DBB_RX_RX_PHY_DBG_CFG_SRC_DBG_COUNTER << MXC_F_DBB_RX_RX_PHY_DBG_CFG_SRC_POS) /**< RX_PHY_DBG_CFG_SRC_DBG_COUNTER Setting */

/**@} end of group DBB_RX_RX_PHY_DBG_CFG_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_CRC_MODE DBB_RX_RX_DL_CRC_MODE
 * @brief    crc_mode
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS            0 /**< RX_DL_CRC_MODE_PHR Position */
#define MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR                ((uint16_t)(0x3UL << MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS)) /**< RX_DL_CRC_MODE_PHR Mask */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_BYP            ((uint16_t)0x0UL) /**< RX_DL_CRC_MODE_PHR_BYP Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PHR_BYP            (MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_BYP << MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS) /**< RX_DL_CRC_MODE_PHR_BYP Setting */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_CRC4           ((uint16_t)0x1UL) /**< RX_DL_CRC_MODE_PHR_CRC4 Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PHR_CRC4           (MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_CRC4 << MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS) /**< RX_DL_CRC_MODE_PHR_CRC4 Setting */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_CRC16          ((uint16_t)0x2UL) /**< RX_DL_CRC_MODE_PHR_CRC16 Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PHR_CRC16          (MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_CRC16 << MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS) /**< RX_DL_CRC_MODE_PHR_CRC16 Setting */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_CRC24          ((uint16_t)0x3UL) /**< RX_DL_CRC_MODE_PHR_CRC24 Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PHR_CRC24          (MXC_V_DBB_RX_RX_DL_CRC_MODE_PHR_CRC24 << MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS) /**< RX_DL_CRC_MODE_PHR_CRC24 Setting */

#define MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS            2 /**< RX_DL_CRC_MODE_PLD Position */
#define MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD                ((uint16_t)(0x3UL << MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS)) /**< RX_DL_CRC_MODE_PLD Mask */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_BYP            ((uint16_t)0x0UL) /**< RX_DL_CRC_MODE_PLD_BYP Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PLD_BYP            (MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_BYP << MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS) /**< RX_DL_CRC_MODE_PLD_BYP Setting */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_CRC4           ((uint16_t)0x1UL) /**< RX_DL_CRC_MODE_PLD_CRC4 Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PLD_CRC4           (MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_CRC4 << MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS) /**< RX_DL_CRC_MODE_PLD_CRC4 Setting */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_CRC16          ((uint16_t)0x2UL) /**< RX_DL_CRC_MODE_PLD_CRC16 Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PLD_CRC16          (MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_CRC16 << MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS) /**< RX_DL_CRC_MODE_PLD_CRC16 Setting */
#define MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_CRC24          ((uint16_t)0x3UL) /**< RX_DL_CRC_MODE_PLD_CRC24 Value */
#define MXC_S_DBB_RX_RX_DL_CRC_MODE_PLD_CRC24          (MXC_V_DBB_RX_RX_DL_CRC_MODE_PLD_CRC24 << MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS) /**< RX_DL_CRC_MODE_PLD_CRC24 Setting */

/**@} end of group DBB_RX_RX_DL_CRC_MODE_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_BCH DBB_RX_RX_DL_BCH
 * @brief    bch
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_BCH_PHR_ENABLE_POS          0 /**< RX_DL_BCH_PHR_ENABLE Position */
#define MXC_F_DBB_RX_RX_DL_BCH_PHR_ENABLE              ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_DL_BCH_PHR_ENABLE_POS)) /**< RX_DL_BCH_PHR_ENABLE Mask */

#define MXC_F_DBB_RX_RX_DL_BCH_PLD_ENABLE_POS          1 /**< RX_DL_BCH_PLD_ENABLE Position */
#define MXC_F_DBB_RX_RX_DL_BCH_PLD_ENABLE              ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_DL_BCH_PLD_ENABLE_POS)) /**< RX_DL_BCH_PLD_ENABLE Mask */

#define MXC_F_DBB_RX_RX_DL_BCH_BIT_ERR_NR_POS          2 /**< RX_DL_BCH_BIT_ERR_NR Position */
#define MXC_F_DBB_RX_RX_DL_BCH_BIT_ERR_NR              ((uint16_t)(0x7FUL << MXC_F_DBB_RX_RX_DL_BCH_BIT_ERR_NR_POS)) /**< RX_DL_BCH_BIT_ERR_NR Mask */

/**@} end of group DBB_RX_RX_DL_BCH_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_SPRD_SEQ_ERR_NR DBB_RX_RX_DL_SPRD_SEQ_ERR_NR
 * @brief    sprd
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_SPRD_SEQ_ERR_NR_SEQ_ERR_NR_POS 0 /**< RX_DL_SPRD_SEQ_ERR_NR_SEQ_ERR_NR Position */
#define MXC_F_DBB_RX_RX_DL_SPRD_SEQ_ERR_NR_SEQ_ERR_NR  ((uint16_t)(0xFFFUL << MXC_F_DBB_RX_RX_DL_SPRD_SEQ_ERR_NR_SEQ_ERR_NR_POS)) /**< RX_DL_SPRD_SEQ_ERR_NR_SEQ_ERR_NR Mask */

/**@} end of group DBB_RX_RX_DL_SPRD_SEQ_ERR_NR_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_BAN_SETTINGS DBB_RX_RX_DL_BAN_SETTINGS
 * @brief    ban_settings
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD_POS 0 /**< RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD Position */
#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD  ((uint32_t)(0x1UL << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD_POS)) /**< RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD Mask */

#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS 1 /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT Position */
#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT  ((uint32_t)(0x7UL << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS)) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT Mask */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 ((uint32_t)0x1UL) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_1 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 ((uint32_t)0x2UL) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_2 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 ((uint32_t)0x3UL) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_4 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 ((uint32_t)0x4UL) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_8 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 ((uint32_t)0x5UL) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_SPRD_FACT_16 Setting */

#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_MDR_LENGTH_POS 4 /**< RX_DL_BAN_SETTINGS_MDR_LENGTH Position */
#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_MDR_LENGTH     ((uint32_t)(0xFUL << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_MDR_LENGTH_POS)) /**< RX_DL_BAN_SETTINGS_MDR_LENGTH Mask */

#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS 8 /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD Position */
#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD   ((uint32_t)(0x7UL << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS)) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD Mask */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK ((uint32_t)0x0UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_OQPSK Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK ((uint32_t)0x1UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DBPSK Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK ((uint32_t)0x2UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_DQPSK Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK ((uint32_t)0x3UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_D8PSK Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK ((uint32_t)0x4UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GMSK Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK ((uint32_t)0x5UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_MOD_GFSK Setting */

#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS 11 /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT Position */
#define MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT ((uint32_t)(0x7UL << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS)) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT Mask */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 ((uint32_t)0x1UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_1 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 ((uint32_t)0x2UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_2 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 ((uint32_t)0x3UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_4 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 ((uint32_t)0x4UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_8 Setting */
#define MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 ((uint32_t)0x5UL) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 Value */
#define MXC_S_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 (MXC_V_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) /**< RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_SPRD_FACT_16 Setting */

/**@} end of group DBB_RX_RX_DL_BAN_SETTINGS_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_BTLE_SETTINGS_WHIT DBB_RX_RX_DL_BTLE_SETTINGS_WHIT
 * @brief    btle_settings_whit
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS 0 /**< RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR Position */
#define MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR  ((uint16_t)(0x3FUL << MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS)) /**< RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR Mask */

#define MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS 6 /**< RX_DL_BTLE_SETTINGS_WHIT_BYPASS Position */
#define MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_BYPASS   ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS)) /**< RX_DL_BTLE_SETTINGS_WHIT_BYPASS Mask */

/**@} end of group DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_PHR_MODULATION DBB_RX_RX_DL_PHR_MODULATION
 * @brief    Header modulation: OQPSK=0,  1 = DBPSK 2 = DQPSK, 3 = D8PSK, 4 = GMSK, 5 = GFSK
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS      0 /**< RX_DL_PHR_MODULATION_MOD Position */
#define MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD          ((uint16_t)(0x7UL << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS)) /**< RX_DL_PHR_MODULATION_MOD Mask */
#define MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_OQPSK    ((uint16_t)0x0UL) /**< RX_DL_PHR_MODULATION_MOD_OQPSK Value */
#define MXC_S_DBB_RX_RX_DL_PHR_MODULATION_MOD_OQPSK    (MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_OQPSK << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS) /**< RX_DL_PHR_MODULATION_MOD_OQPSK Setting */
#define MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_DBPSK    ((uint16_t)0x1UL) /**< RX_DL_PHR_MODULATION_MOD_DBPSK Value */
#define MXC_S_DBB_RX_RX_DL_PHR_MODULATION_MOD_DBPSK    (MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_DBPSK << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS) /**< RX_DL_PHR_MODULATION_MOD_DBPSK Setting */
#define MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_DQPSK    ((uint16_t)0x2UL) /**< RX_DL_PHR_MODULATION_MOD_DQPSK Value */
#define MXC_S_DBB_RX_RX_DL_PHR_MODULATION_MOD_DQPSK    (MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_DQPSK << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS) /**< RX_DL_PHR_MODULATION_MOD_DQPSK Setting */
#define MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_D8PSK    ((uint16_t)0x3UL) /**< RX_DL_PHR_MODULATION_MOD_D8PSK Value */
#define MXC_S_DBB_RX_RX_DL_PHR_MODULATION_MOD_D8PSK    (MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_D8PSK << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS) /**< RX_DL_PHR_MODULATION_MOD_D8PSK Setting */
#define MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_GMSK     ((uint16_t)0x4UL) /**< RX_DL_PHR_MODULATION_MOD_GMSK Value */
#define MXC_S_DBB_RX_RX_DL_PHR_MODULATION_MOD_GMSK     (MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_GMSK << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS) /**< RX_DL_PHR_MODULATION_MOD_GMSK Setting */
#define MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_GFSK     ((uint16_t)0x5UL) /**< RX_DL_PHR_MODULATION_MOD_GFSK Value */
#define MXC_S_DBB_RX_RX_DL_PHR_MODULATION_MOD_GFSK     (MXC_V_DBB_RX_RX_DL_PHR_MODULATION_MOD_GFSK << MXC_F_DBB_RX_RX_DL_PHR_MODULATION_MOD_POS) /**< RX_DL_PHR_MODULATION_MOD_GFSK Setting */

/**@} end of group DBB_RX_RX_DL_PHR_MODULATION_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_OUT_CRC_STAT DBB_RX_RX_DL_OUT_CRC_STAT
 * @brief    crc_stat
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PHR_NOK_POS    0 /**< RX_DL_OUT_CRC_STAT_PHR_NOK Position */
#define MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PHR_NOK        ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PHR_NOK_POS)) /**< RX_DL_OUT_CRC_STAT_PHR_NOK Mask */

#define MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PLD_NOK_POS    1 /**< RX_DL_OUT_CRC_STAT_PLD_NOK Position */
#define MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PLD_NOK        ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PLD_NOK_POS)) /**< RX_DL_OUT_CRC_STAT_PLD_NOK Mask */

/**@} end of group DBB_RX_RX_DL_OUT_CRC_STAT_Register */

/**
 * @ingroup  dbb_rx_registers
 * @defgroup DBB_RX_RX_DL_OUT_CI DBB_RX_RX_DL_OUT_CI
 * @brief    When receiving a BTLE coded phy packet this field will indicate the value of the
 *           CI field (00 = S8 mode, 01=S2 mode, others resevered for future use)
 * @{
 */
#define MXC_F_DBB_RX_RX_DL_OUT_CI_CI_POS               0 /**< RX_DL_OUT_CI_CI Position */
#define MXC_F_DBB_RX_RX_DL_OUT_CI_CI                   ((uint16_t)(0x1UL << MXC_F_DBB_RX_RX_DL_OUT_CI_CI_POS)) /**< RX_DL_OUT_CI_CI Mask */
#define MXC_V_DBB_RX_RX_DL_OUT_CI_CI_S8                ((uint16_t)0x0UL) /**< RX_DL_OUT_CI_CI_S8 Value */
#define MXC_S_DBB_RX_RX_DL_OUT_CI_CI_S8                (MXC_V_DBB_RX_RX_DL_OUT_CI_CI_S8 << MXC_F_DBB_RX_RX_DL_OUT_CI_CI_POS) /**< RX_DL_OUT_CI_CI_S8 Setting */
#define MXC_V_DBB_RX_RX_DL_OUT_CI_CI_S2                ((uint16_t)0x1UL) /**< RX_DL_OUT_CI_CI_S2 Value */
#define MXC_S_DBB_RX_RX_DL_OUT_CI_CI_S2                (MXC_V_DBB_RX_RX_DL_OUT_CI_CI_S2 << MXC_F_DBB_RX_RX_DL_OUT_CI_CI_POS) /**< RX_DL_OUT_CI_CI_S2 Setting */

/**@} end of group DBB_RX_RX_DL_OUT_CI_Register */

#ifdef __cplusplus
}
#endif

#endif // MAX32665_INCLUDE_DBB_RX_REGS_H_

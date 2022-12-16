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
 * @file    pal_bb_phy.c
 * @brief   Functions to get and and set the PHY.
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include "pal_bb_phy.h"
#include "pal_bb_dbb.h"
#include "pal_bb_afe.h"
#include "pal_bb_seq.h"
#include "pal_bb_cfg.h"

#include "afe_regs.h"
#include "dbb_registers.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Workaround for coded mode. Receiver does not recognize packets without */
/* sync_after_sig_det enabled. Works with other transmitters, but not ours... */
#define CODED_SYNC_WORKAROUND 1

/*! \brief      PalBbPhy control block. */
typedef struct {
    uint8_t phy;
    uint8_t options;
} palBbPhyCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

static palBbPhyCb_t palBbPhyCb;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
static void palBbPhySetLpf(uint8_t phy)
{
    uint8_t lpf_mode, reg12, lpf_bw;
    static const uint8_t LPF_2M_SETTING = 0x7;
    static const uint8_t LPF_DEF_SETTING = 0x5;

    PalBbDbbSpiRead(MXC_R_AFE_REG12, &reg12);

    switch (phy) {
    case BB_PHY_BLE_2M:
        lpf_mode = MXC_S_AFE_REG12_LPF_MODE_1M;
        lpf_bw = (LPF_2M_SETTING << MXC_F_AFE_REG12_LPF_BW_POS);
        break;
    default:
        lpf_mode = MXC_S_AFE_REG12_LPF_MODE_500K;
        lpf_bw = (LPF_DEF_SETTING << MXC_F_AFE_REG12_LPF_BW_POS);
        break;
    }

    PAL_BB_SETFIELD(reg12, MXC_F_AFE_REG12_LPF_BW, lpf_bw);
    PAL_BB_SETFIELD(reg12, MXC_F_AFE_REG12_LPF_MODE, lpf_mode);

    PalBbDbbSpiWrite(MXC_R_AFE_REG12, reg12);
}

/*************************************************************************************************/
void palBbPhySet1M(void)
{
    PalBbSeqUpdate(MXC_R_AFE_REGA8, MXC_F_AFE_REGA8_FDEV_D2, MXC_S_AFE_REGA8_FDEV_D2_2K);
    PalBbSeqUpdate(MXC_R_AFE_REGB6, MXC_F_AFE_REGB6_ADC_SF, MXC_S_AFE_REGB6_ADC_SF_D4);

    MXC_DBB_RX->rx_phy_signal_det2_timeout_tmr_periode = RX_PHY_SIGNAL_DET2_TIMEOUT;

    /* LPF bandwidth to ~500 kHz + CFO range */
    palBbPhySetLpf(BB_PHY_BLE_1M);

    /* restore clock standard setting for phy and dl */
    PalBbDbbCmu1M();
    /*  disabled anti aliasing filter enabled the skip */
    MXC_DBB_RFFE->anti_alias_fm_skip_fir = ENABLED;
    MXC_DBB_RX->rx_phy_btlespeed_anti_alias_enable = DISABLED;

    /* tuning for 1Mbps */
    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_signal_det_gear_v2,
                    MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP,
                    (GEAR_V2_STEP_1MBIT << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS));

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_agc5_cntr, MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID,
                    (RX_PHY_AGC5_CNTR_VALID << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS));

    /* Resetting here because we change in high speed */
    MXC_DBB_RX->rx_phy_agc5_thresh2upper = RX_PHY_AGC5_THRESH2UPPER;

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_timing_sync_cfg, MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT,
                    (RX_PHY_TIMING_SYNC_CFG_LIMIT
                     << MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT_POS));

    PAL_BB_SETFIELD(
        MXC_DBB_RX->rx_phy_frame_sync_settings, MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH,
        (RX_PHY_FRAME_SYNC_CORR_LENGTH << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH_POS));

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_frame_sync_settings,
                    MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR,
                    (RX_PHY_FRAME_SYNC_THRESH_FACTOR
                     << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR_POS));

    MXC_DBB_RX->rx_phy_phsmagest_scaling_factor = RX_PHY_PHSMAGEST_SCALING_FACTOR;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    MXC_DBB_RX->rx_phy_ddc_alpha_start[0] = RX_DDC_ALPHA_START0;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[1] = RX_DDC_ALPHA_START1;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[2] = RX_DDC_ALPHA_START2;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[3] = RX_DDC_ALPHA_START3;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[0] = RX_DDC_ALPHA_STOP0;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[1] = RX_DDC_ALPHA_STOP1;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[2] = RX_DDC_ALPHA_STOP2;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[3] = RX_DDC_ALPHA_STOP3;

    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[0] = RX_PHY_PHSMAGEST_FILTER_COEFF_0;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[1] = RX_PHY_PHSMAGEST_FILTER_COEFF_1;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[2] = RX_PHY_PHSMAGEST_FILTER_COEFF_2;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[3] = RX_PHY_PHSMAGEST_FILTER_COEFF_3;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[4] = RX_PHY_PHSMAGEST_FILTER_COEFF_4;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[5] = RX_PHY_PHSMAGEST_FILTER_COEFF_5;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[6] = RX_PHY_PHSMAGEST_FILTER_COEFF_6;

    MXC_DBB_RX->rx_phy_filters_coefficients[0] = RX_PHY_FILTERS_COEFFICIENTS_0;
    MXC_DBB_RX->rx_phy_filters_coefficients[1] = RX_PHY_FILTERS_COEFFICIENTS_1;
    MXC_DBB_RX->rx_phy_filters_coefficients[2] = RX_PHY_FILTERS_COEFFICIENTS_2;
    MXC_DBB_RX->rx_phy_filters_coefficients[3] = RX_PHY_FILTERS_COEFFICIENTS_3;
    MXC_DBB_RX->rx_phy_filters_coefficients[4] = RX_PHY_FILTERS_COEFFICIENTS_4;
    MXC_DBB_RX->rx_phy_filters_coefficients[5] = RX_PHY_FILTERS_COEFFICIENTS_5;
    MXC_DBB_RX->rx_phy_filters_coefficients[6] = RX_PHY_FILTERS_COEFFICIENTS_6;
    MXC_DBB_RX->rx_phy_filters_coefficients[7] = RX_PHY_FILTERS_COEFFICIENTS_7;
    MXC_DBB_RX->rx_phy_filters_coefficients[8] = RX_PHY_FILTERS_COEFFICIENTS_8;
    MXC_DBB_RX->rx_phy_filters_coefficients[9] = RX_PHY_FILTERS_COEFFICIENTS_9;
    MXC_DBB_RX->rx_phy_filters_coefficients[10] = RX_PHY_FILTERS_COEFFICIENTS_10;
    MXC_DBB_RX->rx_phy_filters_coefficients[11] = RX_PHY_FILTERS_COEFFICIENTS_11;
    MXC_DBB_RX->rx_phy_filters_coefficients[12] = RX_PHY_FILTERS_COEFFICIENTS_12;
    MXC_DBB_RX->rx_phy_filters_coefficients[13] = RX_PHY_FILTERS_COEFFICIENTS_13;
    MXC_DBB_RX->rx_phy_filters_coefficients[14] = RX_PHY_FILTERS_COEFFICIENTS_14;
    MXC_DBB_RX->rx_phy_filters_coefficients[15] = RX_PHY_FILTERS_COEFFICIENTS_15;
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    /* signal detection */
    MXC_DBB_RX->rx_phy_general_sig_detect_init_timer = RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_1MBIT;
    MXC_DBB_RX->rx_phy_signal_det_gear_v2 =
        ((GEAR_V2_DELTA << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA_POS) |
         (GEAR_V2_START << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START_POS) |
         (GEAR_V2_MAX << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX_POS) |
         (GEAR_V2_STEP << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS) | 0);
    MXC_DBB_RX->rx_phy_signal_det_thresholds_v2_noise_factor = THRESHOLDS_V2_NOISE_FACTOR;

    PalBbDbbAgcSetDefaultGain();
    PalBbDbbAgcRestoreDcOffs(BB_PHY_BLE_1M);

    /* Set BLE speed mode; 0 = 1Mb/s, 1 = 2Mb/s */
    MXC_DBB_TX->tx_dl2_btle_speed_mode = 0;
    MXC_DBB_TX->tx_dl_btle_longrange = 0;

    MXC_DBB_RX->rx_general_control &= ~(MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN);

    /* set rx sample conversion clk to 8MHz. */
    MXC_DBB_RFFE->cdc_clk_div_en = 0;

    /* adjust for time delay between enable PA and data arriving on AFE */
    MXC_DBB_RFFE->general_delay_freq = 0;

#ifdef CODED_SYNC_WORKAROUND
    /* Disable sync after sig detect */
    MXC_DBB_RX->rx_phy_general_control &=
        ~(MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT);
#endif
}

/*************************************************************************************************/
static void palBbPhySet2M(void)
{
    PalBbSeqUpdate(MXC_R_AFE_REGA8, MXC_F_AFE_REGA8_FDEV_D2, MXC_S_AFE_REGA8_FDEV_D2_4K);
    PalBbSeqUpdate(MXC_R_AFE_REGB6, MXC_F_AFE_REGB6_ADC_SF, MXC_S_AFE_REGB6_ADC_SF_D2);

    MXC_DBB_RX->rx_phy_signal_det2_timeout_tmr_periode = RX_PHY_SIGNAL_DET2_TIMEOUT; /*  RX clocks
                                                                                        cycles  @ 16
                                                                                        MHz   */

    /* LPF bandwidth to ~1000 kHz + CFO range */
    palBbPhySetLpf(BB_PHY_BLE_2M);

    PalBbDbbCmu2M();

    /* disabled anti aliasing filter enabled the skip */
    MXC_DBB_RFFE->anti_alias_fm_skip_fir = ENABLED;
    MXC_DBB_RX->rx_phy_btlespeed_anti_alias_enable = DISABLED;

    /* tuning for 2Mbps */
    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_signal_det_gear_v2,
                    MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP,
                    (GEAR_V2_STEP_2MBIT << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS));

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_agc5_cntr, MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID,
                    (RX_PHY_AGC5_CNTR_VALID_2MBIT << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS));

    MXC_DBB_RX->rx_phy_agc5_thresh2upper = RX_PHY_AGC5_THRESH2UPPER_2MBIT;

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_timing_sync_cfg, MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT,
                    (RX_PHY_TIMING_SYNC_CFG_LIMIT
                     << MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT_POS));

    PAL_BB_SETFIELD(
        MXC_DBB_RX->rx_phy_frame_sync_settings, MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH,
        (RX_PHY_FRAME_SYNC_CORR_LENGTH << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH_POS));

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_frame_sync_settings,
                    MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR,
                    (RX_PHY_FRAME_SYNC_THRESH_FACTOR
                     << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR_POS));

    MXC_DBB_RX->rx_phy_phsmagest_scaling_factor = RX_PHY_PHSMAGEST_SCALING_FACTOR;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    MXC_DBB_RX->rx_phy_ddc_alpha_start[0] = RX_DDC_ALPHA_START0;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[1] = RX_DDC_ALPHA_START1;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[2] = RX_DDC_ALPHA_START2;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[3] = RX_DDC_ALPHA_START3;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[0] = RX_DDC_ALPHA_STOP0;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[1] = RX_DDC_ALPHA_STOP1;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[2] = RX_DDC_ALPHA_STOP2;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[3] = RX_DDC_ALPHA_STOP3;

    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[0] = RX_PHY_PHSMAGEST_FILTER_COEFF_0;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[1] = RX_PHY_PHSMAGEST_FILTER_COEFF_1;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[2] = RX_PHY_PHSMAGEST_FILTER_COEFF_2;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[3] = RX_PHY_PHSMAGEST_FILTER_COEFF_3;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[4] = RX_PHY_PHSMAGEST_FILTER_COEFF_4;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[5] = RX_PHY_PHSMAGEST_FILTER_COEFF_5;
    MXC_DBB_RX->rx_phy_phsmagest_filter_coeff[6] = RX_PHY_PHSMAGEST_FILTER_COEFF_6;

    MXC_DBB_RX->rx_phy_filters_coefficients[0] = RX_2MPHY_FILTERS_COEFFICIENTS_0;
    MXC_DBB_RX->rx_phy_filters_coefficients[1] = RX_2MPHY_FILTERS_COEFFICIENTS_1;
    MXC_DBB_RX->rx_phy_filters_coefficients[2] = RX_2MPHY_FILTERS_COEFFICIENTS_2;
    MXC_DBB_RX->rx_phy_filters_coefficients[3] = RX_2MPHY_FILTERS_COEFFICIENTS_3;
    MXC_DBB_RX->rx_phy_filters_coefficients[4] = RX_2MPHY_FILTERS_COEFFICIENTS_4;
    MXC_DBB_RX->rx_phy_filters_coefficients[5] = RX_2MPHY_FILTERS_COEFFICIENTS_5;
    MXC_DBB_RX->rx_phy_filters_coefficients[6] = RX_2MPHY_FILTERS_COEFFICIENTS_6;
    MXC_DBB_RX->rx_phy_filters_coefficients[7] = RX_2MPHY_FILTERS_COEFFICIENTS_7;
    MXC_DBB_RX->rx_phy_filters_coefficients[8] = RX_2MPHY_FILTERS_COEFFICIENTS_8;
    MXC_DBB_RX->rx_phy_filters_coefficients[9] = RX_2MPHY_FILTERS_COEFFICIENTS_9;
    MXC_DBB_RX->rx_phy_filters_coefficients[10] = RX_2MPHY_FILTERS_COEFFICIENTS_10;
    MXC_DBB_RX->rx_phy_filters_coefficients[11] = RX_2MPHY_FILTERS_COEFFICIENTS_11;
    MXC_DBB_RX->rx_phy_filters_coefficients[12] = RX_2MPHY_FILTERS_COEFFICIENTS_12;
    MXC_DBB_RX->rx_phy_filters_coefficients[13] = RX_2MPHY_FILTERS_COEFFICIENTS_13;
    MXC_DBB_RX->rx_phy_filters_coefficients[14] = RX_2MPHY_FILTERS_COEFFICIENTS_14;
    MXC_DBB_RX->rx_phy_filters_coefficients[15] = RX_2MPHY_FILTERS_COEFFICIENTS_15;
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    /* signal detection */
    MXC_DBB_RX->rx_phy_general_sig_detect_init_timer = RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_2MBIT;
    MXC_DBB_RX->rx_phy_signal_det_gear_v2 =
        ((GEAR_V2_DELTA << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA_POS) |
         (GEAR_V2_START_2MBIT << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START_POS) |
         (GEAR_V2_MAX_2MBIT << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX_POS) |
         (GEAR_V2_STEP << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS) | 0);
    MXC_DBB_RX->rx_phy_signal_det_thresholds_v2_noise_factor = THRESHOLDS_V2_NOISE_FACTOR_2MBIT;

    PalBbDbbAgcSetHsGain();
    PalBbDbbAgcRestoreDcOffs(BB_PHY_BLE_2M);

    MXC_DBB_RX->rx_phy_btlespeed_scaling_factor = 0;

    MXC_DBB_TX->tx_dl2_btle_speed_mode = 1; /* Set BLE speed mode; 0 = 1Mb/s, 1
                                               = 2Mb/s */

    MXC_DBB_TX->tx_dl_btle_longrange = 0;

    MXC_DBB_RX->rx_general_control &= ~(MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN); /* RX long
                                                                                          * range
                                                                                          * off
                                                                                          */
    MXC_DBB_RFFE->cdc_clk_div_en = 1; /* set rx sample conversion clk to 8MHz.
                                       */

    /* adjust for time delay between enable PA and data arriving on AFE */
    static const uint16_t GENERAL_DELAY_FREQ = 0x2D;
    MXC_DBB_RFFE->general_delay_freq =
        ((GENERAL_DELAY_FREQ << MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_LOW_POS) |
         (GENERAL_DELAY_FREQ << MXC_F_DBB_RFFE_GENERAL_DELAY_FREQ_HIGH_POS));

#ifdef CODED_SYNC_WORKAROUND
    /* Disable sync after sig detect */
    MXC_DBB_RX->rx_phy_general_control &=
        ~(MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT);
#endif
}

/*************************************************************************************************/
static void palBbPhySetS2(void)
{
    PalBbSeqUpdate(MXC_R_AFE_REGA8, MXC_F_AFE_REGA8_FDEV_D2, MXC_S_AFE_REGA8_FDEV_D2_2K);
    PalBbSeqUpdate(MXC_R_AFE_REGB6, MXC_F_AFE_REGB6_ADC_SF, MXC_S_AFE_REGB6_ADC_SF_D4);

    MXC_DBB_RX->rx_phy_signal_det2_timeout_tmr_periode = RX_PHY_SIGNAL_DET2_TIMEOUT;

    /* LPF bandwidth to ~500 kHz + CFO range */
    palBbPhySetLpf(BB_PHY_BLE_CODED);

    /* restore clock standard setting for phy and dl */
    PalBbDbbCmu1M();

    MXC_DBB_RFFE->anti_alias_fm_skip_fir = ENABLED; /*  disabled anti aliasing
                                                       filter enabled the skip
                                                     */
    MXC_DBB_RX->rx_phy_btlespeed_anti_alias_enable = DISABLED;

    /* tuning for coded */
    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_signal_det_gear_v2,
                    MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP,
                    (GEAR_V2_STEP << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS));

    PAL_BB_SETFIELD(MXC_DBB_RX->rx_phy_agc5_cntr, MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID,
                    (RX_PHY_AGC5_CNTR_VALID << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS));

    MXC_DBB_RX->rx_phy_agc5_thresh3upper = RX_PHY_AGC5_THRESH3UPPER;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    MXC_DBB_RX->rx_phy_ddc_alpha_start[0] = RX_DDC_ALPHA_START0_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[1] = RX_DDC_ALPHA_START1_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[2] = RX_DDC_ALPHA_START2_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[3] = RX_DDC_ALPHA_START3_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[0] = RX_DDC_ALPHA_STOP0_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[1] = RX_DDC_ALPHA_STOP1_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[2] = RX_DDC_ALPHA_STOP2_CODED;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[3] = RX_DDC_ALPHA_STOP3_CODED;

    MXC_DBB_RX->rx_phy_filters_coefficients[0] = RX_PHY_FILTERS_COEFFICIENTS_0;
    MXC_DBB_RX->rx_phy_filters_coefficients[1] = RX_PHY_FILTERS_COEFFICIENTS_1;
    MXC_DBB_RX->rx_phy_filters_coefficients[2] = RX_PHY_FILTERS_COEFFICIENTS_2;
    MXC_DBB_RX->rx_phy_filters_coefficients[3] = RX_PHY_FILTERS_COEFFICIENTS_3;
    MXC_DBB_RX->rx_phy_filters_coefficients[4] = RX_PHY_FILTERS_COEFFICIENTS_4;
    MXC_DBB_RX->rx_phy_filters_coefficients[5] = RX_PHY_FILTERS_COEFFICIENTS_5;
    MXC_DBB_RX->rx_phy_filters_coefficients[6] = RX_PHY_FILTERS_COEFFICIENTS_6;
    MXC_DBB_RX->rx_phy_filters_coefficients[7] = RX_PHY_FILTERS_COEFFICIENTS_7;
    MXC_DBB_RX->rx_phy_filters_coefficients[8] = RX_PHY_FILTERS_COEFFICIENTS_8;
    MXC_DBB_RX->rx_phy_filters_coefficients[9] = RX_PHY_FILTERS_COEFFICIENTS_9;
    MXC_DBB_RX->rx_phy_filters_coefficients[10] = RX_PHY_FILTERS_COEFFICIENTS_10;
    MXC_DBB_RX->rx_phy_filters_coefficients[11] = RX_PHY_FILTERS_COEFFICIENTS_11;
    MXC_DBB_RX->rx_phy_filters_coefficients[12] = RX_PHY_FILTERS_COEFFICIENTS_12;
    MXC_DBB_RX->rx_phy_filters_coefficients[13] = RX_PHY_FILTERS_COEFFICIENTS_13;
    MXC_DBB_RX->rx_phy_filters_coefficients[14] = RX_PHY_FILTERS_COEFFICIENTS_14;
    MXC_DBB_RX->rx_phy_filters_coefficients[15] = RX_PHY_FILTERS_COEFFICIENTS_15;
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    /* signal detection */
    MXC_DBB_RX->rx_phy_general_sig_detect_init_timer = RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_CODED;
    MXC_DBB_RX->rx_phy_signal_det_gear_v2 =
        ((GEAR_V2_DELTA << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA_POS) |
         (GEAR_V2_START_CODED << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START_POS) |
         (GEAR_V2_MAX_CODED << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX_POS) |
         (GEAR_V2_STEP << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS) | 0);
    MXC_DBB_RX->rx_phy_signal_det_thresholds_v2_noise_factor = THRESHOLDS_V2_NOISE_FACTOR_CODED;

    PalBbDbbAgcSetDefaultGain();
    PalBbDbbAgcRestoreDcOffs(BB_PHY_BLE_CODED);

    MXC_DBB_TX->tx_dl2_btle_speed_mode = 0; /* Set BLE speed mode; 0 = 1Mb/s, 1
                                               = 2Mb/s */

    MXC_DBB_TX->tx_dl_btle_longrange = MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_FECENC_EN;

    MXC_DBB_RX->rx_general_control |= (MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN); /* RX long
                                                                                           range on
                                                                                         */
    MXC_DBB_RFFE->cdc_clk_div_en = 0; /* set rx sample conversion clk to 8MHz.
                                       */

    /* adjust for time delay between enable PA and data arriving on AFE */
    MXC_DBB_RFFE->general_delay_freq = 0;

#ifdef CODED_SYNC_WORKAROUND
    /* Enable sync after signal detect */
    MXC_DBB_RX->rx_phy_general_control |= MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT;
#endif
}

/*************************************************************************************************/
static void palBbPhySetS8(void)
{
    palBbPhySetS2();
    MXC_DBB_TX->tx_dl_btle_longrange |= MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8;
}

/*************************************************************************************************/
bool_t PalBbPhyInit(uint8_t phy, uint8_t options)
{
    /* Clear the control block PHY settings to force the update */
    palBbPhyCb.phy = 0;
    palBbPhyCb.options = BB_PHY_OPTIONS_DEFAULT;

    return PalBbPhySet(phy, options);
}

/*************************************************************************************************/
bool_t PalBbPhySet(uint8_t phy, uint8_t options)
{
    /* Check to see if the current control block settings */
    if ((phy == palBbPhyCb.phy) && (options == palBbPhyCb.options)) {
        return TRUE;
    }

    /* Save the current settings */
    palBbPhyCb.phy = phy;
    palBbPhyCb.options = options;

    switch (phy) {
    default:
    case BB_PHY_BLE_1M:
        palBbPhySet1M();
        break;
    case BB_PHY_BLE_2M:
        palBbPhySet2M();
        break;
    case BB_PHY_BLE_CODED: {
        if (options == BB_PHY_OPTIONS_BLE_S8) {
            palBbPhySetS8();
        } else {
            palBbPhySetS2();
        }
        break;
    }
    }

    return TRUE;
}

/*************************************************************************************************/
uint8_t PalBbPhyGetPhy(void)
{
    return palBbPhyCb.phy;
}

/*************************************************************************************************/
uint8_t PalBbPhyGetOptions(void)
{
    return palBbPhyCb.options;
}

/*************************************************************************************************/
uint32_t PalBbPhyGetTxStartup(void)
{
    switch (palBbPhyCb.phy) {
    default:
    case BB_PHY_BLE_1M:
        return TXSTARTUP_1M_USEC;
        break;
    case BB_PHY_BLE_2M:
        return TXSTARTUP_2M_USEC;
        break;
    case BB_PHY_BLE_CODED: {
        // if (palBbPhyCb.options == BB_PHY_OPTIONS_BLE_S8) {
        // } else { /* BB_PHY_OPTIONS_BLE_S2 */
        // }
        break;
    }
    }

    return 0;
}
/*************************************************************************************************/
void PalBbPhyEnableTIFS(void)
{
    uint32_t txStartup, txEarlyIRQ, rxStartup, rxLateIRQ;
    /* Account for PHY and different TIFS timings */

    switch (palBbPhyCb.phy) {
    default:
    case BB_PHY_BLE_1M:
        txStartup = TXSTARTUP_1M_USEC;
        txEarlyIRQ = TXEARLYIRQ_1M_USEC;
        rxStartup = RXSTARTUP_1M_USEC;
        rxLateIRQ = RXLATEIRQ_1M_USEC;
        break;
    case BB_PHY_BLE_2M:
        txStartup = TXSTARTUP_2M_USEC;
        txEarlyIRQ = TXEARLYIRQ_2M_USEC;
        rxStartup = RXSTARTUP_2M_USEC;
        rxLateIRQ = RXLATEIRQ_2M_USEC;
        break;
    case BB_PHY_BLE_CODED: {
        // if (palBbPhyCb.options == BB_PHY_OPTIONS_BLE_S8) {
        // } else { /* BB_PHY_OPTIONS_BLE_S2 */
        // }
        txStartup = TXSTARTUP_1M_USEC;
        txEarlyIRQ = TXEARLYIRQ_1M_USEC;
        rxStartup = RXSTARTUP_1M_USEC;
        rxLateIRQ = RXLATEIRQ_1M_USEC;
        break;
    }
    }

    MXC_DBB_CTRL->rx_ctrl_dbb_en_dly =
        (RX_ENABLE_DBB_DELAY + txEarlyIRQ - rxStartup) * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->tx_ctrl_dbb_en_dly =
        (TX_ENABLE_DBB_DELAY - rxLateIRQ - txStartup) * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
}

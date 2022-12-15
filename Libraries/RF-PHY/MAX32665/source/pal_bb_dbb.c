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
 * @file    pal_bb_dbb.c
 * @brief   Functions use the digital base band.
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/
#include <string.h>
#include "pal_bb_dbb.h"
#include "pal_bb_afe.h"
#include "pal_bb_phy.h"
#include "pal_bb_cfg.h"
#include "dbb_registers.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#define PAL_BB_DBB_SPI_TIMEOUT 100

/*! \brief      PalBbDbb control block. */
typedef struct {
    uint8_t dc_offs_i_1M[GAIN_STEPS * 2];
    uint8_t dc_offs_q_1M[GAIN_STEPS * 2];

    uint8_t dc_offs_i_2M[GAIN_STEPS * 2];
    uint8_t dc_offs_q_2M[GAIN_STEPS * 2];
} palBbDbbCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

static palBbDbbCb_t palBbDbbCb;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Functions
**************************************************************************************************/

void PalBbDbbDelay(uint32_t us)
{
    uint32_t startCnt;

    startCnt = PalBbGetCurrentTime();
    while ((PalBbGetCurrentTime() - startCnt) < us) {}
}

static void palBbDbbSpiEnable(void)
{
    MXC_DBB_CTRL->rffe_pmu_ctrl |= MXC_F_DBB_CTRL_RFFE_PMU_CTRL_WAKEUP;

    // Wait for the interface to power on
    while (!(MXC_DBB_CTRL->gen_pmu_status & MXC_F_DBB_CTRL_GEN_PMU_STATUS_RFE)) {}

    PalBbDbbDelay(3);
}

bool_t PalBbDbbSpiWrite(uint8_t address, uint8_t data)
{
    bool_t retval = TRUE;
    volatile unsigned loop_count;

    palBbDbbSpiEnable();

    /* Set the write bit */
    address |= PAL_BB_DBB_SPI_WRITE_BIT;

    /* Clear the ready bit */
    MXC_DBB_CTRL->events_status &= ~(MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY);

    /* Write the address and start the transaction */
    MXC_DBB_RFFE->rffe_spim_data_out = ((address << PAL_BB_DBB_SPI_ADDR_POS) | data);

    /* Start data transaction */
    MXC_DBB_RFFE->rffe_spim_start_transaction = 0x1;

    /* Wait for the operation to complete, check for timeout */
    loop_count = PAL_BB_DBB_SPI_TIMEOUT;
    while (!(MXC_DBB_CTRL->events_status & MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY) &&
           loop_count--) {}

    if (loop_count == 0) {
        retval = FALSE;
    }

    return retval;
}

bool_t PalBbDbbSpiRead(uint8_t address, uint8_t *data)
{
    bool_t retval = TRUE;
    volatile unsigned loop_count;

    palBbDbbSpiEnable();

    /* Clear the write bit */
    address &= ~(PAL_BB_DBB_SPI_WRITE_BIT);

    /* Clear the ready bit */
    MXC_DBB_CTRL->events_status &= ~(MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY);

    /* Write the address and start the transaction */
    MXC_DBB_RFFE->rffe_spim_data_out = (address << PAL_BB_DBB_SPI_ADDR_POS);

    /* Start data transaction */
    MXC_DBB_RFFE->rffe_spim_start_transaction = 0x1;

    /* Wait for the operation to complete, check for timeout */
    loop_count = PAL_BB_DBB_SPI_TIMEOUT;
    while (!(MXC_DBB_CTRL->events_status & MXC_F_DBB_CTRL_EVENTS_STATUS_RFE_SPIM_RDY) &&
           loop_count--) {}

    if (loop_count == 0) {
        retval = FALSE;
    }

    *data = MXC_DBB_RFFE->rffe_spim_data_in;

    return retval;
}

/*************************************************************************************************/
static void palBbSpiInit(void)
{
#ifdef RFFE_EXTERNAL
    static const uint8_t endianess = 1;
    static const uint8_t clk_rate = 0;
#else
    static const uint8_t endianess = 0;
    static const uint8_t clk_rate = 2;
#endif

    uint16_t spi_setup =
        ((clk_rate << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CLK_RATE_POS) |
         (endianess << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENDIANESS_POS) |
         (1 << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPOL_POS) |
         (0 << MXC_F_DBB_RFFE_RFFE_SPIM_CFG_CPHA_POS) | MXC_F_DBB_RFFE_RFFE_SPIM_CFG_ENABLE);

    MXC_DBB_RFFE->rffe_spim_cfg = spi_setup;
    MXC_DBB_RFFE->tx_seq_spi = spi_setup;
    MXC_DBB_RFFE->rx_seq_spi = spi_setup;
    MXC_DBB_RFFE->agc_spi_cfg = spi_setup;

    MXC_DBB_RFFE->general_spi_invert_csn = 1;
}

/*************************************************************************************************/
void palBbDbbSetDdcActive(void)
{
    MXC_DBB_RX->rx_phy_general_ddc_rst_timer = RX_PHY_GENERAL_DDC_RST_TIMER;

    MXC_DBB_RX->rx_phy_ddc_alpha_start[0] = RX_DDC_ALPHA_START0;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[1] = RX_DDC_ALPHA_START1;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[2] = RX_DDC_ALPHA_START2;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[3] = RX_DDC_ALPHA_START3;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[4] = RX_DDC_ALPHA_START4;

    MXC_DBB_RX->rx_phy_ddc_alpha_stop[0] = RX_DDC_ALPHA_STOP0;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[1] = RX_DDC_ALPHA_STOP1;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[2] = RX_DDC_ALPHA_STOP2;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[3] = RX_DDC_ALPHA_STOP3;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[4] = RX_DDC_ALPHA_STOP4;

    MXC_DBB_RX->rx_phy_ddc_alpha_step[0] = RX_DDC_ALPHA_STEP0;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[1] = RX_DDC_ALPHA_STEP1;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[2] = RX_DDC_ALPHA_STEP2;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[3] = RX_DDC_ALPHA_STEP3;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[4] = RX_DDC_ALPHA_STEP4;

    MXC_DBB_RX->rx_phy_ddc_beta[0] = RX_DDC_BETA0;
    MXC_DBB_RX->rx_phy_ddc_beta[1] = RX_DDC_BETA1;
    MXC_DBB_RX->rx_phy_ddc_beta[2] = RX_DDC_BETA2;
    MXC_DBB_RX->rx_phy_ddc_beta[3] = RX_DDC_BETA3;
    MXC_DBB_RX->rx_phy_ddc_beta[4] = RX_DDC_BETA4;

    MXC_DBB_RX->rx_phy_ddc_rst_tmr = RX_PHY_DDC_RST_TMR;

    /* DDC offset; digital offset table just in front of the DDC */

    MXC_DBB_RX->rx_phy_ddc_offs_i[0] = RX_PHY_DDC_OFFS_I0;
    MXC_DBB_RX->rx_phy_ddc_offs_i[1] = RX_PHY_DDC_OFFS_I1;
    MXC_DBB_RX->rx_phy_ddc_offs_i[2] = RX_PHY_DDC_OFFS_I2;
    MXC_DBB_RX->rx_phy_ddc_offs_i[3] = RX_PHY_DDC_OFFS_I3;

    MXC_DBB_RX->rx_phy_ddc_offs_q[0] = RX_PHY_DDC_OFFS_Q0;
    MXC_DBB_RX->rx_phy_ddc_offs_q[1] = RX_PHY_DDC_OFFS_Q1;
    MXC_DBB_RX->rx_phy_ddc_offs_q[2] = RX_PHY_DDC_OFFS_Q2;
    MXC_DBB_RX->rx_phy_ddc_offs_q[3] = RX_PHY_DDC_OFFS_Q3;
    MXC_DBB_RX->rx_phy_ddc_offs_i[4] = RX_PHY_DDC_OFFS_I4;
    MXC_DBB_RX->rx_phy_ddc_offs_q[4] = RX_PHY_DDC_OFFS_Q4;

    MXC_DBB_RFFE->agc_enc5_offs_i_bypass[0] = DIBS_AGC5_ENC_OFFS_I_BYPASS0;
    MXC_DBB_RFFE->agc_enc5_offs_i_bypass[1] = DIBS_AGC5_ENC_OFFS_I_BYPASS1;
    MXC_DBB_RFFE->agc_enc5_offs_i_bypass[2] = DIBS_AGC5_ENC_OFFS_I_BYPASS2;
    MXC_DBB_RFFE->agc_enc5_offs_i_bypass[3] = DIBS_AGC5_ENC_OFFS_I_BYPASS3;

    MXC_DBB_RFFE->agc_enc5_offs_q_bypass[0] = DIBS_AGC5_ENC_OFFS_Q_BYPASS0;
    MXC_DBB_RFFE->agc_enc5_offs_q_bypass[1] = DIBS_AGC5_ENC_OFFS_Q_BYPASS1;
    MXC_DBB_RFFE->agc_enc5_offs_q_bypass[2] = DIBS_AGC5_ENC_OFFS_Q_BYPASS2;
    MXC_DBB_RFFE->agc_enc5_offs_q_bypass[3] = DIBS_AGC5_ENC_OFFS_Q_BYPASS3;
    MXC_DBB_RFFE->agc_enc5_offs_i_bypass[4] = DIBS_AGC5_ENC_OFFS_I_BYPASS4;
    MXC_DBB_RFFE->agc_enc5_offs_q_bypass[4] = DIBS_AGC5_ENC_OFFS_Q_BYPASS4;
}

/*************************************************************************************************/
static void palBbDbbSetDdcTrans(void)
{
    MXC_DBB_RX->rx_phy_general_ddc_rst_timer = 0;

    MXC_DBB_RX->rx_phy_ddc_alpha_start[0] = RX_TRANS_DDC_ALPHA_START0;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[1] = RX_TRANS_DDC_ALPHA_START1;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[2] = RX_TRANS_DDC_ALPHA_START2;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[3] = RX_TRANS_DDC_ALPHA_START3;
    MXC_DBB_RX->rx_phy_ddc_alpha_start[4] = RX_TRANS_DDC_ALPHA_START4;

    MXC_DBB_RX->rx_phy_ddc_alpha_stop[0] = RX_TRANS_DDC_ALPHA_STOP0;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[1] = RX_TRANS_DDC_ALPHA_STOP1;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[2] = RX_TRANS_DDC_ALPHA_STOP2;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[3] = RX_TRANS_DDC_ALPHA_STOP3;
    MXC_DBB_RX->rx_phy_ddc_alpha_stop[4] = RX_TRANS_DDC_ALPHA_STOP4;

    MXC_DBB_RX->rx_phy_ddc_alpha_step[0] = RX_TRANS_DDC_ALPHA_STEP0;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[1] = RX_TRANS_DDC_ALPHA_STEP1;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[2] = RX_TRANS_DDC_ALPHA_STEP2;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[3] = RX_TRANS_DDC_ALPHA_STEP3;
    MXC_DBB_RX->rx_phy_ddc_alpha_step[4] = RX_TRANS_DDC_ALPHA_STEP4;

    MXC_DBB_RX->rx_phy_ddc_beta[0] = RX_TRANS_DDC_BETA0;
    MXC_DBB_RX->rx_phy_ddc_beta[1] = RX_TRANS_DDC_BETA1;
    MXC_DBB_RX->rx_phy_ddc_beta[2] = RX_TRANS_DDC_BETA2;
    MXC_DBB_RX->rx_phy_ddc_beta[3] = RX_TRANS_DDC_BETA3;
    MXC_DBB_RX->rx_phy_ddc_beta[4] = RX_TRANS_DDC_BETA4;

    MXC_DBB_RX->rx_phy_ddc_rst_tmr = RX_PHY_TRANS_DDC_RST_TMR;

    MXC_DBB_RX->rx_phy_ddc_offs_i[0] = RX_PHY_TRANS_DDC_OFFS_I0;
    MXC_DBB_RX->rx_phy_ddc_offs_i[1] = RX_PHY_TRANS_DDC_OFFS_I1;
    MXC_DBB_RX->rx_phy_ddc_offs_i[2] = RX_PHY_TRANS_DDC_OFFS_I2;
    MXC_DBB_RX->rx_phy_ddc_offs_i[3] = RX_PHY_TRANS_DDC_OFFS_I3;

    MXC_DBB_RX->rx_phy_ddc_offs_q[0] = RX_PHY_TRANS_DDC_OFFS_Q0;
    MXC_DBB_RX->rx_phy_ddc_offs_q[1] = RX_PHY_TRANS_DDC_OFFS_Q1;
    MXC_DBB_RX->rx_phy_ddc_offs_q[2] = RX_PHY_TRANS_DDC_OFFS_Q2;
    MXC_DBB_RX->rx_phy_ddc_offs_q[3] = RX_PHY_TRANS_DDC_OFFS_Q3;
    MXC_DBB_RX->rx_phy_ddc_offs_i[4] = RX_PHY_TRANS_DDC_OFFS_I4;
    MXC_DBB_RX->rx_phy_ddc_offs_q[4] = RX_PHY_TRANS_DDC_OFFS_Q4;
}

/*************************************************************************************************/
static void palBbRxInit(void)
{
    MXC_DBB_RX->rx_phy_signal_det2_ed_fast_fact = RX_PHY_CFO_HPF_ED_FAST_FACT;
    MXC_DBB_RX->rx_phy_signal_det2_ed_slow_fact = RX_PHY_CFO_HPF_ED_SLOW_FACT;

    MXC_DBB_RX->rx_phy_signal_det2_ed_threshold_fact = RX_PHY_CFO_HPF_ED_THRESHOLD_FACT;
    MXC_DBB_RX->rx_phy_signal_det2_stop_high_filt_corner = RX_PHY_CFO_HPF_STOP_HIGH_FILT_CORNER;

    MXC_DBB_RX->rx_phy_signal_det2_ed_timeout_tmr_periode = RX_PHY_CFO_HPF_ED_TIMEOUT;

    MXC_DBB_RX->rx_phy_signal_det2_beta_mux_en_stage = RX_PHY_CFO_HPF_BETA_MUX_EN_STAGE;
    MXC_DBB_RX->rx_phy_signal_det2_beta_mux_threshold = RX_PHY_CFO_HPF_BETA_MUX_THRESHOLD;

    MXC_DBB_RX->rx_phy_signal_det2_gamma_mux_en_stage = RX_PHY_CFO_HPF_GAMMA_MUX_EN_STAGE;
    MXC_DBB_RX->rx_phy_signal_det2_gamma_mux_threshold = RX_PHY_CFO_HPF_GAMMA_MUX_THRESHOLD;

    MXC_DBB_RX->rx_phy_signal_det2_alpha[0] = RX_PHY_CFO_HPF_ALPHA_0;
    MXC_DBB_RX->rx_phy_signal_det2_alpha[1] = RX_PHY_CFO_HPF_ALPHA_1;
    MXC_DBB_RX->rx_phy_signal_det2_alpha[2] = RX_PHY_CFO_HPF_ALPHA_2;
    MXC_DBB_RX->rx_phy_signal_det2_alpha[3] = RX_PHY_CFO_HPF_ALPHA_3;

    MXC_DBB_RX->rx_phy_signal_det2_beta[0] = RX_PHY_CFO_HPF_BETA_0;
    MXC_DBB_RX->rx_phy_signal_det2_beta[1] = RX_PHY_CFO_HPF_BETA_1;
    MXC_DBB_RX->rx_phy_signal_det2_beta[2] = RX_PHY_CFO_HPF_BETA_2;
    MXC_DBB_RX->rx_phy_signal_det2_beta[3] = RX_PHY_CFO_HPF_BETA_3;

    MXC_DBB_RX->rx_phy_signal_det2_gamma[0] = RX_PHY_CFO_HPF_GAMMA_0;
    MXC_DBB_RX->rx_phy_signal_det2_gamma[1] = RX_PHY_CFO_HPF_GAMMA_1;
    MXC_DBB_RX->rx_phy_signal_det2_gamma[2] = RX_PHY_CFO_HPF_GAMMA_2;
    MXC_DBB_RX->rx_phy_signal_det2_gamma[3] = RX_PHY_CFO_HPF_GAMMA_3;
}

/*************************************************************************************************/
static void palBbRxSetup(void)
{
    uint16_t reg;

    /* Set standard to BTLE */
    MXC_DBB_RX->rx_general_standard = RX_GENERAL_STANDARD;

    reg = (MXC_F_DBB_RX_RX_GENERAL_CONTROL_AGC_ENABLE |
           /* MXC_F_DBB_RX_RX_GENERAL_CONTROL_RESERVED | */
           MXC_F_DBB_RX_RX_GENERAL_CONTROL_PHY_ENABLE | MXC_F_DBB_RX_RX_GENERAL_CONTROL_DL_ENABLE |
           MXC_F_DBB_RX_RX_GENERAL_CONTROL_PH_MG_EST_ENABLE |
           MXC_F_DBB_RX_RX_GENERAL_CONTROL_CFO_HPF_ENABLE |
           /* MXC_F_DBB_RX_RX_GENERAL_CONTROL_BTLE_LNGRNG_EN | */
           /* MXC_F_DBB_RX_RX_GENERAL_CONTROL_DISABLE_TIMING_SYNC | */
           /* MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE | */
           0);
    MXC_DBB_RX->rx_general_control = reg;

    reg = ((RX_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT
            << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT_POS) |
           (RX_GENERAL_CONTROL_START_CCA << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_START_CCA_POS) |
           (RX_GENERAL_CONTROL_CONT_SYNC << MXC_F_DBB_RX_RX_PHY_GENERAL_CONTROL_CONT_SYNC_POS) | 0);
    MXC_DBB_RX->rx_phy_general_control = reg;

    MXC_DBB_RX->rx_phy_general_agc_freeze_timer = RX_GENERAL_AGC_FREEZE_TIMER;
    MXC_DBB_RX->rx_phy_general_sig_detect_init_timer = RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_1MBIT;
    MXC_DBB_RX->rx_phy_general_sig_detect_rst_timer = RX_PHY_GENERAL_SIG_DETECT_RST_TIME;

    palBbDbbSetDdcActive();

    MXC_DBB_RX->rx_phy_agc5_thresh0upper = RX_PHY_AGC5_THRESH0UPPER;
    MXC_DBB_RX->rx_phy_agc5_thresh0lower = RX_PHY_AGC5_THRESH0LOWER;
    MXC_DBB_RX->rx_phy_agc5_thresh1upper = RX_PHY_AGC5_THRESH1UPPER;
    MXC_DBB_RX->rx_phy_agc5_thresh1lower = RX_PHY_AGC5_THRESH1LOWER;
    MXC_DBB_RX->rx_phy_agc5_thresh2upper = RX_PHY_AGC5_THRESH2UPPER;
    MXC_DBB_RX->rx_phy_agc5_thresh2lower = RX_PHY_AGC5_THRESH2LOWER;
    MXC_DBB_RX->rx_phy_agc5_thresh3upper = RX_PHY_AGC5_THRESH3UPPER;
    MXC_DBB_RX->rx_phy_agc5_thresh3lower = RX_PHY_AGC5_THRESH3LOWER;
    MXC_DBB_RX->rx_phy_agc5_thresh4upper = RX_PHY_AGC5_THRESH4UPPER;
    MXC_DBB_RX->rx_phy_agc5_thresh4lower = RX_PHY_AGC5_THRESH4LOWER;

    reg = MXC_DBB_RX->rx_phy_agc5_gain;

    PAL_BB_SETFIELD(reg, MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INIT,
                    (RX_PHY_AGC5_GAIN_INIT << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_INIT_POS));

    PAL_BB_SETFIELD(reg, MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MIN,
                    (RX_PHY_AGC5_GAIN_MIN << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MIN_POS));

    PAL_BB_SETFIELD(reg, MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MAX,
                    (RX_PHY_AGC5_GAIN_MAX << MXC_F_DBB_RX_RX_PHY_AGC5_GAIN_MAX_POS));

    MXC_DBB_RX->rx_phy_agc5_gain = reg;

    reg = ((RX_PHY_AGC5_CNTR_STABLE << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE_POS) |
           (RX_PHY_AGC5_CNTR_VALID << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS) | 0);
    MXC_DBB_RX->rx_phy_agc5_cntr = reg;

    reg = ((PAN2G_RX_PHY_AGC5_ROW_UPPER_LE << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_LE_POS) |
           (PAN2G_RX_PHY_AGC5_ROW_UPPER_INCR << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_INCR_POS) |
           (PAN2G_RX_PHY_AGC5_ROW_UPPER_BLK << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_UPPER_BLK_POS) | 0);
    MXC_DBB_RX->rx_phy_agc5_row_upper = reg;

    reg = ((PAN2G_RX_PHY_AGC5_ROW_LOWER_LE << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_LE_POS) |
           (PAN2G_RX_PHY_AGC5_ROW_LOWER_INCR << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_INCR_POS) |
           (PAN2G_RX_PHY_AGC5_ROW_LOWER_BLK << MXC_F_DBB_RX_RX_PHY_AGC5_ROW_LOWER_BLK_POS) | 0);
    MXC_DBB_RX->rx_phy_agc5_row_lower = reg;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    MXC_DBB_RX->rx_phy_phsmagest_scaling_factor = 6;
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

    reg = ((RX_PHY_FILTERS_SKIP_MUX_SEL << MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SKIP_MUX_SEL_POS) |
           (RX_PHY_FILTERS_SCALING_FACTOR
            << MXC_F_DBB_RX_RX_PHY_FILTERS_SETTINGS_SCALING_FACTOR_POS) |
           0);
    MXC_DBB_RX->rx_phy_filters_settings = reg;

    /* coded frame sync settings */
    MXC_DBB_RX->rx_phy_frame_sync_lr_corr_length = RX_PHY_FRAME_SYNC_LR_CORR_LENGTH;
    MXC_DBB_RX->rx_phy_frame_sync_lr_thresh_factor = RX_PHY_FRAME_SYNC_LR_THRESH_FACTOR_CODED_S8;

    MXC_DBB_RX->rx_phy_rssi_settings = RX_PHY_RSSI_SETTINGS;
    MXC_DBB_RX->rx_phy_rssi_ed_threshold = RX_PHY_RSSI_ED_THRESHOLD;

    MXC_DBB_RX->rx_phy_signal_det_select_v2 = SIGNALDETECT_VERSION;

    reg = ((RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR
            << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR_POS) |
           (RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES
            << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES_POS) |
           0);
    MXC_DBB_RX->rx_phy_signal_det_settings = reg;

    reg = ((RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP
            << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP_POS) |
           (RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR
            << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR_POS) |
           0);

    MXC_DBB_RX->rx_phy_signal_det_thresholds = reg;

    MXC_DBB_RX->rx_phy_signal_det_settings_v2_div_factor = SETTINGS_V2_DIV_FACTOR;

    reg = ((GEAR_V2_DELTA << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_DELTA_POS) |
           (GEAR_V2_START << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_START_POS) |
           (GEAR_V2_MAX << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_MAX_POS) |
           (GEAR_V2_STEP << MXC_F_DBB_RX_RX_PHY_SIGNAL_DET_GEAR_V2_STEP_POS) | 0);
    MXC_DBB_RX->rx_phy_signal_det_gear_v2 = reg;

    MXC_DBB_RX->rx_phy_signal_det_thresholds_v2_noise_factor = THRESHOLDS_V2_NOISE_FACTOR;

    MXC_DBB_RX->rx_phy_signal_det2_timeout_tmr_periode = RX_PHY_SIGNAL_DET2_TIMEOUT;

    MXC_DBB_RX->rx_phy_frame_sync_pd_comp = RX_PHY_FRAME_SYNC_PD;

    MXC_DBB_RX->rx_phy_frame_sync_sfd[0] = RX_PHY_FRAME_SYNC_SFD_0;
    MXC_DBB_RX->rx_phy_frame_sync_sfd[1] = RX_PHY_FRAME_SYNC_SFD_1;
    MXC_DBB_RX->rx_phy_frame_sync_sfd[2] = RX_PHY_FRAME_SYNC_SFD_2;
    MXC_DBB_RX->rx_phy_frame_sync_sfd[3] = RX_PHY_FRAME_SYNC_SFD_3;

    reg = ((RX_PHY_FRAME_SYNC_CORR_LENGTH
            << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_CORR_LENGTH_POS) |
           (RX_PHY_FRAME_SYNC_THRESH_FACTOR
            << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_THRESH_FACTOR_POS) |
           (RX_PHY_FRAME_SYNC_HARD_DETECT
            << MXC_F_DBB_RX_RX_PHY_FRAME_SYNC_SETTINGS_HARD_DETECT_POS) |
           0);
    MXC_DBB_RX->rx_phy_frame_sync_settings = reg;

    MXC_DBB_RX->rx_phy_frame_sync_settings_min_corr_val = RX_PHY_FRAME_SYNC_SETTINGS_MIN_CORR;

    reg = MXC_DBB_RX->rx_phy_timing_sync_cfg;
    PAL_BB_SETFIELD(reg, MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT,
                    (RX_PHY_TIMING_SYNC_CFG_LIMIT
                     << MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_LIMIT_POS));

    PAL_BB_SETFIELD(reg, MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_BITSHIFT,
                    (RX_PHY_TIMING_SYNC_CFG_BITSHIFT
                     << MXC_F_DBB_RX_RX_PHY_TIMING_SYNC_CFG_BITSHIFT_POS));
    MXC_DBB_RX->rx_phy_timing_sync_cfg = reg;

    MXC_DBB_RX->rx_phy_tim_sync_old_cfg = 1;

    MXC_DBB_RX->rx_phy_tim_sync_old_te_thresh = RX_PHY_TIM_SYNC_TE_THRESH;

    MXC_DBB_RX->rx_phy_tim_sync_old_te_thresh_scaling = RX_PHY_TIM_SYNC_TE_THRESH_SCALING;

    reg = ((RX_PHY_TIM_SYNC_INIT_SEL_CNT_MAX
            << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_SEL_CNT_MAX_POS) |
           (RX_PHY_TIM_SYNC_INIT_MIN_NUM_CHIP
            << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_INIT_MIN_NUM_CHIP_POS) |
           0);
    MXC_DBB_RX->rx_phy_tim_sync_old_init = reg;

    reg = ((RX_PHY_TIM_SYNC_TE_CNT_THRESH_VALUE
            << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_VALUE_POS) |
           (RX_PHY_TIM_SYNC_TE_CNT_THRESH_INCR_VALUE
            << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_THRESH_INCR_VALUE_POS) |
           0);
    MXC_DBB_RX->rx_phy_tim_sync_old_te_cnt_thresh = reg;

    reg = ((RX_PHY_TIM_SYNC_TE_CNT_INCR_VALUE
            << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_INCR_VALUE_POS) |
           (RX_PHY_TIM_SYNC_TE_CNT_DECR_VALUE
            << MXC_F_DBB_RX_RX_PHY_TIM_SYNC_OLD_TE_CNT_DECR_VALUE_POS) |
           0);
    MXC_DBB_RX->rx_phy_tim_sync_old_te_cnt = reg;

    MXC_DBB_RX->rx_phy_tim_sync_old_te_fifo_max_ptr = RX_PHY_TIM_SYNC_TE_FIFO_MAX_PTR;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[0] = RX_PHY_CFO_FILTER_COEFFICIENTS_0;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[1] = RX_PHY_CFO_FILTER_COEFFICIENTS_1;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[2] = RX_PHY_CFO_FILTER_COEFFICIENTS_2;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[3] = RX_PHY_CFO_FILTER_COEFFICIENTS_3;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[4] = RX_PHY_CFO_FILTER_COEFFICIENTS_4;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[5] = RX_PHY_CFO_FILTER_COEFFICIENTS_5;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[6] = RX_PHY_CFO_FILTER_COEFFICIENTS_6;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[7] = RX_PHY_CFO_FILTER_COEFFICIENTS_7;
    MXC_DBB_RX->rx_phy_cfo_filter_coefficients[8] = RX_PHY_CFO_FILTER_COEFFICIENTS_8;
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    MXC_DBB_RX->rx_phy_cfo_filter_scaling = RX_PHY_CFO_FILTER_SCALING;
    MXC_DBB_RX->rx_phy_cfo_pa_precnt_num = RX_PHY_CFO_PA_PRECNT_NUM;
    MXC_DBB_RX->rx_phy_cfo_ph_accum_part_num = RX_PHY_CFO_PH_ACCUM_PART_NUM;
    MXC_DBB_RX->rx_phy_cfo_ed_thresh_factor = RX_PHY_CFO_ED_THRESH_FACTOR;
    MXC_DBB_RX->rx_phy_cfo_ed_slow_factor = RX_PHY_CFO_ED_SLOW_FACTOR;
    MXC_DBB_RX->rx_phy_cfo_ed_fast_factor = RX_PHY_CFO_ED_FAST_FACTOR;
    MXC_DBB_RX->rx_phy_cfo_pa_max_limit_val = RX_PHY_CFO_PA_MAX_LIMIT;

    MXC_DBB_RX->rx_dl_bypass = DISABLED;

    reg = ((RX_DL_CRC_MODE_PHR << MXC_F_DBB_RX_RX_DL_CRC_MODE_PHR_POS) |
           (RX_DL_CRC_MODE_PLD << MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS) | 0);
    MXC_DBB_RX->rx_dl_crc_mode = reg;

    MXC_DBB_RX->rx_dl_crc_init_phr = RX_DL_CRC_INIT_PHR;
    MXC_DBB_RX->rx_dl_crc_init_pld = RX_DL_CRC_INIT_PLD;

    reg = ((RX_DL_BCH_PHR_ENABLE << MXC_F_DBB_RX_RX_DL_BCH_PHR_ENABLE_POS) |
           (RX_DL_BCH_PLD_ENABLE << MXC_F_DBB_RX_RX_DL_BCH_PLD_ENABLE_POS) |
           (RX_DL_BCH_BIT_ERR_NR << MXC_F_DBB_RX_RX_DL_BCH_BIT_ERR_NR_POS) | 0);
    MXC_DBB_RX->rx_dl_bch = reg;

    reg = ((RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD_POS) |
           (RX_DL_BAN_SETTINGS_PHR_SPRD_FACT << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PHR_SPRD_FACT_POS) |
           (RX_DL_BAN_SETTINGS_MDR_LENGTH << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_MDR_LENGTH_POS) |
           (RX_DL_BAN_SETTINGS_PROP_PLD_MOD << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_MOD_POS) |
           (RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT
            << MXC_F_DBB_RX_RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT_POS) |
           0);
    MXC_DBB_RX->rx_dl_ban_settings = reg;

    reg = ((RX_DL_BTLE_SETTINGS_CHAN_NR << MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS) |
           (RX_DL_BTLE_SETTINGS_BYPASS << MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS) | 0);
    MXC_DBB_RX->rx_dl_btle_settings_whit = reg;

    MXC_DBB_RX->rx_dl_phr_modulation = RX_DL_PHR_MODULATION;
}

/*************************************************************************************************/
static void palBbTxInit(void)
{
    uint16_t reg;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    MXC_DBB_TX->tx_general_standard = 3;

    MXC_DBB_TX->tx_general_control = 3;

    MXC_DBB_TX->tx_phy_high_freq_coef = 0x3000;
    MXC_DBB_TX->tx_phy_low_freq_coef = 0x00;

    MXC_DBB_TX->tx_phy_freq_carrier = 0x80000;

    MXC_DBB_TX->tx_phy_amp_coef = 0x05;

    reg = ((0x8 << MXC_F_DBB_TX_TX_PHY_FILT_SKIP_MUX_SEL_POS) |
           (0 << MXC_F_DBB_TX_TX_PHY_FILT_SCALING_FACTOR_POS) | 0);
    MXC_DBB_TX->tx_phy_filt = reg;

    MXC_DBB_TX->tx_phy_filt_coefs[0] = 64;
    MXC_DBB_TX->tx_phy_filt_coefs[1] = 57;
    MXC_DBB_TX->tx_phy_filt_coefs[2] = 41;
    MXC_DBB_TX->tx_phy_filt_coefs[3] = 24;
    MXC_DBB_TX->tx_phy_filt_coefs[4] = 11;
    MXC_DBB_TX->tx_phy_filt_coefs[5] = 4;
    MXC_DBB_TX->tx_phy_filt_coefs[6] = 1;
    MXC_DBB_TX->tx_phy_filt_coefs[7] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[8] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[9] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[10] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[11] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[12] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[13] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[14] = 0;
    MXC_DBB_TX->tx_phy_filt_coefs[15] = 0;

    MXC_DBB_TX->tx_phy_freq_offs = 593;

    MXC_DBB_TX->tx_phy_ovrsamp_ratio = 0x08;

    reg = (MXC_S_DBB_TX_TX_DL_CRC_MODE_PHR_CRC24 | MXC_S_DBB_TX_TX_DL_CRC_MODE_PLD_CRC24 | 0);
    MXC_DBB_TX->tx_dl_crc_mode = reg;

    MXC_DBB_TX->tx_dl_crc_init_phr = 0x555555;
    MXC_DBB_TX->tx_dl_crc_init_pld = 0x555555;

    MXC_DBB_TX->tx_dl_bch = 0x00;

    MXC_DBB_TX->tx_dl_btle_settings_acc_addr = 0x8E89BED6;

    reg = ((PROT_DEFAULT_CHANNEL << MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS) |
           (0 << MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS) | 0);
    MXC_DBB_TX->tx_dl_btle_settings_whit = reg;

    MXC_DBB_TX->tx_dl_phr_modulation = 0x05;

    /* test if PA ramp is smooth */
    MXC_DBB_RFFE->general2_skip_fir_amp = 0x0; /*  enabled */
    MXC_DBB_RFFE->general2_fir_coefficients[0] = 109;
    MXC_DBB_RFFE->general2_fir_coefficients[1] = 75;
    MXC_DBB_RFFE->general2_fir_coefficients[2] = 15;
    MXC_DBB_RFFE->general2_fir_coefficients[3] = -16;
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*************************************************************************************************/
void PalBbDbbAgcSetDefaultGain(void)
{
    MXC_DBB_RFFE->agc_enc5_gain_table[0] = AGC_GAIN0;
    MXC_DBB_RFFE->agc_enc5_gain_table[1] = AGC_GAIN1;
    MXC_DBB_RFFE->agc_enc5_gain_table[2] = AGC_GAIN2;
    MXC_DBB_RFFE->agc_enc5_gain_table[3] = AGC_GAIN3;
    MXC_DBB_RFFE->agc_enc5_gain_table[4] = AGC_GAIN4;

    MXC_DBB_RFFE->agc_enc5_gain_table_db[0] = AGC_RSSI_GAINS0;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[1] = AGC_RSSI_GAINS1;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[2] = AGC_RSSI_GAINS2;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[3] = AGC_RSSI_GAINS3;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[4] = AGC_RSSI_GAINS4;
}

/*************************************************************************************************/
void PalBbDbbAgcSetHsGain(void)
{
    MXC_DBB_RFFE->agc_enc5_gain_table[0] = AGC_GAIN_HIGHSPEED_0;
    MXC_DBB_RFFE->agc_enc5_gain_table[1] = AGC_GAIN_HIGHSPEED_1;
    MXC_DBB_RFFE->agc_enc5_gain_table[2] = AGC_GAIN_HIGHSPEED_2;
    MXC_DBB_RFFE->agc_enc5_gain_table[3] = AGC_GAIN_HIGHSPEED_3;
    MXC_DBB_RFFE->agc_enc5_gain_table[4] = AGC_GAIN_HIGHSPEED_4;

    MXC_DBB_RFFE->agc_enc5_gain_table_db[0] = AGC_RSSI_GAINS_HIGHSPEED_0;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[1] = AGC_RSSI_GAINS_HIGHSPEED_1;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[2] = AGC_RSSI_GAINS_HIGHSPEED_2;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[3] = AGC_RSSI_GAINS_HIGHSPEED_3;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[4] = AGC_RSSI_GAINS_HIGHSPEED_4;
}

/*************************************************************************************************/
void palBbDbbAgcSetCodedGain(void)
{
    MXC_DBB_RFFE->agc_enc5_gain_table[0] = AGC_GAIN_CODED_0;
    MXC_DBB_RFFE->agc_enc5_gain_table[1] = AGC_GAIN_CODED_1;
    MXC_DBB_RFFE->agc_enc5_gain_table[2] = AGC_GAIN_CODED_2;

    MXC_DBB_RFFE->agc_enc5_gain_table_db[0] = AGC_RSSI_GAINS_CODED_0;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[1] = AGC_RSSI_GAINS_CODED_1;
    MXC_DBB_RFFE->agc_enc5_gain_table_db[2] = AGC_RSSI_GAINS_CODED_2;
}

/*************************************************************************************************/
void palBbDbbAgcSetup(uint8_t phy)
{
    MXC_DBB_RX->rx_general_control |= MXC_F_DBB_RX_RX_GENERAL_CONTROL_AGC_ENABLE;

    static const uint16_t RSSI_SAMPLES = 0x6;

    MXC_DBB_RX->rx_phy_rssi_settings =
        ((0x1 << MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_NUM_CALC_POS) |
         (RSSI_SAMPLES << MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_POW_TWO_SAMPLES_POS) |
         (AGC_RSSI_CORRECTION_FACTOR << MXC_F_DBB_RX_RX_PHY_RSSI_SETTINGS_COMP_FACT_POS));

    /* new agc5 settings */
    MXC_DBB_RFFE->agc_enc5_gain_addr = DIBS_AGC_ENC5_GAIN_ADDR;

    if (phy == BB_PHY_BLE_2M) {
        PalBbDbbAgcSetHsGain();
    } else {
        PalBbDbbAgcSetDefaultGain();
    }
    MXC_DBB_RFFE->agc_enc5_ready_tmr = AGC_READY_TMR;

    /* Define which AFE register holds the I & Q offsets */
    MXC_DBB_RFFE->agc_enc5_offs_i_addr = AFE_Q_OFFSET_ADDR;
    MXC_DBB_RFFE->agc_enc5_offs_q_addr = AFE_I_OFFSET_ADDR;

    MXC_DBB_RX->rx_general_control |= MXC_F_DBB_RX_RX_GENERAL_CONTROL_AGC_ENABLE;

    MXC_DBB_CTRL->rx_ctrl_dbb_en_dly = RX_ENABLE_DBB_DELAY;
    MXC_DBB_CTRL->rx_ctrl_rffe_en_dly[0] =
        RX_ENABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->rx_ctrl_rffe_en_dly[1] =
        RX_ENABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->rx_ctrl_dbb_dis_dly = RX_DISABLE_DBB_DELAY;
    MXC_DBB_CTRL->rx_ctrl_rffe_dis_dly[0] =
        RX_DISABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->rx_ctrl_rffe_dis_dly[1] =
        RX_DISABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_RX->rx_phy_frame_sync_timeout_tmr_periode = 0;
    MXC_DBB_CTRL->tx_ctrl_dbb_en_dly = 0;

    MXC_DBB_RFFE->agc_enc5_dc_offs &= ~(MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_BYPASS_EN);
}

/*************************************************************************************************/
void palBbDbbAgcInit(uint8_t phy)
{
    palBbDbbAgcSetup(phy);

    palBbDbbSetDdcTrans(); /* disable DDC */

    MXC_DBB_RFFE->agc_enc5_dc_offs |= MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_ENABLE;

    MXC_DBB_RX->rx_phy_agc5_offs_init |= MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_START;

    MXC_DBB_CTRL->rx_ctrl_cmd = 0x1;

    while (!(MXC_DBB_RX->rx_phy_agc5_offs_init & MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_DONE)) {}

    MXC_DBB_RX->rx_phy_agc5_offs_init &= ~(MXC_F_DBB_RX_RX_PHY_AGC5_OFFS_INIT_START);

    MXC_DBB_CTRL->rx_ctrl_cmd = 0;
    MXC_DBB_CTRL->dbb_ctrl_rst |= 1;

    /* Save the offsets */
    if (MXC_DBB_RX->rx_phy_agc5_cntr ==
        ((OFFSET_CAL_RX_PHY_AGC5_CNTR_STABLE << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE_POS) |
         (OFFSET_CAL_RX_PHY_AGC5_CNTR_VALID << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS))) {
        // NOLINTBEGIN(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
        if (phy == BB_PHY_BLE_2M) {
            memcpy(palBbDbbCb.dc_offs_i_2M, (void *)MXC_DBB_RFFE->agc_enc5_curr_dc_offs_i,
                   (int)(GAIN_STEPS * 2));
            memcpy(palBbDbbCb.dc_offs_q_2M, (void *)MXC_DBB_RFFE->agc_enc5_curr_dc_offs_q,
                   (int)(GAIN_STEPS * 2));
        } else {
            memcpy(palBbDbbCb.dc_offs_i_1M, (void *)MXC_DBB_RFFE->agc_enc5_curr_dc_offs_i,
                   (int)(GAIN_STEPS * 2));
            memcpy(palBbDbbCb.dc_offs_q_1M, (void *)MXC_DBB_RFFE->agc_enc5_curr_dc_offs_q,
                   (int)(GAIN_STEPS * 2));
        }
        // NOLINTEND(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    }

    palBbDbbSetDdcActive(); /* enable DDC */
}

/*************************************************************************************************/
void PalBbDbbAgcRestoreDcOffs(uint8_t phy)
{
    MXC_DBB_RFFE->agc_enc5_dc_offs |= MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_ENABLE;
    MXC_DBB_RFFE->agc_enc5_dc_offs |= MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_BYPASS_EN;

    // NOLINTBEGIN(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    if (phy == BB_PHY_BLE_2M) {
        memcpy((void *)MXC_DBB_RFFE->agc_enc5_offs_i_bypass, (void *)palBbDbbCb.dc_offs_i_2M,
               (int)(GAIN_STEPS * 2));
        memcpy((void *)MXC_DBB_RFFE->agc_enc5_offs_q_bypass, (void *)palBbDbbCb.dc_offs_q_2M,
               (int)(GAIN_STEPS * 2));
    } else {
        memcpy((void *)MXC_DBB_RFFE->agc_enc5_offs_i_bypass, (void *)palBbDbbCb.dc_offs_i_1M,
               (int)(GAIN_STEPS * 2));
        memcpy((void *)MXC_DBB_RFFE->agc_enc5_offs_q_bypass, (void *)palBbDbbCb.dc_offs_q_1M,
               (int)(GAIN_STEPS * 2));
    }
    // NOLINTEND(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
}

/*************************************************************************************************/
void palBbDbbAgcRestore(void)
{
    palBbDbbAgcSetup(BB_PHY_BLE_1M);

    MXC_DBB_RFFE->agc_enc5_dc_offs |= MXC_F_DBB_RFFE_AGC_ENC5_DC_OFFS_ENABLE;
}

/*************************************************************************************************/
void PalBbDbbCmu1M(void)
{
    /* configure clocks for DL */
    MXC_DBB_CTRL->cmu_dl_mult_p = DL_CLK_MULT;
    MXC_DBB_CTRL->cmu_dl_div_q = DL_CLK_DIV;

    /* configure clocks for PHY */
    MXC_DBB_CTRL->cmu_phy_mult_p = DL_CLK_MULT;
    MXC_DBB_CTRL->cmu_phy_div_q = DL_CLK_DIV;
}

/*************************************************************************************************/
void PalBbDbbCmu2M(void)
{
    /* configure clocks for DL */
    MXC_DBB_CTRL->cmu_dl_mult_p = DL_HS_CLK_MULT;
    MXC_DBB_CTRL->cmu_dl_div_q = DL_HS_CLK_DIV;

    /* configure clocks for PHY */
    MXC_DBB_CTRL->cmu_phy_mult_p = DL_HS_CLK_MULT;
    MXC_DBB_CTRL->cmu_phy_div_q = DL_HS_CLK_DIV;
}

/*************************************************************************************************/
void PalBbDbbCmuInit(void)
{
    MXC_DBB_CTRL->cmu_gating_on = 0x1;

    /* Configure the main divider/multiplier to run at the input crystal
     * frequency */
    MXC_DBB_CTRL->cmu_main_mult_p = 1;
    MXC_DBB_CTRL->cmu_main_div_q = 1;

    PalBbDbbCmu1M();
}

/*************************************************************************************************/
static void palBbDbbTimerSetup(void)
{
    /* Setup the TIFS timers, enable times are set when the packet is scheduled
     */

    /* Stop any pening operations to prevents TIFS events */
    MXC_DBB_CTRL->rx_ctrl_cmd |= MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP;
    MXC_DBB_CTRL->tx_ctrl_cmd |= MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP;

    MXC_DBB_CTRL->rx_ctrl_cmd |= MXC_F_DBB_CTRL_RX_CTRL_CMD_DISABLE;
    MXC_DBB_CTRL->tx_ctrl_cmd |= MXC_F_DBB_CTRL_TX_CTRL_CMD_DISABLE;

    /* TX ENABLE */
    MXC_DBB_CTRL->tx_ctrl_rffe_en_dly[0] =
        TX_ENABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->tx_ctrl_rffe_en_dly[1] =
        TX_ENABLE_RFFE_1_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);

    /* TX DISABLE */
    MXC_DBB_CTRL->tx_ctrl_dbb_dis_dly = TX_DISABLE_DBB_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->tx_ctrl_rffe_dis_dly[0] =
        (TX_DISABLE_RFFE_0_DELAY + TXEARLYIRQ_USEC) * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->tx_ctrl_rffe_dis_dly[1] =
        TX_DISABLE_RFFE_1_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);

    /* RX ENABLE */
    MXC_DBB_CTRL->rx_ctrl_rffe_en_dly[0] =
        RX_ENABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->rx_ctrl_rffe_en_dly[1] =
        RX_ENABLE_RFFE_1_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);

    /* RX DISABLE */
    MXC_DBB_CTRL->rx_ctrl_dbb_dis_dly = RX_DISABLE_DBB_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->rx_ctrl_rffe_dis_dly[0] =
        RX_ENABLE_RFFE_0_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    MXC_DBB_CTRL->rx_ctrl_rffe_dis_dly[1] =
        RX_ENABLE_RFFE_1_DELAY * (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ);

    /* Clear the RX/TX disable */
    MXC_DBB_CTRL->rx_ctrl_cmd &= ~(MXC_F_DBB_CTRL_RX_CTRL_CMD_DISABLE);
    MXC_DBB_CTRL->tx_ctrl_cmd &= ~(MXC_F_DBB_CTRL_TX_CTRL_CMD_DISABLE);
}

/*************************************************************************************************/
static bool_t palBbDbbSetup(bool_t init)
{
    /* Reset the DBB to obtain a known state */
    MXC_DBB_CTRL->dbb_ctrl_rst = 1;

    PalBbDbbCmuInit();
    static const uint16_t WAKE_DELAY = 0xA;
    /* Setup TX PMU */
    MXC_DBB_CTRL->tx_pmu_wake_up_dly = ((WAKE_DELAY << MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_STAB_POS) |
                                        (WAKE_DELAY << MXC_F_DBB_CTRL_TX_PMU_WAKE_UP_DLY_ISO_POS));
    MXC_DBB_CTRL->tx_pmu_ctrl |= MXC_F_DBB_CTRL_TX_PMU_CTRL_WAKEUP;

    /* Setup RX PMU */
    MXC_DBB_CTRL->rx_pmu_wake_up_dly = ((WAKE_DELAY << MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_STAB_POS) |
                                        (WAKE_DELAY << MXC_F_DBB_CTRL_RX_PMU_WAKE_UP_DLY_ISO_POS));
    MXC_DBB_CTRL->rx_pmu_ctrl |= MXC_F_DBB_CTRL_RX_PMU_CTRL_WAKEUP;

    /* Setup RFFE PMU */
    MXC_DBB_CTRL->rffe_pmu_wake_up_dly =
        ((WAKE_DELAY << MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_STAB_POS) |
         (WAKE_DELAY << MXC_F_DBB_CTRL_RFFE_PMU_WAKE_UP_DLY_ISO_POS));
    MXC_DBB_CTRL->rffe_pmu_ctrl |= MXC_F_DBB_CTRL_RFFE_PMU_CTRL_WAKEUP;

    /* Set event timing clock */
    MXC_DBB_CTRL->event_timing_cntr_clk_mult_p = BB_CLK_MULT;
    MXC_DBB_CTRL->event_timing_cntr_clk_div_q = BB_CLK_DIV;

    /* Start timer */
    MXC_DBB_CTRL->event_timing_ctrl = MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE;

    if (init) {
        static const uint32_t settlingTime = 350;
        /* Wait for counter to start and give some settlement time */
        PalBbDbbDelay(settlingTime);
    }

    /* Stop the control of the TX and RX to be sure both blocks are in the
     * correct state */
    MXC_DBB_CTRL->tx_ctrl_cmd |= MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP;
    MXC_DBB_CTRL->rx_ctrl_cmd |= MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP;

    palBbSpiInit();
    palBbRxInit();

    if (init) {
        PalBbPhyInit(BB_PHY_BLE_1M, BB_PHY_OPTIONS_DEFAULT);
        PalBbAfeSetup();
    }

    /* enable antenna select output */
    MXC_DBB_CTRL->dbb_ctrl_rst |= MXC_F_DBB_CTRL_DBB_CTRL_RST_ANT_SEL;

    palBbTxInit();
    palBbRxSetup();

    if (init) {
        /* Run DC Offset cal */
        MXC_DBB_RX->rx_phy_agc5_cntr =
            ((OFFSET_CAL_RX_PHY_AGC5_CNTR_STABLE << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE_POS) |
             (OFFSET_CAL_RX_PHY_AGC5_CNTR_VALID << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS));

        /* TODO: Not sure why this needs to be run twice */
        palBbDbbAgcInit(BB_PHY_BLE_2M);
        palBbDbbAgcInit(BB_PHY_BLE_1M);

        /* Calibrate on default channel */
        PalBbDbbSetChannel(AGC_OFFSET_CORRECTION_CHANNEL);

        palBbDbbAgcInit(BB_PHY_BLE_2M);
        palBbDbbAgcInit(BB_PHY_BLE_1M);

        MXC_DBB_RX->rx_phy_agc5_cntr =
            ((RX_PHY_AGC5_CNTR_STABLE << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_STABLE_POS) |
             (RX_PHY_AGC5_CNTR_VALID << MXC_F_DBB_RX_RX_PHY_AGC5_CNTR_VALID_POS));

    } else {
        PalBbDbbSetChannel(AGC_OFFSET_CORRECTION_CHANNEL);
        palBbDbbAgcRestore();
    }

    if (init) {
        PalBbAfeSaveCal();
    } else {
        PalBbAfeRestore();
        PalBbPhyInit(BB_PHY_BLE_1M, BB_PHY_OPTIONS_DEFAULT);
    }

    palBbDbbTimerSetup();

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbDbbInit(void)
{
    return palBbDbbSetup(TRUE);
}

/*************************************************************************************************/
bool_t PalBbDbbRestore(void)
{
    return palBbDbbSetup(FALSE);
}

/*************************************************************************************************/
void PalBbDbbRxCancel(void)
{
    /* Cancel current RX operation */
    MXC_DBB_CTRL->rx_ctrl_cmd |= MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP;

    /* Clear the RX enable timer */
    MXC_DBB_CTRL->rx_ctrl_dbb_en_dly = 0;

    /* Stop running RX timer */
    MXC_DBB_CTRL->event_timing_ctrl = (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE |
                                       MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_RX_ENABLE_EVENT);

    /* Stop GP event timer, used for debug */
    MXC_DBB_CTRL->event_timing_ctrl =
        (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE | MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_GP_EVENT);
}

/*************************************************************************************************/
void PalBbDbbTxCancel(void)
{
    /* Cancel current TX operation */
    MXC_DBB_CTRL->tx_ctrl_cmd |= MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP;

    /* Clear the TX enable timer */
    MXC_DBB_CTRL->tx_ctrl_dbb_en_dly = 0;

    /* Stop running TX timer */
    MXC_DBB_CTRL->event_timing_ctrl = (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE |
                                       MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_TX_ENABLE_EVENT);

    /* Stop GP event timer, used for debug */
    MXC_DBB_CTRL->event_timing_ctrl =
        (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE | MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_STOP_GP_EVENT);
}

/*************************************************************************************************/
void PalBbDbbSetAccAddr(uint32_t addr)
{
    /* Set the TX acc addr */
    MXC_DBB_TX->tx_dl_btle_settings_acc_addr = addr;
    MXC_DBB_TX->tx_dl_btle_settings_adv_acc_addr = addr;

    /* Set the RX acc addr with the frame sync pattern */
    MXC_DBB_RX->rx_phy_frame_sync_sfd[0] = 0x0000;

    if (addr & 0x01) {
        static const uint16_t frameSync55 = 0x5500;
        MXC_DBB_RX->rx_phy_frame_sync_sfd[1] = frameSync55;
    } else {
        static const uint16_t frameSyncAA = 0xAA00;
        MXC_DBB_RX->rx_phy_frame_sync_sfd[1] = frameSyncAA;
    }

    MXC_DBB_RX->rx_phy_frame_sync_sfd[2] = (addr & 0xFFFF); // NOLINT
    MXC_DBB_RX->rx_phy_frame_sync_sfd[3] = ((addr >> 16) & 0xFFFF); // NOLINT
}

/*************************************************************************************************/
void PalBbDbbSetCrcInit(uint32_t crcInit)
{
    MXC_DBB_RX->rx_dl_crc_init_phr = crcInit;
    MXC_DBB_RX->rx_dl_crc_init_pld = crcInit;
    MXC_DBB_TX->tx_dl_crc_init_phr = crcInit;
    MXC_DBB_TX->tx_dl_crc_init_pld = crcInit;
}

/*************************************************************************************************/
void PalBbDbbSetChannel(uint32_t channel)
{
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    if (channel <= 39) {
        /* Update channel number in the DataLink implementations */
        PAL_BB_SETFIELD(MXC_DBB_TX->tx_dl_btle_settings_whit,
                        MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR,
                        (channel << MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS));

        PAL_BB_SETFIELD(MXC_DBB_RX->rx_dl_btle_settings_whit,
                        MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR,
                        (channel << MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_CHAN_NR_POS));

        /* Convert channel to channel index */
        if (channel <= 10) {
            channel++;
        } else if (channel <= 36) {
            channel += 2;
        } else if (channel == 37) {
            channel = 0;
        } else if (channel == 38) {
            channel = 12;
        } else if (channel == 39) {
            channel = 39;
        }

        /* Update the channel in the sequencer information */
        PalBbAfeSetChannel(channel);
    }
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*************************************************************************************************/
void PalBbDbbSetRxTimeout(uint32_t timeout, uint8_t phy)
{
    /* TODO: Measure to confirm RX timeout */
    /* TODO: Calculate this based on the delays */
    static const uint32_t rxTurnOnMargin = 12;

    if (timeout == 0) {
        MXC_DBB_RX->rx_phy_frame_sync_timeout_tmr_periode = 0;
        return;
    }

    /* Adjust SFD timeout according to the PHY we're using. */
    if (phy == BB_PHY_BLE_CODED) {
        timeout += BB_BLE_PREAM_TO_ACCESS_CODED;
    } else if (phy == BB_PHY_BLE_2M) {
        timeout += BB_BLE_PREAM_TO_ACCESS_2M;
    } else {
        timeout += BB_BLE_PREAM_TO_ACCESS_1M;
    }

    timeout += rxTurnOnMargin;

    /* Convert the event timing ticks to data link ticks */
    if (phy == BB_PHY_BLE_2M) {
        timeout *= (DL_HS_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    } else {
        timeout *= (DL_CLK_RATE_HZ / BB_CLK_RATE_HZ);
    }

    MXC_DBB_RX->rx_phy_frame_sync_timeout_tmr_periode = timeout;
}

/*************************************************************************************************/
void PalBbDbbDisableTIFS(void)
{
    MXC_DBB_CTRL->rx_ctrl_dbb_en_dly = 0;
    MXC_DBB_CTRL->tx_ctrl_dbb_en_dly = 0;
}

/*************************************************************************************************/
void PalBbDbbEnableTIFS(void)
{
    if (MXC_DBB_CTRL->rx_ctrl_dbb_en_dly == 0) {
        MXC_DBB_CTRL->rx_ctrl_cmd |= MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP;
    }

    if (MXC_DBB_CTRL->tx_ctrl_dbb_en_dly == 0) {
        MXC_DBB_CTRL->tx_ctrl_cmd |= MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP;
    }

    PalBbPhyEnableTIFS();
}

/*************************************************************************************************/
static void palBbDbbSetEncryptionParams(uint64_t pktCnt, uint8_t pktLen, uint32_t aad, uint8_t dir)
{
    /* Copy in 39 bits from packetCounter */
    pktCnt &= 0x7FFFFFFFFFULL; // NOLINT
    memcpy((void *)&(MXC_DBB_CTRL->aes_ctr_blk[1]), (void *)&pktCnt, 5); // NOLINT

    /*
    Bit 6 – Bit 0: Octet4 (7 most significant bits of packetCounter , with Bit 6
    being the most significant bit) Bit7: directionBit
    */
    static const uint8_t dirBit = 0x80;
    static const uint8_t dirByte = 5;
    if (dir) {
        MXC_DBB_CTRL->aes_ctr_blk[dirByte] |= dirBit;
    }

    /* Write transmit length. */
    static const uint8_t lenByte0 = 14;
    static const uint8_t lenByte1 = 15;

    MXC_DBB_CTRL->aes_ctr_blk[lenByte0] = 0;
    MXC_DBB_CTRL->aes_ctr_blk[lenByte1] = pktLen;

    /* Write AAD value */
    MXC_DBB_CTRL->aes_aad = aad;
}

/*************************************************************************************************/
void PalBbDbbEnableEncryption(uint64_t pktCnt, uint8_t pktLen, uint32_t aad, uint8_t dir)
{
    palBbDbbSetEncryptionParams(pktCnt, pktLen, aad, dir);

    MXC_DBB_TX->tx_general_control |= MXC_F_DBB_TX_TX_GENERAL_CONTROL_ENCRYPTION_ENABLE;
}

/*************************************************************************************************/
void PalBbDbbDisableEncryption(void)
{
    MXC_DBB_TX->tx_general_control &= ~MXC_F_DBB_TX_TX_GENERAL_CONTROL_ENCRYPTION_ENABLE;
}

/*************************************************************************************************/
void PalBbDbbEnableDecryption(uint64_t pktCnt, uint8_t dir)
{
    palBbDbbSetEncryptionParams(pktCnt, 0, 0, dir);

    MXC_DBB_RX->rx_general_control |= MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE;
}

/*************************************************************************************************/
void PalBbDbbDisableDecryption(void)
{
    MXC_DBB_RX->rx_general_control &= ~MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE;
}

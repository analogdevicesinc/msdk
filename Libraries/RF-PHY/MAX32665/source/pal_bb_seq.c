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
 * @file    pal_bb_seq.c
 * @brief   Functions to use the AFE sequencers.
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include <string.h>
#include "mxc_device.h"
#include "gcr_regs.h"
#include "simo_regs.h"
#include "afe_regs.h"
#include "dbb_registers.h"
#include "pal_bb_afe.h"
#include "pal_bb_dbb.h"
#include "pal_bb_phy.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#define TX_RFPOWER_OFF 0x00
#define TX_RFPOWER_RAMPUP 0x01

/*! \brief      PalBbSeq control block. */
typedef struct {
    uint8_t txEnableRfPowerIndex;
    uint8_t txEnableChIndex;
    uint8_t rxEnableChIndex;
} palBbSeqCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

static palBbSeqCb_t palBbSeqCb;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
static unsigned int palBbSeqTxChannelSetup(palBbAfeCh_t *chCfg)
{
    unsigned int idx = palBbSeqCb.txEnableChIndex;
    uint16_t fdev_phy, div_frac, div_int;
    uint8_t reg;
    int8_t txPower = PalBbAfeGetTxPower();

    /* If we're using the 2M PHY, use the 2M fdev setting */
    if (PalBbPhyGetPhy() == BB_PHY_BLE_2M) {
        fdev_phy = chCfg->f_dev_2M; // NOLINT
    } else {
        fdev_phy = chCfg->f_dev;
    }

    /* process Register B3 */
    reg = ((chCfg->b_osc << MXC_F_AFE_REGB3_B_OSC_POS) | MXC_F_AFE_REGB3_TX_RXB);

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGB3 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A1 */
    reg = (MXC_F_AFE_REGA1_FRAC_EN | (chCfg->div_frac << MXC_F_AFE_REGA1_DIV_FRAC_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    // Determine which div_frac and div_int values we will use
    if (txPower > 2) {
        div_frac = chCfg->div_frac_4;
        div_int = chCfg->div_int_4;
    } else if (txPower > 0) {
        div_frac = chCfg->div_frac_2;
        div_int = chCfg->div_int_2;
    } else if (txPower > -2) {
        div_frac = chCfg->div_frac_0;
        div_int = chCfg->div_int_0;
    } else if (txPower > -10) { // NOLINT
        div_frac = chCfg->div_frac_n2;
        div_int = chCfg->div_int_n2;
    } else {
        div_frac = chCfg->div_frac;
        div_int = chCfg->div_int;
    }

    /* process Register A2 */
    /* Account for PA->PLL pulling frac bit 4 .. 11 */
    reg = div_frac >> 4; // NOLINT Account for PA->PLL pulling div_frac bit 4 .. 11

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A3 */
    // div_int and div_frac bit 12 .. 14
    static const unsigned A3_bit_shift = 12;
    reg = (((div_frac >> A3_bit_shift) << MXC_F_AFE_REGA3_DIV_FRAC_POS) |
           (div_int << MXC_F_AFE_REGA3_DIV_INT_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA3 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* Process Register A4 */
    /* Tom says we could possibly remove this, as f_dev_low has little impact */
    reg = (MXC_S_AFE_REGA4_SEL_HFP_FM_EN | ((fdev_phy & 1) << MXC_F_AFE_REGA4_F_DEV_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA4 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* Process Register A5 */
    reg = fdev_phy >> 1;
    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA5 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A12 */
    reg = (((chCfg->ib_vco_dac) << MXC_F_AFE_REGA12_IB_VDO_DAC_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA12 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    return idx;
}

/*************************************************************************************************/
static unsigned int palBbSeqRxChannelSetup(palBbAfeCh_t *chCfg)
{
    uint8_t reg;
    unsigned int idx = palBbSeqCb.rxEnableChIndex;

    /* process Register B3 */
    reg = ((chCfg->b_osc << MXC_F_AFE_REGB3_B_OSC_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGB3 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A1 */
    reg = (MXC_F_AFE_REGA1_FRAC_EN | (chCfg->div_frac << MXC_F_AFE_REGA1_DIV_FRAC_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A2 */
    reg = (chCfg->div_frac >> 4); /* bit 4 .. 11 */

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A3 */
    // div_int and div_frac bit 12 .. 14
    static const unsigned A3_bit_shift = 12;
    reg = (((chCfg->div_frac >> A3_bit_shift) << MXC_F_AFE_REGA3_DIV_FRAC_POS) |
           (chCfg->div_int << MXC_F_AFE_REGA3_DIV_INT_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA3 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* Process Register A4 */
    reg = (MXC_S_AFE_REGA4_SEL_HFP_FM_EN | ((chCfg->f_dev & 1) << MXC_F_AFE_REGA4_F_DEV_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA4 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* Process Register A5 */
    reg = chCfg->f_dev >> 1;

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA5 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A12 */
    reg = ((chCfg->ib_vco_dac << MXC_F_AFE_REGA12_IB_VDO_DAC_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA12 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A15 */
    reg = ((chCfg->ib_lobuf_dac << MXC_F_AFE_REGA15_IB_LOBUF_DAC_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA15 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    return idx;
}

/*************************************************************************************************/
static void palBbSeqTxEnableSetup(palBbAfeCh_t *chCfg)
{
    uint8_t reg;
    uint16_t fdev_phy;

    /* If we're using the 2M PHY, use the 2M fdev setting */
    if (PalBbPhyGetPhy() == BB_PHY_BLE_2M) {
        fdev_phy = MXC_S_AFE_REGA8_FDEV_D2_4K;
    } else {
        fdev_phy = MXC_S_AFE_REGA8_FDEV_D2_2K;
    }

    /* set index into sequencer table */
    unsigned int idx = 0;

    /* **********************************************************************************
     *  First Part
     * **********************************************************************************/

    /* process Register 9 */
    reg = (
        /* MXC_F_AFE_REG9_LNA_EN | */
        /* MXC_F_AFE_REG9_MX1_EN | */
        /* MXC_F_AFE_REG9_MX2_EN | */
        /* MXC_F_AFE_REG9_LPF_EN | */
        /* MXC_F_AFE_REG9_CLK_ADC_EN | */
        /* MXC_F_AFE_REG9_D2S_BUF_EN | */
        /* MXC_F_AFE_REG9_CON_SW_RW | */
        MXC_F_AFE_REG9_BIAS_EN);

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG9 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 2 is not directly frequency dependent but mode TX/RX
     * dependent */
    reg = (MXC_F_AFE_REG2_CLK_DAC_EN | MXC_F_AFE_REG2_CLK_PA_EN);

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A8 is not directly frequency dependent but mode TX/RX
     * dependent */
    reg = (
        /* MXC_F_AFE_REGA8_DATA_CLK_EN | */
        /* MXC_F_AFE_REGA8_SDMA_EN | */
        MXC_S_AFE_REGA8_SEL_AMP_INTERNAL | MXC_S_AFE_REGA8_AM_FIX_AMP_COEF |
        // (0x0 << MXC_F_AFE_REGA8_FDEV_D2_POS) |
        MXC_S_AFE_REGA8_BLE_24MHZ_32M | MXC_S_AFE_REGA8_INV_FM_POL_NON_INV |
        /* MXC_F_AFE_REGA8_DCOCBY_EN | */
        0);

    reg |= fdev_phy;

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA8 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* Setup channel specific settings */
    palBbSeqCb.txEnableChIndex = idx;
    idx = palBbSeqTxChannelSetup(chCfg);

    /* process Register A13 */
    static const uint8_t IB_CP_DAC = 0x9;
    reg = ((IB_CP_DAC << MXC_F_AFE_REGA13_IB_CP_DAC_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA13 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 1 */
    reg = (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_CP_EN | MXC_F_AFE_REG1_VCO_EN |
           /* MXC_F_AFE_REG1_LO_BUF_EN | */
           MXC_F_AFE_REG1_DIV_EN | MXC_F_AFE_REG1_PA_EN |
           /* MXC_F_AFE_REG1_TX_SW_EN | */
           MXC_F_AFE_REG1_PLL_BIAS_EN);

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* **********************************************************************************
     *  Set part length
     * **********************************************************************************/
    MXC_DBB_RFFE->tx_seq_ena_seq_lngth_part = idx;

    /* **********************************************************************************
     *  Second Part
     * **********************************************************************************/

    /* process Register 1 */
    reg = (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_CP_EN | MXC_F_AFE_REG1_VCO_EN |
           /* MXC_F_AFE_REG1_LO_BUF_EN | */
           MXC_F_AFE_REG1_DIV_EN | MXC_F_AFE_REG1_PA_EN | MXC_F_AFE_REG1_TX_SW_EN |
           MXC_F_AFE_REG1_PLL_BIAS_EN);

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    palBbSeqCb.txEnableRfPowerIndex = idx; /* !!this value must match with the
                                              sequencer index of this entry */

    /* process Register A7 */
    uint8_t ampCoef = PalBbAfeGetTxPowerPa(PalBbAfeGetTxPower());
    reg = ((ampCoef << MXC_F_AFE_REGA7_AMP_COEF_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA7 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    // process Register A2
    reg = (chCfg->div_frac >> 4); // bit 4..11

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    // process Register A3
    reg = (((chCfg->div_frac >> 12) << MXC_F_AFE_REGA3_DIV_FRAC_POS) | // NOLINT
           (chCfg->div_int << MXC_F_AFE_REGA3_DIV_INT_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA3 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* **********************************************************************************
     *  Set total length
     * **********************************************************************************/
    MXC_DBB_RFFE->tx_seq_ena_seq_lngth_total = idx;
}

/*************************************************************************************************/
static void palBbSeqRxEnableSetup(void)
{
    uint8_t reg;
    /* set index into sequencer table */
    unsigned int idx = 0;
    palBbAfeCh_t chCfg = { 0 };

    /* **********************************************************************************
     *  First Part
     * **********************************************************************************/

    /* process Register 1 */
    reg = (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_CP_EN | MXC_F_AFE_REG1_VCO_EN |
           MXC_F_AFE_REG1_LO_BUF_EN | MXC_F_AFE_REG1_DIV_EN |
           /* MXC_F_AFE_REG1_PA_EN | */
           /* MXC_F_AFE_REG1_TX_SW_EN | */
           MXC_F_AFE_REG1_PLL_BIAS_EN);

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 2 is not directly frequency dependent but mode TX/RX
     * dependent */
    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS));

    /* process Register A8 is not directly frequency dependent but mode TX/RX
     * dependent */
    reg = (
        /* MXC_F_AFE_REGA8_DATA_CLK_EN | */
        /* MXC_F_AFE_REGA8_SDMA_EN | */
        MXC_S_AFE_REGA8_SEL_AMP_INTERNAL | MXC_S_AFE_REGA8_AM_FIX_AMP_COEF |
        (0x0 << MXC_F_AFE_REGA8_FDEV_D2_POS) | MXC_S_AFE_REGA8_BLE_24MHZ_32M |
        MXC_S_AFE_REGA8_INV_FM_POL_NON_INV |
        /* MXC_F_AFE_REGA8_DCOCBY_EN | */
        0);

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA8 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* Setup channel specific settings */
    palBbSeqCb.rxEnableChIndex = idx;
    idx = palBbSeqRxChannelSetup(&chCfg);

    /* process Register A13 */
    static const uint8_t IB_CP_DAC = 0x17;
    reg = ((IB_CP_DAC << MXC_F_AFE_REGA13_IB_CP_DAC_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGA13 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 11 */
    PalBbDbbSpiRead(MXC_R_AFE_REG11, &reg);
    reg |= (MXC_F_AFE_REG11_DC_DAC_EN | MXC_F_AFE_REG11_CLK_DC_DAC);

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG11 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 9 */
    reg = (MXC_F_AFE_REG9_LNA_EN | MXC_F_AFE_REG9_MX1_EN | MXC_F_AFE_REG9_MX2_EN |
           MXC_F_AFE_REG9_LPF_EN | MXC_F_AFE_REG9_CLK_ADC_EN | MXC_F_AFE_REG9_D2S_BUF_EN |
           MXC_F_AFE_REG9_CON_SW_RW | MXC_F_AFE_REG9_BIAS_EN);

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REG9 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register B6 */
    static const uint8_t PA_CP = 0x4;
    reg = ((PA_CP << MXC_F_AFE_REGB6_PA_CP_POS) | (0x2 << MXC_F_AFE_REGB6_ADC_SF_POS));

    MXC_DBB_RFFE->rx_seq_ena_cmd[idx++] =
        (((MXC_R_AFE_REGB6 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* **********************************************************************************
     *  Second Part
     * **********************************************************************************/

    /* empty second part */

    /* **********************************************************************************
     *  Set total and part length
     * **********************************************************************************/
    MXC_DBB_RFFE->rx_seq_ena_seq_lngth_part = 0;
    MXC_DBB_RFFE->rx_seq_ena_seq_lngth_total = idx;
}

/*************************************************************************************************/
static void palBbSeqTxDisableSetup(void)
{
    uint8_t reg;

    /* set index into sequencer table */
    unsigned int idx = 0;

    /* **********************************************************************************
     *  First Part
     * **********************************************************************************/

    /* process Register 2 is not directly frequency dependent but mode TX/RX
     * dependent */
    reg = (MXC_F_AFE_REG2_CLK_DAC_EN | MXC_F_AFE_REG2_CLK_PA_EN);

    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A8 is not directly frequency dependent but mode TX/RX
     * dependent */
    reg = (
        /* MXC_F_AFE_REGA8_DATA_CLK_EN | */
        /* MXC_F_AFE_REGA8_SDMA_EN | */
        MXC_S_AFE_REGA8_SEL_AMP_INTERNAL | MXC_S_AFE_REGA8_AM_FIX_AMP_COEF |
        (0x0 << MXC_F_AFE_REGA8_FDEV_D2_POS) | MXC_S_AFE_REGA8_BLE_24MHZ_32M |
        MXC_S_AFE_REGA8_INV_FM_POL_NON_INV |
        /* MXC_F_AFE_REGA8_DCOCBY_EN | */
        0);

    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REGA8 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 1 */
    reg = (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_CP_EN | MXC_F_AFE_REG1_VCO_EN |
           MXC_F_AFE_REG1_LO_BUF_EN | MXC_F_AFE_REG1_DIV_EN |
           /* MXC_F_AFE_REG1_PA_EN | */
           /* MXC_F_AFE_REG1_TX_SW_EN | */
           MXC_F_AFE_REG1_PLL_BIAS_EN);

    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register B3 */
    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REGB3 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS));

    /* process Register 9 */
    reg = (
        /* MXC_F_AFE_REG9_LNA_EN | */
        /* MXC_F_AFE_REG9_MX1_EN | */
        MXC_F_AFE_REG9_MX2_EN | MXC_F_AFE_REG9_LPF_EN |
        /* MXC_F_AFE_REG9_CLK_ADC_EN | */
        /* MXC_F_AFE_REG9_D2S_BUF_EN | */
        /* MXC_F_AFE_REG9_CON_SW_RW | */
        MXC_F_AFE_REG9_BIAS_EN);

    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG9 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* **********************************************************************************
     *  Set part length
     * **********************************************************************************/
    MXC_DBB_RFFE->tx_seq_dis_seq_lngth_part = idx;

    /* **********************************************************************************
     *  Second Part
     * **********************************************************************************/

    /* process Register 9 */
    reg = (
        /* MXC_F_AFE_REG9_LNA_EN | */
        /* MXC_F_AFE_REG9_MX1_EN | */
        MXC_F_AFE_REG9_MX2_EN | MXC_F_AFE_REG9_LPF_EN |
        /* MXC_F_AFE_REG9_CLK_ADC_EN | */
        /* MXC_F_AFE_REG9_D2S_BUF_EN | */
        /* MXC_F_AFE_REG9_CON_SW_RW | */
        MXC_F_AFE_REG9_BIAS_EN);

    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG9 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register A7 */
    reg = ((TX_RFPOWER_OFF << MXC_F_AFE_REGA7_AMP_COEF_POS));

    MXC_DBB_RFFE->tx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REGA7 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* **********************************************************************************
     *  Set total length
     * **********************************************************************************/
    MXC_DBB_RFFE->tx_seq_dis_seq_lngth_total = idx;
}

/*************************************************************************************************/
static void palBbSeqRxDisableSetup(void)
{
    uint8_t reg;
    /* set index into sequencer table */
    unsigned int idx = 0;

    /* **********************************************************************************
     *  First Part
     * **********************************************************************************/

    /* process Register 9 */
    reg = (
        /* MXC_F_AFE_REG9_LNA_EN | */
        /* MXC_F_AFE_REG9_MX1_EN | */
        /* MXC_F_AFE_REG9_MX2_EN | */
        /* MXC_F_AFE_REG9_LPF_EN | */
        /* MXC_F_AFE_REG9_CLK_ADC_EN | */
        /* MXC_F_AFE_REG9_D2S_BUF_EN | */
        /* MXC_F_AFE_REG9_CON_SW_RW | */
        MXC_F_AFE_REG9_BIAS_EN);

    MXC_DBB_RFFE->rx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG9 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 1 */
    reg = (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_CP_EN | MXC_F_AFE_REG1_VCO_EN |
           /* MXC_F_AFE_REG1_LO_BUF_EN | */
           MXC_F_AFE_REG1_DIV_EN |
           /* MXC_F_AFE_REG1_PA_EN | */
           /* MXC_F_AFE_REG1_TX_SW_EN | */
           MXC_F_AFE_REG1_PLL_BIAS_EN);

    MXC_DBB_RFFE->rx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG1 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 11 */
    PalBbDbbSpiRead(MXC_R_AFE_REG11, &reg);
    reg |= (MXC_F_AFE_REG11_DC_DAC_EN | MXC_F_AFE_REG11_CLK_DC_DAC);

    MXC_DBB_RFFE->rx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG11 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* process Register 2 is not directly frequency dependent but mode TX/RX
     * dependent */
    MXC_DBB_RFFE->rx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REG2 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS));

    /* process Register A8 is not directly frequency dependent but mode TX/RX
     * dependent */
    reg = (
        /* MXC_F_AFE_REGA8_DATA_CLK_EN | */
        /* MXC_F_AFE_REGA8_SDMA_EN | */
        MXC_S_AFE_REGA8_SEL_AMP_INTERNAL | MXC_S_AFE_REGA8_AM_FIX_AMP_COEF |
        (0x0 << MXC_F_AFE_REGA8_FDEV_D2_POS) | MXC_S_AFE_REGA8_BLE_24MHZ_32M |
        MXC_S_AFE_REGA8_INV_FM_POL_NON_INV |
        /* MXC_F_AFE_REGA8_DCOCBY_EN | */
        0);

    MXC_DBB_RFFE->rx_seq_dis_cmd[idx++] =
        (((MXC_R_AFE_REGA8 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);

    /* **********************************************************************************
     *  Second Part
     * **********************************************************************************/

    /* empty second part */

    /* **********************************************************************************
     *  Set total and part length
     * **********************************************************************************/
    MXC_DBB_RFFE->rx_seq_dis_seq_lngth_part = 0;
    MXC_DBB_RFFE->rx_seq_dis_seq_lngth_total = idx;
}

/*************************************************************************************************/
void PalBbSeqUpdate(uint8_t addr, uint8_t mask, uint8_t value)
{
    int i;

    /* Set the write bit */
    addr |= PAL_BB_DBB_SPI_WRITE_BIT;

    /* Search through the sequencer values to update the register */
    for (i = 0; i < MXC_DBB_RFFE->tx_seq_ena_seq_lngth_total; i++) {
        if (((MXC_DBB_RFFE->tx_seq_ena_cmd[i] & PAL_BB_DBB_SPI_ADDR_MASK) >>
             PAL_BB_DBB_SPI_ADDR_POS) == addr) {
            MXC_DBB_RFFE->tx_seq_ena_cmd[i] = (MXC_DBB_RFFE->tx_seq_ena_cmd[i] & ~mask) | value;
        }
    }
    for (i = 0; i < MXC_DBB_RFFE->tx_seq_dis_seq_lngth_total; i++) {
        if (((MXC_DBB_RFFE->tx_seq_dis_cmd[i] & PAL_BB_DBB_SPI_ADDR_MASK) >>
             PAL_BB_DBB_SPI_ADDR_POS) == addr) {
            MXC_DBB_RFFE->tx_seq_dis_cmd[i] = (MXC_DBB_RFFE->tx_seq_dis_cmd[i] & ~mask) | value;
        }
    }
    for (i = 0; i < MXC_DBB_RFFE->rx_seq_ena_seq_lngth_total; i++) {
        if (((MXC_DBB_RFFE->rx_seq_ena_cmd[i] & PAL_BB_DBB_SPI_ADDR_MASK) >>
             PAL_BB_DBB_SPI_ADDR_POS) == addr) {
            MXC_DBB_RFFE->rx_seq_ena_cmd[i] = (MXC_DBB_RFFE->rx_seq_ena_cmd[i] & ~mask) | value;
        }
    }
    for (i = 0; i < MXC_DBB_RFFE->rx_seq_dis_seq_lngth_total; i++) {
        if (((MXC_DBB_RFFE->rx_seq_dis_cmd[i] & PAL_BB_DBB_SPI_ADDR_MASK) >>
             PAL_BB_DBB_SPI_ADDR_POS) == addr) {
            MXC_DBB_RFFE->rx_seq_dis_cmd[i] = (MXC_DBB_RFFE->rx_seq_dis_cmd[i] & ~mask) | value;
        }
    }
}

/*************************************************************************************************/
void PalBbSeqInit(void)
{
    memset(&palBbSeqCb, 0, sizeof(palBbSeqCb_t)); // NOLINT

    // Use an empty chCfg, will be reset once channel is programmed
    palBbAfeCh_t chCfg = { 0 };

    palBbSeqTxEnableSetup(&chCfg);
    palBbSeqRxEnableSetup();
    palBbSeqTxDisableSetup();
    palBbSeqRxDisableSetup();
}

/*************************************************************************************************/
bool_t PalBbSeqSetChannel(palBbAfeCh_t *rxChCfg, palBbAfeCh_t *txChCfg)
{
    palBbSeqTxEnableSetup(txChCfg);
    palBbSeqRxChannelSetup(rxChCfg);

    return TRUE;
}

/*************************************************************************************************/
void PalBbSeqSetPA(uint8_t reg)
{
    /* Set the PA settings in the TX enable sequencer */
    reg = (
        /* (0 << MXC_F_AFE_REGA7_DL_AMP_POS) | */
        (reg << MXC_F_AFE_REGA7_AMP_COEF_POS));

    MXC_DBB_RFFE->tx_seq_ena_cmd[palBbSeqCb.txEnableRfPowerIndex] =
        (((MXC_R_AFE_REGA7 | PAL_BB_DBB_SPI_WRITE_BIT) << PAL_BB_DBB_SPI_ADDR_POS) | reg);
}

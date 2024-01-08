/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 ******************************************************************************/

#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "audio.h"

/* **** Definitions **** */

/* **** Variable Declaration **** */

const mxc_gpio_cfg_t gpio_cfg_audio = { MXC_GPIO0,
                                        (MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 |
                                         MXC_GPIO_PIN_27),
                                        MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_PULL_UP,
                                        MXC_GPIO_VSSEL_VDDIO,
                                        MXC_GPIO_DRVSTR_0 };

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
static int MXC_AUDIO_SetMasterClock(mxc_audio_I2S_config_t *config)
{
    uint16_t mVal, nVal = 0;

    if (config->audio == NULL) {
        return E_NULL_PTR;
    }

    /* I2S Clock generation */
    // Clear GLOBAL_ENABLE to 0 to disable the audio system.
    config->audio->global_en = 0;
    // Set GCR_PCKDIV.audclksel for clock source, 96MHz high-frequency internal oscillato is selected

    switch (config->masterClockSource) {
    case MXC_AUDIO_CLK_SRC_HSCLK:
        MXC_GCR->pckdiv &= ~(0x03 << 16);
        if (config->clock == MXC_AUDIO_CLK_12_288MHz) {
            mVal = 125;
            nVal = 16;
        } else if (config->clock == MXC_AUDIO_CLK_11_2896MHz) {
            ///////////////////////
            mVal = 147;
            nVal = 17;
        } else {
            return E_NOT_SUPPORTED;
        }
        break;
    case MXC_AUDIO_CLK_SRC_32MHZ:
        MXC_GCR->pckdiv &= ~(0x01 << 17);
        MXC_GCR->pckdiv |= (0x01 << 16);

        if (config->clock == MXC_AUDIO_CLK_12_288MHz) {
            mVal = 125;
            nVal = 48;
        } else if (config->clock == MXC_AUDIO_CLK_11_2896MHz) {
            ///////////////////////
            mVal = 1250;
            nVal = 441;
        } else {
            return E_NOT_SUPPORTED;
        }

        break;
    case MXC_AUDIO_CLK_SRC_GPIO_0_23:
        return E_NOT_SUPPORTED;

    default:
        return E_NOT_SUPPORTED;
    }

    // Set GLOBAL_ENABLE.enable to 1 to enable the audio subsystem
    config->audio->global_en = 1;

    //Set M and N values for master clock(Set to 12288MHz)
    config->audio->m_val = mVal;
    config->audio->n_val = nVal;

    return E_NO_ERROR;
}

static int MXC_AUDIO_EnablePCMMode(mxc_audio_I2S_config_t *init)
{
    if (init->audio == NULL) {
        return E_NULL_PTR;
    }

    //Enable the PCM Mode
    init->audio->modulator_controls |= MXC_F_PDM_PCM_SELECT;

    return E_NO_ERROR;
}

static int MXC_AUDIO_PCM_Config(mxc_audio_I2S_config_t *init)
{
    if (init->audio == NULL) {
        return E_NULL_PTR;
    }

    //PCM Clock config, BCLK polarity high, 32 BCLK per LRCLK
    init->audio->pcm_clock_setup =
        (init->LRCLKDivider | (init->BCLKPolarity << MXC_F_PCM_BCLKEDGE_POS));

    //PCM Config, I2S mode
    init->audio->pcm_config = (MXC_F_PCM_FORMATX_I2S | (init->channelSize << MXC_F_PCM_CHANSZ_POS));

    //Set PCM Clock Dividers
    init->audio->pcm_clock_dividers_msb =
        ((init->BCLKSource << MXC_F_PCM_BCLK_MASTER_SEL_POS) |
         (init->BCLKSourceSelect << MXC_F_PCM_BCLK_SEL_POS) | ((init->BCLKDivisor >> 10) & 0x03));
    init->audio->pcm_clock_dividers_lsb = init->BCLKDivisor & 0x3FF;

    return E_NO_ERROR;
}

static int MXC_AUDIO_PCM_Tx_Enable(mxc_audio_I2S_config_t *init)
{
    if (init->audio == NULL) {
        return E_NULL_PTR;
    }

    init->audio->pcm_tx_sample_rates =
        (init->TxInterfaceSampleRates |
         (init->TxDataportSampleRates << MXC_F_PCM_TX_DATAPORT_SR_POS));

    init->audio->pcm_tx_enables_byte0 = (MXC_F_PCM_TX_CH0_EN | MXC_F_PCM_TX_CH1_EN);
    init->audio->dataport1_slot_mapping_ch0 |= MXC_F_PCM_DPORT1_CH0_EN;
    init->audio->dataport1_slot_mapping_ch1 |= MXC_F_PCM_DPORT1_CH1_EN;

    return E_NO_ERROR;
}

static int MXC_AUDIO_PCM_Rx_Enable(mxc_audio_I2S_config_t *init)
{
    if (init->audio == NULL) {
        return E_NULL_PTR;
    }

    init->audio->pcm_rx_sample_rates =
        (init->RxInterfaceSampleRates |
         (init->RxDataportSampleRates << MXC_F_PCM_RX_DATAPORT_SR_POS));

    init->audio->pcm_rx_enables_byte0 = (MXC_F_PCM_RX_CH0_EN | MXC_F_PCM_RX_CH1_EN);
    init->audio->dataport2_slot_mapping_ch0 |= MXC_F_PCM_DPORT2_CH0_EN;
    init->audio->dataport2_slot_mapping_ch1 |= MXC_F_PCM_DPORT2_CH1_EN;

    return E_NO_ERROR;
}

int MXC_AUDIO_EnableInterrupts(mxc_audio_regs_t *audio, uint32_t interrupts)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }
    audio->int_en |= interrupts;

    return E_NO_ERROR;
}

int MXC_AUDIO_Init(mxc_audio_regs_t *audio)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC96M_EN)) {
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC96M_EN;
        while (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC96M_RDY)) {}
    }

    //Enable audio subsytem peripheral clock
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_AUDIO);
    //Enable gpio clock and configure as alternate function
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_GPIO_Config(&gpio_cfg_audio);

    return E_NO_ERROR;
}

int MXC_AUDIO_I2S_Configure(mxc_audio_I2S_config_t *config)
{
    int err = E_NO_ERROR;

    if (config->audio == NULL) {
        return E_NULL_PTR;
    }

    if ((err = MXC_AUDIO_SetMasterClock(config)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_AUDIO_EnablePCMMode(config)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_PCM_Config(config)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_PCM_Tx_Enable(config)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_PCM_Rx_Enable(config)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_AUDIO_I2S_Receive(mxc_audio_regs_t *audio, uint32_t *leftData, uint32_t *rightData,
                          uint16_t len)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    if ((rightData == NULL) && (leftData == NULL)) {
        return E_NULL_PTR;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (leftData != NULL)
            leftData[i] = audio->rx_pcm_ch0_addr;
        if (rightData != NULL)
            rightData[i] = audio->rx_pcm_ch1_addr;
    }

    return E_NO_ERROR;
}
int MXC_AUDIO_I2S_Transmit(mxc_audio_regs_t *audio, uint32_t *leftData, uint32_t *rightData,
                           uint16_t len)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    if ((rightData == NULL) && (leftData == NULL)) {
        return E_NULL_PTR;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (leftData != NULL)
            audio->tx_pcm_ch0_addr = leftData[i];
        if (rightData != NULL)
            audio->tx_pcm_ch1_addr = rightData[i];
    }

    return E_NO_ERROR;
}

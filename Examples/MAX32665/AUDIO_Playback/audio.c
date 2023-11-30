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

#include "audio.h"
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_errors.h"

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

static int MXC_AUDIO_SetMasterClock(mxc_audio_regs_t *audio)
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

    /* I2S Clock generation */
    // Clear GLOBAL_ENABLE to 0 to disable the audio system.
    audio->global_en = 0;
    // Set GCR_PCKDIV.audclksel for clock source, 96MHz high-frequency internal oscillato is selected
    MXC_GCR->pckdiv &= ~(0x03 << 16);
    // Set GLOBAL_ENABLE.enable to 1 to enable the audio subsystem
    audio->global_en = 1;

    //Set M and N values for master clock(Set to 12288MHz)
    audio->m_val = 125;
    audio->n_val = 16;

    return E_NO_ERROR;
}

static int MXC_AUDIO_EnablePCMMode(mxc_audio_regs_t *audio)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    //Enable the PCM Mode
    audio->modulator_controls |= MXC_F_PDM_PCM_SELECT;

    return E_NO_ERROR;
}

static int MXC_AUDIO_PCM_Config(mxc_audio_regs_t *audio)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    //PCM Clock config, BCLK polarity high, 32 BCLK per LRCLK
    audio->pcm_clock_setup = MXC_F_PCM_BSEL_32;

    //PCM Confif, 16 bit I2S mode
    audio->pcm_config = (MXC_F_PCM_FORMATX_I2S | MXC_F_PCM_CHANSZ_16);

    //Set PCM Clock Dividers
    audio->pcm_clock_dividers_msb =
        (MXC_F_PCM_BCLK_SEL_BCLK_TOGGLE | MXC_F_PCM_BCLK_MASTER_SEL_MODULATOR_CLK);
    audio->pcm_clock_dividers_lsb = 0x06;

    return E_NO_ERROR;
}

static int MXC_AUDIO_PCM_Tx_Enable(mxc_audio_regs_t *audio)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    //Set PCM TX dataport and interface sample rates to 192kHz
    audio->pcm_tx_sample_rates = (MXC_F_PCM_TX_INTERFACE_SR_192 | MXC_F_PCM_TX_DATAPORT_SR_192);

    //Enable PCM Tx CH0 and CH1 Channels
    audio->pcm_tx_enables_byte0 = (MXC_F_PCM_TX_CH0_EN | MXC_F_PCM_TX_CH1_EN);

    //Enables the PCM TRANSMIT dataport channel 0, which is connected to APB Channel TX_PCM_CH 0
    audio->dataport1_slot_mapping_ch0 |= MXC_F_PCM_DPORT1_CH0_EN;

    //Enables the PCM TRANSMIT dataport channel 1, which is connected to APB Channel TX_PCM_CH 1
    audio->dataport1_slot_mapping_ch1 |= MXC_F_PCM_DPORT1_CH1_EN;

    return E_NO_ERROR;
}

static int MXC_AUDIO_PCM_Rx_Enable(mxc_audio_regs_t *audio)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }

    //Set PCM TX dataport and interface sample rates to 192kHz
    audio->pcm_rx_sample_rates = (MXC_F_PCM_RX_INTERFACE_SR_192 | MXC_F_PCM_RX_DATAPORT_SR_192);

    //Enable PCM Tx CH0 and CH1 Channels
    audio->pcm_rx_enables_byte0 = (MXC_F_PCM_RX_CH0_EN | MXC_F_PCM_RX_CH1_EN);

    //Enables the PCM TRANSMIT dataport channel 0, which is connected to APB Channel TX_PCM_CH 0
    audio->dataport2_slot_mapping_ch0 |= MXC_F_PCM_DPORT2_CH0_EN;

    //Enables the PCM TRANSMIT dataport channel 1, which is connected to APB Channel TX_PCM_CH 1
    audio->dataport2_slot_mapping_ch1 |= MXC_F_PCM_DPORT2_CH1_EN;

    return E_NO_ERROR;
}

static int MXC_AUDIO_EnableInterrupts(mxc_audio_regs_t *audio)
{
    if (audio == NULL) {
        return E_NULL_PTR;
    }
    audio->int_en |= MXC_F_EN_AE_PCM_RX;

    return E_NO_ERROR;
}

int MXC_AUDIO_Init(mxc_audio_regs_t *audio)
{
    int err = E_NO_ERROR;

    if (audio == NULL) {
        return E_NULL_PTR;
    }

    if ((err = MXC_AUDIO_SetMasterClock(audio)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_EnablePCMMode(audio)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_PCM_Config(audio)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_PCM_Tx_Enable(audio)) != E_NO_ERROR) {
        return err;
    }
    if ((err = MXC_AUDIO_PCM_Rx_Enable(audio)) != E_NO_ERROR) {
        return err;
    }

    return MXC_AUDIO_EnableInterrupts(audio);
}

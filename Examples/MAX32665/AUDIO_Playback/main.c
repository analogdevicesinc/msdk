/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

/**
 * @file        main.c
 * @brief       Audio Playback Example
 * @details     This example uses the I2C Master to initialize MAX9867. 
 *              For this example, if user wants to get audio data from Line-in
 *              they must connect microphone to line in jack. For output sound,
 *              user also must connect headphone to HD_PHONE jack. Refer UART
 *              messages for more information.
 * @note        Digital microphone input only works with MAX32666 EV KIT REV D.
 * 
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2c.h"
#include "board.h"
#include "audio.h"
#include "max9867.h"

/***** Definitions *****/

#define AUDIO MXC_AUDIO
#define I2C_MASTER MXC_I2C0_BUS0

#define AUDIO_IRQ AUDIO_IRQn

#define I2C_FREQ 100000

/***** Globals *****/
volatile bool lineIn = false;
uint32_t leftBuffer = 0;
uint32_t rightBuffer = 0;
uint16_t len = 0;

/***** Protottypes *****/
static int max9867ConfigureHeadphone(void);
static int max9867ConfigureDigitalMicrophone(void);
static int max9867ConfigureLineIn(void);
static int max9867Configure(void);

/***** Functions *****/

void AUDIO_IRQHandler(void)
{
    if (MXC_AUDIO_I2S_Receive(AUDIO, &leftBuffer, &rightBuffer, 1) == E_NO_ERROR) {
        MXC_AUDIO_I2S_Transmit(AUDIO, &leftBuffer, &rightBuffer, 1);
    }

    //Clear RX interrupts
    AUDIO->int_pcm_rx_clr |= 0xFFFFFFFF;
}

int main()
{
    int err;
    SystemCoreClockUpdate();

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);

    printf("\n************************ Audio Subsystem I2S Example ************************\n");

    printf(
        "\nIn this example, the device reads audio data simultaneously from the LINE_IN input or\n");
    printf("the Digital Microphone, then writes it to HD_PHONE using the MAX9867 Audio Codec.\n");
    printf(
        "\nThis example utilizes the audio subsystem peripheral to receive and transmit audio\n");
    printf("data via the I2S protocol. If you wish to use 'LINE_IN' as the input source, \n");
    printf("you must connect a microphone to the 'LINE_IN' port and listen to the sound using \n");
    printf("headphones connected to 'HD_PHONE'. \n");
    printf("\nAlternatively, you can switch the input source to the Digital Microphone by \n");
    printf("setting the 'lineIn' global variable to 'false'. \n");
    printf(
        "\n\nIt's important to note that the Digital microphone input source is only compatible \n");
    printf("with MAX32666 EV KIT REV D. \n");

    // Initialize I2C peripheral to initialize the MAX9867 Audio Codec
    err = MXC_I2C_Init(I2C_MASTER, TRUE, 0);
    if (err != E_NO_ERROR) {
        printf("\nError I2C init: %i", err);
        while (1) {}
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

    err = max9867Configure();
    if (err != E_NO_ERROR) {
        printf("\nError Configuring Audio Codec: %i", err);
        while (1) {}
    }

    if (lineIn) {
        err = max9867ConfigureLineIn();
        if (err != E_NO_ERROR) {
            printf("\nError Configuring Audio Codec Line In: %i", err);
            while (1) {}
        }
    } else {
        err = max9867ConfigureDigitalMicrophone();
        if (err != E_NO_ERROR) {
            printf("\nError Configuring Digital Microphone: %i", err);
            while (1) {}
        }
    }

    err = max9867ConfigureHeadphone();
    if (err != E_NO_ERROR) {
        printf("\nError Configuring Headphone: %i", err);
        while (1) {}
    }

    err = MXC_AUDIO_Init(AUDIO);
    if (err != E_NO_ERROR) {
        printf("\nError Configuring Adudio Subsystem peripheral: %i", err);
        while (1) {}
    }

    mxc_audio_I2S_config_t config = { 0 };

    config.audio = AUDIO;
    config.masterClockSource = MXC_AUDIO_CLK_SRC_HSCLK;
    config.clock = MXC_AUDIO_CLK_12_288MHz;
    config.BCLKSourceSelect = MXC_AUDIO_BCLK_GENERATOR_TOGGLE;
    config.BCLKSource = MXC_AUDIO_BCLK_SOURCE_F_AUDIO_MN;
    config.BCLKPolarity = MXC_AUDIO_CLK_POL_HIGH;
    config.BCLKDivisor = 0x06;
    config.LRCLKPolarity = MXC_AUDIO_CLK_POL_HIGH;
    config.LRCLKDivider = MXC_AUDIO_LRCLK_DIV_32;
    config.channelSize = MXC_AUDIO_PCM_CHANNEL_SIZE_16;
    config.TxInterfaceSampleRates = MXC_AUDIO_PCM_SAMPLE_RATE_192kHz;
    config.RxInterfaceSampleRates = MXC_AUDIO_PCM_SAMPLE_RATE_192kHz;
    config.TxDataportSampleRates = MXC_AUDIO_PCM_SAMPLE_RATE_192kHz;
    config.RxDataportSampleRates = MXC_AUDIO_PCM_SAMPLE_RATE_192kHz;
    config.TxExtraBitsFormat = MXC_AUDIO_TX_EXTRA_BITS_1;

    MXC_AUDIO_I2S_Configure(&config);
    uint32_t interrupts = MXC_F_EN_AE_PCM_RX;
    MXC_AUDIO_EnableInterrupts(AUDIO, interrupts);

    //Enable Audio Subsytem interrupts
    NVIC_ClearPendingIRQ(AUDIO_IRQ);
    NVIC_DisableIRQ(AUDIO_IRQ);
    MXC_NVIC_SetVector(AUDIO_IRQ, AUDIO_IRQHandler);
    NVIC_EnableIRQ(AUDIO_IRQ);

    while (1) {}
}

static int max9867ConfigureHeadphone(void)
{
    int err = E_NO_ERROR;

    if ((err = max9867_headphone_mode(HPMODE_STEREO_SE_CLICKLESS)) != E_NO_ERROR) {
        return err;
    }

    return max9867_playback_volume(6, 6);
}

static int max9867ConfigureDigitalMicrophone(void)
{
    int err = E_NO_ERROR;

    if ((err = max9867_microphone_enable(1, 0)) != E_NO_ERROR) {
        return err;
    }
    if ((err = max9867_adc_level(0, 0)) != E_NO_ERROR) {
        return err;
    }
    if ((err = max9867_linein_mute(1, 1)) != E_NO_ERROR) {
        return err;
    }

    if ((err = max9867_power_enable(0, EN_LEFT_DAC | EN_RIGHT_DAC | EN_RIGHT_ADC | EN_LEFT_ADC |
                                           EN_LEFT_LINEIN | EN_RIGHT_LINEIN)) != E_NO_ERROR) {
        return err;
    }

    return max9867_linein_gain(0, 0);
}

static int max9867ConfigureLineIn(void)
{
    int err = E_NO_ERROR;
    if ((err = max9867_adc_level(-12, -12)) != E_NO_ERROR) {
        return err;
    }
    if ((err = max9867_linein_mute(0, 0)) != E_NO_ERROR) {
        return err;
    }
    if ((err = max9867_linein_gain(24, 24)) != E_NO_ERROR) {
        return err;
    }

    if ((err = max9867_power_enable(0, EN_LEFT_LINEIN | EN_RIGHT_LINEIN)) != E_NO_ERROR) {
        return err;
    }

    return max9867_adc_input(ADC_IN_LINE_IN, ADC_IN_LINE_IN);
}

static int max9867Configure(void)
{
    int err = E_NO_ERROR;
    // Initialize audio codec
    if ((err = max9867_init(I2C_MASTER, 12288000, 0)) != E_NO_ERROR) {
        return err;
    }

    return max9867_digital_filter(1, BUTTERWORTH_SR16KHZ, BUTTERWORTH_SR16KHZ);
}

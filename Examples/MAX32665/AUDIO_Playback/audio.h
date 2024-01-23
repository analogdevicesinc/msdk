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

#ifndef EXAMPLES_MAX32665_AUDIO_PLAYBACK_AUDIO_H_
#define EXAMPLES_MAX32665_AUDIO_PLAYBACK_AUDIO_H_

/* **** Includes **** */
#include <stdint.h>
#include <stdbool.h>
#include "mxc_sys.h"
#include "audio_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */

// clang-format off
typedef enum _CLK_FREQ {
     MXC_AUDIO_CLK_12_288MHz,
     MXC_AUDIO_CLK_11_2896MHz
} mxc_audio_clock_frequency_t;

typedef enum _CLK_SRC {
    MXC_AUDIO_CLK_SRC_HSCLK,
    MXC_AUDIO_CLK_SRC_32MHZ,
    MXC_AUDIO_CLK_SRC_GPIO_0_23
} mxc_audio_master_clock_source_t;

typedef enum _CLK_POL {
    MXC_AUDIO_CLK_POL_HIGH,
    MXC_AUDIO_CLK_POL_LOW
    } mxc_audio_clock_polarity_t;

typedef enum _EXTRA_BITS {
    MXC_AUDIO_TX_EXTRA_BITS_0,
    MXC_AUDIO_TX_EXTRA_BITS_1
} mxc_audio_tx_extra_bits_t;

typedef enum _CH_SIZE {
    MXC_AUDIO_PCM_CHANNEL_SIZE_16 = 0x01,
    MXC_AUDIO_PCM_CHANNEL_SIZE_24,
    MXC_AUDIO_PCM_CHANNEL_SIZE_32
} mxc_audio_PCM_Channel_size;

typedef enum _LRCLK_DIV {
    MXC_AUDIO_LRCLK_DIV_32 = 0x02,
    MXC_AUDIO_LRCLK_DIV_48,
    MXC_AUDIO_LRCLK_DIV_64,
    MXC_AUDIO_LRCLK_DIV_96,
    MXC_AUDIO_LRCLK_DIV_128,
    MXC_AUDIO_LRCLK_DIV_192,
    MXC_AUDIO_LRCLK_DIV_256,
    MXC_AUDIO_LRCLK_DIV_384,
    MXC_AUDIO_LRCLK_DIV_512,
    MXC_AUDIO_LRCLK_DIV_320
} mxc_audio_LRCLK_divider;

typedef enum _PCM_SAMPLE_RATES {
    MXC_AUDIO_PCM_SAMPLE_RATE_8kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_11_025kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_12kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_16kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_22_05kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_24kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_32kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_44_1kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_48kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_88_2kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_96kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_176_4kHz,
    MXC_AUDIO_PCM_SAMPLE_RATE_192kHz,
}mxc_audio_PCM_sample_rates;

typedef enum _BCLK_SOURCE {
    MXC_AUDIO_BCLK_SOURCE_F_AUDIO,  ///< Clock source = fAudio_Clock
    MXC_AUDIO_BCLK_SOURCE_F_AUDIO_MN   ///< Clock source = fAudio_Clock *(N/M)
}mxc_audio_BCLK_source;

typedef enum _BCLK_SOURCE_SELECT {
    MXC_AUDIO_BCLK_GENERATOR_TICK,  ///<  BCLK generator, tick out
    MXC_AUDIO_BCLK_GENERATOR_TOGGLE,  ///< BCLK generator, toggle out
    MXC_AUDIO_BCLK_MASTER  ///< BCLK master
}mxc_audio_BCLK_source_Select;

// clang-format on

/**
 * @brief
 */

typedef struct {
    mxc_audio_regs_t *audio; ///< Pointer to AUDIO registers

    mxc_audio_clock_frequency_t clock; ///< AUDIO Master Clock Frequency
    mxc_audio_master_clock_source_t masterClockSource; ///< AUDIO Master Clock Source

    mxc_audio_BCLK_source_Select BCLKSourceSelect; ///< BCLK source select
    mxc_audio_BCLK_source BCLKSource; ///< BCLK source
    mxc_audio_clock_polarity_t BCLKPolarity; ///< BCLK polarity
    uint16_t BCLKDivisor; ///< BCLKDivisor

    mxc_audio_clock_polarity_t LRCLKPolarity; ///< LRCLK polarity
    mxc_audio_LRCLK_divider LRCLKDivider; ///< LRCLK divider

    mxc_audio_PCM_Channel_size channelSize; ///< PCM Channel Size

    mxc_audio_PCM_sample_rates TxInterfaceSampleRates; ///< TX Interface sample rates
    mxc_audio_PCM_sample_rates RxInterfaceSampleRates; ///< RX Interface sample rates
    mxc_audio_PCM_sample_rates TxDataportSampleRates; ///< TX Dataport sample rates
    mxc_audio_PCM_sample_rates RxDataportSampleRates; ///< TX Dataport sample rate
    mxc_audio_tx_extra_bits_t
        TxExtraBitsFormat; ///< Indicates the extra bits at the end of a PCM transmission will be transmitted as 0 or high impedance
} mxc_audio_I2S_config_t;

/* **** Globals **** */

/* **** Functions **** */
/**
 * @brief   Initialize and enable Audio Subsysyem peripheral.
 *
 * @param   audio       Pointer to Audio Subsystem registers 
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AUDIO_Init(mxc_audio_regs_t *audio);

/**
 * @brief   Initialize and enable I2S protocol.
 *
 * @param   config      Pointer to I2S config struct instance 
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AUDIO_I2S_Configure(mxc_audio_I2S_config_t *config);

/**
 * @brief   Enables the Audio Subsystem interrupts
 *
 *
 * @param   audio       Pointer to Audio Subsystem registers 
 * @param   interrupts  Interrupts to be enabled
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AUDIO_EnableInterrupts(mxc_audio_regs_t *audio, uint32_t interrupts);

/**
 * @brief   I2S receive function
 *
 *
 * @param   audio       Pointer to Audio Subsystem registers 
 * @param   leftData    Pointer to the buffer to read into left channel data
 * @param   rightData   Pointer to the buffer to read into right channel data
 * @param   len         The number of bytes to receive.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AUDIO_I2S_Receive(mxc_audio_regs_t *audio, uint32_t *leftData, uint32_t *rightData,
                          uint16_t len);

/**
 * @brief   I2S transmit function
 *
 *
 * @param   audio       Pointer to Audio Subsystem registers 
 * @param   leftData    Pointer to the data of the left channel to be transmitted
 * @param   rightData   Pointer to the data of the right channel to be transmitted
 * @param   len         The number of bytes to transmit.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AUDIO_I2S_Transmit(mxc_audio_regs_t *audio, uint32_t *leftData, uint32_t *rightData,
                           uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // EXAMPLES_MAX32665_AUDIO_PLAYBACK_AUDIO_H_

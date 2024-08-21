/**
 * @file    audio_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AUDIO Peripheral Module.
 */

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

#ifndef EXAMPLES_MAX32665_AUDIO_PLAYBACK_AUDIO_REGS_H_
#define EXAMPLES_MAX32665_AUDIO_PLAYBACK_AUDIO_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__ICCARM__)
#pragma system_include
#endif

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile /*!< Defines 'read / write' permissions */
#endif
#ifndef __I
#define __I volatile const /*!< Defines 'read only' permissions */
#endif
#ifndef __O
#define __O volatile /*!< Defines 'write only' permissions */
#endif
#ifndef __R
#define __R volatile const /*!< Defines 'read only' permissions */
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     audio
 * @defgroup    audio_registers PCM_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AUDIO Subsystem Peripheral Module.
 * @details     Audio Subsystem.
 */

/**
 * @ingroup audio_registers
 * Structure type to access the AUDIO Registers.
 */

typedef struct {
    __IO uint32_t tx_pdm_ch0_addr; /**< <tt>\b 0x00:</tt> PDM Tx FIFO Data Channel 0 Register */
    __IO uint32_t tx_pdm_ch1_addr; /**< <tt>\b 0x04:</tt> PDM Tx FIFO Data Channel 1 Register */
    __I uint32_t rx_pdm_1_ch0_addr; /**< <tt>\b 0x08:</tt> PDM Rx1 FIFO Data Channel 0 Register */
    __I uint32_t rx_pdm_1_ch1_addr; /**< <tt>\b 0x0C:</tt> PDM Rx1 FIFO Data Channel 1 Register */
    __I uint32_t rx_pdm_2_ch0_addr; /**< <tt>\b 0x10:</tt> PDM Rx2 FIFO Data Channel 0 Register */
    __I uint32_t rx_pdm_2_ch1_addr; /**< <tt>\b 0x14:</tt> PDM Rx2 FIFO Data Channel 1 Register */
    __I uint32_t rx_pdm_3_ch0_addr; /**< <tt>\b 0x18:</tt> PDM Rx3 FIFO Data Channel 0 Register */
    __I uint32_t rx_pdm_3_ch1_addr; /**< <tt>\b 0x1C:</tt> PDM Rx3 FIFO Data Channel 1 Register */

    __IO uint32_t tx_pcm_ch0_addr; /**< <tt>\b 0x20:</tt> PCM Tx FIFO Data Channel 0 Register */
    __IO uint32_t tx_pcm_ch1_addr; /**< <tt>\b 0x24:</tt> PCM Tx FIFO Data Channel 1 Register */
    __IO uint32_t tx_pcm_ch2_addr; /**< <tt>\b 0x28:</tt> PCM Tx FIFO Data Channel 2 Register */
    __IO uint32_t tx_pcm_ch3_addr; /**< <tt>\b 0x2C:</tt> PCM Tx FIFO Data Channel 3 Register */
    __I uint32_t rx_pcm_ch0_addr; /**< <tt>\b 0x30:</tt> PCM Rx FIFO Data Channel 0 Register */
    __I uint32_t rx_pcm_ch1_addr; /**< <tt>\b 0x34:</tt> PCM Rx FIFO Data Channel 1 Register */
    __I uint32_t rx_pcm_ch2_addr; /**< <tt>\b 0x38:</tt> PCM Rx FIFO Data Channel 2 Register */
    __I uint32_t rx_pcm_ch3_addr; /**< <tt>\b 0x3C:</tt> PCM Rx FIFO Data Channel 3 Register */
    __I uint32_t rx_pcm_ch4_addr; /**< <tt>\b 0x40:</tt> PCM Rx FIFO Data Channel 4 Register */
    __I uint32_t rx_pcm_ch5_addr; /**< <tt>\b 0x44:</tt> PCM Rx FIFO Data Channel 5 Register */
    __I uint32_t rx_pcm_ch6_addr; /**< <tt>\b 0x48:</tt> PCM Rx FIFO Data Channel 6 Register */
    __I uint32_t rx_pcm_ch7_addr; /**< <tt>\b 0x4C:</tt> PCM Rx FIFO Data Channel 7 Register */

    __I uint32_t int_pdm_status; /**< <tt>\b 0x50:</tt> PDM FIFO Status Register */
    __IO uint32_t int_pdm_clr; /**< <tt>\b 0x54:</tt> PDM Interrupt Clear Register */

    __I uint32_t int_pcm_tx_status; /**< <tt>\b 0x58:</tt> PCM Tx FIFO Status Register */
    __IO uint32_t int_pcm_tx_clr; /**< <tt>\b 0x5C:</tt> PCM Tx Interrupt Clear Register */
    __I uint32_t int_pcm_rx_status; /**< <tt>\b 0x60:</tt> PCM Rx FIFO Status Register */
    __IO uint32_t int_pcm_rx_clr; /**< <tt>\b 0x64:</tt> PCM Rx Interrupt Clear Register */
    __IO uint32_t int_en; /**< <tt>\b 0x68:</tt> Interrupt Enable Register */

    __I uint8_t rsv_0x6C_0x7F[0x14];

    __IO uint32_t m_val; /**< <tt>\b 0x80:</tt> NM Generator Denominator M Register */
    __I uint32_t rsv_0x840_0x87[0x01];
    __IO uint32_t n_val; /**< <tt>\b 0x88:</tt> NM Generator Denominator N Register */

    __I uint8_t rsv_0x8C_0x93[0x08];

    __IO uint32_t pdm_tx_rate_setup; /**< <tt>\b 0x94:</tt> PDM Tx Rate Setup Register */

    __IO uint32_t modulator_controls; /**< <tt>\b 0x98:</tt> Modulator Configuration Register */

    __IO uint32_t pdm_tx_control_1; /**< <tt>\b 0x9C:</tt> PDM Tx Configuration 0 Register */
    __IO uint32_t pdm_rx_rate_setup; /**< <tt>\b 0xA0:</tt> PDM Rx Clock Configuration Register */
    __IO uint32_t pdm_tx_control_2; /**< <tt>\b 0xA4:</tt> PDM Tx Configuration 1 Register */
    __IO uint32_t pdm_rx_1_control; /**< <tt>\b 0xA8:</tt> PDM Rx1 Configuration Register */
    __IO uint32_t pdm_rx_2_control; /**< <tt>\b 0xAC:</tt> PDM Rx2 Configuration Register */
    __IO uint32_t pdm_rx_3_control; /**< <tt>\b 0xB0:</tt> PDM Rx3 Configuration Register */

    __IO uint32_t pcm_clock_dividers_msb; /**< <tt>\b 0xB4:</tt> PCM BCLK Configuration 0 Register */
    __IO uint32_t pcm_clock_dividers_lsb; /**< <tt>\b 0xB8:</tt> PCM BCLK Configuration 1 Register */
    __IO uint32_t pcm_clock_setup; /**< <tt>\b 0xBC:</tt> PCM Clock Configuration Register */
    __IO uint32_t pcm_config; /**< <tt>\b 0xC0:</tt> PCM Configuration Register */

    __IO uint32_t
        pcm_tx_sample_rates; /**< <tt>\b 0xC4:</tt> PCM Tx Sample Rate Configuration Register */
    __IO uint32_t
        pcm_rx_sample_rates; /**< <tt>\b 0xC8:</tt> PCM Rx Sample Rate Configuration Register */

    __IO uint32_t pcm_rx_enables_byte0; /**< <tt>\b 0xCC:</tt> PCM Rx Channel Enable 0 Register */
    __IO uint32_t pcm_rx_enables_byte1; /**< <tt>\b 0xD0:</tt> PCM Rx Channel Enable 1 Register */

    __IO uint32_t pcm_tx_enables_byte0; /**< <tt>\b 0xD4:</tt> PCM Tx Channel Enable 0 Register */
    __IO uint32_t pcm_tx_enables_byte1; /**< <tt>\b 0xD8:</tt> PCM Tx Channel Enable 1 Register */

    __IO uint32_t pcm_tx_hiz_byte0; /**< <tt>\b 0xDC:</tt> PCM Tx Channel HI-Z Enable 0 Register */
    __IO uint32_t pcm_tx_hiz_byte1; /**< <tt>\b 0xE0:</tt> PCM Tx Channel HI-Z Enable 1 Register */

    __IO uint32_t
        dataport1_slot_mapping_ch0; /**< <tt>\b 0xE4:</tt>  Dataport 1 Slot Mapping Channel 0 Register */
    __IO uint32_t
        dataport1_slot_mapping_ch1; /**< <tt>\b 0xE8:</tt>  Dataport 1 Slot Mapping Channel 1 Register */
    __IO uint32_t
        dataport1_slot_mapping_ch2; /**< <tt>\b 0xEC:</tt>  Dataport 1 Slot Mapping Channel 2 Register */
    __IO uint32_t
        dataport1_slot_mapping_ch3; /**< <tt>\b 0xF0:</tt>  Dataport 1 Slot Mapping Channel 3 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch0; /**< <tt>\b 0xF4:</tt>  Dataport 2 Slot Mapping Channel 0 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch1; /**< <tt>\b 0xF8:</tt>  Dataport 2 Slot Mapping Channel 1 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch2; /**< <tt>\b 0xFC:</tt>  Dataport 2 Slot Mapping Channel 2 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch3; /**< <tt>\b 0x100:</tt> Dataport 2 Slot Mapping Channel 3 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch4; /**< <tt>\b 0x104:</tt> Dataport 2 Slot Mapping Channel 4 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch5; /**< <tt>\b 0x108:</tt> Dataport 2 Slot Mapping Channel 5 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch6; /**< <tt>\b 0x10C:</tt> Dataport 2 Slot Mapping Channel 6 Register */
    __IO uint32_t
        dataport2_slot_mapping_ch7; /**< <tt>\b 0x110:</tt> Dataport 2 Slot Mapping Channel 7 Register */

    __IO uint32_t pcm_data_controls; /**< <tt>\b 0x114:</tt> PCM Data Controls Register */
    __IO uint32_t global_en; /**< <tt>\b 0x118:</tt> Audio Enable Register */
} mxc_audio_regs_t;

/* Register offsets for module AUDIO */
/**
 * @ingroup    audio_registers
 * @defgroup   AUDIO_Register_Offsets Register Offsets
 * @brief      AUDIO Peripheral Register Offsets from the I2S_REVA Base Peripheral Address. 
 * @{
 */

#define MXC_R_AUDIO_TX_PDM_CH0_ADDR \
    ((uint32_t)0x00000000UL) /**< Offset from AUDIO Base Address: <tt> 0x0000</tt> */
#define MXC_R_AUDIO_TX_PDM_CH1_ADDR \
    ((uint32_t)0x00000004UL) /**< Offset from AUDIO Base Address: <tt> 0x0004</tt> */
#define MXC_R_AUDIO_RX_PDM_1_CH0_ADDR \
    ((uint32_t)0x00000008UL) /**< Offset from AUDIO Base Address: <tt> 0x0008</tt> */
#define MXC_R_AUDIO_RX_PDM_1_CH1_ADDR \
    ((uint32_t)0x0000000CUL) /**< Offset from AUDIO Base Address: <tt> 0x000C</tt> */
#define MXC_R_AUDIO_RX_PDM_2_CH0_ADDR \
    ((uint32_t)0x00000010UL) /**< Offset from AUDIO Base Address: <tt> 0x0010</tt> */
#define MXC_R_AUDIO_RX_PDM_2_CH1_ADDR \
    ((uint32_t)0x00000014UL) /**< Offset from AUDIO Base Address: <tt> 0x0014</tt> */
#define MXC_R_AUDIO_RX_PDM_3_CH0_ADDR \
    ((uint32_t)0x00000018UL) /**< Offset from AUDIO Base Address: <tt> 0x0018</tt> */
#define MXC_R_AUDIO_RX_PDM_3_CH1_ADDR \
    ((uint32_t)0x0000001CUL) /**< Offset from AUDIO Base Address: <tt> 0x001C</tt> */
#define MXC_R_AUDIO_TX_PCM_CH0_ADDR \
    ((uint32_t)0x00000020UL) /**< Offset from AUDIO Base Address: <tt> 0x0020</tt> */
#define MXC_R_AUDIO_TX_PCM_CH1_ADDR \
    ((uint32_t)0x00000024UL) /**< Offset from AUDIO Base Address: <tt> 0x0024</tt> */
#define MXC_R_AUDIO_TX_PCM_CH2_ADDR \
    ((uint32_t)0x00000028UL) /**< Offset from AUDIO Base Address: <tt> 0x0028</tt> */
#define MXC_R_AUDIO_TX_PCM_CH3_ADDR \
    ((uint32_t)0x0000002CUL) /**< Offset from AUDIO Base Address: <tt> 0x002C</tt> */
#define MXC_R_AUDIO_RX_PCM_CH0_ADDR \
    ((uint32_t)0x00000030UL) /**< Offset from AUDIO Base Address: <tt> 0x0030</tt> */
#define MXC_R_AUDIO_RX_PCM_CH1_ADDR \
    ((uint32_t)0x00000034UL) /**< Offset from AUDIO Base Address: <tt> 0x0034</tt> */
#define MXC_R_AUDIO_RX_PCM_CH2_ADDR \
    ((uint32_t)0x00000038UL) /**< Offset from AUDIO Base Address: <tt> 0x0038</tt> */
#define MXC_R_AUDIO_RX_PCM_CH3_ADDR \
    ((uint32_t)0x0000003CUL) /**< Offset from AUDIO Base Address: <tt> 0x003C</tt> */
#define MXC_R_AUDIO_RX_PCM_CH4_ADDR \
    ((uint32_t)0x00000040UL) /**< Offset from AUDIO Base Address: <tt> 0x0040</tt> */
#define MXC_R_AUDIO_RX_PCM_CH5_ADDR \
    ((uint32_t)0x00000044UL) /**< Offset from AUDIO Base Address: <tt> 0x0044</tt> */
#define MXC_R_AUDIO_RX_PCM_CH6_ADDR \
    ((uint32_t)0x00000048UL) /**< Offset from AUDIO Base Address: <tt> 0x0048</tt> */
#define MXC_R_AUDIO_RX_PCM_CH7_ADDR \
    ((uint32_t)0x0000004CUL) /**< Offset from AUDIO Base Address: <tt> 0x004C</tt> */
#define MXC_R_AUDIO_INT_PDM_STATUS \
    ((uint32_t)0x00000050UL) /**< Offset from AUDIO Base Address: <tt> 0x0050</tt> */
#define MXC_R_AUDIO_INT_PDM_CLR \
    ((uint32_t)0x00000054UL) /**< Offset from AUDIO Base Address: <tt> 0x0054</tt> */
#define MXC_R_AUDIO_INT_PCM_TX_STATUS \
    ((uint32_t)0x00000058UL) /**< Offset from AUDIO Base Address: <tt> 0x0058</tt> */
#define MXC_R_AUDIO_INT_PCM_TX_CLR \
    ((uint32_t)0x0000005CUL) /**< Offset from AUDIO Base Address: <tt> 0x005C</tt> */
#define MXC_R_AUDIO_INT_PCM_RX_STATUS \
    ((uint32_t)0x00000060UL) /**< Offset from AUDIO Base Address: <tt> 0x0060</tt> */
#define MXC_R_AUDIO_INT_PCM_RX_CLR \
    ((uint32_t)0x00000064UL) /**< Offset from AUDIO Base Address: <tt> 0x0064</tt> */
#define MXC_R_AUDIO_INT_EN \
    ((uint32_t)0x00000068UL) /**< Offset from AUDIO Base Address: <tt> 0x0068</tt> */
#define MXC_R_AUDIO_M_VAL \
    ((uint32_t)0x00000080UL) /**< Offset from AUDIO Base Address: <tt> 0x0080</tt> */
#define MXC_R_AUDIO_RES \
    ((uint32_t)0x00000084UL) /**< Offset from AUDIO Base Address: <tt> 0x0084</tt> */
#define MXC_R_AUDIO_N_VAL \
    ((uint32_t)0x00000088UL) /**< Offset from AUDIO Base Address: <tt> 0x0088</tt> */
#define MXC_R_AUDIO_PDM_TX_RATE_SETUP \
    ((uint32_t)0x00000094UL) /**< Offset from AUDIO Base Address: <tt> 0x0094</tt> */
#define MXC_R_AUDIO_MODULATOR_CONTROLS \
    ((uint32_t)0x00000098UL) /**< Offset from AUDIO Base Address: <tt> 0x0098</tt> */
#define MXC_R_AUDIO_PDM_TX_CONTROL_1 \
    ((uint32_t)0x0000009CUL) /**< Offset from AUDIO Base Address: <tt> 0x009C</tt> */
#define MXC_R_AUDIO_PDM_RX_RATE_SETUP \
    ((uint32_t)0x000000A0UL) /**< Offset from AUDIO Base Address: <tt> 0x00A0</tt> */
#define MXC_R_AUDIO_PDM_TX_CONTROL_2 \
    ((uint32_t)0x000000A4UL) /**< Offset from AUDIO Base Address: <tt> 0x00A4</tt> */
#define MXC_R_AUDIO_PDM_RX_1_CONTROL \
    ((uint32_t)0x000000A8UL) /**< Offset from AUDIO Base Address: <tt> 0x00A8</tt> */
#define MXC_R_AUDIO_PDM_RX_2_CONTROL \
    ((uint32_t)0x000000ACUL) /**< Offset from AUDIO Base Address: <tt> 0x00AC</tt> */
#define MXC_R_AUDIO_PDM_RX_3_CONTROL \
    ((uint32_t)0x000000B0UL) /**< Offset from AUDIO Base Address: <tt> 0x00B0</tt> */
#define MXC_R_AUDIO_PCM_CLOCK_DIVIDERS_MSB \
    ((uint32_t)0x000000B4UL) /**< Offset from AUDIO Base Address: <tt> 0x00B4</tt> */
#define MXC_R_AUDIO_PCM_CLOCK_DIVIDERS_LSB \
    ((uint32_t)0x000000B8UL) /**< Offset from AUDIO Base Address: <tt> 0x00B8</tt> */
#define MXC_R_AUDIO_PCM_CLOCK_SET_UP \
    ((uint32_t)0x000000BCUL) /**< Offset from AUDIO Base Address: <tt> 0x00BC</tt> */
#define MXC_R_AUDIO_PCM_CONFIG \
    ((uint32_t)0x000000C0UL) /**< Offset from AUDIO Base Address: <tt> 0x00C0</tt> */
#define MXC_R_AUDIO_PCM_TX_SAMPLE_RATES \
    ((uint32_t)0x000000C4UL) /**< Offset from AUDIO Base Address: <tt> 0x00C4</tt> */
#define MXC_R_AUDIO_PCM_RX_SAMPLE_RATES \
    ((uint32_t)0x000000C8UL) /**< Offset from AUDIO Base Address: <tt> 0x00C8</tt> */
#define MXC_R_AUDIO_PCM_RX_ENABLES_BYTE0 \
    ((uint32_t)0x000000CCUL) /**< Offset from AUDIO Base Address: <tt> 0x00CC</tt> */
#define MXC_R_AUDIO_PCM_RX_ENABLES_BYTE1 \
    ((uint32_t)0x000000D0UL) /**< Offset from AUDIO Base Address: <tt> 0x00D0</tt> */
#define MXC_R_AUDIO_PCM_TX_ENABLES_BYTE0 \
    ((uint32_t)0x000000D4UL) /**< Offset from AUDIO Base Address: <tt> 0x00D4</tt> */
#define MXC_R_AUDIO_PCM_TX_ENABLES_BYTE1 \
    ((uint32_t)0x000000D8UL) /**< Offset from AUDIO Base Address: <tt> 0x00D8</tt> */
#define MXC_R_AUDIO_PCM_TX_HIZ_BYTE0 \
    ((uint32_t)0x000000DCUL) /**< Offset from AUDIO Base Address: <tt> 0x00DC</tt> */
#define MXC_R_AUDIO_PCM_TX_HIZ_BYTE1 \
    ((uint32_t)0x000000E0UL) /**< Offset from AUDIO Base Address: <tt> 0x00E0</tt> */
#define MXC_R_AUDIO_DATAPORT1_SLOT_MAPPING_CH0 \
    ((uint32_t)0x000000E4UL) /**< Offset from AUDIO Base Address: <tt> 0x00E4</tt> */
#define MXC_R_AUDIO_DATAPORT1_SLOT_MAPPING_CH1 \
    ((uint32_t)0x000000E8UL) /**< Offset from AUDIO Base Address: <tt> 0x00E8</tt> */
#define MXC_R_AUDIO_DATAPORT1_SLOT_MAPPING_CH2 \
    ((uint32_t)0x000000ECUL) /**< Offset from AUDIO Base Address: <tt> 0x00EC</tt> */
#define MXC_R_AUDIO_DATAPORT1_SLOT_MAPPING_CH3 \
    ((uint32_t)0x000000F0UL) /**< Offset from AUDIO Base Address: <tt> 0x00F0</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH0 \
    ((uint32_t)0x000000F4UL) /**< Offset from AUDIO Base Address: <tt> 0x00F4</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH1 \
    ((uint32_t)0x000000F8UL) /**< Offset from AUDIO Base Address: <tt> 0x00F8</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH2 \
    ((uint32_t)0x000000FCUL) /**< Offset from AUDIO Base Address: <tt> 0x00FC</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH3 \
    ((uint32_t)0x00000100UL) /**< Offset from AUDIO Base Address: <tt> 0x0100</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH4 \
    ((uint32_t)0x00000104UL) /**< Offset from AUDIO Base Address: <tt> 0x0104</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH5 \
    ((uint32_t)0x00000108UL) /**< Offset from AUDIO Base Address: <tt> 0x0108</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH6 \
    ((uint32_t)0x0000010CUL) /**< Offset from AUDIO Base Address: <tt> 0x010C</tt> */
#define MXC_R_AUDIO_DATAPORT2_SLOT_MAPPING_CH7 \
    ((uint32_t)0x00000110UL) /**< Offset from AUDIO Base Address: <tt> 0x0110</tt> */
#define MXC_R_AUDIO_PCM_DATA_CONTROLS \
    ((uint32_t)0x00000114UL) /**< Offset from AUDIO Base Address: <tt> 0x0114</tt> */
#define MXC_R_AUDIO_GLOBAL_EN \
    ((uint32_t)0x00000118UL) /**< Offset from AUDIO Base Address: <tt> 0x0000</tt> */

/**@} end of group audio_registers */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_INT_PDM_STATUS AUDIO_INT_PDM_STATUS
 * @brief    PDM Interrupt Status.
 * @{
 */
#define MXC_F_PDM_RX3_FIFO_CH1_ALMOST_EMPTY_POS \
    8 /**< MXC_F_PDM_RX3_FIFO_CH1_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_RX3_FIFO_CH1_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                       \
                << MXC_F_PDM_RX3_FIFO_CH1_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_RX3_FIFO_CH1_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_RX3_FIFO_CH0_ALMOST_EMPTY_POS \
    9 /**< MXC_F_PDM_RX3_FIFO_CH0_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_RX3_FIFO_CH0_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                       \
                << MXC_F_PDM_RX3_FIFO_CH0_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_RX3_FIFO_CH0_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_RX2_FIFO_CH1_ALMOST_EMPTY_POS \
    10 /**< MXC_F_PDM_RX2_FIFO_CH1_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_RX2_FIFO_CH1_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                       \
                << MXC_F_PDM_RX2_FIFO_CH1_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_RX2_FIFO_CH1_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_RX02FIFO_CH0_ALMOST_EMPTY_POS \
    11 /**< MXC_F_PDM_RX02FIFO_CH0_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_RX02FIFO_CH0_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                       \
                << MXC_F_PDM_RX02FIFO_CH0_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_RX02FIFO_CH0_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_RX1_FIFO_CH1_ALMOST_EMPTY_POS \
    12 /**< MXC_F_PDM_RX1_FIFO_CH1_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_RX1_FIFO_CH1_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                       \
                << MXC_F_PDM_RX1_FIFO_CH1_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_RX1_FIFO_CH1_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_RX1_FIFO_CH0_ALMOST_EMPTY_POS \
    13 /**< MXC_F_PDM_RX1_FIFO_CH0_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_RX1_FIFO_CH0_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                       \
                << MXC_F_PDM_RX1_FIFO_CH0_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_RX1_FIFO_CH0_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_TX_FIFO_CH1_ALMOST_EMPTY_POS \
    14 /**< MXC_F_PDM_TX_FIFO_CH1_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_TX_FIFO_CH1_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_TX_FIFO_CH1_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_TX_FIFO_CH1_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_TX_FIFO_CH0_ALMOST_EMPTY_POS \
    15 /**< MXC_F_PDM_TX_FIFO_CH0_ALMOST_EMPTY_POS Position */
#define MXC_F_PDM_TX_FIFO_CH0_ALMOST_EMPTY \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_TX_FIFO_CH0_ALMOST_EMPTY_POS)) /**< MXC_F_PDM_TX_FIFO_CH0_ALMOST_EMPTY_POS Mask */

#define MXC_F_PDM_RX3_FIFO_CH1_HALF_FULL_POS \
    16 /**< MXC_F_PDM_RX3_FIFO_CH1_HALF_FULL_POS Position */
#define MXC_F_PDM_RX3_FIFO_CH1_HALF_FULL \
    ((uint32_t)(0x1UL                    \
                << MXC_F_PDM_RX3_FIFO_CH1_HALF_FULL_POS)) /**< MXC_F_PDM_RX3_FIFO_CH1_HALF_FULL_POS Mask */

#define MXC_F_PDM_RX3_FIFO_CH0_HALF_FULL_POS \
    17 /**< MXC_F_PDM_RX3_FIFO_CH0_HALF_FULL_POS Position */
#define MXC_F_PDM_RX3_FIFO_CH0_HALF_FULL \
    ((uint32_t)(0x1UL                    \
                << MXC_F_PDM_RX3_FIFO_CH0_HALF_FULL_POS)) /**< MXC_F_PDM_RX3_FIFO_CH0_HALF_FULL_POS Mask */

#define MXC_F_PDM_RX2_FIFO_CH1_HALF_FULL_POS \
    18 /**< MXC_F_PDM_RX2_FIFO_CH1_HALF_FULL_POS Position */
#define MXC_F_PDM_RX2_FIFO_CH1_HALF_FULL \
    ((uint32_t)(0x1UL                    \
                << MXC_F_PDM_RX2_FIFO_CH1_HALF_FULL_POS)) /**< MXC_F_PDM_RX2_FIFO_CH1_HALF_FULL_POS Mask */

#define MXC_F_PDM_RX2_FIFO_CH0_HALF_FULL_POS \
    19 /**< MXC_F_PDM_RX2_FIFO_CH0_HALF_FULL_POS Position */
#define MXC_F_PDM_RX2_FIFO_CH0_HALF_FULL \
    ((uint32_t)(0x1UL                    \
                << MXC_F_PDM_RX2_FIFO_CH0_HALF_FULL_POS)) /**< MXC_F_PDM_RX2_FIFO_CH0_HALF_FULL_POS Mask */

#define MXC_F_PDM_RX1_FIFO_CH1_HALF_FULL_POS \
    20 /**< MXC_F_PDM_RX1_FIFO_CH1_HALF_FULL_POS Position */
#define MXC_F_PDM_RX1_FIFO_CH1_HALF_FULL \
    ((uint32_t)(0x1UL                    \
                << MXC_F_PDM_RX1_FIFO_CH1_HALF_FULL_POS)) /**< MXC_F_PDM_RX1_FIFO_CH1_HALF_FULL_POS Mask */

#define MXC_F_PDM_RX1_FIFO_CH0_HALF_FULL_POS \
    21 /**< MXC_F_PDM_RX1_FIFO_CH0_HALF_FULL_POS Position */
#define MXC_F_PDM_RX1_FIFO_CH0_HALF_FULL \
    ((uint32_t)(0x1UL                    \
                << MXC_F_PDM_RX1_FIFO_CH0_HALF_FULL_POS)) /**< MXC_F_PDM_RX1_FIFO_CH0_HALF_FULL_POS Mask */

#define MXC_F_PDM_TX_FIFO_CH1_HALF_FULL_POS 22 /**< MXC_F_PDM_TX_FIFO_CH1_HALF_FULL_POS Position */
#define MXC_F_PDM_TX_FIFO_CH1_HALF_FULL \
    ((uint32_t)(0x1UL                   \
                << MXC_F_PDM_TX_FIFO_CH1_HALF_FULL_POS)) /**< MXC_F_PDM_TX_FIFO_CH1_HALF_FULL_POS Mask */

#define MXC_F_PDM_TX_FIFO_CH0_HALF_FULL_POS 23 /**< MXC_F_PDM_TX_FIFO_CH0_HALF_FULL_POS Position */
#define MXC_F_PDM_TX_FIFO_CH0_HALF_FULL \
    ((uint32_t)(0x1UL                   \
                << MXC_F_PDM_TX_FIFO_CH0_HALF_FULL_POS)) /**< MXC_F_PDM_TX_FIFO_CH0_HALF_FULL_POS Mask */

#define MXC_F_PDM_RX3_FIFO_CH1_ALMOST_FULL_POS \
    24 /**< MXC_F_PDM_RX3_FIFO_CH1_ALMOST_FULL_POS Position */
#define MXC_F_PDM_RX3_FIFO_CH1_ALMOST_FULL \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_RX3_FIFO_CH1_ALMOST_FULL_POS)) /**< MXC_F_PDM_RX3_FIFO_CH1_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_RX3_FIFO_CH0_ALMOST_FULL_POS \
    25 /**< MXC_F_PDM_RX3_FIFO_CH0_ALMOST_FULL_POS Position */
#define MXC_F_PDM_RX3_FIFO_CH0_ALMOST_FULL \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_RX3_FIFO_CH0_ALMOST_FULL_POS)) /**< MXC_F_PDM_RX3_FIFO_CH0_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_RX2_FIFO_CH1_ALMOST_FULL_POS \
    26 /**< MXC_F_PDM_RX2_FIFO_CH1_ALMOST_FULL_POS Position */
#define MXC_F_PDM_RX2_FIFO_CH1_ALMOST_FULL \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_RX2_FIFO_CH1_ALMOST_FULL_POS)) /**< MXC_F_PDM_RX2_FIFO_CH1_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_RX2_FIFO_CH0_ALMOST_FULL_POS \
    27 /**< MXC_F_PDM_RX2_FIFO_CH0_ALMOST_FULL_POS Position */
#define MXC_F_PDM_RX2_FIFO_CH0_ALMOST_FULL \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_RX2_FIFO_CH0_ALMOST_FULL_POS)) /**< MXC_F_PDM_RX2_FIFO_CH0_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_RX1_FIFO_CH1_ALMOST_FULL_POS \
    28 /**< MXC_F_PDM_RX1_FIFO_CH1_ALMOST_FULL_POS Position */
#define MXC_F_PDM_RX1_FIFO_CH1_ALMOST_FULL \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_RX1_FIFO_CH1_ALMOST_FULL_POS)) /**< MXC_F_PDM_RX1_FIFO_CH1_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_RX1_FIFO_CH0_ALMOST_FULL_POS \
    29 /**< MXC_F_PDM_RX1_FIFO_CH0_ALMOST_FULL_POS Position */
#define MXC_F_PDM_RX1_FIFO_CH0_ALMOST_FULL \
    ((uint32_t)(0x1UL                      \
                << MXC_F_PDM_RX1_FIFO_CH0_ALMOST_FULL_POS)) /**< MXC_F_PDM_RX1_FIFO_CH0_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_TX_FIFO_CH1_ALMOST_FULL_POS \
    30 /**< MXC_F_PDM_TX_FIFO_CH1_ALMOST_FULL_POS Position */
#define MXC_F_PDM_TX_FIFO_CH1_ALMOST_FULL \
    ((uint32_t)(0x1UL                     \
                << MXC_F_PDM_TX_FIFO_CH1_ALMOST_FULL_POS)) /**< MXC_F_PDM_TX_FIFO_CH1_ALMOST_FULL_POS Mask */

#define MXC_F_PDM_TX_FIFO_CH0_ALMOST_FULL_POS \
    31 /**< MXC_F_PDM_TX_FIFO_CH0_ALMOST_FULL_POS Position */
#define MXC_F_PDM_TX_FIFO_CH0_ALMOST_FULL \
    ((uint32_t)(0x1UL                     \
                << MXC_F_PDM_TX_FIFO_CH0_ALMOST_FULL_POS)) /**< MXC_F_PDM_TX_FIFO_CH0_ALMOST_FULL_POS Mask */

/**@} end of group AUDIO_INT_PDM_STATUS */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_INT_EN AUDIO_INT_EN
 * @brief    Interrupt Enable Register
 * @{
 */

#define MXC_F_EN_AE_PCM_RX_POS 10 /**< MXC_F_EN_AE_PCM_RX Position */
#define MXC_F_EN_AE_PCM_RX ((uint32_t)(0x01UL << MXC_F_EN_AE_PCM_RX_POS)) /**< MXC_F_EN_AE_PCM_RX */

/**@} end of group AUDIO_INT_EN */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_MODULATOR_CONTROLS AUDIO_MODULATOR_CONTROLS
 * @brief    Modulator control register
 * @{
 */

#define MXC_F_TX_MOD_ENABLE_POS 0 /**< MXC_F_TX_MOD_ENABLE Position */
#define MXC_F_TX_MOD_ENABLE \
    ((uint32_t)(0x01UL << MXC_F_TX_MOD_ENABLE_POS)) /**< MXC_F_TX_MOD_ENABLE */

#define MXC_F_TX_MONO_ENABLE_POS 1 /**< MXC_F_TX_MONO_ENABLE Position */
#define MXC_F_TX_MONO_ENABLE \
    ((uint32_t)(0x01UL << MXC_F_TX_MONO_ENABLE_POS)) /**< MXC_F_TX_MONO_ENABLE */

#define MXC_F_TX_MOD_BYPASS_POS 2 /**< MXC_F_TX_MOD_BYPASS Position */
#define MXC_F_TX_MOD_BYPASS \
    ((uint32_t)(0x01UL << MXC_F_TX_MOD_BYPASS_POS)) /**< MXC_F_TX_MOD_BYPASS */

#define MXC_F_PDM_TX_ALT_ENCODE_POS 3 /**< MXC_F_PDM_TX_ALT_ENCODE Position */
#define MXC_F_PDM_TX_ALT_ENCODE \
    ((uint32_t)(0x01UL << MXC_F_PDM_TX_ALT_ENCODE_POS)) /**< MXC_F_PDM_TX_ALT_ENCODE */

#define MXC_F_PDM_PCM_SELECT_POS 7 /**< MXC_F_PDM_PCM_SELECT Position */
#define MXC_F_PDM_PCM_SELECT \
    ((uint32_t)(0x01UL << MXC_F_PDM_PCM_SELECT_POS)) /**< MXC_F_PDM_PCM_SELECT */

/**@} end of group AUDIO_MODULATOR_CONTROLS */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_CLOCK_DIVIDERS_MSB AUDIO_PCM_CLOCK_DIVIDERS_MSB
 * @brief    PCM Clock Dividers MSB
 * @{
 */

#define MXC_F_PCM_BCLK_DIV_MSB_POS 0 /**< MXC_F_PCM_BCLK_DIV_MSB Position */
#define MXC_F_PCM_BCLK_DIV_MSB \
    ((uint32_t)(0x00UL << MXC_F_PCM_BCLK_DIV_MSB_POS)) /**< MXC_F_PCM_BCLK_DIV_MSB */

#define MXC_F_PCM_BCLK_SEL_POS 5 /**< MXC_F_PCM_BCLK_SEL Position */
#define MXC_F_PCM_BCLK_SEL_BCLK_TICK \
    ((uint32_t)(0x00UL               \
                << MXC_F_PCM_BCLK_SEL_POS)) /**< MXC_F_PCM_BCLK_SEL BCLK generator, tick out*/
#define MXC_F_PCM_BCLK_SEL_BCLK_TOGGLE \
    ((uint32_t)(0x01UL                 \
                << MXC_F_PCM_BCLK_SEL_POS)) /**< MXC_F_PCM_BCLK_SEL BCLK generator, toggle out*/
#define MXC_F_PCM_BCLK_SEL_BCLK_MASTER \
    ((uint32_t)(0x02UL << MXC_F_PCM_BCLK_SEL_POS)) /**< MXC_F_PCM_BCLK_SEL BCLK master*/

#define MXC_F_PCM_BCLK_MASTER_SEL_POS 7 /**< MXC_F_PCM_BCLK_MASTER_SEL Position */
#define MXC_F_PCM_BCLK_MASTER_SEL_AUDIO_CLK \
    ((uint32_t)(0x00UL                      \
                << MXC_F_PCM_BCLK_MASTER_SEL_POS)) /**< MXC_F_PCM_BCLK_MASTER_SEL Audio Clock*/
#define MXC_F_PCM_BCLK_MASTER_SEL_MODULATOR_CLK \
    ((uint32_t)(0x01UL                          \
                << MXC_F_PCM_BCLK_MASTER_SEL_POS)) /**< MXC_F_PCM_BCLK_MASTER_SEL Modulator Clock*/

/**@} end of group AUDIO_PCM_CLOCK_DIVIDERS_MSB */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_CLOCK_DIVIDERS_LSB AUDIO_PCM_CLOCK_DIVIDERS_LSB
 * @brief    PCM Clock Dividers LSB
 * @{
 */

#define MXC_F_PCM_BCLK_DIV_LSB_POS 0 /**< MXC_F_PCM_BCLK_DIV_MSB Position */
#define MXC_F_PCM_BCLK_DIV_LSB \
    ((uint32_t)(0x01UL << MXC_F_PCM_BCLK_DIV_MSB_POS)) /**< MXC_F_PCM_BCLK_DIV_MSB */

/**@} end of group AUDIO_PCM_CLOCK_DIVIDERS_LSB */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_CLOCK_SET_UP AUDIO_PCM_CLOCK_SET_UP
 * @brief    PCM Clock Config
 * @{
 */

#define MXC_F_PCM_BSEL_POS 0 /**< MXC_F_PCM_BSEL Position */
#define MXC_F_PCM_BSEL_32 \
    ((uint32_t)(0x02UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 32 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_48 \
    ((uint32_t)(0x03UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 48 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_64 \
    ((uint32_t)(0x04UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 64 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_96 \
    ((uint32_t)(0x05UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 96 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_128 \
    ((uint32_t)(0x06UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 128 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_192 \
    ((uint32_t)(0x07UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 192 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_256 \
    ((uint32_t)(0x08UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 256 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_384 \
    ((uint32_t)(0x09UL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 384 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_512 \
    ((uint32_t)(0x0AUL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 512 BCLK pulses per LRCLK*/
#define MXC_F_PCM_BSEL_320 \
    ((uint32_t)(0x0BUL << MXC_F_PCM_BSEL_POS)) /**< MXC_F_PCM_BSEL Mask 320 BCLK pulses per LRCLK*/

#define MXC_F_PCM_BCLKEDGE_POS 4 /**< MXC_F_PCM_BCLKEDGE Position */
#define MXC_F_PCM_BCLKEDGE \
    ((uint32_t)(0x01UL << MXC_F_PCM_BCLKEDGE_POS)) /**< MXC_F_PCM_BCLKEDGE Mask */

/**@} end of group AUDIO_PCM_CLOCK_SET_UP */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_CONFIG AUDIO_PCM_CONFIG
 * @brief    PCM Config
 * @{
 */

#define MXC_F_PCM_TX_EXTRA_HIZ_POS 0 /**< MXC_F_PCM_TX_EXTRA_HIZ Position */
#define MXC_F_PCM_TX_EXTRA_HIZ \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_EXTRA_HIZ_POS)) /**< MXC_F_PCM_TX_EXTRA_HIZ Mask */

#define MXC_F_PCM_CHANSEL_POS 2 /**< MXC_F_PCM_CHANSEL Position */
#define MXC_F_PCM_CHANSEL \
    ((uint32_t)(0x01UL << MXC_F_PCM_CHANSEL_POS)) /**< MXC_F_PCM_CHANSEL Mask */

#define MXC_F_PCM_FORMATX_POS 3 /**< MXC_F_PCM_BSEL Position */
#define MXC_F_PCM_FORMATX_I2S \
    ((uint32_t)(0x00UL << MXC_F_PCM_FORMATX_POS)) /**< MXC_F_PCM_FORMATX I2S Format*/
#define MXC_F_PCM_FORMATX_PCM_LEFT \
    ((uint32_t)(0x01UL << MXC_F_PCM_FORMATX_POS)) /**< MXC_F_PCM_FORMATX PCM justified*/
#define MXC_F_PCM_FORMATX_PCM_RIGT \
    ((uint32_t)(0x02UL << MXC_F_PCM_FORMATX_POS)) /**< MXC_F_PCM_FORMATX PCM right justified*/
#define MXC_F_PCM_FORMATX_TDM_MODE_1 \
    ((uint32_t)(0x03UL << MXC_F_PCM_FORMATX_POS)) /**< MXC_F_PCM_FORMATX TDM mode 1*/
#define MXC_F_PCM_FORMATX_TDM_MODE_2 \
    ((uint32_t)(0x04UL << MXC_F_PCM_FORMATX_POS)) /**< MXC_F_PCM_FORMATX TDM mode 2*/
#define MXC_F_PCM_FORMATX_TDM_MODE_3 \
    ((uint32_t)(0x05UL << MXC_F_PCM_FORMATX_POS)) /**< MXC_F_PCM_FORMATX TDM mode 3*/

#define MXC_F_PCM_CHANSZ_POS 6 /**< MXC_F_PCM_CHANSZ Position */
#define MXC_F_PCM_CHANSZ_16 \
    ((uint32_t)(0x01UL << MXC_F_PCM_CHANSZ_POS)) /**< MXC_F_PCM_CHANSZ 16-bit*/
#define MXC_F_PCM_CHANSZ_24 \
    ((uint32_t)(0x02UL << MXC_F_PCM_CHANSZ_POS)) /**< MXC_F_PCM_CHANSZ 24-bit*/
#define MXC_F_PCM_CHANSZ_32 \
    ((uint32_t)(0x03UL << MXC_F_PCM_CHANSZ_POS)) /**< MXC_F_PCM_CHANSZ 32-bit*/

/**@} end of group AUDIO_PCM_CONFIG */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_TX_SAMPLE_RATES AUDIO_PCM_TX_SAMPLE_RATES
 * @brief    PCM Tx Sample Rates
 * @{
 */

#define MXC_F_PCM_TX_INTERFACE_SR_POS 0 /**< MXC_F_PCM_TX_INTERFACE_SR Position */
#define MXC_F_PCM_TX_INTERFACE_SR_8 \
    ((uint32_t)(0x00UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 8kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_11_025 \
    ((uint32_t)(0x01UL                   \
                << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 11.025kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_12 \
    ((uint32_t)(0x02UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 12kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_16 \
    ((uint32_t)(0x03UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 16kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_22_05 \
    ((uint32_t)(0x04UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 22.05kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_24 \
    ((uint32_t)(0x05UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 24kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_32 \
    ((uint32_t)(0x06UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 32kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_44_1 \
    ((uint32_t)(0x07UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 44.1kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_48 \
    ((uint32_t)(0x08UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 48kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_88_2 \
    ((uint32_t)(0x09UL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 88.2kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_96 \
    ((uint32_t)(0x0AUL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 96kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_176_4 \
    ((uint32_t)(0x0BUL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 176.4kHz */
#define MXC_F_PCM_TX_INTERFACE_SR_192 \
    ((uint32_t)(0x0CUL << MXC_F_PCM_TX_INTERFACE_SR_POS)) /**< MXC_F_PCM_TX_INTERFACE_SR 192kHz */

#define MXC_F_PCM_TX_DATAPORT_SR_POS 4 /**< MXC_F_PCM_TX_DATAPORT_SR Position */
#define MXC_F_PCM_TX_DATAPORT_SR_8 \
    ((uint32_t)(0x00UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 8kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_11_025 \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 11.025kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_12 \
    ((uint32_t)(0x02UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 12kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_16 \
    ((uint32_t)(0x03UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 16kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_22_05 \
    ((uint32_t)(0x04UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 22.05kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_24 \
    ((uint32_t)(0x05UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 24kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_32 \
    ((uint32_t)(0x06UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 32kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_44_1 \
    ((uint32_t)(0x07UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 44.1kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_48 \
    ((uint32_t)(0x08UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 48kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_88_2 \
    ((uint32_t)(0x09UL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 88.2kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_96 \
    ((uint32_t)(0x0AUL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 96kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_176_4 \
    ((uint32_t)(0x0BUL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 176.4kHz */
#define MXC_F_PCM_TX_DATAPORT_SR_192 \
    ((uint32_t)(0x0CUL << MXC_F_PCM_TX_DATAPORT_SR_POS)) /**< MXC_F_PCM_TX_DATAPORT_SR 192kHz */

/**@} end of group AUDIO_PCM_TX_SAMPLE_RATES */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_RX_SAMPLE_RATES AUDIO_PCM_RX_SAMPLE_RATES
 * @brief    PCM Rx Sample Rates
 * @{
 */

#define MXC_F_PCM_RX_INTERFACE_SR_POS 0 /**< MXC_F_PCM_RX_INTERFACE_SR Position */
#define MXC_F_PCM_RX_INTERFACE_SR_8 \
    ((uint32_t)(0x00UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 8kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_11_025 \
    ((uint32_t)(0x01UL                   \
                << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 11.025kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_12 \
    ((uint32_t)(0x02UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 12kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_16 \
    ((uint32_t)(0x03UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 16kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_22_05 \
    ((uint32_t)(0x04UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 22.05kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_24 \
    ((uint32_t)(0x05UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 24kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_32 \
    ((uint32_t)(0x06UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 32kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_44_1 \
    ((uint32_t)(0x07UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 44.1kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_48 \
    ((uint32_t)(0x08UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 48kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_88_2 \
    ((uint32_t)(0x09UL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 88.2kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_96 \
    ((uint32_t)(0x0AUL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 96kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_176_4 \
    ((uint32_t)(0x0BUL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 176.4kHz */
#define MXC_F_PCM_RX_INTERFACE_SR_192 \
    ((uint32_t)(0x0CUL << MXC_F_PCM_RX_INTERFACE_SR_POS)) /**< MXC_F_PCM_RX_INTERFACE_SR 192kHz */

#define MXC_F_PCM_RX_DATAPORT_SR_POS 4 /**< MXC_F_PCM_RX_DATAPORT_SR Position */
#define MXC_F_PCM_RX_DATAPORT_SR_8 \
    ((uint32_t)(0x00UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 8kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_11_025 \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 11.025kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_12 \
    ((uint32_t)(0x02UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 12kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_16 \
    ((uint32_t)(0x03UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 16kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_22_05 \
    ((uint32_t)(0x04UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 22.05kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_24 \
    ((uint32_t)(0x05UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 24kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_32 \
    ((uint32_t)(0x06UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 32kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_44_1 \
    ((uint32_t)(0x07UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 44.1kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_48 \
    ((uint32_t)(0x08UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 48kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_88_2 \
    ((uint32_t)(0x09UL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 88.2kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_96 \
    ((uint32_t)(0x0AUL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 96kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_176_4 \
    ((uint32_t)(0x0BUL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 176.4kHz */
#define MXC_F_PCM_RX_DATAPORT_SR_192 \
    ((uint32_t)(0x0CUL << MXC_F_PCM_RX_DATAPORT_SR_POS)) /**< MXC_F_PCM_RX_DATAPORT_SR 192kHz */

/**@} end of group AUDIO_PCM_RX_SAMPLE_RATES */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_RX_ENABLES_BYTE_0 AUDIO_PCM_RX_ENABLES_BYTE_0
 * @brief    PCM Rx Channels Enable Byte 0
 * @{
 */

#define MXC_F_PCM_RX_CH0_EN_POS 0 /**< MXC_F_PCM_RX_CH0_EN Position */
#define MXC_F_PCM_RX_CH0_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH0_EN_POS)) /**< MXC_F_PCM_RX_CH0_EN */

#define MXC_F_PCM_RX_CH1_EN_POS 1 /**< MXC_F_PCM_RX_CH1_EN Position */
#define MXC_F_PCM_RX_CH1_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH1_EN_POS)) /**< MXC_F_PCM_RX_CH1_EN */

#define MXC_F_PCM_RX_CH2_EN_POS 2 /**< MXC_F_PCM_RX_CH2_EN Position */
#define MXC_F_PCM_RX_CH2_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH2_EN_POS)) /**< MXC_F_PCM_RX_CH2_EN */

#define MXC_F_PCM_RX_CH3_EN_POS 3 /**< MXC_F_PCM_RX_CH3_EN Position */
#define MXC_F_PCM_RX_CH3_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH3_EN_POS)) /**< MXC_F_PCM_RX_CH3_EN */

#define MXC_F_PCM_RX_CH4_EN_POS 4 /**< MXC_F_PCM_RX_CH4_EN Position */
#define MXC_F_PCM_RX_CH4_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH4_EN_POS)) /**< MXC_F_PCM_RX_CH4_EN */

#define MXC_F_PCM_RX_CH5_EN_POS 5 /**< MXC_F_PCM_RX_CH5_EN Position */
#define MXC_F_PCM_RX_CH5_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH5_EN_POS)) /**< MXC_F_PCM_RX_CH5_EN */

#define MXC_F_PCM_RX_CH6_EN_POS 6 /**< MXC_F_PCM_RX_CH6_EN Position */
#define MXC_F_PCM_RX_CH6_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH6_EN_POS)) /**< MXC_F_PCM_RX_CH6_EN */

#define MXC_F_PCM_RX_CH7_EN_POS 7 /**< MXC_F_PCM_RX_CH7_EN Position */
#define MXC_F_PCM_RX_CH7_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH7_EN_POS)) /**< MXC_F_PCM_RX_CH7_EN */

/**@} end of group AUDIO_PCM_RX_ENABLES_BYTE_0 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_RX_ENABLES_BYTE_1 AUDIO_PCM_RX_ENABLES_BYTE_1
 * @brief    PCM Rx Channels Enable Byte 1
 * @{
 */

#define MXC_F_PCM_RX_CH8_EN_POS 0 /**< MXC_F_PCM_RX_CH8_EN Position */
#define MXC_F_PCM_RX_CH8_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH8_EN_POS)) /**< MXC_F_PCM_RX_CH8_EN */

#define MXC_F_PCM_RX_CH9_EN_POS 1 /**< MXC_F_PCM_RX_CH9_EN Position */
#define MXC_F_PCM_RX_CH9_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH9_EN_POS)) /**< MXC_F_PCM_RX_CH9_EN */

#define MXC_F_PCM_RX_CH10_EN_POS 2 /**< MXC_F_PCM_RX_CH10_EN Position */
#define MXC_F_PCM_RX_CH10_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH10_EN_POS)) /**< MXC_F_PCM_RX_CH10_EN */

#define MXC_F_PCM_RX_CH11_EN_POS 3 /**< MXC_F_PCM_RX_CH11_EN Position */
#define MXC_F_PCM_RX_CH11_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH11_EN_POS)) /**< MXC_F_PCM_RX_CH11_EN */

#define MXC_F_PCM_RX_CH12_EN_POS 4 /**< MXC_F_PCM_RX_CH12_EN Position */
#define MXC_F_PCM_RX_CH12_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH12_EN_POS)) /**< MXC_F_PCM_RX_CH12_EN */

#define MXC_F_PCM_RX_CH13_EN_POS 5 /**< MXC_F_PCM_RX_CH13_EN Position */
#define MXC_F_PCM_RX_CH13_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH13_EN_POS)) /**< MXC_F_PCM_RX_CH13_EN */

#define MXC_F_PCM_RX_CH14_EN_POS 6 /**< MXC_F_PCM_RX_CH14_EN Position */
#define MXC_F_PCM_RX_CH14_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH14_EN_POS)) /**< MXC_F_PCM_RX_CH14_EN */

#define MXC_F_PCM_RX_CH15_EN_POS 7 /**< MXC_F_PCM_RX_CH15_EN Position */
#define MXC_F_PCM_RX_CH15_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_RX_CH15_EN_POS)) /**< MXC_F_PCM_RX_CH15_EN */

/**@} end of group AUDIO_PCM_RX_ENABLES_BYTE_1 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_TX_ENABLES_BYTE_0 AUDIO_PCM_TX_ENABLES_BYTE_0
 * @brief    PCM Tx Channels Enable Byte 0
 * @{
 */

#define MXC_F_PCM_TX_CH0_EN_POS 0 /**< MXC_F_PCM_TX_CH0_EN Position */
#define MXC_F_PCM_TX_CH0_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH0_EN_POS)) /**< MXC_F_PCM_TX_CH0_EN */

#define MXC_F_PCM_TX_CH1_EN_POS 1 /**< MXC_F_PCM_TX_CH1_EN Position */
#define MXC_F_PCM_TX_CH1_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH1_EN_POS)) /**< MXC_F_PCM_TX_CH1_EN */

#define MXC_F_PCM_TX_CH2_EN_POS 2 /**< MXC_F_PCM_TX_CH2_EN Position */
#define MXC_F_PCM_TX_CH2_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH2_EN_POS)) /**< MXC_F_PCM_TX_CH2_EN */

#define MXC_F_PCM_TX_CH3_EN_POS 3 /**< MXC_F_PCM_TX_CH3_EN Position */
#define MXC_F_PCM_TX_CH3_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH3_EN_POS)) /**< MXC_F_PCM_TX_CH3_EN */

#define MXC_F_PCM_TX_CH4_EN_POS 4 /**< MXC_F_PCM_TX_CH4_EN Position */
#define MXC_F_PCM_TX_CH4_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH4_EN_POS)) /**< MXC_F_PCM_TX_CH4_EN */

#define MXC_F_PCM_TX_CH5_EN_POS 5 /**< MXC_F_PCM_TX_CH5_EN Position */
#define MXC_F_PCM_TX_CH5_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH5_EN_POS)) /**< MXC_F_PCM_TX_CH5_EN */

#define MXC_F_PCM_TX_CH6_EN_POS 6 /**< MXC_F_PCM_TX_CH6_EN Position */
#define MXC_F_PCM_TX_CH6_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH6_EN_POS)) /**< MXC_F_PCM_TX_CH6_EN */

#define MXC_F_PCM_TX_CH7_EN_POS 7 /**< MXC_F_PCM_TX_CH7_EN Position */
#define MXC_F_PCM_TX_CH7_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH7_EN_POS)) /**< MXC_F_PCM_TX_CH7_EN */

/**@} end of group AUDIO_PCM_TX_ENABLES_BYTE_0 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_TX_ENABLES_BYTE_1 AUDIO_PCM_TX_ENABLES_BYTE_1
 * @brief    PCM Tx Channels Enable Byte 1
 * @{
 */

#define MXC_F_PCM_TX_CH8_EN_POS 0 /**< MXC_F_PCM_TX_CH8_EN Position */
#define MXC_F_PCM_TX_CH8_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH8_EN_POS)) /**< MXC_F_PCM_TX_CH8_EN */

#define MXC_F_PCM_TX_CH9_EN_POS 1 /**< MXC_F_PCM_TX_CH9_EN Position */
#define MXC_F_PCM_TX_CH9_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH9_EN_POS)) /**< MXC_F_PCM_TX_CH9_EN */

#define MXC_F_PCM_TX_CH10_EN_POS 2 /**< MXC_F_PCM_TX_CH10_EN Position */
#define MXC_F_PCM_TX_CH10_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH10_EN_POS)) /**< MXC_F_PCM_TX_CH10_EN */

#define MXC_F_PCM_TX_CH11_EN_POS 3 /**< MXC_F_PCM_TX_CH11_EN Position */
#define MXC_F_PCM_TX_CH11_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH11_EN_POS)) /**< MXC_F_PCM_TX_CH11_EN */

#define MXC_F_PCM_TX_CH12_EN_POS 4 /**< MXC_F_PCM_TX_CH12_EN Position */
#define MXC_F_PCM_TX_CH12_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH12_EN_POS)) /**< MXC_F_PCM_TX_CH12_EN */

#define MXC_F_PCM_TX_CH13_EN_POS 5 /**< MXC_F_PCM_TX_CH13_EN Position */
#define MXC_F_PCM_TX_CH13_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH13_EN_POS)) /**< MXC_F_PCM_TX_CH13_EN */

#define MXC_F_PCM_TX_CH14_EN_POS 6 /**< MXC_F_PCM_TX_CH14_EN Position */
#define MXC_F_PCM_TX_CH14_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH14_EN_POS)) /**< MXC_F_PCM_TX_CH14_EN */

#define MXC_F_PCM_TX_CH15_EN_POS 7 /**< MXC_F_PCM_TX_CH15_EN Position */
#define MXC_F_PCM_TX_CH15_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_TX_CH15_EN_POS)) /**< MXC_F_PCM_TX_CH15_EN */

/**@} end of group AUDIO_PCM_TX_ENABLES_BYTE_1 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_0 AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_0
 * @brief    Dataport 1 Slot Mapping Registers Channel 0
 * @{
 */

#define MXC_F_PCM_DPORT1_SLOT_MAP_CH0_POS 0 /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH0 Position */
#define MXC_F_PCM_DPORT1_SLOT_MAP_CH0 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT1_SLOT_MAP_CH0_POS)) /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH0 */

#define MXC_F_PCM_DPORT1_CH0_EN_POS 7 /**< MXC_F_PCM_DPORT1_CH0_EN Position */
#define MXC_F_PCM_DPORT1_CH0_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT1_CH0_EN_POS)) /**< MXC_F_PCM_DPORT1_CH0_EN */

/**@} end of group AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_0 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_1 AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_1
 * @brief    Dataport 1 Slot Mapping Registers Channel 1
 * @{
 */

#define MXC_F_PCM_DPORT1_SLOT_MAP_CH1_POS 0 /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH1 Position */
#define MXC_F_PCM_DPORT1_SLOT_MAP_CH1 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT1_SLOT_MAP_CH1_POS)) /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH1 */

#define MXC_F_PCM_DPORT1_CH1_EN_POS 7 /**< MXC_F_PCM_DPORT1_CH1_EN Position */
#define MXC_F_PCM_DPORT1_CH1_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT1_CH1_EN_POS)) /**< MXC_F_PCM_DPORT1_CH1_EN */

/**@} end of group AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_1 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_2 AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_2
 * @brief    Dataport 1 Slot Mapping Registers Channel 2
 * @{
 */

#define MXC_F_PCM_DPORT1_SLOT_MAP_CH2_POS 0 /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH2 Position */
#define MXC_F_PCM_DPORT1_SLOT_MAP_CH2 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT1_SLOT_MAP_CH2_POS)) /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH2 */

#define MXC_F_PCM_DPORT1_CH2_EN_POS 7 /**< MXC_F_PCM_DPORT1_CH2_EN Position */
#define MXC_F_PCM_DPORT1_CH2_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT1_CH2_EN_POS)) /**< MXC_F_PCM_DPORT1_CH2_EN */

/**@} end of group AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_2 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_3 AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_3
 * @brief    Dataport 1 Slot Mapping Registers Channel 3
 * @{
 */

#define MXC_F_PCM_DPORT1_SLOT_MAP_CH3_POS 0 /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH3 Position */
#define MXC_F_PCM_DPORT1_SLOT_MAP_CH3 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT1_SLOT_MAP_CH3_POS)) /**< MXC_F_PCM_DPORT1_SLOT_MAP_CH3 */

#define MXC_F_PCM_DPORT1_CH3_EN_POS 7 /**< MXC_F_PCM_DPORT1_CH3_EN Position */
#define MXC_F_PCM_DPORT1_CH3_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT1_CH3_EN_POS)) /**< MXC_F_PCM_DPORT1_CH3_EN */

/**@} end of group AUDIO_PCM_DATAPORT_1_SLOT_MAPPING_CHANNEL_3 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_0 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_0
 * @brief    Dataport 1 Slot Mapping Registers Channel 0
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH0_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH0 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH0 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH0_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH0 */

#define MXC_F_PCM_DPORT2_CH0_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH0_EN Position */
#define MXC_F_PCM_DPORT2_CH0_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH0_EN_POS)) /**< MXC_F_PCM_DPORT2_CH0_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_0 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_1 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_1
 * @brief    Dataport 1 Slot Mapping Registers Channel 1
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH1_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH1 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH1 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH1_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH1 */

#define MXC_F_PCM_DPORT2_CH1_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH1_EN Position */
#define MXC_F_PCM_DPORT2_CH1_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH1_EN_POS)) /**< MXC_F_PCM_DPORT2_CH1_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_1 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_2 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_2
 * @brief    Dataport 1 Slot Mapping Registers Channel 2
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH2_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH2 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH2 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH2_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH2 */

#define MXC_F_PCM_DPORT2_CH2_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH2_EN Position */
#define MXC_F_PCM_DPORT2_CH2_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH2_EN_POS)) /**< MXC_F_PCM_DPORT2_CH2_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_2 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_3 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_3
 * @brief    Dataport 1 Slot Mapping Registers Channel 3
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH3_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH3 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH3 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH3_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH3 */

#define MXC_F_PCM_DPORT2_CH3_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH3_EN Position */
#define MXC_F_PCM_DPORT2_CH3_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH3_EN_POS)) /**< MXC_F_PCM_DPORT2_CH3_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_3 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_4 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_4
 * @brief    Dataport 1 Slot Mapping Registers Channel 4
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH4_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH4 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH4 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH4_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH4 */

#define MXC_F_PCM_DPORT2_CH4_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH4_EN Position */
#define MXC_F_PCM_DPORT2_CH4_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH4_EN_POS)) /**< MXC_F_PCM_DPORT2_CH4_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_4 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_5 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_5
 * @brief    Dataport 1 Slot Mapping Registers Channel 5
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH5_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH5 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH5 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH5_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH5 */

#define MXC_F_PCM_DPORT2_CH5_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH5_EN Position */
#define MXC_F_PCM_DPORT2_CH5_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH5_EN_POS)) /**< MXC_F_PCM_DPORT2_CH5_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_5 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_6 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_6
 * @brief    Dataport 1 Slot Mapping Registers Channel 6
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH6_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH6 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH6 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH6_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH6 */

#define MXC_F_PCM_DPORT2_CH6_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH6_EN Position */
#define MXC_F_PCM_DPORT2_CH6_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH6_EN_POS)) /**< MXC_F_PCM_DPORT2_CH6_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_6 */

/**
 * @ingroup  audio_registers
 * @defgroup AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_7 AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_7
 * @brief    Dataport 1 Slot Mapping Registers Channel 7
 * @{
 */

#define MXC_F_PCM_DPORT2_SLOT_MAP_CH7_POS 0 /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH7 Position */
#define MXC_F_PCM_DPORT2_SLOT_MAP_CH7 \
    ((uint32_t)(0x000UL << MXC_F_PCM_DPORT2_SLOT_MAP_CH7_POS)) /**< MXC_F_PCM_DPORT2_SLOT_MAP_CH7 */

#define MXC_F_PCM_DPORT2_CH7_EN_POS 7 /**< MXC_F_PCM_DPORT2_CH7_EN Position */
#define MXC_F_PCM_DPORT2_CH7_EN \
    ((uint32_t)(0x01UL << MXC_F_PCM_DPORT2_CH7_EN_POS)) /**< MXC_F_PCM_DPORT2_CH7_EN */

/**@} end of group AUDIO_PCM_DATAPORT_2_SLOT_MAPPING_CHANNEL_7 */

#ifdef __cplusplus
}
#endif

#endif // EXAMPLES_MAX32665_AUDIO_PLAYBACK_AUDIO_REGS_H_

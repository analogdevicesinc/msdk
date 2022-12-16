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
 * @file    pal_bb_cfg.h
 * @brief   PAL_BB configuration parameters
 */

#ifndef MAX32665_INCLUDE_PAL_BB_CFG_H_
#define MAX32665_INCLUDE_PAL_BB_CFG_H_

/*******************************************************************************
  Includes
*******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
  Macros
*******************************************************************************/
#ifndef BB_CLK_RATE_HZ
#define BB_CLK_RATE_HZ 1000000
#endif

#if (BB_CLK_RATE_HZ != 1000000)
#error "BB_CLK_RATE_HZ must be 1000000"
#endif

#define DBB_CLK_RATE_HZ 32000000
#define DL_CLK_RATE_HZ 8000000
#define DL_HS_CLK_RATE_HZ 16000000

#define DL_CLK_MULT 1
#define DL_CLK_DIV (DBB_CLK_RATE_HZ / DL_CLK_RATE_HZ)

#define DL_HS_CLK_MULT 1
#define DL_HS_CLK_DIV (DBB_CLK_RATE_HZ / DL_HS_CLK_RATE_HZ)

#define BB_CLK_MULT 1
#define BB_CLK_DIV (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ)

/* BB LED definitions */
#ifndef BB_LED_ENA
#define BB_LED_ENA 1
#endif

#if (BB_LED_ENA == 1)
#define BB_LED_ON(x) PalLedOn(x)
#define BB_LED_OFF(x) PalLedOff(x)
#else
#define BB_LED_ON(x)
#define BB_LED_OFF(x)
#endif

/* Distinguish the BB LEDs with the MSB */
/* TODO: Use int instead of bits */
#define BB_LED_PIN_TX (0x80 | 0x1)
#define BB_LED_PIN_RX (0x80 | 0x2)
#define BB_LED_PIN_RX_OK (0x80 | 0x4)
#define BB_LED_PIN_RX_CRC (0x80 | 0x8)
#define BB_LED_PIN_RX_TO (0x80 | 0x10)
#define BB_LED_PIN_ISR (0x80 | 0x20)

#define TIMESTAMP_WRAP_DIFF 1000000

#define BB_DEFAULT_TX_POWER 0

#define ENABLED 0x01
#define DISABLED 0x00

#define PROT_DEFAULT_CHANNEL 37

/* general settings like standard and modulation type */

/*
 * please notice that the RX ad clock is ALWAYS 8 MHz regardless the mode *
 */

/* number of RX clocks in one symbol in 1 MBit/sec mode */
#define CLK_PER_SAMPLE_1MBIT 8
/* number of RX clocks in one symbol in 1 MBit/sec mode */
#define CLK_PER_SAMPLE_2MBIT 16

#define CLK_PER_SAMPLE CLK_PER_SAMPLE_1MBIT

#define AFE_SELECT_SWAP_IQ ENABLED
#define AFE_SELECT_SWAP_AM_FM DISABLED

#define AFE_I_OFFSET 0xFF
#define AFE_Q_OFFSET 0xFE

#define START_GAIN 4 /* control the gain in which the agc starts */
#define MAXIMUM_GAIN 4
#define MINIMUM_GAIN 0

#define GAIN_STEPS (((MAXIMUM_GAIN - MINIMUM_GAIN) + 1))

#define RUN_AGC_OFFSET_CORRECTION ENABLED /* IQ offset correction routine */

#define AGC_GAIN4 0x9F
#define AGC_GAIN3 0x6B
#define AGC_GAIN2 0x72
#define AGC_GAIN1 0x90
#define AGC_GAIN0 0x40

#define AGC_GAIN_HIGHSPEED_4 0x9F
#define AGC_GAIN_HIGHSPEED_3 0x6B
#define AGC_GAIN_HIGHSPEED_2 0xB2
#define AGC_GAIN_HIGHSPEED_1 0x71
#define AGC_GAIN_HIGHSPEED_0 0x13

#define AGC_GAIN_CODED_2 ((0x9A))
#define AGC_GAIN_CODED_1 ((0x68))
#define AGC_GAIN_CODED_0 ((0x18))

#define AGC_FINE(x) (((((x)&0xF0) >> 4) * 3))
#define AGC_COARSE(x) \
    ((((((x)&0x08) >> 3) + (((x)&0x04) >> 2) + (((x)&0x02) >> 1) + (((x)&0x01))) * 12))

/* gains RSSI correction = fine gain + coarse gain + correction factor */
#define AGC_RSSI_GAINS0 0x05 /*-10dBm */
#define AGC_RSSI_GAINS1 0x14 /*-30dBm */
#define AGC_RSSI_GAINS2 0x28 /*-50dBm */
#define AGC_RSSI_GAINS3 0x3C /*-70dBm */
#define AGC_RSSI_GAINS4 0x4D /*-90dBm */

#define AGC_RSSI_GAINS_HIGHSPEED_0 0x0D /*-10dBm */
#define AGC_RSSI_GAINS_HIGHSPEED_1 0x1F /*-30dBm */
#define AGC_RSSI_GAINS_HIGHSPEED_2 0x31 /*-50dBm */
#define AGC_RSSI_GAINS_HIGHSPEED_3 0x3C /*-70dBm */
#define AGC_RSSI_GAINS_HIGHSPEED_4 0x4D /*-90dBm */

#define AGC_RSSI_GAINS_CODED_0 \
    (((AGC_FINE(AGC_GAIN_CODED_0) + AGC_COARSE(AGC_GAIN_CODED_0)) << 8) + 0x0200) /* add 2 dBm  */
#define AGC_RSSI_GAINS_CODED_1 \
    (((AGC_FINE(AGC_GAIN_CODED_1) + AGC_COARSE(AGC_GAIN_CODED_1)) << 8) + 0x0200) /* add 2 dBm  */
#define AGC_RSSI_GAINS_CODED_2 \
    (((AGC_FINE(AGC_GAIN_CODED_2) + AGC_COARSE(AGC_GAIN_CODED_2)) << 8) + 0x0200) /* add 2 dBm  */

#define AGC_RSSI_CORRECTION_FACTOR ((0xD4)) /* correction 0dBm */

#define RECEIVEDEBUG 1

#define AGC_OFFSET_CORRECTION_CHANNEL \
    19 /* single channel for offset calibration, center channel chosen */

#define AGC_OFFSET_CORRECTION_TIMED 1

#if START_GAIN == 4
#define AFE_START_GAIN AGC_GAIN4
#else
#define AFE_START_GAIN AGC_GAIN0
#endif

#define DIBS_AGC_ENC5_GAIN_ADDR 0x8A
#define AFE_I_OFFSET_ADDR 0xA8
#define AFE_Q_OFFSET_ADDR 0xA9

#define ZIGBEE (15.4)1 /*  ZIGBEE (15.4),              */
#define BAN (15.6)2 /*  BAN (15.6), not available any more */
#define BLUETOOTH_LOW_ENERGY 3 /*  BLUETOOTH_LOW_ENERGY (BTLE) */

#define RX_GENERAL_STANDARD BLUETOOTH_LOW_ENERGY

/* Enabled for Coded PHY */
#define RX_GENERAL_CONTROL_SYNC_AFTER_SIG_DETECT DISABLED
#define RX_GENERAL_CONTROL_START_CCA DISABLED
#define RX_GENERAL_CONTROL_CONT_SYNC DISABLED

#define AGC_READY_TMR 8
#define AGC_READY_TMR_2MBIT 27 /* 31 is highest value */

/* was 26, AGC may switch until 26/8  us after signal detect */
#define RX_GENERAL_AGC_FREEZE_TIMER 255
/* AGC may switch until 85/8  us after signal detect 2MBIT mode  */
#define RX_GENERAL_AGC_FREEZE_TIMER_2MBIT 85
/* AGC may switch until 255/8 us after signal detect */
#define RX_GENERAL_AGC_FREEZE_TIMER_CODED 255

#define RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_1MBIT 16 /* rx clocks */
#define RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_2MBIT 16 /* rx clocks */
#define RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER_CODED 16

// #define RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER       64         //rx clocks
#define RX_PHY_GENERAL_SIG_DETECT_RST_TIME 128 /* was 64, rx clocks */

/*rx clocks (time in preample) */
#define RX_PHY_GENERAL_SIG_DETECT_RST_TIME_CODED (80 * 8)

/* time to block the AGC from responding to the DDC filter  */
#define RX_PHY_DDC_RST_TMR 12
/* time to block the AGC from responding to the DDC filter  */
#define RX_PHY_DDC_RST_TMR_2MBIT 18

/* time the DDC stay valid; a signal detect stops this time */
#define RX_PHY_GENERAL_DDC_RST_TIMER 0

/* time the DDC stay valid; a signal detect stops this time */
#define RX_PHY_GENERAL_DDC_RST_TIMER_2MBIT ((80 * 16))

/* time the DDC stay valid; a signal detect stops this time */
#define RX_PHY_GENERAL_DDC_RST_TIMER_CODED ((80 + 100) * 8)

/* low pass FIR filter settings after adc */
#define RX_PHY_FILTERS_COEFFICIENTS_0 190
#define RX_PHY_FILTERS_COEFFICIENTS_1 179
#define RX_PHY_FILTERS_COEFFICIENTS_2 150
#define RX_PHY_FILTERS_COEFFICIENTS_3 109
#define RX_PHY_FILTERS_COEFFICIENTS_4 63
#define RX_PHY_FILTERS_COEFFICIENTS_5 21
#define RX_PHY_FILTERS_COEFFICIENTS_6 (-10)
#define RX_PHY_FILTERS_COEFFICIENTS_7 (-28)
#define RX_PHY_FILTERS_COEFFICIENTS_8 (-34)
#define RX_PHY_FILTERS_COEFFICIENTS_9 (-30)
#define RX_PHY_FILTERS_COEFFICIENTS_10 (-22)
#define RX_PHY_FILTERS_COEFFICIENTS_11 (-13)
#define RX_PHY_FILTERS_COEFFICIENTS_12 (-5)
#define RX_PHY_FILTERS_COEFFICIENTS_13 (-1)
#define RX_PHY_FILTERS_COEFFICIENTS_14 1
#define RX_PHY_FILTERS_COEFFICIENTS_15 1

#define RX_2MPHY_FILTERS_COEFFICIENTS_0 179
#define RX_2MPHY_FILTERS_COEFFICIENTS_1 171
#define RX_2MPHY_FILTERS_COEFFICIENTS_2 149
#define RX_2MPHY_FILTERS_COEFFICIENTS_3 117
#define RX_2MPHY_FILTERS_COEFFICIENTS_4 81
#define RX_2MPHY_FILTERS_COEFFICIENTS_5 48
#define RX_2MPHY_FILTERS_COEFFICIENTS_6 20
#define RX_2MPHY_FILTERS_COEFFICIENTS_7 3
#define RX_2MPHY_FILTERS_COEFFICIENTS_8 (-6)
#define RX_2MPHY_FILTERS_COEFFICIENTS_9 (-8)
#define RX_2MPHY_FILTERS_COEFFICIENTS_10 (-7)
#define RX_2MPHY_FILTERS_COEFFICIENTS_11 (-3)
#define RX_2MPHY_FILTERS_COEFFICIENTS_12 (-2)
#define RX_2MPHY_FILTERS_COEFFICIENTS_13 1
#define RX_2MPHY_FILTERS_COEFFICIENTS_14 0
#define RX_2MPHY_FILTERS_COEFFICIENTS_15 0

#define RX_PHY_FILTERS_SKIP_MUX_SEL 1
#define RX_PHY_FILTERS_SCALING_FACTOR 6

/* Signal strength settings */
#define RX_PHY_RSSI_SETTINGS 0xc367
#define RX_PHY_RSSI_ED_THRESHOLD 15649

/* Signal  detection settings */

#define SIGNALDETECT_VERSION_V1 0 /* legacy  signal detect */
#define SIGNALDETECT_VERSION_V2 1 /* updated  signal detect */

/* select new signals detect version  */
#define SIGNALDETECT_VERSION SIGNALDETECT_VERSION_V2

/* setting belonging to Signals detect version 1 */
#define RX_PHY_SIGNAL_DET_SETTINGS_DIV_FACTOR 6
#define RX_PHY_SIGNAL_DET_SETTINGS_DELTA_SAMPLES 6 /* was 7 */
#define RX_PHY_SIGNAL_DET_THRESHOLDS_DELTA_AMP 0x30 /* was 0x15 */
#define RX_PHY_SIGNAL_DET_THRESHOLDS_NOISE_FACTOR 0x03 /* was 0x0a */

/* settings belonging to signals detect version 2 */
/* Threshold that the average signal power needs to cross, before a signal is
 * detected */
#define SETTINGS_V2_DIV_FACTOR 6
#define GEAR_V2_DELTA 1 /* Slow IIR gear increment */
/* Slow IIR initial gear value (-2 ~ 0) is always a negative value.
 * 0x3 is -1 2s complement
 */
#define GEAR_V2_START (0x3)

#define GEAR_V2_MAX 3 /* Slow IIR final gear value */
#define GEAR_V2_MAX_CODED 3 /* Slow IIR final gear value */

/* Slow IIR initial gear value (-2 ~ 0) is always a negative
 * value. 0x3 is -1 2s complement
 */
#define GEAR_V2_START_2MBIT (0x3)

#define GEAR_V2_MAX_2MBIT 3 /*Slow IIR final gear value */

/* Slow IIR initial gear value (-2 ~ 0) is always a negative \
 * value. 0x3 is -1 2s complement
 */
#define GEAR_V2_START_CODED (0x3)

/* was 64, Slow IIR gear update step (input samples) per gear step; \
 * base band clock is 8 MHz
 */
#define GEAR_V2_STEP_1MBIT 16

/* Slow IIR gear update step (input samples) per gear step; base \
 * band clock is 16 MHz
 */
#define GEAR_V2_STEP_2MBIT 32

/* Slow IIR gear update step (input samples) per gear step; base \
 * band clock is 8 MHz
 */
#define GEAR_V2_STEP_CODED 32

#define GEAR_V2_STEP GEAR_V2_STEP_1MBIT

/* is 2.0      Threshold factor for scaling the threshold up (2 \
 * integer and 3 fractional bits)
 */
#define THRESHOLDS_V2_NOISE_FACTOR 0x10

/* is 2.0      Threshold factor for scaling the threshold up (2 \
 * integer and 3 fractional bits)
 */
#define THRESHOLDS_V2_NOISE_FACTOR_2MBIT 0x10

/* was 0x18 is 3.0    Threshold factor for scaling the threshold up \
 * (2 integer and 3 fractional bits)
 */
#define THRESHOLDS_V2_NOISE_FACTOR_CODED 0x10

/*
 *  In case we do not start in the highst again mode,
 *  the signal detect must get so more time to learn the new noise level
 */
#if START_GAIN >= 3

#define AGC_INIT_TIME_FOR_STARTUP (0)
#define AGC_INIT_TIME_FOR_STARTUP_2MBIT (0)
#else

#define AGC_AFE_SWITCH_TIME_HIGHER_GAIN (15)
#define AGC_INIT_TIME_FOR_STARTUP \
    ((RX_PHY_GENERAL_SIG_DETECT_RST_TIME / CLK_PER_SAMPLE_1MBIT) + AGC_AFE_SWITCH_TIME_HIGHER_GAIN)

#define AGC_INIT_TIME_FOR_STARTUP_2MBIT \
    ((RX_PHY_GENERAL_SIG_DETECT_RST_TIME / CLK_PER_SAMPLE_2MBIT) + AGC_AFE_SWITCH_TIME_HIGHER_GAIN)

#endif

#define SIGNAL_DETECT_LEARNING_DURATION_1MBIT                                        \
    ((((((GEAR_V2_MAX - GEAR_V2_START) + 1) / GEAR_V2_DELTA) * GEAR_V2_STEP_1MBIT) / \
      CLK_PER_SAMPLE_1MBIT) +                                                        \
     AGC_INIT_TIME_FOR_STARTUP) /* in usec */

#define SIGNAL_DETECT_LEARNING_DURATION_2MBIT                                                    \
    ((((((GEAR_V2_MAX_2MBIT - GEAR_V2_START_2MBIT) + 1) / GEAR_V2_DELTA) * GEAR_V2_STEP_2MBIT) / \
      CLK_PER_SAMPLE_2MBIT) +                                                                    \
     AGC_INIT_TIME_FOR_STARTUP_2MBIT) /* in usec */

#define SIGNAL_DETECT_LEARNING_DURATION SIGNAL_DETECT_LEARNING_DURATION_1MBIT

/* in usec ; time needed for filling the correlator in coded mode */
#define SIGNAL_DETECT_CORRELATION_DELAY_CODED 16

#define SIGNAL_DETECT_LEARNING_DURATION_CODED                                                     \
    (((((((GEAR_V2_MAX_CODED - GEAR_V2_START_CODED) + 1) / GEAR_V2_DELTA) * GEAR_V2_STEP_CODED) / \
       CLK_PER_SAMPLE_1MBIT) +                                                                    \
      SIGNAL_DETECT_CORRELATION_DELAY_CODED + AGC_INIT_TIME_FOR_STARTUP)) /* in usec */

/*
 * time from energy detect to SDF detect
 */

/* 100 usec time out specify in 8 Mhz clk's */
#define RX_PHY_SIGNAL_DET2_TIMEOUT (100 * 8)
/* 400 usec time out specify in 8 Mhz clk's */
#define RX_PHY_SIGNAL_DET2_TIMEOUT_CODED (400 * 8)

/* SFD detect settings */
#define RX_PHY_FRAME_SYNC_PD 8

#define RX_PHY_FRAME_SYNC_SFD_0 0x0000
#define RX_PHY_FRAME_SYNC_SFD_1 0xAA00
#define RX_PHY_FRAME_SYNC_SFD_2 0xBED6
#define RX_PHY_FRAME_SYNC_SFD_3 0x8E89

#define RX_PHY_FRAME_SYNC_CORR_LENGTH \
    32 /* fixed for 1MBit and 2 Mbit modes \
        */
#define RX_PHY_FRAME_SYNC_THRESH_FACTOR 29 /* Changed for AFE optimization */

#define RX_PHY_FRAME_SYNC_LR_CORR_LENGTH 256 /* fixed for these Coded modes */

/* max is equal to RX_PHY_FRAME_SYNC_LR_CORR_LENGTH-1 */
#define RX_PHY_FRAME_SYNC_LR_THRESH_FACTOR_CODED_S2 210

/* max is equal to RX_PHY_FRAME_SYNC_LR_CORR_LENGTH-1 */
#define RX_PHY_FRAME_SYNC_LR_THRESH_FACTOR_CODED_S8 128

#define RX_PHY_FRAME_SYNC_HARD_DETECT 0x01

#define RX_PHY_FRAME_SYNC_SETTINGS_MIN_CORR 70 /* only used in soft detect */

/* 255 is pi;  128 ~ 0.5 pi (90 degree) */
#define RX_PHY_TIMING_SYNC_CFG_LIMIT 128

/* 255 is pi;  128 ~ 0.5 pi (90 degree) */
#define RX_PHY_TIMING_SYNC_CFG_LIMIT_CODED 128
/* inverse loop gain of algorithm */
#define RX_PHY_TIMING_SYNC_CFG_BITSHIFT 6

/* old timing synchronization/ Gardner  settings */
#define RX_PHY_TIM_SYNC_TE_THRESH 60
#define RX_PHY_TIM_SYNC_TE_THRESH_SCALING 0x1C2

#define RX_PHY_TIM_SYNC_INIT_SEL_CNT_MAX 0x08
#define RX_PHY_TIM_SYNC_INIT_MIN_NUM_CHIP 0x08

#define RX_PHY_TIM_SYNC_TE_CNT_THRESH_VALUE 0x04
#define RX_PHY_TIM_SYNC_TE_CNT_THRESH_INCR_VALUE 0x00

#define RX_PHY_TIM_SYNC_TE_CNT_INCR_VALUE 0x00
#define RX_PHY_TIM_SYNC_TE_CNT_DECR_VALUE 0x00

#define RX_PHY_TIM_SYNC_TE_FIFO_MAX_PTR 3

/* RX cfo filter settings */
#define RX_PHY_CFO_FILTER_COEFFICIENTS_0 193
#define RX_PHY_CFO_FILTER_COEFFICIENTS_1 172
#define RX_PHY_CFO_FILTER_COEFFICIENTS_2 123
#define RX_PHY_CFO_FILTER_COEFFICIENTS_3 71
#define RX_PHY_CFO_FILTER_COEFFICIENTS_4 32
#define RX_PHY_CFO_FILTER_COEFFICIENTS_5 12
#define RX_PHY_CFO_FILTER_COEFFICIENTS_6 3
#define RX_PHY_CFO_FILTER_COEFFICIENTS_7 1
#define RX_PHY_CFO_FILTER_COEFFICIENTS_8 0

#define RX_PHY_CFO_FILTER_SCALING 10
#define RX_PHY_CFO_PA_PRECNT_NUM 41
#define RX_PHY_CFO_PH_ACCUM_PART_NUM 22
#define RX_PHY_CFO_ED_THRESH_FACTOR 6
#define RX_PHY_CFO_ED_SLOW_FACTOR 61
#define RX_PHY_CFO_ED_FAST_FACTOR 3
#define RX_PHY_CFO_PA_MAX_LIMIT 7721

/* CRC types PHR and PLD  */
#define CRC_BYPASSED 0
#define CRC_4 1
#define CRC_16 2
#define CRC_24 3

/* phr and pld crc settings */
#define RX_DL_CRC_MODE_PHR CRC_24
#define RX_DL_CRC_MODE_PLD CRC_24

#define RX_DL_CRC_INIT_PHR 0x555555
#define RX_DL_CRC_INIT_PLD 0x555555

/* BAN radio settings */
#define RX_DL_BCH_PHR_ENABLE 0
#define RX_DL_BCH_PLD_ENABLE 0
#define RX_DL_BCH_BIT_ERR_NR 0

#define RX_DL_BAN_SETTINGS_PHR_SCRMBL_SD 0
#define RX_DL_BAN_SETTINGS_PHR_SPRD_FACT 0
#define RX_DL_BAN_SETTINGS_MDR_LENGTH 0
#define RX_DL_BAN_SETTINGS_PROP_PLD_MOD 0
#define RX_DL_BAN_SETTINGS_PROP_PLD_SPRD_FACT 0

/* ble whitening */
#define RX_DL_BTLE_SETTINGS_CHAN_NR PROT_DEFAULT_CHANNEL
#define RX_DL_BTLE_SETTINGS_BYPASS DISABLED

/*
 * Header modulation phr:
 */
#define HEADER_MODULATION_OQPSK 0
#define HEADER_MODULATION_DBPSK 1
#define HEADER_MODULATION_DQPSK 2
#define HEADER_MODULATION_D8PSK 3
#define HEADER_MODULATION_GMSK 4
#define HEADER_MODULATION_GFSK 5

#define RX_DL_PHR_MODULATION HEADER_MODULATION_GFSK

/*
 *  Gear shifted High Pass Filter (HPF) to compensate the DC offset in IQ
 * signals
 *
 *
 *  The lower the filter value (alpha and beta ) the more aggressive (faster)
 * the filter is. max value is 0x3FFF 1     = 0x4000 1/2   = 0x2000 1/16  =
 * 0x0400
 *
 *
 *
 *
 */

#define RX_PHY_AGC2_HPF_ALPHA_START 0x3800 /* start value filter corner */
#define RX_PHY_AGC2_HPF_ALPHA_STOP 0x3FB0 /* 0x3F80 */
#define RX_PHY_AGC2_HPF_ALPHA_STEP 0x200 /* 0x0200 */
#define RX_PHY_AGC2_HPF_BETA 0x3FFE /* tracking value */

/*#define  RX_DDC_ALPHA_START0   0x3FF0 */
#define RX_DDC_ALPHA_START0 0x3FFF /* was 0x3F00 */
#define RX_DDC_ALPHA_START1 0x3FFF
#define RX_DDC_ALPHA_START2 0x3FFF
#define RX_DDC_ALPHA_START3 0x3FFF /* was 0x2800 */
#define RX_DDC_ALPHA_START4 0x3FF0
#define RX_DDC_ALPHA_START0_CODED 0x3FF0
#define RX_DDC_ALPHA_START1_CODED 0x3FF0
#define RX_DDC_ALPHA_START2_CODED 0x3FF0
#define RX_DDC_ALPHA_START3_CODED 0x3FF0

#define RX_DDC_ALPHA_STOP0 0x3FFF
#define RX_DDC_ALPHA_STOP1 0x3FFF
#define RX_DDC_ALPHA_STOP2 0x3FFF
#define RX_DDC_ALPHA_STOP3 0x3FFF
#define RX_DDC_ALPHA_STOP4 0x3FF0
#define RX_DDC_ALPHA_STOP0_CODED 0x3FF0
#define RX_DDC_ALPHA_STOP1_CODED 0x3FF0
#define RX_DDC_ALPHA_STOP2_CODED 0x3FF0
#define RX_DDC_ALPHA_STOP3_CODED 0x3FF0

#define RX_DDC_ALPHA_STEP0 0x100
#define RX_DDC_ALPHA_STEP1 0x080
#define RX_DDC_ALPHA_STEP2 0x080
#define RX_DDC_ALPHA_STEP3 0x080
#define RX_DDC_ALPHA_STEP4 0x100

#define RX_DDC_BETA0 0x3FFF
#define RX_DDC_BETA1 0x3FFF
#define RX_DDC_BETA2 0x3FFF
#define RX_DDC_BETA3 0x3FFF
#define RX_DDC_BETA4 0x3FF8

#define RX_PHY_DDC_OFFS_I0 0
#define RX_PHY_DDC_OFFS_Q0 0
#define RX_PHY_DDC_OFFS_I1 0
#define RX_PHY_DDC_OFFS_Q1 0
#define RX_PHY_DDC_OFFS_I2 0
#define RX_PHY_DDC_OFFS_Q2 0
#define RX_PHY_DDC_OFFS_I3 0
#define RX_PHY_DDC_OFFS_Q3 0
#define RX_PHY_DDC_OFFS_I4 0
#define RX_PHY_DDC_OFFS_Q4 0

/*
 *
 * Bypass offset values to be written to the AFE
 *
 *
 */
#define DIBS_AGC5_ENC_OFFS_I_BYPASS4 0
#define DIBS_AGC5_ENC_OFFS_Q_BYPASS4 0
#define DIBS_AGC5_ENC_OFFS_I_BYPASS3 0
#define DIBS_AGC5_ENC_OFFS_Q_BYPASS3 0
#define DIBS_AGC5_ENC_OFFS_I_BYPASS2 0
#define DIBS_AGC5_ENC_OFFS_Q_BYPASS2 0
#define DIBS_AGC5_ENC_OFFS_I_BYPASS1 0
#define DIBS_AGC5_ENC_OFFS_Q_BYPASS1 0
#define DIBS_AGC5_ENC_OFFS_I_BYPASS0 0
#define DIBS_AGC5_ENC_OFFS_Q_BYPASS0 0

/* DDC transparent settings */

#define RX_TRANS_DDC_ALPHA_START0 0x3FFF
#define RX_TRANS_DDC_ALPHA_START1 0x3FFF
#define RX_TRANS_DDC_ALPHA_START2 0x3FFF
#define RX_TRANS_DDC_ALPHA_START3 0x3FFF
#define RX_TRANS_DDC_ALPHA_START4 0x3FFF

#define RX_TRANS_DDC_ALPHA_STOP0 0x3FFF
#define RX_TRANS_DDC_ALPHA_STOP1 0x3FFF
#define RX_TRANS_DDC_ALPHA_STOP2 0x3FFF
#define RX_TRANS_DDC_ALPHA_STOP3 0x3FFF
#define RX_TRANS_DDC_ALPHA_STOP4 0x3FFF

#define RX_TRANS_DDC_ALPHA_STEP0 0x100
#define RX_TRANS_DDC_ALPHA_STEP1 0x100
#define RX_TRANS_DDC_ALPHA_STEP2 0x100
#define RX_TRANS_DDC_ALPHA_STEP3 0x100
#define RX_TRANS_DDC_ALPHA_STEP4 0x100

#define RX_TRANS_DDC_BETA0 0x3FFF
#define RX_TRANS_DDC_BETA1 0x3FFF
#define RX_TRANS_DDC_BETA2 0x3FFF
#define RX_TRANS_DDC_BETA3 0x3FFF
#define RX_TRANS_DDC_BETA4 0x3FFF

#define RX_PHY_TRANS_DDC_RST_TMR 12

#define RX_PHY_TRANS_DDC_OFFS_I0 0
#define RX_PHY_TRANS_DDC_OFFS_Q0 0
#define RX_PHY_TRANS_DDC_OFFS_I1 0
#define RX_PHY_TRANS_DDC_OFFS_Q1 0
#define RX_PHY_TRANS_DDC_OFFS_I2 0
#define RX_PHY_TRANS_DDC_OFFS_Q2 0
#define RX_PHY_TRANS_DDC_OFFS_I3 0
#define RX_PHY_TRANS_DDC_OFFS_Q3 0
#define RX_PHY_TRANS_DDC_OFFS_I4 0
#define RX_PHY_TRANS_DDC_OFFS_Q4 0

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/* AGC 5 settings */

#define RX_PHY_AGC5_GAIN_INIT START_GAIN
#define RX_PHY_AGC5_GAIN_MIN MINIMUM_GAIN /* strongest power signal */
#define RX_PHY_AGC5_GAIN_MAX MAXIMUM_GAIN /* lowest power signal */

#define RX_PHY_AGC5_THRESH4UPPER 640
#define RX_PHY_AGC5_THRESH4LOWER 0
#define RX_PHY_AGC5_THRESH3UPPER 640
#define RX_PHY_AGC5_THRESH3LOWER 0
#define RX_PHY_AGC5_THRESH2UPPER_2MBIT 870
#define RX_PHY_AGC5_THRESH2UPPER 640
#define RX_PHY_AGC5_THRESH2LOWER 0
#define RX_PHY_AGC5_THRESH1UPPER 480
#define RX_PHY_AGC5_THRESH1LOWER 0
#define RX_PHY_AGC5_THRESH0UPPER 512
#define RX_PHY_AGC5_THRESH0LOWER 64

/* AGC steps down if the upper levels is exceeded
 * Gain DOWn Run 2-1-0 (low  power signal to higher power signal)
 * Gain UP Run 0-1-2   (High power signal to lower power signal)
 */
#define AGC_RUNS_DOWN 1

#if AGC_RUNS_DOWN

/* For switching to a lower gain
 * specifies if gain is incremented when samples are
 * larger-or-equal to this threshold (smaller otherwise)
 */
#define PAN2G_RX_PHY_AGC5_ROW_UPPER_LE 1

/* specifies the increment value (signed -> can be
 * used to increase and decrease) 0xF -> -1 2's complement
 */
#define PAN2G_RX_PHY_AGC5_ROW_UPPER_INCR ((0xF))

/*Specifies the number of blocks the threshold has to
 * be reached before the gain is changed.
 * In this case the gain is decreased
 */
#define PAN2G_RX_PHY_AGC5_ROW_UPPER_BLK 1

/* For switching to a higher gain
 * specifies if gain is incremented when samples are
 * larger-or-equal to this threshold (smaller otherwise)
 */
#define PAN2G_RX_PHY_AGC5_ROW_LOWER_LE 0

/* specifies the increment value (signed -> can be
 * used to increase and decrease)
 */
#define PAN2G_RX_PHY_AGC5_ROW_LOWER_INCR ((1))

/* Specifies the number of blocks the threshold has to
 * be reached before the gain is changed.
 *  In this case the gain is  increased
 */
#define PAN2G_RX_PHY_AGC5_ROW_LOWER_BLK (4)

#else

/* For switching to a lower gain
 * specifies if gain is incremented when samples are
 * larger-or-equal to this threshold (smaller otherwise)
 */
#define PAN2G_RX_PHY_AGC5_ROW_UPPER_LE 0

/* specifies the increment value (signed -> can be
 * used to increase and decrease)
 */
#define PAN2G_RX_PHY_AGC5_ROW_UPPER_INCR ((1))

/* Specifies the number of blocks the threshold has to
 * be reached before the gain is changed.
 * In this case the gain is decreased
 */
#define PAN2G_RX_PHY_AGC5_ROW_UPPER_BLK (3)

/* For switching to a higher gain
 * specifies if gain is incremented when samples are
 * larger-or-equal to this threshold (smaller otherwise)
 */
#define PAN2G_RX_PHY_AGC5_ROW_LOWER_LE 1

/* specifies the increment value (signed -> can be
 *      used to increase and decrease)
 */
#define PAN2G_RX_PHY_AGC5_ROW_LOWER_INCR ((-1))

/* Specifies the number of blocks the threshold has to
 * be reached before the gain is changed.
 *  In this case the gain is  increased
 */
#define PAN2G_RX_PHY_AGC5_ROW_LOWER_BLK (12)

#endif

/* Number of samples in a block
 * and also number valid samples before gain is considered stable
 * actual value is programmed value -1 */
#define RX_PHY_AGC5_CNTR_STABLE 6

/* Number of samples needed after a gain change before samples are considered
 * valid */
#define RX_PHY_AGC5_CNTR_VALID 6

/* Number of samples needed after a gain change before samples are considered
 * valid */
#define RX_PHY_AGC5_CNTR_VALID_2MBIT 12

/* Number of samples needed after a gain change before samples are considered
 * valid */
#define RX_PHY_AGC5_CNTR_VALID_CODED 4

#define OFFSET_CAL_RX_PHY_AGC5_CNTR_STABLE 128
#define OFFSET_CAL_RX_PHY_AGC5_CNTR_VALID 255

/*
 * High pass Filter (HPF) in phase domain
 * This filter removes the CFO from the signal
 *
 *
 * There are three HPF filters in parallel:
 *         alpha for no or little CFO
 *         beta for medium CFO
 *         gamma for major CFO
 *
 * A mux at the end evaluates the results of the 3 HPF's
 * and chooses the best results to correction
 *
 *
 */

/*
 * The CFO HPF  filter has it own energy detect
 */

#define RX_PHY_CFO_HPF_ED_FAST_FACT 0x03
#define RX_PHY_CFO_HPF_ED_SLOW_FACT 0x3D
#define RX_PHY_CFO_HPF_ED_THRESHOLD_FACT 0x1C
#define RX_PHY_CFO_HPF_STOP_HIGH_FILT_CORNER 0x40

/* 8 samples per symbol  with  40 bits (4*8 bytes) */
#define RX_PHY_CFO_HPF_ED_TIMEOUT 0x00000140

#define RX_PHY_CFO_HPF_BETA_MUX_EN_STAGE 1
#define RX_PHY_CFO_HPF_BETA_MUX_THRESHOLD (0x12000 * 2) /* 0x12000 ~ 10.5 kHz*/

/*
 *   Filter multiplexer enable stage
 *   RTL starts at 1 ( instead of 0)
 */
#define MULTIPLEXER_DISABLED 0
#define ALPHA0 1 /* filter stage 1 */
#define ALPHA1 2 /* filter stage 2 */
#define ALPHA2 3 /* filter stage 3 */
#define ALPHA3 4 /* filter stage 4 */

#define RX_PHY_CFO_HPF_GAMMA_MUX_EN_STAGE ALPHA3
#define RX_PHY_CFO_HPF_GAMMA_MUX_THRESHOLD (0x12000 * 4)

#define RX_PHY_CFO_HPF_ALPHA_0 0x4000
#define RX_PHY_CFO_HPF_ALPHA_1 0x4000
#define RX_PHY_CFO_HPF_ALPHA_2 0x4000
#define RX_PHY_CFO_HPF_ALPHA_3 0x4000

#define RX_PHY_CFO_HPF_BETA_0 0x4000
#define RX_PHY_CFO_HPF_BETA_1 0x3800
#define RX_PHY_CFO_HPF_BETA_2 0x3f80
#define RX_PHY_CFO_HPF_BETA_3 0x3ff8

#define RX_PHY_CFO_HPF_GAMMA_0 0x4000
#define RX_PHY_CFO_HPF_GAMMA_1 0x3800
#define RX_PHY_CFO_HPF_GAMMA_2 0x3f00
#define RX_PHY_CFO_HPF_GAMMA_3 0x3ff0

/* global needed defines */

/*
 * The afe should needs time to prepare for a receive.
 * This  time consists of pLl locking time + agc noise level learing
 */
#define RX_ENABLE_DELAY                                         \
    ((RX_ENABLE_RFFE_0_DELAY + RX_ENABLE_RFFE_1_DELAY +         \
      (RX_PHY_GENERAL_SIG_DETECT_INIT_TIMER / CLK_PER_SAMPLE) + \
      SIGNAL_DETECT_LEARNING_DURATION)) /* in usec */

#define RX_START_OF_FRAME_TO_SDF_DETECT (45) /* in usec from simulation */
#define RX_START_OF_FRAME_TO_SDF_DETECT_2MBIT (27) /* in usec from simulation */

/* in usec from simulation */
#define RX_START_OF_FRAME_TO_SDF_DETECT_CODED (341)

#define SPI_WRITE_DELAY (1.25) /* usec */

#define RX_PHY_PHSMAGEST_FILTER_COEFF_0 112
#define RX_PHY_PHSMAGEST_FILTER_COEFF_1 102
#define RX_PHY_PHSMAGEST_FILTER_COEFF_2 77
#define RX_PHY_PHSMAGEST_FILTER_COEFF_3 45
#define RX_PHY_PHSMAGEST_FILTER_COEFF_4 17
#define RX_PHY_PHSMAGEST_FILTER_COEFF_5 (-1)
#define RX_PHY_PHSMAGEST_FILTER_COEFF_6 (-16)

#define RX_PHY_PHSMAGEST_SCALING_FACTOR 6

#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_0 102
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_1 94
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_2 70
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_3 41
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_4 15
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_5 (-1)
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_6 (-15)
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_7 0
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_8 0
#define RX_PHY_PHSMAGEST_FILTER_COEFF_CODED_9 0

/*
 *
 *  PROTOCOL SPECIFIC COMMON SETTINGS [BLE]
 *
 */

#define PROT_DEFAULT_CHANNEL 37

#define PROT_CRC_INIT 0x555555 /* CRC seed */
#define PROT_ACC_ADDR 0x8e89bed6 /* Advertisement access address */

#define PROT_HDR_SIZE_BYTES 2 /* size of packet header */
#define PROT_PAYLOAD_MAX_BYTES 255 /* max size of payload */
#define PROT_PDU_MAX_BYTES (PROT_HDR_SIZE_BYTES + PROT_PAYLOAD_MAX_BYTES)

/* TODO: Account for coded PHY TIFS timings */
#define TXSTARTUP_1M_USEC 21
#define TXEARLYIRQ_1M_USEC 45
#define RXSTARTUP_1M_USEC 25
#define RXLATEIRQ_1M_USEC 3

#define TXSTARTUP_2M_USEC 16
#define TXEARLYIRQ_2M_USEC 18
#define RXSTARTUP_2M_USEC 6
#define RXLATEIRQ_2M_USEC 0

/* Used for RFFE disable delay, set to longest delay required */
#define TXEARLYIRQ_USEC TXEARLYIRQ_1M_USEC

/* Time from Preamble to Access Address for different PHY */
#define BB_BLE_PREAM_TO_ACCESS_1M 64
#define BB_BLE_PREAM_TO_ACCESS_2M 39
#define BB_BLE_PREAM_TO_ACCESS_CODED 574

/*
 *
 *  TX PROTOCOL SPECIFIC SETTINGS
 *
 */

#define TX_ENABLE_DBB_DELAY (96) /* in usec */
#define TX_ENABLE_RFFE_0_DELAY (30) /* in usec */
#define TX_ENABLE_RFFE_1_DELAY (20) /* in usec */

/* in usec This value does not affect TRX timing, leave alone */
#define TX_DISABLE_DBB_DELAY (1)
#define TX_DISABLE_RFFE_0_DELAY (2) /* in usec */
#define TX_DISABLE_RFFE_1_DELAY (10) /* in usec */

/*
 *
 *  RX PROTOCOL SPECIFIC SETTINGS
 *
 */

/* RX_MARGIN : defines the time the RX is already receiving before the Tifs time
 * expires rx early turn on to settle things in receiver and not miss begin of
 * received frame)
 */

/* in usec */
#define RX_MARGIN (10 + SIGNAL_DETECT_LEARNING_DURATION)

/* in usec */
#define RX_ENABLE_DBB_DELAY (102 - SIGNAL_DETECT_LEARNING_DURATION)
#define RX_ENABLE_RFFE_0_DELAY (10) /* in usec */
#define RX_ENABLE_RFFE_1_DELAY (28) /* was 28 in usec */

#define RX_DISABLE_DBB_DELAY 1 /* in usec */
#define RX_DISABLE_RFFE_0_DELAY 1 /* in usec */
#define RX_DISABLE_RFFE_1_DELAY 0 /* in usec */

/*
 *
 *  RX TIMEOUT SETTINGS
 *
 */
#define RX_TIMEOUT_DELAY 1000 /* in usec */

#define CFO_MAX_KHZ 200 /* Filter out gross CFO error */
#define CFO_SFD_TIMEOUT_ENABLE 1 /* Set to 0 to disable, other to enable */
#define CFO_SFD_TIMEOUT (350) /* 1/2 of the packet interval,  */
/* with 60 usec for the receiver to turn on */

#define PAL_BB_SETFIELD(reg, mask, setting) ((reg) = ((reg) & ~(mask)) | ((setting) & (mask)))

#ifdef __cplusplus
};
#endif

#endif // MAX32665_INCLUDE_PAL_BB_CFG_H_

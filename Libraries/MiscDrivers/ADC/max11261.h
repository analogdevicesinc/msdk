/**
 * @file        max11261.h
 * @brief       MAX11261 driver API definitions.
 * @details     This header file contains the MAX11261 driver API definitions.
 */

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

#ifndef MAX11261_H_
#define MAX11261_H_

#include <stdint.h>

/**
 * @brief MAX11261 device access function prototype.
 */
typedef int (*max11261_transfer_func_t) (uint8_t *txbuf, uint8_t txsize,
        uint8_t *rxbuf, uint8_t rxsize, uint8_t slave);

/**
 * @brief MAX11261 reset function prototype.
 */
typedef void (*max11261_reset_ctrl_t) (int);

/**
 * @brief MAX11261 microseconds delay function prototype.
 */
typedef void (*max11261_delay_func_t) (uint32_t);

/**
 * @brief MAX11261 RDYB pin status check function.
 */
typedef int (*max11261_ready_func_t) (void);

/**
 * Channel enumeration.
 * There are six ADC channels available in this chip.
 */
typedef enum {
    MAX11261_ADC_CHANNEL_0 = 0,
    MAX11261_ADC_CHANNEL_1,
    MAX11261_ADC_CHANNEL_2,
    MAX11261_ADC_CHANNEL_3,
    MAX11261_ADC_CHANNEL_4,
    MAX11261_ADC_CHANNEL_5,
    MAX11261_ADC_CHANNEL_MAX
} max11261_adc_channel_t;

/**
 * Conversion modes.
 * Sequencer modes 2, 3 and 4 can only run in \a MAX11261_SINGLE_CYCLE mode.
 */
typedef enum {
    MAX11261_LATENT_CONTINUOUS,         /**< Latent continuous single cycle conversion. Valid only in 1 */
    MAX11261_SINGLE_CYCLE,              /**< No-latency single cycle conversion. Valid in 1, 2, 3 and 4 */
    MAX11261_SINGLE_CYCLE_CONTINUOUS,   /**< Continuous no-latency single cycle conversion. Valid only in 1. */
} max11261_conversion_mode_t;

/**
 * Sequencer modes.
 * 2, 3 and 4 are only valid with MAX11261_SINGLE_CYCLE.
 */
typedef enum {
    MAX11261_SEQ_MODE_1,    /**< Single-Channel Conversion with GPO Control and MUX Delays. */
    MAX11261_SEQ_MODE_2,    /**< Multichannel Scan with GPO Control and MUX Delays */
    MAX11261_SEQ_MODE_3,    /**< Scan, with Sequenced GPO Controls */
    MAX11261_SEQ_MODE_4,    /**< Autoscan with GPO Controls (CHMAP) and Interrupt */
    MAX11261_SEQ_MODE_MAX,  /**< Autoscan with GPO Controls (CHMAP) and Interrupt */
} max11261_sequencer_mode_t;

/**
 * Sample rates available for continuous conversion.
 * \a MAX11261_CONT_RATE_16000 is only supported with serial interface speeds
 * equal to or greater than 1MHz.
 */
typedef enum {
    MAX11261_CONT_RATE_1_9 = 0,
    MAX11261_CONT_RATE_3_9,
    MAX11261_CONT_RATE_7_8,
    MAX11261_CONT_RATE_15_6,
    MAX11261_CONT_RATE_31_2,
    MAX11261_CONT_RATE_62_5,
    MAX11261_CONT_RATE_125,
    MAX11261_CONT_RATE_250,
    MAX11261_CONT_RATE_500,
    MAX11261_CONT_RATE_1000,
    MAX11261_CONT_RATE_2000,
    MAX11261_CONT_RATE_4000,
    MAX11261_CONT_RATE_8000,
    MAX11261_CONT_RATE_16000,
} max11261_cont_rate_t;

/**
 * Sample rates available for single cycle conversion.
 */
typedef enum {
    MAX11261_SINGLE_RATE_50 = 0,
    MAX11261_SINGLE_RATE_62_5,
    MAX11261_SINGLE_RATE_100,
    MAX11261_SINGLE_RATE_125,
    MAX11261_SINGLE_RATE_200,
    MAX11261_SINGLE_RATE_250,
    MAX11261_SINGLE_RATE_400,
    MAX11261_SINGLE_RATE_500,
    MAX11261_SINGLE_RATE_800,
    MAX11261_SINGLE_RATE_1000,
    MAX11261_SINGLE_RATE_1600,
    MAX11261_SINGLE_RATE_2000,
    MAX11261_SINGLE_RATE_3200,
    MAX11261_SINGLE_RATE_4000,
    MAX11261_SINGLE_RATE_6400,
    MAX11261_SINGLE_RATE_12800,
    MAX11261_SINGLE_RATE_MAX,
} max11261_single_rate_t;

typedef enum {
    MAX11261_PDSTATE_CONVERSION = 0,
    MAX11261_PDSTATE_SLEEP,
    MAX11261_PDSTATE_STANDBY,
    MAX11261_PDSTATE_RESET
} max11261_pd_state_t;

typedef enum {
    MAX11261_SIF_FREQ_100_400 = 0,
    MAX11261_SIF_FREQ_401_1000,
    MAX11261_SIF_FREQ_1001_3000,
    MAX11261_SIF_FREQ_3001_5000,
} max11261_sif_freq_t;

/**
 * Input polarity.
 */
typedef enum {
    MAX11261_POL_UNIPOLAR = 0,  /**< 0 to +VREF */
    MAX11261_POL_BIPOLAR,       /**< -VREF to +VREF */
} max11261_pol_t;

/**
 * Digital format of the bipolar range data.
 */
typedef enum {
    MAX11261_FMT_TWOS_COMPLEMENT = 0,
    MAX11261_FMT_OFFSET_BINARY,
} max11261_fmt_t;

/**
 * PGA gain values.
 */
typedef enum {
    MAX11261_PGA_GAIN_1 = 0,
    MAX11261_PGA_GAIN_2,
    MAX11261_PGA_GAIN_4,
    MAX11261_PGA_GAIN_8,
    MAX11261_PGA_GAIN_16,
    MAX11261_PGA_GAIN_32,
    MAX11261_PGA_GAIN_64,
    MAX11261_PGA_GAIN_128,
    MAX11261_PGA_GAIN_MAX,
} max11261_pga_gain_t;

/**
 * General-purpose outputs.
 * There are six GPOs available in this chip. Each enum corresponds to a
 * bit in GPO_DIR:GPO[5:0].
 */
typedef enum {
    MAX11261_GPO_INVALID = -1,
    MAX11261_GPO_0 = 0,
    MAX11261_GPO_1,
    MAX11261_GPO_2,
    MAX11261_GPO_3,
    MAX11261_GPO_4,
    MAX11261_GPO_5,
    MAX11261_GPO_MAX,
} max11261_gpo_t;

/**
 * ADC conversion result and miscelleanous info.
 */
typedef struct {
    int32_t val; /**< Conversion result */
    uint8_t chn:3; /**< Channel number */
    uint8_t aor:1; /**< Analog input overrange detection */
    uint8_t dor:1; /**< Data overrun detection */
    uint8_t rfu:3;
} max11261_adc_result_t;

/**
 * @brief Initializes MAX11261 platform configuration.
 *
 * These are platform-specific functions and must be provided by the user. The
 * first parameter, @p transferFunc is the I2C transfer function that takes a
 * transmit buffer, receive buffer and slave address and should return 0 if
 * I2C transaction succeeds.
 *
 * @p resetCtrl provides the hardware reset functionality. Passing 1 to this
 * function should assert the reset line and passing 0 should clear it.
 *
 * @p delayFunc should provide delay functionality in microseconds resolution.
 *
 * @param transferFunc I2C transfer function.
 * @param resetCtrl Function that controls the reset line.
 * @param delayFunc Microseconds delay function.
 */
void max11261_adc_platform_init(max11261_transfer_func_t transferFunc,
        max11261_reset_ctrl_t resetCtrl, max11261_delay_func_t delayFunc
);

/**
 * @brief Configures MAX11261 hardware parameters.
 *
 * @param vavdd V_AVDD in millivolts.
 * @param vref  REFP - REFN in millivolts.
 * @param freq Interface frequency.
 * @param slaveAddr I2C slave address of the MAX11261 chip.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_config_init(uint16_t vavdd, uint16_t vref, uint16_t freq,
        uint8_t slaveAddr);

/**
 * @brief Sets the function that checks if data is available by monitoring the
 * RDYB pin.
 *
 * If RDYB pin is not connected and ready function is not set, the STAT
 * register will be polled until RDY bit is set. It is recommended to use the
 * RDYB pin as polling over serial interface takes longer.
 *
 * @param readyFunc Function that monitors the level of the ready pin. Should
 *                  return 1 if ready pin is asserted, 0 otherwise.
 */
void max11261_adc_set_ready_func(max11261_ready_func_t readyFunc);

/**
 * @brief Resets MAX11261 using the reset function provided earlier.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_reset(void);

/**
 * @brief Sets the polarity of the input. Default polarity is
 * \a MAX11261_POL_UNIPOLAR.
 *
 * Unipolar mode is not supported in sequencer mode 4.
 *
 * @param pol Input polarity. See \ref max11261_pol_t.
 *
 * @return Success/Fail, 0 if success, -EINVAL if the polarity is not
 * compatible with the current sequencer mode.
 */
int max11261_adc_set_polarity(max11261_pol_t pol);

/**
 * @brief Sets the format of the conversion results. Default format is
 * \a MAX11261_FMT_OFFSET_BINARY.
 *
 * In unipolar mode, data format is always offset binary. In offset binary
 * format the most negative value is 0x000000, the midscale value is
 * 0x800000 and the most positive value is 0xFFFFFF. In two's complement,
 * the negative full-scale value is 0x800000, the midscale is 0x000000,
 * and the positive full scale is 0x7FFFFF.
 *
 * @param fmt Data format. See \ref max11261_fmt_t.
 *
 * @return Success/Fail, 0 if success, -EINVAL if the format is not compatible
 * with the current polarity selection.
 */
int max11261_adc_set_format(max11261_fmt_t fmt);

/**
 * @brief Set the output data rate used during single cycle conversion.
 *
 * The maximum data rate depends on the I2C interface speed.
 *
 * @param rate Data rate to be used.
 *
 */
void max11261_adc_set_rate_single(max11261_single_rate_t rate);

/**
 * @brief Set the output data rate used during continuous conversion.
 *
 * The maximum data rate depends on the I2C interface speed.
 *
 * @param ctx ADC context structure.
 * @param rate Data rate to be used.
 *
 * @return Success/Fail, 0 if success, -EINVAL if the rate is invalid.
 */
int max11261_adc_set_rate_continuous(max11261_cont_rate_t rate);

/**
 * @brief Sets the ADC channel to be scanned in sequencer mode 1. Default is
 * \a MAX11261_ADC_CHANNEL_0.
 *
 * @param chan Channel to scan.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_set_channel(max11261_adc_channel_t chan);

/**
 * @brief Sets the scan order of ADC channel in sequencer modes 2, 3 and 4.
 *
 * The allowed values range from 1 to 6. Scanning will be disabled for \a chan
 * if \a order is 0. The order numbers must start from 1 and be sequential,
 * i.e. there should not be any missing numbers in the sequence. For example,
 * if there are two channels to be scanned, they must be ordered 1 and 2. 2
 * and 3 will be invalid as number 1 has not been assigned to any channel.
 *
 * @return Success/Fail, 0 if success, -EINVAL if \a chan or \a order is
 * invalid, -EACCES if \a gpo is already mapped to another channel.
 */
int max11261_adc_set_channel_order(max11261_adc_channel_t chan, uint8_t order);

/**
 * @brief Map given general-purpose output to a channel.
 *
 * A GPO can only be mapped to a single channel.
 *
 * @param chan Channel to map the GPO to.
 * @param gpo GPO to map. Pass MAX11261_GPO_INVALID to disable the mapping for
 * channel \a chan.
 *
 * @return Success/Fail, 0 if success, -EINVAL if \a chan or \a gpo is invalid,
 * or \a gpo is already mapped to another channel.
 */
int max11261_adc_set_channel_gpo(max11261_adc_channel_t chan,
        max11261_gpo_t gpo);

/**
 * @brief Sets conversion and sequencer mode to be used. Default values are
 * \a MAX11261_SINGLE_CYCLE and MAX11261_SEQ_MODE_1.
 *
 * @param convMode Conversion mode to use.
 * @param seqMode Sequencer mode to use.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_set_mode(max11261_conversion_mode_t convMode,
        max11261_sequencer_mode_t seqMode);

/**
 * @brief Sets the gain of the PGA. PGA is automatically enabled when this
 * function is called.
 *
 * @param gain Gain value.
 *
 * @return Success/Fail, 0 if success, -EINVAL if \a gain is out of range.
 */
int max11261_adc_set_gain(max11261_pga_gain_t gain);

/**
 * @brief Disables the amplifier.
 */
void max11261_adc_disable_pga(void);

/**
 * @brief Sets the multiplexer delay.
 *
 * Start of conversion is delayed by the set amount of time to allow for input
 * to become stable. Recommended to use with high impedance source networks.
 *
 * @param delay Delay value in microseconds. Must be less than 1021
 * microseconds.
 *
 * @return Success/Fail, 0 if success, -EINVAL if the value is out
 * of supported range.
 */
int max11261_adc_set_mux_delay(uint16_t delay);

/**
 * @brief Sets the GPO delay.
 *
 * Start of conversion is delayed by the set amount of time to allow sufficient
 * settling time if GPOs are used to bias external circuitry such as bridge
 * sensors.
 *
 * GPO delay is only used in sequencer modes 3 and 4.
 *
 * @param delay Delay value in microseconds. Must be less than or equal to 5100
 * microseconds.
 *
 * @return Success/Fail, 0 if success, -EINVAL if the value is out
 * of supported range.
 */
int max11261_adc_set_gpo_delay(uint16_t delay);

/**
 * @brief Returns the MAX11261 powerdown state.
 *
 * @return Current power-down state, error code otherwise.
 */
max11261_pd_state_t max11261_adc_pd_state(void);

/**
 * @brief Read and print the values inside the MAX11261 registers.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_dump_regs(void);

/**
 * @brief Puts the MAX11261 into sleep mode.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_sleep(void);

/**
 * @brief Puts the MAX11261 into standby mode.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_standby(void);

/**
 * @brief Performs a self-calibration operation.
 */
int max11261_adc_calibrate_self(void);

/**
 * @brief Enable the given general-purpose output.
 *
 * GPOs can only be controlled directly in sequencer modes 1 and 2. In sleep
 * state GPOs are inactive regardless of the setting.
 *
 * @param gpo Output to enable. One of \ref max11261_gpo_t.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_enable_gpo(max11261_gpo_t gpo);

/**
 * @brief Disable the given general-purpose output.
 *
 * @param gpo Output to disable. One of \ref max11261_gpo_t.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_disable_gpo(max11261_gpo_t gpo);

/**
 * @brief Prepares MAX11261 internal registers for conversion.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_convert_prepare(void);

/**
 * @brief Reads the available conversion result into @p val.
 *
 * A conversion command must have already been sent otherwise the function will
 * timeout.
 * For sequencer modes 2 and 3, SEQ:RDYBEN is enabled, meaning that all
 * channels will have been converted before the ready bit is asserted.
 *
 * @param res A \a max11261_adc_result_t buffer to write the conversion
 * results into.
 * @param count Number of elements in \a max11261_adc_result_t buffer.
 *
 * @return Success/Fail, returns the number of conversion results read,
 * negative error code otherwise.
 */
int max11261_adc_result(max11261_adc_result_t *res, int count);

/**
 * @brief Starts conversion by sending a sequencer command.
 *
 * @return Success/Fail, see \ref errno.h for a list of return codes.
 */
int max11261_adc_convert(void);

#endif /* MAX11261_H_ */

/*******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
*******************************************************************************/

#ifndef __MAX9867_H__
#define __MAX9867_H__

#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// clang-format off
typedef enum _AVFLT_DVFLT {
  FILTER_DISABLED,
  ELLIPTICAL_SR16KHZ,
  BUTTERWORTH_SR16KHZ,
  ELLIPTICAL_SR8KHZ,
  BUTTERWORTH_SR8KHZ,
  BUTTERWORTH_SR8TO24KHZ,
  DC_BLOCKING = BUTTERWORTH_SR8TO24KHZ
} digital_filter_t;

typedef enum _DSTS {
  DSTS_NONE,
  DSTS_LEFT_ADC,
  DSTS_RIGHT_ADC,
  DSTS_LEFT_RIGHT_ADC
} digital_sidetone_source_t;

typedef enum _DACG {
  DAC_GAIN_0,
  DAC_GAIN_6,
  DAC_GAIN_12,
  DAC_GAIN_18
} dac_gain_t;

typedef enum _MXINL_MXINR {
  ADC_IN_NONE,
  ADC_IN_ANALOG_MIC,
  ADC_IN_LINE_IN,
  ADC_IN_LINE_IN_ANALOG_MIC
} adc_input_t;

typedef enum _PGAML_PGAMR {
  MIC_PA_GAIN_DISABLE,
  MIC_PA_GAIN_0,
  MIC_PA_GAIN_20,
  MIC_PA_GAIN_30
} mic_pa_gain_t;

typedef enum _MICCLK {
  MICCLK_PCLK_DIV8,
  MICCLK_PCLK_DIV6
} mic_clk_t;

typedef enum _PMREG {
  EN_RIGHT_ADC    = 0x01,
  EN_LEFT_ADC     = 0x02,
  EN_RIGHT_DAC    = 0x04,
  EN_LEFT_DAC     = 0x08,
  EN_LEFT_LINEIN  = 0x20,
  EN_RIGHT_LINEIN = 0x40
} codec_enables_t;

typedef enum _HPMODE {
  HPMODE_STEREO_DIFF_CLICKLESS,
  HPMODE_MONO_DIFF_CLICKLESS,
  HPMODE_STEREO_NOCAP_CLICKLESS,
  HPMODE_MONO_NOCAP_CLICKLESS,
  HPMODE_STEREO_SE_CLICKLESS,
  HPMODE_MONO_SE_CLICKLESS,
  HPMODE_STEREO_SE_FAST_ON,
  HPMODE_MONO_SE_FAST_ON
} headphone_mode_t;
// clang-format on

/**
 * @brief Select IIR Voice or FIR Audio filters for ADC and DAC
 *
 * @param fir Filter type selection 0 = IIR, 1 = FIR.
 * @param fltra ADC digital audio filter, one of digital_filter_t.
 * @param fltrd DAC digital audio filter, one of digital_filter_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_digital_filter(int fir, digital_filter_t fltra, digital_filter_t fltrd);

/**
 * @brief Select digital sidetone source
 *
 * @param src Sidetone source, one of digital_sidetone_source_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_digital_sidetone_source(digital_sidetone_source_t src);

/**
 * @brief Select digital sidetone level
 *
 * @note Valid sidetone levels are [-60,0] in steps of +2 for differential mode and
 * [-65,-5] in steps of +2 for capacitorless and single-ended modes.
 *
 * @param differential Current mode headphone mode, one of headphone_mode_t.
 * @param level Sidetone level, levels greater than zero are taken as OFF.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_digital_sidetone_level(headphone_mode_t differential, int level);

/**
 * @brief DAC mute control
 *
 * @param mute Mute state 0 = unmuted, 1 = muted.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_dac_mute(int mute);

/**
 * @brief Set DAC gain
 *
 * @param gain Gain value, one of dac_gain_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_dac_gain(dac_gain_t gain);

/**
 * @brief Set DAC level
 *
 * @note Valid DAC levels are [-15,0]
 *
 * @param level Level value
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_dac_level(int level);

/**
 * @brief Set ADC left and right level
 *
 * @note Valid left and right levels are [-12,+3]
 *
 * @param left Left level value
 * @param right Right level value
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_adc_level(int left, int right);

/**
 * @brief Set line-in playback mute
 *
 * @param left Mute state 0 = unmuted, 1 = muted.
 * @param right Mute state 0 = unmuted, 1 = muted.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_linein_mute(int left, int right);

/**
 * @brief Set line-in left and right gain
 *
 * @note Valid line-in gain values are [-6,+24] in steps of +2 ie -6, -4, -2, 0, +2
 *
 * @param left Gain value.
 * @param right Gain value.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_linein_gain(int left, int right);

/**
 * @brief Set line-in to headphone output volume tracking
 *
 * @param fix Fix value 0 = headphone output volume tracks line-in, 1 headphone output volume fixed.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_linein_vol_fix(int fix);

/**
 * @brief Set playback mute of DAC and line-in signals
 *
 * @param left Mute state 0 = unmuted, 1 = muted
 * @param right Mute state 0 = unmuted, 1 = muted
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_playback_mute(int left, int right);

/**
 * @brief Set playback volume of DAC and line-in signals
 *
 * @note Valid left and right volume values are
 *       +6.0, +1.0,  -8.0, -26.0, -58.0,
 *       +5.5,  0.0, -10.0, -30.0, -62.0,
 *       +5.0, -1.0, -12.0, -34.0, -66.0,
 *       +4.5, -2.0, -14.0, -38.0, -70.0,
 *       +4.0, -3.0, -16.0, -42.0, -74.0,
 *       +3.5, -4.0, -18.0, -46.0, -78.0,
 *       +3.0, -5.0, -20.0, -50.0, -82.0,
 *       +2.0, -6.0, -22.0, -54.0, -84.0
 *       A value of -85.0 mutes the channel.
 *
 * @param left Volume value.
 * @param right Volume value.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_playback_volume(float left, float right);

/**
 * @brief Set microphone preamplifier gain
 *
 * @param left Gain value, one of mic_pa_gain_t.
 * @param right Gain value, one of mic_pa_gain_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_microphone_preamp(mic_pa_gain_t left, mic_pa_gain_t right);

/**
 * @brief Set microphone PGA gain
 *
 * @note Valid gain values are [0,+20]
 *
 * @param left Gain value.
 * @param right Gain value.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_microphone_gain(int left, int right);

/**
 * @brief Set ADC mixer input
 *
 * @param left Mixer input selection, one of adc_input_t.
 * @param right Mixer input selection, one of adc_input_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_adc_input(adc_input_t left, adc_input_t right);

/**
 * @brief Set digital microphone clock
 *
 * @param mclk Clock value, one of mic_clk_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_microphone_clock(mic_clk_t mclk);

/**
 * @brief Digital microphone enable
 *
 * @param left Enable value 0 = disabled, 1 = enabled.
 * @param right Enable value 0 = disabled, 1 = enabled.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_microphone_enable(int left, int right);

/**
 * @brief Set digital volume slew speed
 *
 * @param slew_80 Slew speed 0 = 10ms, 1 = 80ms.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_digital_volume_slew(int slew_80);

/**
 * @brief Set volume smoothing
 *
 * @param one_step Step rate 0 = slew through intermediate values, 1 = change on one step.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_volume_change_smoothing(int one_step);

/**
 * @brief Set line-in zero crossing detection
 *
 * @param disable Detection state 0 = change volume at zero crossings or after 62ms if no zero crossing,
 *                1 = volume changes occur immediately.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_linein_zero_crossing_disable(int disable);

/**
 * @brief Jack detection enable
 *
 * @param enable Enable state
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_jack_detect_enable(int enable);

/**
 * @brief Set headphone amplifier mode
 *
 * @param mode Mode selection, headphone_mode_t.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_headphone_mode(headphone_mode_t mode);

/**
 * @brief Set power state
 *
 * @param shutdown Shutdown 0 = normal operation, 1 = low-power shutdown mode.
 * @param enable Enabled device functions. A bitfield of devices to be enabled.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_power_enable(int shutdown, int enable);

/**
 * @brief Enable playback signal path
 *
 * @note Headphone output is configured for single-ended clickless mode, playback volume is set to 0dB.
 * The CODEC must be initialized by calling max9867_init() prior to calling this function.
 *
 * @param stereo Enable left and right channels when = 1, enable left channel only when = 0.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_enable_playback(int stereo);

/**
 * @brief Enable record signal path
 *
 * @note ADC and line-in circuits are enabled, ADC and line-in levels are set to 0dB. The CODEC must
 * be initialized by calling max9867_init() prior to calling this function.
 *
 * @param stereo Enable left and right channels when = 1, enable left channel only when = 0.
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_enable_record(int stereo);

/**
 * @brief Initialize MAX9867 CODEC
 *
 * @details The initialization function first verifies communication with a MAX9867 CODEC by reading
 * the part revision identification register. If a suitable device is found then the digital
 * audio interface (DAI) is configured for I2S data. The MCLK prescaler is configured based on
 * the provided mclk parameter.
 * On return if no errors are encountered the DAI and MCLK are configured and the device is in
 * low-power mode.
 *
 * @param i2c_inst I2C instance for communication with CODEC
 * @param mclk MCLK input frequency in Hz
 * @param controller configure as I2S controller
 * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int controller);

#ifdef __cplusplus
}
#endif

#endif

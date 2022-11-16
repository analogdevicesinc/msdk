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

#include <stdint.h>
#include "i2c.h"
#include "max9867.h"

// clang-format off
#define MAX9867_00_STATUS            0x00
#define MAX9867_01_JACKSENSE         0x01
#define MAX9867_02_AUX_HIGH          0x02
#define MAX9867_03_AUX_LOW           0x03
#define MAX9867_04_INT_EN            0x04
#define MAX9867_05_SYS_CLK           0x05
#define MAX9867_06_CLK_HIGH          0x06
#define MAX9867_07_CLK_LOW           0x07
#define MAX9867_08_DAI_FORMAT        0x08
#define MAX9867_09_DAI_CLOCK         0x09
#define MAX9867_0A_DIG_FILTER        0x0A
#define MAX9867_0B_SIDETONE          0x0B
#define MAX9867_0C_LVL_DAC           0x0C
#define MAX9867_0D_LVL_ADC           0x0D
#define MAX9867_0E_LVL_LINE_IN_LEFT  0x0E
#define MAX9867_0F_LVL_LINE_IN_RIGHT 0x0F
#define MAX9867_10_VOL_LEFT          0x10
#define MAX9867_11_VOL_RIGHT         0x11
#define MAX9867_12_MIC_GAIN_LEFT     0x12
#define MAX9867_13_MIC_GAIN_RIGHT    0x13
#define MAX9867_14_ADC_INPUT         0x14
#define MAX9867_15_MIC               0x15
#define MAX9867_16_MODE              0x16
#define MAX9867_17_PWR_SYS           0x17
#define MAX9867_FF_REV_ID            0xFF
// clang-format on

#define MAX9867_ADDR 0x18
#define MAX9867_REVID 0x42

#define MHZ(N) (N*1000000)

static mxc_i2c_req_t i2c_req;

static int reg_write(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { reg, val };

  i2c_req.tx_buf = buf;
  i2c_req.tx_len = sizeof(buf);
  i2c_req.rx_len = 0;

  return MXC_I2C_MasterTransaction(&i2c_req);
}

static int reg_read(uint8_t reg, uint8_t *dest)
{
  uint8_t buf[1] = { reg };

  i2c_req.tx_buf = buf;
  i2c_req.tx_len = sizeof(buf);
  i2c_req.rx_buf = dest;
  i2c_req.rx_len = 1;

  return MXC_I2C_MasterTransaction(&i2c_req);
}

static int reg_update(uint8_t reg, uint8_t mask, uint8_t val)
{
  int err;
  uint8_t tmp;

  if ((err = reg_read(reg, &tmp)) != E_NO_ERROR)
    return err;

  tmp &= ~mask;
  tmp |= val & mask;

  return reg_write(reg, tmp);
}

int max9867_digital_filter(int fir, digital_filter_t fltra, digital_filter_t fltrd)
{
  return reg_write(MAX9867_0A_DIG_FILTER, (!!fir << 7) | (fltra << 4) | (fltrd << 0));
}

int max9867_digital_sidetone_source(digital_sidetone_source_t src)
{
  return reg_update(MAX9867_0B_SIDETONE, 0xC0, src << 6);
}

int max9867_digital_sidetone_level(headphone_mode_t mode, int level)
{
  uint8_t dvst;

  if (level > 0)
    dvst = 0;
  else
    if ((mode == HPMODE_STEREO_DIFF_CLICKLESS) || (mode == HPMODE_MONO_DIFF_CLICKLESS))
      dvst = (0 - level) / 2 + 1;
    else
      dvst = ((0 - level) - 5) / 2 + 1;

  return reg_update(MAX9867_0B_SIDETONE, 0x1F, dvst);
}

int max9867_dac_mute(int mute)
{
  return reg_update(MAX9867_0C_LVL_DAC, 0x40, !!mute << 6);
}

int max9867_dac_gain(dac_gain_t gain)
{
  return reg_update(MAX9867_0C_LVL_DAC, 0x30, gain << 4);
}

int max9867_dac_level(int level)
{
  uint8_t daca;

  daca = 0 - level;

  return reg_update(MAX9867_0C_LVL_DAC, 0x0F, daca);
}

int max9867_adc_level(int left, int right)
{
  uint8_t avl, avr;

  avl = 3 - left;
  avr = 3 - right;

  return reg_update(MAX9867_0D_LVL_ADC, 0xFF, (avl << 4) | (avr & 0x0F));
}

int max9867_linein_mute(int left, int right)
{
  int err;

  if ((err = reg_update(MAX9867_0E_LVL_LINE_IN_LEFT, 0x40, !!left << 6)) != E_NO_ERROR)
    return err;

  return reg_update(MAX9867_0F_LVL_LINE_IN_RIGHT, 0x40, !!right << 6);
}

int max9867_linein_gain(int left, int right)
{
  int err;
  uint8_t ligl, ligr;

  ligl = (24 - left) / 2;
  ligr = (24 - right) / 2;

  if ((err = reg_update(MAX9867_0E_LVL_LINE_IN_LEFT, 0x0F, ligl)) != E_NO_ERROR)
    return err;

  return reg_update(MAX9867_0F_LVL_LINE_IN_RIGHT, 0x0F, ligr);
}

int max9867_linein_vol_fix(int fix)
{
  return reg_update(MAX9867_09_DAI_CLOCK, 0x01, !!fix << 4);
}

int max9867_playback_mute(int left, int right)
{
  int err;

  if ((err = reg_update(MAX9867_10_VOL_LEFT, 0x40, !!left << 6)) != E_NO_ERROR)
    return err;

  return reg_update(MAX9867_11_VOL_RIGHT, 0x40, !!right << 6);
}

static uint8_t map_vol(float vol)
{
  int tmp;
  uint8_t res;

  tmp = vol * 10;

  if (tmp >= 30)
    res = (60 - tmp) / 5;
  else if (tmp >= -60)
    res = (2 - (int)vol) + 7;
  else if (tmp >= -220)
    res = (8 - (int)vol) / 2 + 8;
  else if (tmp >= -860)
    res = (26 - (int)vol) / 4 + 11;
  else
    res = 0x3F;

  return res;
}

int max9867_playback_volume(float left, float right)
{
  int err;
  uint8_t voll, volr;

  voll = map_vol(left);
  volr = map_vol(right);

  if ((err = reg_update(MAX9867_10_VOL_LEFT, 0x3F, voll)) != E_NO_ERROR)
    return err;

  return reg_update(MAX9867_11_VOL_RIGHT, 0x3F, volr);
}

int max9867_microphone_preamp(mic_pa_gain_t left, mic_pa_gain_t right)
{
  int err;

  if ((err = reg_update(MAX9867_12_MIC_GAIN_LEFT, 0x60, left << 5)) != E_NO_ERROR)
    return err;

  return reg_update(MAX9867_13_MIC_GAIN_RIGHT, 0x60, right << 5);
}

int max9867_microphone_gain(int left, int right)
{
  int err;
  uint8_t pgaml, pgamr;

  pgaml = 20 - left;
  pgamr = 20 - right;

  if ((err = reg_update(MAX9867_12_MIC_GAIN_LEFT, 0x1F, pgaml)) != E_NO_ERROR)
    return err;

  return reg_update(MAX9867_13_MIC_GAIN_RIGHT, 0x1F, pgamr);
}

int max9867_adc_input(adc_input_t left, adc_input_t right)
{
  return reg_update(MAX9867_14_ADC_INPUT, 0xF0, (left << 6) | (right << 4));
}

int max9867_microphone_clock(mic_clk_t mclk)
{
  return reg_update(MAX9867_15_MIC, 0xC0, mclk << 6);
}

int max9867_microphone_enable(int left, int right)
{
  return reg_update(MAX9867_15_MIC, 0x30, (!!left << 5) | (!!right << 4));
}

int max9867_digital_volume_slew(int slew_80)
{
  return reg_update(MAX9867_16_MODE, 0x80, !!slew_80 << 7);
}

int max9867_volume_change_smoothing(int one_step)
{
  return reg_update(MAX9867_16_MODE, 0x40, !!one_step << 6);
}

int max9867_linein_zero_crossing_disable(int disable)
{
  return reg_update(MAX9867_16_MODE, 0x20, !!disable << 5);
}

int max9867_jack_detect_enable(int enable)
{
  return reg_update(MAX9867_16_MODE, 0x08, !!enable << 3);
}

int max9867_headphone_mode(headphone_mode_t mode)
{
  return reg_update(MAX9867_16_MODE, 0x07, mode);
}

int max9867_power_enable(int shutdown, int enable)
{
  int err;

  if (shutdown)
    if ((err = reg_write(MAX9867_17_PWR_SYS, 0x00)) != E_NO_ERROR)
      return err;

  return reg_write(MAX9867_17_PWR_SYS, 0x80 | enable);
}

int max9867_enable_playback(int stereo)
{
  int err;
  uint8_t en;

  if (!i2c_req.i2c)
    return E_NULL_PTR;

  if ((err = max9867_headphone_mode(HPMODE_STEREO_SE_CLICKLESS)) != E_NO_ERROR)
    return err;

  if ((err = max9867_playback_volume(0, 0)) != E_NO_ERROR)
    return err;

  /* Assert SHDN as first step in toggling SHDN when changing enabled circuitry */
  if ((err = reg_update(MAX9867_17_PWR_SYS, 0x80, 0x00)) != E_NO_ERROR)
    return err;

  en = EN_LEFT_DAC;
  if (stereo)
    en |= EN_RIGHT_DAC;

  return reg_update(MAX9867_17_PWR_SYS, 0x8C, 0x80 | en);
}

int max9867_enable_record(int stereo)
{
  int err;
  uint8_t en;

  if (!i2c_req.i2c)
    return E_NULL_PTR;

  if ((err = max9867_adc_input(ADC_IN_LINE_IN, stereo ? ADC_IN_LINE_IN : ADC_IN_NONE)) != E_NO_ERROR)
    return err;

  if ((err = max9867_adc_level(0, 0)) != E_NO_ERROR)
    return err;

  /* disconnect line inputs from headphone amplifiers */
  if ((err = max9867_linein_mute(1, 1)) != E_NO_ERROR)
    return err;

  if ((err = max9867_linein_gain(0, 0)) != E_NO_ERROR)
    return err;

  /* Assert SHDN as first step in toggling SHDN when changing enabled circuitry */
  if ((err = reg_update(MAX9867_17_PWR_SYS, 0x80, 0x00)) != E_NO_ERROR)
    return err;

  en = EN_LEFT_ADC | EN_LEFT_LINEIN;
  if (stereo)
    en |= EN_RIGHT_ADC | EN_RIGHT_LINEIN;

  return reg_update(MAX9867_17_PWR_SYS, 0xE3, 0x80 | en);
}

int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int controller)
{
  int err;
  uint8_t rev_id;
  int psclk;

  if (!i2c_inst)
    return E_NULL_PTR;

  /* Static I2C request values */
  i2c_req.i2c = i2c_inst;
  i2c_req.addr = MAX9867_ADDR;
  i2c_req.restart = 0;
  i2c_req.callback = (void *)0;

  /* Probe for MAX9867 */
  if ((err = reg_read(MAX9867_FF_REV_ID, &rev_id)) != E_NO_ERROR)
    return err;

  if (rev_id != MAX9867_REVID)
    return E_NOT_SUPPORTED;

  /* Shutdown for configuration */
  if ((err = max9867_power_enable(1, 0)) != E_NO_ERROR)
    return err;

  /* Clear all regs to POR state */
  for (int i = MAX9867_04_INT_EN; i < MAX9867_17_PWR_SYS; i++)
    if ((err = reg_write(i, 0x00)) != E_NO_ERROR)
      return err;

  /* Select MCLK prescaler */
  if ((mclk < MHZ(10)) || (mclk > MHZ(60)))
    return E_INVALID;
  else if (mclk < MHZ(20))
    psclk = 0x1;
  else if (mclk < MHZ(40))
    psclk = 0x2;
  else
    psclk = 0x3;

  /* Set prescaler, FREQ field is 0 for Normal clock mode */
  if ((err = reg_write(MAX9867_05_SYS_CLK, psclk << 4)) != E_NO_ERROR)
    return err;

  /* Use PLL in target mode or appropriate NI in controller mode.
     NI=0x3000 when MCLK=12.288MHz and LRCLK=24KHz
     See MAX9867 datasheet for NI selection */
  if ((err = reg_write(MAX9867_06_CLK_HIGH, controller ? 0x30 : 0x80)) != E_NO_ERROR)
    return err;

  if (controller)
    if ((err = reg_write(MAX9867_09_DAI_CLOCK, 0x02)) != E_NO_ERROR)
      return err;

  /* I2S format, data is delayed 1 bit clock, HI-Z mode disabled */
  if ((err = reg_write(MAX9867_08_DAI_FORMAT, controller ? 0x98 : 0x18)) != E_NO_ERROR)
    return err;

  return E_NO_ERROR;
}

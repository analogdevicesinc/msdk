/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "adc.h"
#include "adc_regs.h"
#include "adc_revb.h"
#include "gcr_regs.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include "mxc_pins.h"
#include "pwrseq_regs.h"

/*TODO-
 * This is work in progress file and has not finalized yet. Expected to change some parameters as well as some function prototypes.
 */

#define MXC_F_MCR_ADCCFG2_CH 0x3
#define TEMP_FACTOR 530.582f / 4096.0f
#define TEMP_FACTOR1V25 1.25f * TEMP_FACTOR
#define TEMP_FACTOR2V048 2.048f * TEMP_FACTOR

static void initGPIOForChannel(mxc_adc_chsel_t channel)
{
    switch (channel) {
    case MXC_ADC_CH_0:
        MXC_GPIO_Config(&gpio_cfg_adc_ain0);
        break;

    case MXC_ADC_CH_1:
        MXC_GPIO_Config(&gpio_cfg_adc_ain1);
        break;

    case MXC_ADC_CH_2:
        MXC_GPIO_Config(&gpio_cfg_adc_ain2);
        break;

    case MXC_ADC_CH_3:
        MXC_GPIO_Config(&gpio_cfg_adc_ain3);
        break;

    case MXC_ADC_CH_4:
        MXC_GPIO_Config(&gpio_cfg_adc_ain4);
        break;

    case MXC_ADC_CH_5:
        MXC_GPIO_Config(&gpio_cfg_adc_ain5);
        break;

    case MXC_ADC_CH_6:
        MXC_GPIO_Config(&gpio_cfg_adc_ain6);
        break;

    case MXC_ADC_CH_7:
        MXC_GPIO_Config(&gpio_cfg_adc_ain7);
        break;

    default:
        break;
    }
}

static void initGPIOforHWTrig(mxc_adc_trig_sel_t hwTrig)
{
    mxc_gpio_cfg_t gpioCfg;
    switch (hwTrig) {
    case MXC_ADC_TRIG_SEL_TMR0:
    case MXC_ADC_TRIG_SEL_TMR1:
    case MXC_ADC_TRIG_SEL_TMR2:
    case MXC_ADC_TRIG_SEL_TMR3:
    case MXC_ADC_TRIG_SEL_TEMP_SENS:
        break;
    case MXC_ADC_TRIG_SEL_P1_12:
        gpioCfg.port = MXC_GPIO1;
        gpioCfg.mask = MXC_GPIO_PIN_12;
        gpioCfg.func = MXC_GPIO_FUNC_ALT2;
        gpioCfg.pad = MXC_GPIO_PAD_NONE;
        gpioCfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
        gpioCfg.drvstr = MXC_GPIO_DRVSTR_0;
        MXC_GPIO_Config(&gpioCfg);
        break;
    case MXC_ADC_TRIG_SEL_P1_13:
        gpioCfg.port = MXC_GPIO1;
        gpioCfg.mask = MXC_GPIO_PIN_13;
        gpioCfg.func = MXC_GPIO_FUNC_ALT2;
        gpioCfg.pad = MXC_GPIO_PAD_NONE;
        gpioCfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
        gpioCfg.drvstr = MXC_GPIO_DRVSTR_0;
        MXC_GPIO_Config(&gpioCfg);
        break;
    case MXC_ADC_TRIG_SEL_P1_14:
        gpioCfg.port = MXC_GPIO1;
        gpioCfg.mask = MXC_GPIO_PIN_14;
        gpioCfg.func = MXC_GPIO_FUNC_ALT2;
        gpioCfg.pad = MXC_GPIO_PAD_NONE;
        gpioCfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
        gpioCfg.drvstr = MXC_GPIO_DRVSTR_0;
        MXC_GPIO_Config(&gpioCfg);
        break;
    }
}

int MXC_ADC_Init(mxc_adc_req_t *req)
{
    switch (req->clock) {
    case MXC_ADC_CLK_HCLK:
        break;
    case MXC_ADC_CLK_EXT:
        MXC_GPIO_Config(&gpio_cfg_extclk);
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_EXTCLK);
        break;
    case MXC_ADC_CLK_IBRO:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        break;
    default:
        return E_BAD_PARAM;
    }

    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_ADC);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_ADC);

    /* This is required for temperature sensor only */
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);

    MXC_ADC_ReferenceSelect(req->ref);

    return MXC_ADC_RevB_Init((mxc_adc_revb_regs_t *)MXC_ADC, req);
}

int MXC_ADC_Shutdown(void)
{
    MXC_ADC_RevB_Shutdown((mxc_adc_revb_regs_t *)MXC_ADC);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ADC);

    return E_NO_ERROR;
}

void MXC_ADC_EnableInt(uint32_t flags)
{
    MXC_ADC_RevB_EnableInt((mxc_adc_revb_regs_t *)MXC_ADC, flags);
}

void MXC_ADC_DisableInt(uint32_t flags)
{
    MXC_ADC_RevB_DisableInt((mxc_adc_revb_regs_t *)MXC_ADC, flags);
}

int MXC_ADC_GetFlags(void)
{
    return MXC_ADC_RevB_GetFlags((mxc_adc_revb_regs_t *)MXC_ADC);
}

void MXC_ADC_ClearFlags(uint32_t flags)
{
    MXC_ADC_RevB_ClearFlags((mxc_adc_revb_regs_t *)MXC_ADC, flags);
}

void MXC_ADC_ClockSelect(mxc_adc_clock_t clock)
{
    MXC_ADC_RevB_ClockSelect((mxc_adc_revb_regs_t *)MXC_ADC, clock);
}

int MXC_ADC_StartConversion(void)
{
    return MXC_ADC_RevB_StartConversion((mxc_adc_revb_regs_t *)MXC_ADC);
}

int MXC_ADC_StartConversionAsync(mxc_adc_complete_cb_t callback)
{
    return MXC_ADC_RevB_StartConversionAsync((mxc_adc_revb_regs_t *)MXC_ADC, callback);
}

int MXC_ADC_StartConversionDMA(mxc_adc_conversion_req_t *req, int *data, void (*callback)(int, int))
{
    return MXC_ADC_RevB_StartConversionDMA((mxc_adc_revb_regs_t *)MXC_ADC, req, data, callback);
}

int MXC_ADC_Handler(void)
{
    return MXC_ADC_RevB_Handler((mxc_adc_revb_regs_t *)MXC_ADC);
}

int MXC_ADC_GetData(int *outdata)
{
    return MXC_ADC_RevB_GetData((mxc_adc_revb_regs_t *)MXC_ADC, outdata);
}

int MXC_ADC_InputDividerSelect(mxc_adc_chsel_t ch, mxc_adc_divsel_t div,
                               mxc_adc_dynamic_pullup_t lpEn)
{
    uint32_t bitOffset;

    if (ch > MXC_ADC_CH_7 || div > MXC_ADC_DIV2_50K) {
        return E_BAD_PARAM;
    }

    bitOffset = ch << 1;
    MXC_MCR->adccfg2 &= ~(MXC_F_MCR_ADCCFG2_CH << bitOffset);
    MXC_MCR->adccfg2 |= (div << bitOffset);

    if (lpEn == MXC_ADC_PY_DN_DISABLE) {
        MXC_ADC_DynamicModeDis(ch);
    } else {
        MXC_ADC_DynamicModeEn(ch);
    }

    return E_NO_ERROR;
}

int MXC_ADC_ReferenceSelect(mxc_adc_refsel_t ref)
{
    switch (ref) {
    case MXC_ADC_REF_EXT:
        MXC_MCR->adccfg0 |= MXC_F_MCR_ADCCFG0_EXT_REF;
        break;
    case MXC_ADC_REF_INT_1V25:
        MXC_MCR->adccfg0 &= ~(MXC_F_MCR_ADCCFG0_EXT_REF | MXC_F_MCR_ADCCFG0_REF_SEL);
        break;
    case MXC_ADC_REF_INT_2V048:
        MXC_MCR->adccfg0 &= ~MXC_F_MCR_ADCCFG0_EXT_REF;
        MXC_MCR->adccfg0 |= MXC_F_MCR_ADCCFG0_REF_SEL;
        break;
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_ADC_LowPowerModeDividerSelect(mxc_adc_div_lpmode_t div_lpmode)
{
    MXC_MCR->adccfg0 &= ~(MXC_F_MCR_ADCCFG0_LP_5K_DIS | MXC_F_MCR_ADCCFG0_LP_50K_DIS);
    MXC_MCR->adccfg0 |= div_lpmode;

    return E_NO_ERROR;
}

void MXC_ADC_EnableConversion(void)
{
    MXC_ADC_RevB_EnableConversion((mxc_adc_revb_regs_t *)MXC_ADC);
}

void MXC_ADC_DisableConversion(void)
{
    MXC_ADC_RevB_DisableConversion((mxc_adc_revb_regs_t *)MXC_ADC);
}

void MXC_ADC_TS_SelectEnable(void)
{
    MXC_ADC_RevB_TS_SelectEnable((mxc_adc_revb_regs_t *)MXC_ADC);
}

void MXC_ADC_TS_SelectDisable(void)
{
    MXC_ADC_RevB_TS_SelectDisable((mxc_adc_revb_regs_t *)MXC_ADC);
}

uint16_t MXC_ADC_FIFO_Level(void)
{
    return MXC_ADC_RevB_FIFO_Level((mxc_adc_revb_regs_t *)MXC_ADC);
}

int MXC_ADC_FIFO_Threshold_Config(uint32_t fifo_threshold)
{
    return MXC_ADC_RevB_FIFO_Threshold_Config((mxc_adc_revb_regs_t *)MXC_ADC, fifo_threshold);
}

int MXC_ADC_AverageConfig(mxc_adc_avg_t avg_number)
{
    return MXC_ADC_RevB_AverageConfig((mxc_adc_revb_regs_t *)MXC_ADC, avg_number);
}

void MXC_ADC_Clear_ChannelSelect(void)
{
    MXC_ADC_RevB_Clear_ChannelSelect((mxc_adc_revb_regs_t *)MXC_ADC);
}

void MXC_ADC_TriggerConfig(mxc_adc_conversion_req_t *req)
{
    initGPIOforHWTrig(req->hwTrig);
    MXC_ADC_RevB_TriggerConfig((mxc_adc_revb_regs_t *)MXC_ADC, req);
}

void MXC_ADC_ConversionModeConfig(mxc_adc_conversion_req_t *req)
{
    MXC_ADC_RevB_ConversionModeConfig((mxc_adc_revb_regs_t *)MXC_ADC, req);
}

void MXC_ADC_SetConversionDelay(int delay)
{
    MXC_ADC_RevB_SetConversionDelay((mxc_adc_revb_regs_t *)MXC_ADC, delay);
}

int MXC_ADC_SlotsConfig(mxc_adc_conversion_req_t *req)
{
    return MXC_ADC_RevB_SlotsConfig((mxc_adc_revb_regs_t *)MXC_ADC, req);
}

int MXC_ADC_ChSelectConfig(mxc_adc_chsel_t ch, uint32_t slot_num)
{
    return MXC_ADC_RevB_ChSelectConfig((mxc_adc_revb_regs_t *)MXC_ADC, ch, slot_num);
}

int MXC_ADC_Configuration(mxc_adc_conversion_req_t *req)
{
    MXC_ADC_ConversionModeConfig(req);

    MXC_ADC_TriggerConfig(req);

    MXC_ADC_FIFO_Threshold_Config(req->fifo_threshold);

    MXC_ADC_SlotsConfig(req);

    MXC_ADC_Clear_ChannelSelect();

    //number of samples to average
    MXC_ADC_AverageConfig(req->avg_number);

    MXC_ADC_LowPowerModeDividerSelect(req->lpmode_divder);

    return E_NO_ERROR;
}

int MXC_ADC_SlotConfiguration(mxc_adc_slot_req_t *req, uint32_t slot_length)
{
    uint32_t loop_counter = 0;

    for (loop_counter = 0; loop_counter <= slot_length; loop_counter++) {
        initGPIOForChannel(req->channel);

        if (req->channel <= MAX_ADC_RES_DIV_CH) {
            MXC_ADC_InputDividerSelect(req->channel, req->div, req->pullup_dyn);
        }

        MXC_ADC_ChSelectConfig(req->channel, loop_counter);
        req++;
    }
    return E_NO_ERROR;
}

int MXC_ConvertTemperature_ToK(uint16_t tempSensor_Readout, mxc_adc_refsel_t ref, float ext_ref,
                               float *temp_k)
{
    switch (ref) {
    case MXC_ADC_REF_EXT:
        *temp_k = tempSensor_Readout * TEMP_FACTOR * ext_ref;
        break;

    case MXC_ADC_REF_INT_1V25:
        *temp_k = tempSensor_Readout * TEMP_FACTOR1V25;
        break;

    case MXC_ADC_REF_INT_2V048:
        *temp_k = tempSensor_Readout * TEMP_FACTOR2V048;
        break;

    default:
        return E_BAD_PARAM;
    }
    return E_NO_ERROR;
}

int MXC_ConvertTemperature_ToC(uint16_t tempSensor_Readout, mxc_adc_refsel_t ref, float ext_ref,
                               float *temp)
{
    if (MXC_ConvertTemperature_ToK(tempSensor_Readout, ref, ext_ref, temp) == E_NO_ERROR) {
        *temp = *temp - 273.15f;
        return E_NO_ERROR;
    } else {
        return E_BAD_PARAM;
    }
}

int MXC_ConvertTemperature_ToF(uint16_t tempSensor_Readout, mxc_adc_refsel_t ref, float ext_ref,
                               float *temp)
{
    if (MXC_ConvertTemperature_ToK(tempSensor_Readout, ref, ext_ref, temp) == E_NO_ERROR) {
        *temp = (*temp * 1.8f) - 459.67f;
        return E_NO_ERROR;
    } else {
        return E_BAD_PARAM;
    }
}

int MXC_ADC_DynamicModeEn(mxc_adc_chsel_t ch)
{
    if (ch > MXC_ADC_CH_7) {
        return E_BAD_PARAM;
    }

    MXC_MCR->adccfg1 |= (1 << ch);

    return E_NO_ERROR;
}

int MXC_ADC_DynamicModeDis(mxc_adc_chsel_t ch)
{
    if (ch > MXC_ADC_CH_7) {
        return E_BAD_PARAM;
    }

    MXC_MCR->adccfg1 &= ~(1 << ch);

    return E_NO_ERROR;
}

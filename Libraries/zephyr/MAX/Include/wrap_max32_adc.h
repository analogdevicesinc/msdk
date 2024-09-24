/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_ADC_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_ADC_H_

/***** Includes *****/
#include <adc.h>
#include <wrap_utils.h>
#include <mxc_delay.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t clock; ///< clock to use
    uint8_t clkdiv; ///< clock divider
    uint8_t cal; ///< skip calibration
    uint8_t ref; ///< ADC reference voltage
    uint32_t trackCount; ///< Sample Clock High time
    uint32_t idleCount; ///< Sample Clock Low time
} wrap_mxc_adc_req_t;

typedef enum {
    WRAP_MXC_ADC_SCALE_2X, // ADC Scale by 2x (this scales ADC Reference by 1/2)
    WRAP_MXC_ADC_SCALE_1, // ADC Scale by 1x (no scaling)
    WRAP_MXC_ADC_SCALE_2, // ADC Scale by 1/2
    WRAP_MXC_ADC_SCALE_3, // ADC Scale by 1/3
    WRAP_MXC_ADC_SCALE_4, // ADC Scale by 1/4
    WRAP_MXC_ADC_SCALE_6, // ADC Scale by 1/6 (this uses 1/3 and an additional 1/2 scaling)
} wrap_mxc_adc_scale_t;

#define ADI_MAX32_ADC_REF_EXT0 0
#define ADI_MAX32_ADC_REF_INTERNAL 1
#define ADI_MAX32_ADC_REF_VDD_1_2 2

/*
 *  MAX32655, MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32655) || (CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666) || \
    (CONFIG_SOC_MAX32680)

#define WRAP_MXC_F_ADC_CONV_DONE_IE MXC_F_ADC_INTR_DONE_IE
#define WRAP_MXC_F_ADC_CONV_DONE_IF MXC_F_ADC_INTR_DONE_IF

static inline int Wrap_MXC_ADC_Init(wrap_mxc_adc_req_t *req)
{
    (void)req;
    return MXC_ADC_Init();
}

static inline void Wrap_MXC_ADC_ChannelSelect(uint32_t *sample_channels)
{
    mxc_adc_regs_t *adc = MXC_ADC;
    int channel_id;

    channel_id = wrap_utils_find_lsb_set(*sample_channels);
    if (channel_id == 0) {
        return;
    }
    --channel_id;

    adc->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    adc->ctrl |= (channel_id << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
}

static inline int Wrap_MXC_ADC_SetExtScale(wrap_mxc_adc_scale_t scale)
{
    MXC_ADC_SetExtScale(scale);
    return 0;
}

static inline int Wrap_MXC_ADC_AverageConfig(uint8_t oversampling)
{
    if (oversampling != 0) {
        return -1; /* Oversampling is not supported */
    }

    return 0;
}

static inline int Wrap_MXC_ADC_ReferenceSelect(uint8_t ref)
{
    mxc_adc_regs_t *adc = MXC_ADC;

    if (ref == ADI_MAX32_ADC_REF_INTERNAL) {
        adc->ctrl &= ~MXC_F_ADC_CTRL_REF_SEL;
    } else if (ref == ADI_MAX32_ADC_REF_VDD_1_2) {
        adc->ctrl |= MXC_F_ADC_CTRL_REF_SEL;
    } else {
        return -1;
    }

    return 0;
}

static inline void Wrap_MXC_ADC_DisableConversion(void)
{
    return;
}

static inline int Wrap_MXC_ADC_StartConversion(uint32_t *sample_channels)
{
    int channel_id;

    channel_id = wrap_utils_find_lsb_set(*sample_channels);
    if (channel_id == 0) {
        return -1;
    }
    --channel_id;
    *sample_channels &= ~(1 << channel_id);

    return MXC_ADC_StartConversion((mxc_adc_chsel_t)channel_id);
}

static inline int Wrap_MXC_ADC_StartConversionAsync(uint32_t *sample_channels,
                                                    mxc_adc_complete_cb_t callback)
{
    int channel_id;

    channel_id = wrap_utils_find_lsb_set(*sample_channels);
    if (channel_id == 0) {
        return -1;
    }
    --channel_id;
    *sample_channels &= ~(1 << channel_id);

    return MXC_ADC_StartConversionAsync((mxc_adc_chsel_t)channel_id, callback);
}

static inline void Wrap_MXC_ADC_GetData(uint16_t **outdata)
{
    MXC_ADC_GetData(*outdata);
    *outdata += 1;
}

/*
 *  MAX32690,  related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32662) || \
    (CONFIG_SOC_MAX78002)

#define WRAP_MXC_F_ADC_CONV_DONE_IE MXC_F_ADC_INTEN_SEQ_DONE
#define WRAP_MXC_F_ADC_CONV_DONE_IF MXC_F_ADC_INTFL_SEQ_DONE

#define MAX_OVERSAMPLING_VALUE 7

static inline int Wrap_MXC_ADC_Init(wrap_mxc_adc_req_t *req)
{
    mxc_adc_req_t mxc_req;

    mxc_req.clock = (mxc_adc_clock_t)req->clock;
    mxc_req.cal = (mxc_adc_calibration_t)req->cal;
    mxc_req.ref = (mxc_adc_refsel_t)req->ref;
    mxc_req.trackCount = req->trackCount;
    mxc_req.idleCount = req->idleCount;

    switch (req->clkdiv) {
    case 1:
        mxc_req.clkdiv = MXC_ADC_CLKDIV_1;
        break;
    case 2:
        mxc_req.clkdiv = MXC_ADC_CLKDIV_2;
        break;
    case 4:
        mxc_req.clkdiv = MXC_ADC_CLKDIV_4;
        break;
    case 8:
        mxc_req.clkdiv = MXC_ADC_CLKDIV_8;
        break;
    case 16:
        mxc_req.clkdiv = MXC_ADC_CLKDIV_16;
        break;
    default:
        return -1;
    }

    /*
    TODO: MXC_ADC_RevB_Init function calls MXC_Delay function which uses SysTick and
    program stucks in this function. Added following line to solve the problem until
    develop a Zephyr compatible 'mxc_delay.c' file.
    */
    MXC_DelayAsync(1000, NULL);

    return MXC_ADC_Init(&mxc_req);
}

static inline int Wrap_MXC_ADC_SetExtScale(wrap_mxc_adc_scale_t scale)
{
    if (scale != WRAP_MXC_ADC_SCALE_1) {
        return -1; /* Scaling is not supported */
    }
    return 0;
}

static inline int Wrap_MXC_ADC_AverageConfig(uint8_t oversampling)
{
    if (oversampling > MAX_OVERSAMPLING_VALUE) {
        return -EINVAL; /* Oversampling value too high */
    }

    MXC_ADC_AverageConfig(oversampling << MXC_F_ADC_CTRL1_AVG_POS);
    return 0;
}

static inline void Wrap_MXC_ADC_ChannelSelect(uint32_t *sample_channels)
{
    const uint8_t num_of_channels = POPCOUNT(*sample_channels);
    mxc_adc_slot_req_t slots[MAX_ADC_SLOT_NUM];
    mxc_adc_conversion_req_t req;
    uint8_t slot_index = 0;
    int channel_id;

    req.num_slots = num_of_channels - 1;

    for (slot_index = 0; slot_index < num_of_channels; slot_index++) {
        channel_id = wrap_utils_find_lsb_set(*sample_channels);
        if (channel_id == 0) {
            continue;
        }
        --channel_id;

        slots[slot_index].channel = (mxc_adc_chsel_t)channel_id;
        *sample_channels &= ~(1 << channel_id);
    }

    MXC_ADC_Clear_ChannelSelect();
    MXC_ADC_SlotsConfig(&req);
    MXC_ADC_SlotConfiguration(slots, num_of_channels - 1);
}

static inline int Wrap_MXC_ADC_ReferenceSelect(uint8_t ref)
{
    if ((ref != ADI_MAX32_ADC_REF_EXT0) && (ref != ADI_MAX32_ADC_REF_INTERNAL)) {
        return -1;
    }

    MXC_ADC_ReferenceSelect(ref); /* Sets MXC_ADC_REF_EXT or MXC_ADC_REF_INT_1V25 for reference. */
    return 0;
}

static inline void Wrap_MXC_ADC_DisableConversion(void)
{
    MXC_ADC_DisableConversion();
}

static inline int Wrap_MXC_ADC_StartConversion(uint32_t *sample_channels)
{
    int ret = 0;

    MXC_ADC_FIFO_Threshold_Config(MAX_ADC_FIFO_LEN >> 1);
    Wrap_MXC_ADC_ChannelSelect(sample_channels);

    ret = MXC_ADC_StartConversion();
    while ((MXC_ADC_GetFlags() & WRAP_MXC_F_ADC_CONV_DONE_IF) != WRAP_MXC_F_ADC_CONV_DONE_IF) {
        {
        }
    }

    return ret;
}

static inline int Wrap_MXC_ADC_StartConversionAsync(uint32_t *sample_channels,
                                                    mxc_adc_complete_cb_t callback)
{
    MXC_ADC_FIFO_Threshold_Config(MAX_ADC_FIFO_LEN >> 1);
    Wrap_MXC_ADC_ChannelSelect(sample_channels);
    return MXC_ADC_StartConversionAsync(callback);
}

static inline void Wrap_MXC_ADC_GetData(uint16_t **outdata)
{
    mxc_adc_regs_t *adc = MXC_ADC;
    uint32_t loop_counter, length;

    length = (adc->status & MXC_F_ADC_STATUS_FIFO_LEVEL) >> MXC_F_ADC_STATUS_FIFO_LEVEL_POS;

    for (loop_counter = 0; loop_counter < length; loop_counter++) {
        **outdata = adc->data & MXC_F_ADC_DATA_DATA;
        *outdata += 1;
    }
}

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_ADC_H_

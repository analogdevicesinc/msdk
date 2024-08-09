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
#include "adc.h"
#include "dma.h"
#include "adc_revb.h"
#include "adc_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"

// Mask for all Interrupt Enable Fields
#define ADC_IE_MASK                                                                             \
    (MXC_F_ADC_REVB_INTEN_READY | MXC_F_ADC_REVB_INTEN_ABORT | MXC_F_ADC_REVB_INTEN_START_DET | \
     MXC_F_ADC_REVB_INTEN_SEQ_STARTED | MXC_F_ADC_REVB_INTEN_SEQ_DONE |                         \
     MXC_F_ADC_REVB_INTEN_CONV_DONE | MXC_F_ADC_REVB_INTEN_CLIPPED |                            \
     MXC_F_ADC_REVB_INTEN_FIFO_LVL | MXC_F_ADC_REVB_INTEN_FIFO_UFL |                            \
     MXC_F_ADC_REVB_INTEN_FIFO_OFL)

#define ADC_IF_MASK                                                                             \
    (MXC_F_ADC_REVB_INTFL_READY | MXC_F_ADC_REVB_INTFL_ABORT | MXC_F_ADC_REVB_INTFL_START_DET | \
     MXC_F_ADC_REVB_INTFL_SEQ_STARTED | MXC_F_ADC_REVB_INTFL_SEQ_DONE |                         \
     MXC_F_ADC_REVB_INTFL_CONV_DONE | MXC_F_ADC_REVB_INTFL_CLIPPED |                            \
     MXC_F_ADC_REVB_INTFL_FIFO_LVL | MXC_F_ADC_REVB_INTFL_FIFO_UFL |                            \
     MXC_F_ADC_REVB_INTFL_FIFO_OFL)

static mxc_adc_complete_cb_t async_callback;
static mxc_adc_conversion_req_t *async_req;
// static volatile uint8_t flag;      //indicates  to irqhandler where to store data

static bool g_is_clock_locked = false;

int MXC_ADC_RevB_Init(mxc_adc_revb_regs_t *adc, mxc_adc_req_t *req)
{
    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->trackCount < 4) {
        return E_BAD_PARAM;
    }

    //Enter reset mode
    adc->ctrl0 &= ~MXC_F_ADC_REVB_CTRL0_RESETB;

    MXC_ADC_RevB_SetClockSource(adc, req->clock);
    MXC_ADC_RevB_SetClockDiv(adc, req->clkdiv);

    //Power up to Sleep State

    adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_RESETB;

    //Move to NAP state
    adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_BIAS_EN;
    MXC_Delay(500);

    if (req->cal == MXC_ADC_EN_CAL) {
        adc->ctrl0 &= ~MXC_F_ADC_REVB_CTRL0_SKIP_CAL;
    } else {
        adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_SKIP_CAL;
    }

    MXC_SETFIELD(adc->sampclkctrl, MXC_F_ADC_REVB_SAMPCLKCTRL_TRACK_CNT,
                 (req->trackCount << MXC_F_ADC_REVB_SAMPCLKCTRL_TRACK_CNT_POS));
    MXC_SETFIELD(adc->sampclkctrl, MXC_F_ADC_REVB_SAMPCLKCTRL_IDLE_CNT,
                 (req->idleCount << MXC_F_ADC_REVB_SAMPCLKCTRL_IDLE_CNT_POS));

    adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_ADC_EN;

    //Wait for 30us
    MXC_Delay(30);

    //wait for calibration to complete
    while (!(adc->intfl & MXC_F_ADC_REVB_INTFL_READY)) {}

    async_callback = NULL;

    async_req = NULL;

    return E_NO_ERROR;
}

int MXC_ADC_RevB_SetClockSource(mxc_adc_revb_regs_t *adc, mxc_adc_clock_t clock_source)
{
    if (!g_is_clock_locked) {
        MXC_SETFIELD(adc->clkctrl, MXC_F_ADC_REVB_CLKCTRL_CLKSEL,
                     clock_source << MXC_F_ADC_REVB_CLKCTRL_CLKSEL_POS);
    }
    return E_NO_ERROR;
}

int MXC_ADC_RevB_SetClockDiv(mxc_adc_revb_regs_t *adc, mxc_adc_clkdiv_t div)
{
    if (!g_is_clock_locked) {
        MXC_SETFIELD(adc->clkctrl, MXC_F_ADC_REVB_CLKCTRL_CLKDIV,
                     div << MXC_F_ADC_REVB_CLKCTRL_CLKDIV_POS);
    }
    return E_NO_ERROR;
}

int MXC_ADC_RevB_LockClockSource(mxc_adc_revb_regs_t *adc, bool lock)
{
    g_is_clock_locked = lock;
    if (g_is_clock_locked == lock) {
        return E_FAIL;
    } else {
        return E_NO_ERROR;
    }
}

int MXC_ADC_RevB_Shutdown(mxc_adc_revb_regs_t *adc)
{
    if (async_callback != NULL) {
        MXC_FreeLock((uint32_t *)&async_callback);
    }

    if (async_req != NULL) {
        MXC_FreeLock((uint32_t *)&async_req);
    }

    adc->ctrl0 &=
        ~(MXC_F_ADC_REVB_CTRL0_ADC_EN | MXC_F_ADC_REVB_CTRL0_RESETB | MXC_F_ADC_REVB_CTRL0_BIAS_EN);

    return E_NO_ERROR;
}

void MXC_ADC_RevB_EnableInt(mxc_adc_revb_regs_t *adc, uint32_t flags)
{
    adc->inten |= (flags & ADC_IE_MASK);
}

void MXC_ADC_RevB_DisableInt(mxc_adc_revb_regs_t *adc, uint32_t flags)
{
    adc->inten &= ~(flags & ADC_IE_MASK);
}

int MXC_ADC_RevB_GetFlags(mxc_adc_revb_regs_t *adc)
{
    return (adc->intfl & ADC_IF_MASK);
}

void MXC_ADC_RevB_ClearFlags(mxc_adc_revb_regs_t *adc, uint32_t flags)
{
    // Write 1 to clear flags
    adc->intfl |= (flags & ADC_IF_MASK);
}

void MXC_ADC_RevB_ClockSelect(mxc_adc_revb_regs_t *adc, mxc_adc_clock_t clock)
{
    MXC_SETFIELD(adc->clkctrl, MXC_F_ADC_REVB_CLKCTRL_CLKSEL,
                 (clock << MXC_F_ADC_REVB_CLKCTRL_CLKSEL_POS));
}

int MXC_ADC_RevB_StartConversion(mxc_adc_revb_regs_t *adc)
{
    adc->fifodmactrl |= MXC_F_ADC_REVB_FIFODMACTRL_FLUSH; //Flush data FIFO

    MXC_ADC_RevB_ClearFlags(adc, ADC_IF_MASK);

    adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_START;

    return E_NO_ERROR;
}

int MXC_ADC_RevB_StartConversionAsync(mxc_adc_revb_regs_t *adc, mxc_adc_complete_cb_t callback)
{
    if (callback == NULL) {
        return E_BAD_PARAM;
    }

    adc->fifodmactrl |= MXC_F_ADC_REVB_FIFODMACTRL_FLUSH; //Flush data FIFO

    MXC_ADC_RevB_ClearFlags(adc, ADC_IF_MASK);

    while (MXC_GetLock((uint32_t *)&async_callback, (uint32_t)callback) != E_NO_ERROR) {}

    MXC_ADC_RevB_EnableInt(adc, (MXC_F_ADC_REVB_INTEN_SEQ_DONE | MXC_F_ADC_REVB_INTEN_FIFO_LVL));

    adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_START;

    return E_NO_ERROR;
}

int MXC_ADC_RevB_StartConversionDMA(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req,
                                    int *data, void (*callback)(int, int))
{
    if (callback == NULL) {
        return E_BAD_PARAM;
    }

    if (data == NULL) {
        return E_NULL_PTR;
    }

    uint8_t channel;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    uint8_t num_bytes;

    // Clear interrupt flags
    MXC_ADC_RevB_ClearFlags(adc, ADC_IF_MASK);

    adc->fifodmactrl |= MXC_F_ADC_REVB_FIFODMACTRL_FLUSH; //Flush data FIFO

    adc->fifodmactrl |=
        MXC_S_ADC_REVB_FIFODMACTRL_DATA_FORMAT_DATA_STATUS; //Transfer data and status bits

    adc->fifodmactrl |= MXC_F_ADC_REVB_FIFODMACTRL_DMA_EN; //Enable ADC DMA

    num_bytes = (req->num_slots + 1) * 4; //Support 8 slots (32 bytes) only. (TODO)

    channel = req->dma_channel;

    config.reqsel = MXC_DMA_REQUEST_ADC;
    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = channel;
    srcdst.source = NULL;
    srcdst.dest = data;
    srcdst.len = num_bytes;

    MXC_DMA_ConfigChannel(config, srcdst);

    MXC_DMA_SetCallback(channel, callback);

    //TODO(ADI): This supports 32 bytes transfer. In MXC_ADC_DATA_STATUS if all channels are used 64 bytes may need to read.
    MXC_DMA->ch[channel].ctrl |= (num_bytes - 1) << MXC_F_DMA_CTRL_BURST_SIZE_POS;

    MXC_DMA_EnableInt(channel);

    MXC_DMA_Start(channel);

    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);

    adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_START;

    return E_NO_ERROR;
}

int MXC_ADC_RevB_Handler(mxc_adc_revb_regs_t *adc)
{
    uint32_t flags;

    flags = MXC_ADC_RevB_GetFlags(adc);

    if (flags & (MXC_F_ADC_REVB_INTEN_SEQ_DONE | MXC_F_ADC_REVB_INTEN_CONV_DONE)) {
        mxc_adc_complete_cb_t cb = async_callback;

        if (flags & MXC_F_ADC_REVB_INTEN_SEQ_DONE) {
            MXC_ADC_RevB_ClearFlags(adc, flags);

            // Disable interrupts only when in single conversion mode
            if (!(adc->ctrl1 & MXC_F_ADC_REVB_CTRL1_CNV_MODE)) {
                MXC_ADC_RevB_DisableInt(adc, (MXC_F_ADC_REVB_INTFL_SEQ_DONE |
                                              MXC_F_ADC_REVB_INTFL_CONV_DONE |
                                              MXC_F_ADC_REVB_INTEN_FIFO_LVL));

                MXC_FreeLock((uint32_t *)&async_callback);
            }
        }

        if (flags & MXC_F_ADC_REVB_INTEN_CONV_DONE) {
            MXC_ADC_RevB_ClearFlags(adc, MXC_F_ADC_REVB_INTFL_CONV_DONE);
        }

        if (flags) {
            (cb)(NULL, flags);
        }
    }

    return E_NO_ERROR;
}

int MXC_ADC_RevB_GetData(mxc_adc_revb_regs_t *adc, int *outdata)
{
    uint32_t loop_counter, length;

    length = adc->status & MXC_F_ADC_REVB_STATUS_FIFO_LEVEL;
    length = length >> MXC_F_ADC_REVB_STATUS_FIFO_LEVEL_POS;

    for (loop_counter = 0; loop_counter < length; loop_counter++) {
        *outdata = adc->data;
        outdata++;
    }
    return length;
}

void MXC_ADC_RevB_EnableConversion(mxc_adc_revb_regs_t *adc)
{
    adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_START;
}

void MXC_ADC_RevB_DisableConversion(mxc_adc_revb_regs_t *adc)
{
    adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_START;
}

void MXC_ADC_RevB_TS_SelectEnable(mxc_adc_revb_regs_t *adc)
{
    adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_TS_SEL;
}

void MXC_ADC_RevB_TS_SelectDisable(mxc_adc_revb_regs_t *adc)
{
    adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_TS_SEL;
}

uint16_t MXC_ADC_RevB_FIFO_Level(mxc_adc_revb_regs_t *adc)
{
    return ((adc->status & MXC_F_ADC_REVB_STATUS_FIFO_LEVEL) >>
            MXC_F_ADC_REVB_STATUS_FIFO_LEVEL_POS);
}

int MXC_ADC_RevB_FIFO_Threshold_Config(mxc_adc_revb_regs_t *adc, uint32_t fifo_threshold)
{
    if (fifo_threshold > MAX_ADC_FIFO_LEN) {
        return E_BAD_PARAM;
    }

    adc->fifodmactrl &= ~MXC_F_ADC_REVB_FIFODMACTRL_THRESH;
    adc->fifodmactrl |= (uint32_t)(fifo_threshold << MXC_F_ADC_REVB_FIFODMACTRL_THRESH_POS);

    return E_NO_ERROR;
}

int MXC_ADC_RevB_AverageConfig(mxc_adc_revb_regs_t *adc, mxc_adc_avg_t avg_number)
{
    //number of samples to average
    adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_AVG;
    adc->ctrl1 |= (avg_number);

    return E_NO_ERROR;
}

void MXC_ADC_RevB_Clear_ChannelSelect(mxc_adc_revb_regs_t *adc)
{
    //Clear channel select registers
    adc->chsel0 = 0;
    adc->chsel1 = 0;
    adc->chsel2 = 0;
    adc->chsel3 = 0;

#if TARGET_NUM == 32690
    adc->chsel4 = 0;
#endif
}

void MXC_ADC_RevB_TriggerConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req)
{
    if (req->trig == MXC_ADC_TRIG_SOFTWARE) {
        adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_TRIG_MODE;
    } else {
        adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_TRIG_MODE;
        MXC_SETFIELD(adc->ctrl1, MXC_F_ADC_REVB_CTRL1_TRIG_SEL,
                     (req->hwTrig << MXC_F_ADC_REVB_CTRL1_TRIG_SEL_POS));
    }
}

void MXC_ADC_RevB_ConversionModeConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req)
{
    if (req->mode == MXC_ADC_ATOMIC_CONV) {
        adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_CNV_MODE;
    } else {
        adc->ctrl1 |= MXC_F_ADC_REVB_CTRL1_CNV_MODE;
    }
}

int MXC_ADC_RevB_SetConversionDelay(mxc_adc_revb_regs_t *adc, int delay)
{
    if (delay > MXC_F_ADC_REVB_RESTART_CNT) {
        return E_BAD_PARAM;
    }

    adc->restart &= ~MXC_F_ADC_REVB_RESTART_CNT;
    adc->restart |= delay << MXC_F_ADC_REVB_RESTART_CNT_POS;

    return E_NO_ERROR;
}

int MXC_ADC_RevB_SlotsConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req)
{
    if (req->num_slots >= MAX_ADC_SLOT_NUM) {
        return E_BAD_PARAM;
    }

    adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_NUM_SLOTS;
    adc->ctrl1 |= (uint32_t)(req->num_slots) << MXC_F_ADC_REVB_CTRL1_NUM_SLOTS_POS;

    return E_NO_ERROR;
}

//TODO(ADI): Need to find out better way to handle this.
int MXC_ADC_RevB_ChSelectConfig(mxc_adc_revb_regs_t *adc, mxc_adc_chsel_t ch, uint32_t slot_num)
{
    uint32_t *pointer = (uint32_t *)(MXC_BASE_ADC + MXC_R_ADC_CHSEL0);
    uint32_t offset;
    uint32_t bitposition;

    if (slot_num >= MAX_ADC_SLOT_NUM) {
        return E_BAD_PARAM;
    }

    offset = slot_num >> 2;

    bitposition = ch << ((slot_num & 0x03) << 3);

    *(pointer + offset) |= bitposition;

    return E_NO_ERROR;
}

//End

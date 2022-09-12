/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

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
#include "mxc_delay.h"
#include <stdio.h>

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

    //Power up to Sleep State
    adc->clkctrl |= (req->clock << MXC_F_ADC_REVB_CLKCTRL_CLKSEL) & MXC_F_ADC_REVB_CLKCTRL_CLKSEL;

    adc->clkctrl |= (req->clkdiv << MXC_F_ADC_REVB_CLKCTRL_CLKDIV_POS) &
                    MXC_F_ADC_REVB_CLKCTRL_CLKDIV;

    adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_RESETB;

    //Move to NAP state
    adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_BIAS_EN;
    MXC_Delay(500);

    if (req->cal == MXC_ADC_EN_CAL) {
        adc->ctrl0 &= ~MXC_F_ADC_REVB_CTRL0_SKIP_CAL;
    } else {
        adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_SKIP_CAL;
    }

    adc->sampclkctrl |= (req->trackCount << MXC_F_ADC_REVB_SAMPCLKCTRL_TRACK_CNT_POS) &
                        MXC_F_ADC_REVB_SAMPCLKCTRL_TRACK_CNT;
    adc->sampclkctrl |= (req->idleCount << MXC_F_ADC_REVB_SAMPCLKCTRL_IDLE_CNT_POS) &
                        MXC_F_ADC_REVB_SAMPCLKCTRL_IDLE_CNT;

    adc->ctrl0 |= MXC_F_ADC_REVB_CTRL0_ADC_EN;

    //Wait for 30us
    MXC_Delay(30);

    //wait for calibration to complete
    while (!(adc->intfl & MXC_F_ADC_REVB_INTFL_READY)) {}

    async_callback = NULL;

    async_req = NULL;

    return E_NO_ERROR;
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
    adc->clkctrl |= (clock << MXC_F_ADC_REVB_CLKCTRL_CLKSEL) & MXC_F_ADC_REVB_CLKCTRL_CLKSEL;
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

    channel = MXC_DMA_AcquireChannel();

    config.reqsel = MXC_S_DMA_CTRL_REQUEST_ADC;
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

    //TODO: This supports 32 bytes transfer. In MXC_ADC_DATA_STATUS if all channels are used 64 bytes may need to read.
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
            MXC_ADC_RevB_DisableInt(adc, (MXC_F_ADC_REVB_INTFL_SEQ_DONE |
                                          MXC_F_ADC_REVB_INTFL_CONV_DONE |
                                          MXC_F_ADC_REVB_INTEN_FIFO_LVL));
            MXC_FreeLock((uint32_t *)&async_callback);
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

int MXC_ADC_RevB_SlotsConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req)
{
    if (req->num_slots >= MAX_ADC_SLOT_NUM) {
        return E_BAD_PARAM;
    }

    adc->ctrl1 &= ~MXC_F_ADC_REVB_CTRL1_NUM_SLOTS;
    adc->ctrl1 |= (uint32_t)(req->num_slots) << MXC_F_ADC_REVB_CTRL1_NUM_SLOTS_POS;

    return E_NO_ERROR;
}

//TODO: Need to find out better way to handle this.
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

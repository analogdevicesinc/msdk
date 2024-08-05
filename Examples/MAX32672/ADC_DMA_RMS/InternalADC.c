/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
#include "adc.h"
#include "dma.h"
#include "fcr_regs.h"
#include "mcr_regs.h"
#include "mxc_device.h"
#include "nvic_table.h"

#include "Debug.h"
#include "InternalADC.h"

/* Define the data type of the samples. */
#define DMA_SAMPLE_TYPE uint32_t

/* Size of each DMA buffer in bytes */
#define DMA_BUFFER_BYTES (sizeof(DMA_SAMPLE_TYPE) * INTERNAL_ADC_DMA_BLOCK_SIZE)

/* Helper constant to know if a DMA channel isn't allocated */
#define DMA_CHANNEL_INVALID -1

/* MACRO to help bank swapping the DMA buffers */
#define DMA_QUEUE_NEXT(idx) (idx ^ 0x1)

/* Bank swapping DMA buffer allocation */
static DMA_SAMPLE_TYPE adcDMA_Buffers[2][INTERNAL_ADC_DMA_BLOCK_SIZE];

/* Indexes of current buffer uses */
static volatile int activeIndex = 0;
static volatile int reloadIndex = 1;

/* Currently assigned DMA channel */
static int adcDMA_ChannelID = DMA_CHANNEL_INVALID;

/* Configuration of the ADC conversion slot - FTHR AIN3 */
static mxc_adc_slot_req_t adcSingleSlot = { MXC_ADC_CH_3, MXC_ADC_DIV2_5K, MXC_ADC_PY_DN_DISABLE };

/* User callback function for sample blocks */
static ADC_Callback dmaCallback = NULL;

/* Prototypes */
static void InternalADC_ISR(void);
static void InternalADC_DMA_SetupAndStart(void);
static void InternalADC_DMA_Callback(int ch, int err);
static void InternalADC_Reload(void);
static void InternalADC_TrimRef(void);

void InternalADC_Init()
{
    mxc_adc_req_t adcConfig;

    adcConfig.clock = MXC_ADC_CLK_HCLK; //100MHz System Clock
    adcConfig.clkdiv = MXC_ADC_CLKDIV_4; //25MHz, maximum allowed per datasheet
    adcConfig.cal = MXC_ADC_EN_CAL; //Enable calibration values

    //Effective Sample rate 25/51 = 490196
    //Minimum values: Track + Idle = 8 + 17 = 25
    adcConfig.trackCount = 13; //Really is 4 + 13 = 17
    adcConfig.idleCount = 17; //Really is 17 + 17 = 34
    adcConfig.ref = MXC_ADC_REF_INT_2V048;

    /* Initialize ADC */
    if (MXC_ADC_Init(&adcConfig) != E_NO_ERROR) {
        //Error Handling
    }

    //Calibrate the reference voltage
    InternalADC_TrimRef();

    //Enable Interrupts
    MXC_NVIC_SetVector(ADC_IRQn, InternalADC_ISR);
    NVIC_EnableIRQ(ADC_IRQn);
}

void InternalADC_StartSampling(ADC_Callback callback)
{
    mxc_adc_conversion_req_t adcConv;

    if (adcDMA_ChannelID != DMA_CHANNEL_INVALID) {
        //Already running
        return;
    }

    dmaCallback = callback;

    adcConv.mode = MXC_ADC_CONTINUOUS_CONV; //Run forever
    adcConv.trig = MXC_ADC_TRIG_SOFTWARE; //Started by software
    adcConv.avg_number = MXC_ADC_AVG_1; //No averaging
    adcConv.fifo_format = MXC_ADC_DATA; //Data only.
    adcConv.lpmode_divder = MXC_ADC_DIV_2_5K_50K_DISABLE;
    adcConv.num_slots = 0; //1 slot/single channel
    adcConv.fifo_threshold = 0;
    MXC_ADC_Configuration(&adcConv);
    MXC_ADC_SlotConfiguration(&adcSingleSlot, 0);

    //Default the buffer indexes
    activeIndex = 0;
    reloadIndex = 1;

    //Start the DMA conversions
    InternalADC_DMA_SetupAndStart();

    //Reload to prime the next block
    InternalADC_Reload();
}

void InternalADC_StopSampling()
{
    if (adcDMA_ChannelID != DMA_CHANNEL_INVALID) {
        MXC_ADC_DisableConversion();
        MXC_DMA_Stop(adcDMA_ChannelID);
        MXC_DMA_ReleaseChannel(adcDMA_ChannelID);
        NVIC_DisableIRQ(MXC_DMA_CH_GET_IRQ(adcDMA_ChannelID));
        adcDMA_ChannelID = DMA_CHANNEL_INVALID;
    }
}

/**
 * Reloads the DMA to configure the next buffer on completion. This can happen
 * at any point after the current DMA operation starts.
*/
static void InternalADC_Reload()
{
    mxc_dma_srcdst_t srcdst;
    srcdst.ch = adcDMA_ChannelID;
    srcdst.dest = adcDMA_Buffers[reloadIndex];
    srcdst.len = DMA_BUFFER_BYTES;
    MXC_DMA_SetSrcReload(srcdst);
    reloadIndex = DMA_QUEUE_NEXT(reloadIndex);
}

/**
 * ISR for completion of DMA conversions
 */
static void InternalADC_DMA_Callback(int ch, int err)
{
    DBG_IRQ_START(); //GPIO for timing
    //At this point the reloaded ADC DMA conversions should be underway, so go
    //ahead and reload again.
    InternalADC_Reload();

    if (dmaCallback) {
        dmaCallback(adcDMA_Buffers[activeIndex], INTERNAL_ADC_DMA_BLOCK_SIZE);
    }

    activeIndex = DMA_QUEUE_NEXT(activeIndex);
    DBG_IRQ_END(); //GPIO for timing
}

/**
 * This is a custom implementation of the MSDKs MXC_ADC_StartConversionDMA call
 * to support getting multiple samples from a single channel, as well as only
 * grabbing 16 bits of data without channel or status information
*/
#include "adc_revb.h"
//This constant taken from adc_revb.c
#define ADC_IF_MASK                                                                             \
    (MXC_F_ADC_REVB_INTFL_READY | MXC_F_ADC_REVB_INTFL_ABORT | MXC_F_ADC_REVB_INTFL_START_DET | \
     MXC_F_ADC_REVB_INTFL_SEQ_STARTED | MXC_F_ADC_REVB_INTFL_SEQ_DONE |                         \
     MXC_F_ADC_REVB_INTFL_CONV_DONE | MXC_F_ADC_REVB_INTFL_CLIPPED |                            \
     MXC_F_ADC_REVB_INTFL_FIFO_LVL | MXC_F_ADC_REVB_INTFL_FIFO_UFL |                            \
     MXC_F_ADC_REVB_INTFL_FIFO_OFL)
void InternalADC_DMA_SetupAndStart()
{
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;

    // Clear interrupt flags
    MXC_ADC_ClearFlags(ADC_IF_MASK);

    MXC_ADC->fifodmactrl |= MXC_F_ADC_REVB_FIFODMACTRL_FLUSH; //Flush data FIFO
    MXC_ADC->fifodmactrl |= MXC_S_ADC_REVB_FIFODMACTRL_DATA_FORMAT_DATA_ONLY; //Transfer data only
    MXC_ADC->fifodmactrl |= MXC_F_ADC_REVB_FIFODMACTRL_DMA_EN; //Enable ADC DMA

    //Get an ADC Channel
    adcDMA_ChannelID = MXC_DMA_AcquireChannel();

    config.reqsel = MXC_DMA_REQUEST_ADC;
    config.ch = adcDMA_ChannelID;

    //ADC FIFO must be read in 32-bit words per the hardware design
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = adcDMA_ChannelID;
    srcdst.source = NULL;
    srcdst.dest = adcDMA_Buffers[activeIndex];
    srcdst.len = DMA_BUFFER_BYTES;

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(adcDMA_ChannelID, InternalADC_DMA_Callback);

    MXC_DMA->ch[adcDMA_ChannelID].ctrl |= (sizeof(DMA_SAMPLE_TYPE) - 1)
                                          << MXC_F_DMA_CTRL_BURST_SIZE_POS;

    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(adcDMA_ChannelID));
    MXC_DMA_EnableInt(adcDMA_ChannelID);
    MXC_DMA_Start(adcDMA_ChannelID);
    MXC_DMA_SetChannelInterruptEn(adcDMA_ChannelID, 0, 1);

    MXC_ADC->ctrl1 |= MXC_F_ADC_REVB_CTRL1_START;
}

/**
 * Performs the trim/calibration for the internal reference. This procedure was
 * taken from the MAX32672 Users Manual
 */
static void InternalADC_TrimRef()
{
    uint32_t tempData;
    uint32_t tempReg;
    //Enter nap state
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_ADC_EN;

    //Clear Skip Cal
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_SKIP_CAL;

    //Clear ADC Interrupt Flags
    MXC_ADC_ClearFlags(0xFFFFFFFF);

    //The following is for the 2.048V Reference. Procedure would need updating
    //for other references
    MXC_ADC->sfraddr = 0x01;
    tempData = MXC_ADC->sfrrddata;
    tempData |= 0x0F;
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0B;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xC0;
    tempData |= ((MXC_FCR->adcreftrim0 & MXC_F_FCR_ADCREFTRIM0_VX2_TUNE) >>
                 MXC_F_FCR_ADCREFTRIM0_VX2_TUNE_POS);
    MXC_ADC->sfrwrdata = tempData;

    tempReg = 0;
    tempData = (MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VCM) >> MXC_F_FCR_ADCREFTRIM1_VCM_POS;
    tempReg |= ((tempData << MXC_F_MCR_ADC_CFG3_VCM_POS) & MXC_F_MCR_ADC_CFG3_VCM);
    tempData = (MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VREFM) >>
               MXC_F_FCR_ADCREFTRIM1_VREFM_POS;
    tempReg |= ((tempData << MXC_F_MCR_ADC_CFG3_VREFM_POS) & MXC_F_MCR_ADC_CFG3_VREFM);
    tempData = (MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VREFP) >>
               MXC_F_FCR_ADCREFTRIM1_VREFP_POS;
    tempReg |= ((tempData << MXC_F_MCR_ADC_CFG3_VREFP_POS) & MXC_F_MCR_ADC_CFG3_VREFP);
    tempData = (MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048) >>
               MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048_POS;
    tempReg |= ((tempData << MXC_F_MCR_ADC_CFG3_D_IBOOST_POS) & MXC_F_MCR_ADC_CFG3_D_IBOOST);
    tempData = (MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_IDRV_2P048) >>
               MXC_F_FCR_ADCREFTRIM2_IDRV_2P048_POS;
    tempReg |= ((tempData << MXC_F_MCR_ADC_CFG3_IDRV_POS) & MXC_F_MCR_ADC_CFG3_IDRV);
    MXC_MCR->adc_cfg3 = tempReg;

    MXC_ADC->sfraddr = 0x05;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= 6; //Closest value for 500ksps
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x06;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= 6; //Closest value for 500ksps
    MXC_ADC->sfrwrdata = tempData;

    //Enable
    MXC_ADC->ctrl0 |= MXC_F_ADC_CTRL0_ADC_EN;

    //wait for calibration to complete
    while (!(MXC_ADC->intfl & MXC_F_ADC_INTFL_READY)) {}
}

/**
 * ADC ISR handler. Just clear the flags since DMA drives the real data
 */
static void InternalADC_ISR()
{
    MXC_ADC_ClearFlags(0xFFFFFFFF);
}

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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "spi.h"
#include "tmr.h"

#include "Debug.h"
#include "InternalADC.h"
#include "MovingAverage.h"
#include "RMS.h"

/* Size of the moving average output filter */
#define MA_WINDOW_SIZE 8

/* Number of bits to RSH the RMS when determining a MA reset threshold */
#define RESET_THRESH_SHIFT 3 // 1/8th

/* New output data is ready for processing */
static volatile int dataReady = 0;

/* Track the number of ADC interrupts, for testing/info */
static volatile uint32_t irqCount = 0;

/* Track if we are taking too long in the main loop */
static volatile uint32_t overflowCount = 0;

/* Moving average buffer and instance */
static uint16_t movingAvgWindow[MA_WINDOW_SIZE];
static moving_average_instance_t movingAvgInstance;

/* RMS Instance */
rms_instance_t rmsInstance;

/* Platform support for DMA in the MSDK */
static void InitializeDMA(void);
static void GlobalDMA_ISR_Handler(void);

/* Callback from the internal ADC */
static void ADC_Handler(uint32_t *dataBuffer, uint32_t count);

/* Prototypes if using the AD5592 for Output*/
#ifdef AD5592_OUTPUT
static void InitializeAD5592(void);
static void AD5592_WriteDAC(uint8_t ch, uint16_t localvalue);
#endif

/**
 * Main entry point into the application. Configures the system and runs the
 * main loop
 */
int main(void)
{
    uint16_t lastRms = 0;

    //Setup the hardware
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);

    DebugInitialize();
    InitializeDMA();

#ifdef AD5592_OUTPUT
    InitializeAD5592();
    AD5592_WriteDAC(0, 0);
    AD5592_WriteDAC(1, 0);
#endif

    RMS_Initialize(&rmsInstance);
    MovingAverageInitialize(&movingAvgInstance, movingAvgWindow, MA_WINDOW_SIZE);

    //Initialize the internal ADC
    InternalADC_Init();
    InternalADC_StartSampling(ADC_Handler);

    while (1) //Loop forever
    {
        //In a RTOS or other more complete application, wait for a flag here
        while (dataReady == 0) {}

        //Determine if the MA should be reset by taking the delta, and comparing
        //it to the threshold
        if (abs((int32_t)rmsInstance.lastRMS - (int32_t)lastRms) >
            (rmsInstance.lastRMS >> RESET_THRESH_SHIFT)) {
            MovingAverageReset(&movingAvgInstance);
        }
        lastRms = rmsInstance.lastRMS;

        MovingAverageFilter(&movingAvgInstance, rmsInstance.lastRMS);

#ifdef CONSOLE_OUTPUT
        printf("%d %d\n", rmsInstance.lastRMS, movingAvgInstance.lastVal);
#endif

#ifdef AD5592_OUTPUT
        AD5592_WriteDAC(0, rmsInstance.lastRMS);
        AD5592_WriteDAC(1, movingAvgInstance.lastVal);
#endif

        dataReady = 0;
    }
    return 0;
}

/**
 * Handles a new block of data coming from the ADC.  Performs the RMS filter
 * operation directly.
 *
 * It is expected (but not required) this is called from an interrupt context
 *
 * @param dataBuffer - Buffer of sample data
 * @param count - Number of samples
*/
static void ADC_Handler(uint32_t *dataBuffer, uint32_t count)
{
    irqCount++;

    if (dataReady) {
        overflowCount++;
    }

    DBG_SQ_START(); //GPIO for timing analysis
    if (RMS_ProcessSamples(&rmsInstance, dataBuffer, count)) {
        //Notify the main loop
        dataReady = 1;
    }
    DBG_SQ_END(); //GPIO for timing analysis
}

/**
 * Initializes all the DMA channels to support interrupts, routing through the
 * handler into the MSDK
 */
static void InitializeDMA()
{
    int i;

    MXC_DMA_Init();

    /* Configure all DMA channels. */
    for (i = 0; i < MXC_DMA_CHANNELS; i++) {
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(i), GlobalDMA_ISR_Handler);
    }
}

/**
 * DMA/DMA0 Interrupt handler connected to the NVIC. Call the MXC DMA handler
 */
static void GlobalDMA_ISR_Handler()
{
    MXC_DMA_Handler();
}

#ifdef AD5592_OUTPUT
/* Need to re-define these since we may need a different drive strength for the bus */
static const mxc_gpio_cfg_t spiPins = { MXC_GPIO0,
                                        (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 |
                                         MXC_GPIO_PIN_17),
                                        MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIOH,
                                        MXC_GPIO_DRVSTR_2 };

/* Define the SPI instance to use */
#define AD5592_SPI_INST MXC_SPI1
#define AD5592_SPI_CS_IDX 0

/* Static SPI request struct to ensure persistance between function calls */
static mxc_spi_req_t ad5592SpiReq;

/**
 * Writes a word (16-bits) to the AD5592
 *
 * @param data - Word to write
*/
static void AD5592_SPI_Write(uint16_t data)
{
    uint8_t dataBytes[2];

    //Big endian format
    dataBytes[0] = ((data >> 8) & 0xFF);
    dataBytes[1] = (data & 0xFF);

    ad5592SpiReq.spi = AD5592_SPI_INST;
    ad5592SpiReq.ssIdx = AD5592_SPI_CS_IDX;
    ad5592SpiReq.ssDeassert = 1;
    ad5592SpiReq.rxData = 0;
    ad5592SpiReq.txData = dataBytes;
    ad5592SpiReq.txCnt = 0;
    ad5592SpiReq.rxCnt = 0;
    ad5592SpiReq.txLen = 2;
    ad5592SpiReq.rxLen = 0;
    ad5592SpiReq.completeCB = NULL;

    MXC_SPI_MasterTransaction(&ad5592SpiReq);
}

/**
 * Initializes the AD5592 and the associated SPI interface
 */
static void InitializeAD5592()
{
    uint16_t regData;

    /* Configure the SPI port */
    MXC_SPI_Init(AD5592_SPI_INST, 1, 0, 1, 0, 5000000); //Master, Not Quad, 1 Slave, 0 Polarity
    MXC_SPI_SetWidth(AD5592_SPI_INST, SPI_WIDTH_STANDARD);
    MXC_SPI_SetDataSize(AD5592_SPI_INST, 8);
    MXC_SPI_SetMode(AD5592_SPI_INST, SPI_MODE_2);
    MXC_GPIO_Config(&spiPins);

    //Power Down/Ref Ctrl - Internal Reference Powered Up
    regData = (0xB << 11) | (1 << 9);
    AD5592_SPI_Write(regData);

    //DAC Config, Ch 0 & 1 are outputs
    regData = (0x5 << 11) | (0x3);
    AD5592_SPI_Write(regData);
}

/**
 * Writes the value out the AD5592 channel.  It is assumed the value passed to
 * this function is scaled to the INTERNAL ADC scale factor.  This function
 * will rescale the value to the AD5592's reference so can easily be compared
 * on a scope.
 *
 * @param ch - Channel to write
 * @param localvalue - Value (scaled to internal ADC)
 */
static void AD5592_WriteDAC(uint8_t ch, uint16_t localvalue)
{
    uint32_t value;

    //Our internal ADC is configured for 4.096 volts full scale, AD5592r is 2.5
    //Adjust accordingly.
    value = (localvalue * 4096) / (2500);
    AD5592_SPI_Write((0x1 << 15) | (ch << 12) | (value & 0xFFF));
}
#endif //AD5592_OUTPUT

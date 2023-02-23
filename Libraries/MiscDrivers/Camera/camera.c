/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mxc_device.h"
#include "board.h"
#include "nvic_table.h"
#include "pt.h"
#include "cameraif.h"
#include "dma.h"
#include "uart.h"
#include "mxc_delay.h"
#include "gpio.h"
#include "camera.h"
#include "sccb.h"
#include "dma_regs.h"
#include "spi_regs.h"
#include "spi.h"

/*******************************      DEFINES      ***************************/
#define FIFO_THRES_HOLD (4)
#define PCIF_DATA_BUS_WITH MXC_S_CAMERAIF_CTRL_DATA_WIDTH_8BIT
#define CAMERA_STARTUP_DELAY (300)
#define CAMERA_DATA_BIT_WIDTH MXC_PCIF_GPIO_DATAWIDTH_8_BIT
//#define TFT_DMA_DEBUG //uncomment to enable the debug GPIOs P1.6 and P3.1
/******************************** Static Functions ***************************/

static uint8_t *rx_data = NULL;
static volatile uint32_t rx_data_index = 0;
static volatile uint32_t g_is_img_rcv = 0;
static int g_total_img_size = 0;
static int g_framesize_width = 64;
static int g_framesize_height = 64;
static int g_stream_buffer_size = 64;
static int g_pixel_format = PIXFORMAT_RGB888;
static fifomode_t g_fifo_mode = FIFO_THREE_BYTE;
static dmamode_t g_dma_mode = NO_DMA;
static int g_dma_channel = 0;
static int g_dma_channel_tft = 1;
static int g_dma_already_setup = 0;
static camera_t camera;
extern int sensor_register(camera_t *camera);

//----------------------------------------
// PCIF ISR
//----------------------------------------
void camera_irq_handler(void)
{
    uint32_t i;
    uint32_t data;
    uint32_t step_size;

    if (g_dma_mode == NO_DMA) {
        // Check fifo threshold flag.
        if (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_THRESH) {
            for (i = 0; i < FIFO_THRES_HOLD; i++) {
                data = MXC_PCIF->fifo_data;

                rx_data[rx_data_index + 0] = data >> 0;
                rx_data[rx_data_index + 1] = data >> 8;
                rx_data[rx_data_index + 2] = data >> 16;

                if (g_fifo_mode == FIFO_FOUR_BYTE) {
                    rx_data[rx_data_index + 3] = data >> 24;
                    step_size = 4;
                } else {
                    step_size = 3;
                }

                if ((rx_data_index + step_size) < g_total_img_size) {
                    rx_data_index += step_size; // increase it only if there is free space
                }
            }
        }
    }

    // check img_done flag
    if (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_IMG_DONE) {
        /* Flush the fifo. */
        if (g_dma_mode == NO_DMA) {
            while (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_NOT_EMPTY) {
                data = MXC_PCIF->fifo_data;
                MXC_PCIF->int_fl = MXC_F_CAMERAIF_INT_FL_FIFO_NOT_EMPTY;

                rx_data[rx_data_index + 0] = data >> 0;
                rx_data[rx_data_index + 1] = data >> 8;
                rx_data[rx_data_index + 2] = data >> 16;

                if (g_fifo_mode == FIFO_FOUR_BYTE) {
                    rx_data[rx_data_index + 3] = data >> 24;
                    step_size = 4;
                } else {
                    step_size = 3;
                }

                if ((rx_data_index + step_size) < g_total_img_size) {
                    rx_data_index += step_size; // increase it only if there is free space
                }
            }
        } else {
            while (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_NOT_EMPTY) {
                data = MXC_PCIF->fifo_data;
                MXC_PCIF->int_fl = MXC_F_CAMERAIF_INT_FL_FIFO_NOT_EMPTY;
            }

            rx_data_index = g_total_img_size;
        }

        g_is_img_rcv = 1;
    }

    // clear flags, tmp flag is used to pass coverity check
    uint32_t flags = MXC_PCIF->int_fl;
    MXC_PCIF->int_fl = flags; // clear flags
}

void stream_irq_handler(void)
{
    MXC_DMA_Handler();
}

#ifdef __riscv
void __attribute__((interrupt("machine"))) PCIF_IRQHandler(void)
{
    camera_irq_handler();
    NVIC_ClearPendingIRQ(PCIF_IRQn);
}

void __attribute__((interrupt("machine"))) DMA0_IRQHandler(void)
{
    stream_irq_handler();
    NVIC_ClearPendingIRQ(DMA0_IRQn);
}
#endif

static stream_stat_t statistic;
static volatile uint32_t current_stream_buffer = 0;
static uint8_t *stream_buffer_ptr = NULL;

#ifdef TFT_DMA_DEBUG
const mxc_gpio_cfg_t debug_pin[] = {
        { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
        { MXC_GPIO3, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_debugs = (sizeof(debug_pin) / sizeof(mxc_gpio_cfg_t));

int debug_Init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;

    /* setup 2 GPIOs for the debug */
    for (i = 0; i < num_debugs; i++) {
        if (MXC_GPIO_Config(&debug_pin[i]) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }

    return retval;
}
#endif

static void stream_callback(int a, int b)
{
    if (MXC_DMA->ch[g_dma_channel].status & MXC_F_DMA_STATUS_CTZ_IF) {
        MXC_DMA->ch[g_dma_channel].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag

        // Check current streaming buffer and reconfigure DMA
        if (current_stream_buffer) {
            // Set buffer[0] for next DMA transfer
            MXC_DMA->ch[g_dma_channel].dst = (uint32_t)rx_data;

            // Set current streaming buffer[1] as full, otherwise report overflow
            if (stream_buffer_ptr == NULL) {
                stream_buffer_ptr = rx_data + g_stream_buffer_size;
            } else {
                statistic.overflow_count++;
            }
        } else {
            // Set buffer[1] for next DMA transfer
            MXC_DMA->ch[g_dma_channel].dst = (uint32_t)(rx_data + g_stream_buffer_size);

            // Set current streaming buffer[0] as full, otherwise report overflow
            if (stream_buffer_ptr == NULL) {
                stream_buffer_ptr = rx_data;
            } else {
                statistic.overflow_count++;
            }
        }

        // Alternate streaming buffers
        current_stream_buffer ^= 1;
        // Set DMA counter
        MXC_DMA->ch[g_dma_channel].cnt = g_stream_buffer_size;
        // Re-enable DMA channel
        MXC_DMA->ch[g_dma_channel].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);

        statistic.dma_transfer_count++;
    }
}

#ifndef __riscv
static void stream_callback_tft(int a, int b)
{
	if (MXC_DMA->ch[g_dma_channel].status & MXC_F_DMA_STATUS_CTZ_IF) {
        MXC_DMA->ch[g_dma_channel].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag

        // Check current streaming buffer and reconfigure DMA
        if (current_stream_buffer) {
#ifdef TFT_DMA_DEBUG
            MXC_GPIO_OutSet(MXC_GPIO3, MXC_GPIO_PIN_1); //debug GPIO
#endif
            // Set buffer[0] for next DMA transfer 
            MXC_DMA->ch[g_dma_channel].dst = (uint32_t)rx_data;
        	// wait until TFT is done
            while((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS));
            if (MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_CTZ_IF)
            	MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF;
            //MXC_DMA->ch[g_dma_channel].dst = (uint32_t)rx_data;
            MXC_DMA->ch[g_dma_channel_tft].cnt = g_stream_buffer_size;
            MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)(rx_data + g_stream_buffer_size);
            MXC_DMA->ch[g_dma_channel_tft].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
            MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;

            // Set current streaming buffer[1] as full, otherwise report overflow
            if (stream_buffer_ptr == NULL) {
                stream_buffer_ptr = rx_data + g_stream_buffer_size;
            }
            else {
                statistic.overflow_count++;
            }
        }
        else {
#ifdef TFT_DMA_DEBUG
            MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_6);  //debug GPIO
#endif
            // Set buffer[1] for next DMA transfer
            MXC_DMA->ch[g_dma_channel].dst = (uint32_t)(rx_data + g_stream_buffer_size);
        	// wait until TFT is done
			while((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS));
			if (MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_CTZ_IF)
				MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF;
			//MXC_DMA->ch[g_dma_channel].dst = (uint32_t)(rx_data + g_stream_buffer_size);
			MXC_DMA->ch[g_dma_channel_tft].cnt = g_stream_buffer_size;
			MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)rx_data;
			MXC_DMA->ch[g_dma_channel_tft].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
			MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;

			// Set current streaming buffer[0] as full, otherwise report overflow
            if (stream_buffer_ptr == NULL) {
                stream_buffer_ptr  = rx_data;
            }
            else {
                statistic.overflow_count++;
            }
        }

        // Alternate streaming buffers
        current_stream_buffer ^= 1;
        // Set DMA counter
        MXC_DMA->ch[g_dma_channel].cnt = g_stream_buffer_size;

        // Re-enable DMA channel
        MXC_DMA->ch[g_dma_channel].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);

        statistic.dma_transfer_count++;
#ifdef TFT_DMA_DEBUG
        MXC_GPIO_OutClr(MXC_GPIO3, MXC_GPIO_PIN_1);
        MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_6);
#endif
    }
}
#endif

static void setup_dma(void)
{
    MXC_DMA->ch[g_dma_channel].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_channel].dst = (uint32_t)rx_data; // Cast Pointer

    if (g_dma_mode == STREAMING_DMA) {
        if (PCIF_DATA_BUS_WITH == MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT) {
            MXC_DMA->ch[g_dma_channel].cnt = g_stream_buffer_size;
        } else {
            MXC_DMA->ch[g_dma_channel].cnt =
                g_stream_buffer_size * 2; // 10 and 12 bit use 2 bytes per word in the fifo
        }

        // Set the initial streaming buffer to 1
        current_stream_buffer = 1;
        MXC_DMA->ch[g_dma_channel].dst = (uint32_t)(rx_data + g_stream_buffer_size);

        stream_buffer_ptr = NULL;
        statistic.dma_transfer_count = 0;

    } else {
        if (PCIF_DATA_BUS_WITH == MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT) {
            MXC_DMA->ch[g_dma_channel].cnt = g_total_img_size;
        } else {
            MXC_DMA->ch[g_dma_channel].cnt =
                g_total_img_size * 2; // 10 and 12 bit use 2 bytes per word in the fifo
        }
    }

    MXC_DMA->ch[g_dma_channel].ctrl =
        ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS) + (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS) +
         (0x3 << MXC_F_DMA_CTRL_BURST_SIZE_POS) + (0x1 << MXC_F_DMA_CTRL_DSTINC_POS) +
         (0x2 << MXC_F_DMA_CTRL_DSTWD_POS) + (0x0 << MXC_F_DMA_CTRL_SRCINC_POS) +
         (0x2 << MXC_F_DMA_CTRL_SRCWD_POS) + (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS) +
         (0x0 << MXC_F_DMA_CTRL_TO_WAIT_POS) + (0xD << MXC_F_DMA_CTRL_REQUEST_POS) + // From PCIF_RX
         (0x0 << MXC_F_DMA_CTRL_PRI_POS) + // High Priority
         (0x0 << MXC_F_DMA_CTRL_RLDEN_POS) + // Reload disabled
         (0x1 << MXC_F_DMA_CTRL_EN_POS)); // Enable DMA channel

    MXC_DMA->inten |= (1 << g_dma_channel);
}

#ifndef __riscv
static void setup_dma_tft(void)
{
    MXC_DMA->ch[g_dma_channel].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_channel].dst = (uint32_t) rx_data; // Cast Pointer
    // TFT DMA
    MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_channel_tft].dst = (uint32_t) rx_data; // Cast Pointer

    if (g_dma_mode == STREAMING_DMA) {

    	//camera DMA
    	if (PCIF_DATA_BUS_WITH == MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT) {
            MXC_DMA->ch[g_dma_channel].cnt = g_stream_buffer_size;
        }
        else {
            // 10 and 12 bit use 2 bytes per word in the fifo
            MXC_DMA->ch[g_dma_channel].cnt = g_stream_buffer_size * 2; 
        }

        // Set the initial streaming buffer to 1
        current_stream_buffer = 1 ;
        MXC_DMA->ch[g_dma_channel].dst = (uint32_t)(rx_data + g_stream_buffer_size);

        stream_buffer_ptr = NULL;
        statistic.dma_transfer_count = 0;

        //TFT DMA
		if (PCIF_DATA_BUS_WITH == MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT) {
			MXC_DMA->ch[g_dma_channel_tft].cnt = g_stream_buffer_size;
		}
		else {
			// 10 and 12 bit use 2 bytes per word in the fifo
            MXC_DMA->ch[g_dma_channel_tft].cnt = g_stream_buffer_size * 2; 
		}

		MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)(rx_data + g_stream_buffer_size);

		//stream_buffer_ptr = NULL;
		//statistic.dma_transfer_count = 0;

    }
    else {
        if (PCIF_DATA_BUS_WITH == MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT) {
            MXC_DMA->ch[g_dma_channel].cnt = g_total_img_size;
        }
        else {
            // 10 and 12 bit use 2 bytes per word in the fifo
            MXC_DMA->ch[g_dma_channel].cnt = g_total_img_size * 2;
        }
    }

    MXC_DMA->ch[g_dma_channel].ctrl = ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS)  +
                                       (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS)  +
                                       (0x3 << MXC_F_DMA_CTRL_BURST_SIZE_POS) +
                                       (0x1 << MXC_F_DMA_CTRL_DSTINC_POS)  +
                                       (0x2 << MXC_F_DMA_CTRL_DSTWD_POS)   +
                                       (0x0 << MXC_F_DMA_CTRL_SRCINC_POS)  +
                                       (0x2 << MXC_F_DMA_CTRL_SRCWD_POS)   +
                                       (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS) +
                                       (0x0 << MXC_F_DMA_CTRL_TO_WAIT_POS) +
                                       (0xD << MXC_F_DMA_CTRL_REQUEST_POS) +  // From PCIF_RX
                                       (0x0 << MXC_F_DMA_CTRL_PRI_POS)     +  // High Priority
                                       (0x0 << MXC_F_DMA_CTRL_RLDEN_POS)   +  // Reload disabled
                                       (0x1 << MXC_F_DMA_CTRL_EN_POS)         // Enable DMA channel
                                      );

    MXC_DMA->ch[g_dma_channel_tft].ctrl = ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS)  +
                                       (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS)  +
                                       (0x1 << MXC_F_DMA_CTRL_BURST_SIZE_POS) +
                                       (0x0 << MXC_F_DMA_CTRL_DSTINC_POS)  +
                                       (0x1 << MXC_F_DMA_CTRL_DSTWD_POS)   +
                                       (0x1 << MXC_F_DMA_CTRL_SRCINC_POS)  +
                                       (0x1 << MXC_F_DMA_CTRL_SRCWD_POS)   +
                                       (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS) +
                                       (0x0 << MXC_F_DMA_CTRL_TO_WAIT_POS) +
                                       (0x2F << MXC_F_DMA_CTRL_REQUEST_POS) +  // To SPI0 -> TFT
                                       (0x0 << MXC_F_DMA_CTRL_PRI_POS)     +  // High Priority
                                       (0x0 << MXC_F_DMA_CTRL_RLDEN_POS)   //+  // Reload disabled
                                      // (0x1 << MXC_F_DMA_CTRL_EN_POS)         // Enable DMA channel
                                      );
    MXC_SPI0->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);
    MXC_SETFIELD(MXC_SPI0->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR, (g_total_img_size/4) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    MXC_SPI0->dma   |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);
        // QSPIn port is enabled
    //MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);

        // Clear master done flag
    MXC_SPI0->intfl = MXC_F_SPI_INTFL_MST_DONE;
    MXC_SETFIELD (MXC_SPI0->dma, MXC_F_SPI_DMA_TX_THD_VAL, 0x10 << MXC_F_SPI_DMA_TX_THD_VAL_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_DMA_TX_EN);
    MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);
    MXC_DMA->inten |= (1 << g_dma_channel);
#ifdef TFT_DMA_DEBUG
    debug_Init();
    MXC_GPIO_OutClr(MXC_GPIO3, MXC_GPIO_PIN_1);
    MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_6);
#endif
    g_dma_already_setup = 1;
}
#endif
/******************************** Public Functions ***************************/
int camera_init(uint32_t freq)
{
    int ret = 0;
    mxc_pt_cfg_t pt_cfg;

    Camera_Power(1);

    // initialize XCLK for camera
    MXC_PT_Init(MXC_PT_CLK_DIV1);

    // Camera requires a delay after starting its input clock.  Since a
    // pulse train is already used for the intput clock, that same
    // pulse train can also be used to time the delay.  Once the pulse train
    // completes the necessary number of pulses for the delay, switch to
    // continuous pulse train mode.
    pt_cfg.channel = 0;
    pt_cfg.bps = freq * 2;
    pt_cfg.pattern = 0x55555555;
    pt_cfg.ptLength = 0;
    pt_cfg.loop = (((freq / 1000) * CAMERA_STARTUP_DELAY) + 15) / 16;
    pt_cfg.loopDelay = 0;
    MXC_PT_Config(&pt_cfg);
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT0);

    MXC_GPIO_SetVSSEL(gpio_cfg_pt0.port, MXC_GPIO_VSSEL_VDDIOH, gpio_cfg_pt0.mask);

    // Camera requires a delay after starting its input clock.
    while (MXC_PT_IsActive(1)) {}

    MXC_PT_SqrWaveConfig(0, freq);
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT0);

    MXC_PCIF->ctrl |= 0;
    // Initialize serial camera communication bus.
    sccb_init();
    // Register functions
    sensor_register(&camera);

    ret = camera.init();

    if (ret == 0) {
        ret |= camera.reset();
        ret |= camera.set_hmirror(1);
        ret |= camera.set_vflip(1);
    }

    if (ret == 0) {
        MXC_PCIF_Init(CAMERA_DATA_BIT_WIDTH);

        MXC_GPIO_SetVSSEL(gpio_cfg_pcif_vsync.port, MXC_GPIO_VSSEL_VDDIOH,
                          gpio_cfg_pcif_vsync.mask);
        MXC_GPIO_SetVSSEL(gpio_cfg_pcif_hsync.port, MXC_GPIO_VSSEL_VDDIOH,
                          gpio_cfg_pcif_hsync.mask);
        MXC_GPIO_SetVSSEL(gpio_cfg_pcif_P0_BITS_0_7.port, MXC_GPIO_VSSEL_VDDIOH,
                          gpio_cfg_pcif_P0_BITS_0_7.mask);
        MXC_GPIO_SetVSSEL(gpio_cfg_pcif_xclk.port, MXC_GPIO_VSSEL_VDDIOH, gpio_cfg_pcif_xclk.mask);

        // Default to reading 8 bits of data from the camera.
        MXC_PCIF_SetDataWidth(MXC_PCIF_DATAWIDTH_8_BIT);
        MXC_PCIF_SetTimingSel(MXC_PCIF_TIMINGSEL_HSYNC_and_VSYNC);
        MXC_PCIF_SetThreshold(FIFO_THRES_HOLD);

        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_PCIF_SYS, MXC_F_CAMERAIF_CTRL_PCIF_SYS);

        MXC_PCIF_EnableInt(MXC_F_CAMERAIF_INT_EN_IMG_DONE);
#ifndef __riscv
        MXC_NVIC_SetVector(PCIF_IRQn, camera_irq_handler);
#else
        __enable_irq();
        NVIC_EnableIRQ(PCIF_IRQn);
#endif
    }

    return ret;
}

int camera_reset(void)
{
    return camera.reset();
}

int camera_sleep(int enable)
{
    return camera.sleep(enable);
}

int camera_setup(int xres, int yres, pixformat_t pixformat, fifomode_t fifo_mode,
                 dmamode_t dma_mode, int dma_channel)
{
    int ret = STATUS_OK;
    int bytes_per_pixel = 2;

    // Setup fifo mode.
    g_fifo_mode = fifo_mode;

    switch (g_fifo_mode) {
    case FIFO_THREE_BYTE: // data is 3 bytes in FIFO, it will be converted to 32-bit with MSB set to zero
        MXC_PCIF->ctrl |= MXC_F_CAMERAIF_CTRL_THREE_CH_EN; // CNN mode enabled
        break;

    case FIFO_FOUR_BYTE: // data is 4 bytes in FIFO, no need to convert to 32-bit
        MXC_PCIF->ctrl &= ~MXC_F_CAMERAIF_CTRL_THREE_CH_EN; // CNN mode disabled

        if (pixformat == PIXFORMAT_RGB888) // cannot be 4 bytes in FIFO in RGB888 case
            return -1;
        break;

    default:
        return -1;
    }

    // Setup DMA.
    g_dma_mode = dma_mode;

    g_pixel_format = pixformat;

    // Setup hardware bit expansion from rgb565 to rgb888 conversion.
    // If enabled then hardware will read in rgb565 from the camera
    // and treat it as rgb888.
    if (pixformat == PIXFORMAT_RGB888) {
        MXC_PCIF_SetDataWidth(MXC_PCIF_DATAWIDTH_12_BIT);

        // Bit expansion mode will yield three bytes per pixel.
        if ((g_dma_mode == USE_DMA) || (g_dma_mode == STREAMING_DMA)) {
            bytes_per_pixel = 4;
        } else {
            bytes_per_pixel = 3;
        }
    } else if (pixformat == PIXFORMAT_BAYER) {
        bytes_per_pixel = 1;
    }

    // Setup camera resolution and allocate a camera frame buffer.
    g_framesize_width = xres;
    g_framesize_height = yres;
    // Calculate the number of bytes for the frame buffer.
    g_total_img_size = g_framesize_width * g_framesize_height * bytes_per_pixel;

    // Free up memory if a camera frame buffer was previously allocated.
    if (rx_data != NULL) {
        free(rx_data);
        rx_data = NULL;
    }

    if (g_dma_mode == STREAMING_DMA) {
        statistic.overflow_count = 0;
        // Stream buffer size
        g_stream_buffer_size = g_framesize_width * bytes_per_pixel;

        // Allocate memory with a buffer just to keep two horizontal
        // lines of a camera image for CNN streaming
        rx_data = (uint8_t *)malloc(2 * g_stream_buffer_size);

        // Register streaming callback function
        MXC_DMA_SetCallback(dma_channel, stream_callback);

#ifndef __riscv
        //MXC_NVIC_SetVector(DMA0_IRQn, stream_irq_handler);
        switch (dma_channel) {
        case 0:
            MXC_NVIC_SetVector(DMA0_IRQn, stream_irq_handler);
            break;
        case 1:
            MXC_NVIC_SetVector(DMA1_IRQn, stream_irq_handler);
            break;
        case 2:
            MXC_NVIC_SetVector(DMA2_IRQn, stream_irq_handler);
            break;
        case 3:
            MXC_NVIC_SetVector(DMA3_IRQn, stream_irq_handler);
            break;
        default:
            printf("DMA channel not supported!\n");
            while (1) {}
        }

#else
        NVIC_EnableIRQ(DMA0_IRQn);
#endif
    } else {
        // Allocate memory with a buffer large enough for a camera frame.
        rx_data = (uint8_t *)malloc(g_total_img_size);
    }

    if (rx_data == NULL) {
        // Return error if unable to allocate memory.
        return STATUS_ERROR_ALLOCATING;
    }

    // Initialize buffer
    if (g_dma_mode == STREAMING_DMA) {
        memset((uint8_t *)rx_data, 0xff, 2 * g_stream_buffer_size);
    } else {
        memset((uint8_t *)rx_data, 0xff, g_total_img_size);
    }

    if ((g_dma_mode == USE_DMA) || (g_dma_mode == STREAMING_DMA)) {
        g_dma_channel = dma_channel;
        setup_dma();
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA_THRSH,
                     (0x1 << MXC_F_CAMERAIF_CTRL_RX_DMA_THRSH_POS));
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA, MXC_F_CAMERAIF_CTRL_RX_DMA);
        MXC_PCIF_DisableInt(MXC_F_CAMERAIF_INT_EN_FIFO_THRESH);
    } else {
        // Slow down clock if not using dma
#if defined(CAMERA_OV7692)
        ret |= camera.write_reg(0x11, 0x4); // clock prescaler
#endif
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA, 0x0);
        MXC_PCIF_EnableInt(MXC_F_CAMERAIF_INT_EN_FIFO_THRESH);
    }

    camera.set_pixformat(pixformat);
    camera.set_framesize(g_framesize_width, g_framesize_height);

    return ret;
}

#ifndef __riscv
int camera_setup_tft(int xres, int yres, pixformat_t pixformat, fifomode_t fifo_mode, dmamode_t dma_mode, int dma_channel)
{
    int ret = STATUS_OK;
    int bytes_per_pixel = 2;

    // Setup fifo mode.
    g_fifo_mode = fifo_mode;

    switch (g_fifo_mode) {
    case FIFO_THREE_BYTE: // data is 3 bytes in FIFO, it will be converted to 32-bit with MSB set to zero
        MXC_PCIF->ctrl |= MXC_F_CAMERAIF_CTRL_THREE_CH_EN;  // CNN mode enabled
        break;

    case FIFO_FOUR_BYTE: // data is 4 bytes in FIFO, no need to convert to 32-bit
        MXC_PCIF->ctrl &= ~MXC_F_CAMERAIF_CTRL_THREE_CH_EN; // CNN mode disabled

        if (pixformat == PIXFORMAT_RGB888)  // cannot be 4 bytes in FIFO in RGB888 case
        	return -1;
        break;

    default:
        return -1;
    }

    // Setup DMA.
    g_dma_mode = dma_mode;

    g_pixel_format = pixformat;

    // Setup hardware bit expansion from rgb565 to rgb888 conversion.
    // If enabled then hardware will read in rgb565 from the camera
    // and treat it as rgb888.
    if (pixformat == PIXFORMAT_RGB888) {
        MXC_PCIF_SetDataWidth(MXC_PCIF_DATAWIDTH_12_BIT);

        // Bit expansion mode will yield three bytes per pixel.
        if ((g_dma_mode == USE_DMA) || (g_dma_mode == STREAMING_DMA)) {
            bytes_per_pixel = 4;
        }
        else {
            bytes_per_pixel = 3;
        }
    }
    else if (pixformat == PIXFORMAT_BAYER) {
        bytes_per_pixel = 1;
    }

    // Setup camera resolution and allocate a camera frame buffer.
    g_framesize_width = xres;
    g_framesize_height = yres;
    // Calculate the number of bytes for the frame buffer.
    g_total_img_size = g_framesize_width * g_framesize_height * bytes_per_pixel;

    // Free up memory if a camera frame buffer was previously allocated.
    if (rx_data != NULL) {
        free(rx_data);
        rx_data = NULL;
    }

    if (g_dma_mode == STREAMING_DMA) {
        statistic.overflow_count = 0;
        // Stream buffer size
        g_stream_buffer_size = g_framesize_width * bytes_per_pixel;

        // Allocate memory with a buffer just to keep two horizontal
        // lines of a camera image for CNN streaming
        rx_data = (uint8_t*)malloc(2 * g_stream_buffer_size);

        // Register streaming callback function
        MXC_DMA_SetCallback(dma_channel, stream_callback_tft);

#ifndef __riscv
        //MXC_NVIC_SetVector(DMA0_IRQn, stream_irq_handler);
        switch(dma_channel)
        {
        	case 0:
        		 MXC_NVIC_SetVector(DMA0_IRQn, stream_irq_handler);
        		 break;
        	case 1:
        	     MXC_NVIC_SetVector(DMA1_IRQn, stream_irq_handler);
        	     break;
        	case 2:
        	     MXC_NVIC_SetVector(DMA2_IRQn, stream_irq_handler);
        	     break;
        	case 3:
        	     MXC_NVIC_SetVector(DMA3_IRQn, stream_irq_handler);
        	     break;
        	default:
        		printf("DMA channel not supported!\n");
        		while(1);

        }

#else
        NVIC_EnableIRQ(DMA0_IRQn);
#endif
    }
    else {

        // Allocate memory with a buffer large enough for a camera frame.
        rx_data = (uint8_t*)malloc(g_total_img_size);
    }

    if (rx_data == NULL) {
        // Return error if unable to allocate memory.
        return STATUS_ERROR_ALLOCATING;
    }

    // Initialize buffer
    if (g_dma_mode == STREAMING_DMA) {
        memset((uint8_t*)rx_data, 0xff, 2 * g_stream_buffer_size);
    }
    else {
        memset((uint8_t*)rx_data, 0xff, g_total_img_size);
    }

    if ((g_dma_mode == USE_DMA) || (g_dma_mode == STREAMING_DMA)) {
        g_dma_channel = dma_channel;
        setup_dma_tft();
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA_THRSH, (0x1 << MXC_F_CAMERAIF_CTRL_RX_DMA_THRSH_POS));
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA, MXC_F_CAMERAIF_CTRL_RX_DMA);
        MXC_PCIF_DisableInt(MXC_F_CAMERAIF_INT_EN_FIFO_THRESH);
    }
    else {
        // Slow down clock if not using dma
#if defined(CAMERA_OV7692)
        ret |= camera.write_reg(0x11, 0x4); // clock prescaler
#endif
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA, 0x0);
        MXC_PCIF_EnableInt(MXC_F_CAMERAIF_INT_EN_FIFO_THRESH);
    }

    camera.set_pixformat(pixformat);
    camera.set_framesize(g_framesize_width, g_framesize_height);

    return ret;
}
#endif

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360_MONO) || defined(CAMERA_HM0360_COLOR) || \
    defined(CAMERA_OV5642)
int camera_read_reg(uint16_t reg_addr, uint8_t *reg_data)
{
    return camera.read_reg(reg_addr, reg_data);
}

int camera_write_reg(uint16_t reg_addr, uint8_t reg_data)
{
    return camera.write_reg(reg_addr, reg_data);
}
#else //CAMERA_OV7642
int camera_read_reg(uint8_t reg_addr, uint8_t *reg_data)
{
    return camera.read_reg(reg_addr, reg_data);
}

int camera_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    return camera.write_reg(reg_addr, reg_data);
}
#endif

int camera_get_slave_address(void)
{
    return camera.get_slave_address();
}

int camera_get_product_id(int *id)
{
    return camera.get_product_id(id);
}

int camera_get_manufacture_id(int *id)
{
    return camera.get_manufacture_id(id);
}

int camera_dump_registers(void)
{
    return camera.dump_registers();
}

int camera_set_hmirror(int enable)
{
    return camera.set_hmirror(enable);
}

int camera_set_vflip(int enable)
{
    return camera.set_vflip(enable);
}

int camera_set_colorbar(int enable)
{
    return camera.set_colorbar(enable);
}

int camera_start_capture_image(void)
{
    int ret = STATUS_OK;

    if ((g_dma_mode == USE_DMA) || (g_dma_mode == STREAMING_DMA)) {
        setup_dma();
    }

    // Clear flags.
    g_is_img_rcv = 0;
    rx_data_index = 0;
    MXC_PCIF->int_fl = MXC_PCIF->int_fl;
    MXC_PCIF_Start(MXC_PCIF_READMODE_SINGLE_MODE);
    return ret;
}

#ifndef __riscv
int camera_start_capture_image_tft(void)
{
    int ret = STATUS_OK;

    if ((g_dma_mode == USE_DMA) || (g_dma_mode == STREAMING_DMA) & (g_dma_already_setup == 0)) {
        setup_dma();
    }

    // Clear flags.
    g_is_img_rcv = 0;
    rx_data_index = 0;
    MXC_PCIF->int_fl = MXC_PCIF->int_fl;
    MXC_PCIF_Start(MXC_PCIF_READMODE_CONTINUES_MODE);
    return ret;
}
#endif

int camera_is_image_rcv(void)
{
    return (g_is_img_rcv) ? 1 : 0;
}

uint8_t *camera_get_pixel_format(void)
{
    pixformat_t pix_format;

    // Get the pixel format from the camera driver.
    camera.get_pixformat(&pix_format);

    if (pix_format == PIXFORMAT_RGB444) {
        return (uint8_t *)"RGB444";
    } else if (pix_format == PIXFORMAT_RGB565) {
        return (uint8_t *)"RGB565";
    } else if (pix_format == PIXFORMAT_RGB888) {
        return (uint8_t *)"RGB888";
    } else if (pix_format == PIXFORMAT_YUV422) {
        return (uint8_t *)"YUV422";
    } else if (pix_format == PIXFORMAT_BAYER) {
        return (uint8_t *)"BAYER";
    }

    return (uint8_t *)"INVALID";
}

void camera_get_image(uint8_t **img, uint32_t *imgLen, uint32_t *w, uint32_t *h)
{
    // int n = 0, index;
    MXC_PCIF->int_fl |= MXC_PCIF->int_fl;

    /*    if (g_dma_mode == USE_DMA && g_pixel_format == PIXFORMAT_RGB888) {
        // Filter image data
        index = 4;

        for (n = 3; n < ((g_framesize_width) * (g_framesize_height) * 3); n++) {
            index++;

            if (index % 4 == 0) {
                index++;
            }

            rx_data[n] = rx_data[index - 1];
        }
    }
*/
    *img = (uint8_t *)rx_data;
    *imgLen = g_total_img_size;

    *w = g_framesize_width;
    *h = g_framesize_height;
}

int camera_get_luminance_level(int *lum)
{
    return camera.get_luminance(lum);
}

uint8_t *get_camera_stream_buffer(void)
{
    return stream_buffer_ptr;
}

int camera_get_stream_buffer_size(void)
{
    return g_stream_buffer_size;
}

void release_camera_stream_buffer(void)
{
    stream_buffer_ptr = NULL;
}

stream_stat_t *get_camera_stream_statistic(void)
{
    return &statistic;
}

int camera_set_contrast(int level)
{
    return camera.set_contrast(level);
}

int camera_set_brightness(int level)
{
    return camera.set_brightness(level);
}

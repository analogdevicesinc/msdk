/**
 * @file    main.c
 * @brief   MIPI CSI-2 Demo.
 * @details This example captures an image using a MIPI Camera and an CSI-2 Receiver.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "csi2.h"
#include "csi2_regs.h"
#include "dma.h"
#include "icc.h"
#include "pb.h"
#include "led.h"
#include "board.h"
#include "nvic_table.h"
#include "mipi_camera.h"
#include "utils.h"
#include "gcr_regs.h"
#include "mcr_regs.h"
#include "console.h"

/***** Definitions *****/

#define IMAGE_WIDTH 160
#define IMAGE_HEIGHT 120

// Check CSI-2 Standard and your color format for these values.
#define BITS_PER_PIXEL_ODD 8 // e.g. RGB888
#define BYTES_PER_PIXEL_ODD (BITS_PER_PIXEL_ODD >> 3)
#define BITS_PER_PIXEL_EVEN 8
 // e.g. RGB888
#define BYTES_PER_PIXEL_EVEN (BITS_PER_PIXEL_EVEN >> 3)

// CSI-2 Peripheral Configuration
#define NUM_DATA_LANES 2
#define FLUSH_COUNT 3
#define VIRTUAL_CHANNEL 0x00
#define RX_THRESHOLD 0x10
#define WAIT_CYCLE 0x2000

#define FLOW_CTRL                                                                  \
    (MXC_F_CSI2_VFIFO_CFG1_WAIT_FIRST_FS | MXC_F_CSI2_VFIFO_CFG1_ACCU_FRAME_CTRL | \
    MXC_F_CSI2_VFIFO_CFG1_ACCU_LINE_CNT | MXC_F_CSI2_VFIFO_CFG1_ACCU_PIXEL_CNT)

// Select corresponding pixel format and output sequence for camera settings.
//    Check OV5640 (or selected camera's) Datasheet for more information.
#define PIXEL_FORMAT MIPI_PIXFORMAT_RAW
#define OUT_SEQ 0x0 // BGBG GRGR
#define MUX_CTRL 1 // ISP RGB

// Streaming pixel format may not match pixel format the camera originally processed.
//    For example, the camera can processes RAW then converts to RGB888.
// #define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RGB888
//#define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RGB565
#define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RAW

// Select RGB Type and RAW Format for the CSI2 Peripheral.
#define RGB_TYPE MXC_CSI2_TYPE_RGB888
#define RAW_FORMAT MXC_CSI2_FORMAT_RGRG_GBGB

// Select corresponding Payload data. Only one type can be selected across payload 0 and 1.
// #define PAYLOAD0_DATA_TYPE MXC_CSI2_PL0_RAW10
#define PAYLOAD0_DATA_TYPE MXC_CSI2_PL0_RAW8
#define PAYLOAD1_DATA_TYPE MXC_CSI2_PL1_DISABLE_ALL

#if (PAYLOAD0_DATA_TYPE != MXC_CSI2_PL0_DISABLE_ALL || \
     PAYLOAD1_DATA_TYPE != MXC_CSI2_PL1_DISABLE_ALL)
#error Invalid Payload Data Configuration.
#endif

// Size of Image Buffer
// This buffer will hold the full output image after the CSI2 has converted it
// to color.
// #define IMAGE_SIZE ((BITS_PER_PIXEL_ODD + BITS_PER_PIXEL_EVEN) * IMAGE_WIDTH * IMAGE_HEIGHT) >> 4
#define IMAGE_SIZE \
    (IMAGE_WIDTH * IMAGE_HEIGHT * BYTES_PER_PIXEL_EVEN) + (IMAGE_WIDTH * BYTES_PER_PIXEL_EVEN)
// ^ The addition of the extra row above is a workaround to another CSI2 hardware and/or driver issue.
// When writing to the output image buffer the CSI2 block will overflow beyond the bounds of the array
// by a variable amount and cause a hard fault.  At 160x120 it overflows by exactly 5 bytes.  At 320x240
// It overflows by somewhere between 64 and 96 bytes (Exact # TBD by sheer trial and error).
// Extending the output array by this extra row seems to be a reliable workaround to this issue.

// Update for future cameras
#if defined(CAMERA_OV5640)
#define CAMERA_ID 0x5640
#else // Default
#define CAMERA_ID 0x5640
#endif

/***** Globals *****/

volatile int DMA_FLAG = 1;

// RAW Line Buffers
// These buffers are used by the CSI2 hardware for debayering.
// The size of these is hard-coded to 2048 because there appears
// to be some instability in the CSI2 hardware if they are scaled
// based on the image dimensions
__attribute__((section(".csi2_buff_raw0"))) uint32_t RAW_ADDR0[2048];
__attribute__((section(".csi2_buff_raw1"))) uint32_t RAW_ADDR1[2048];

// Buffer for processed image
__attribute__((section(".csi2_img_buff"))) uint8_t IMAGE[IMAGE_SIZE] = { 0 };
unsigned int g_index = 0;

/***** Functions *****/

void DMA_Handler(void)
{
    MXC_DMA_Handler();
}

void CSI2_line_handler(volatile uint8_t* data, unsigned int len)
{
    for (unsigned int i = 0; i < len; i++) {
        IMAGE[g_index++] = data[i];
    }
}

// void CSI2_Handler(void)
// {
//     MXC_CSI2_Handler();
// }
volatile unsigned int count = 0;

void process_img(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    printf("Capturing image...\n");

    g_index = 0;
    MXC_TMR_SW_Start(MXC_TMR0);
    MXC_CSI2_CaptureFrameDMA(NUM_DATA_LANES);

    while (!MXC_CSI2_DMA_Frame_Complete()) {}

    unsigned int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %u us)\n", elapsed);
    DMA_FLAG = 1;

    // Get the details of the image from the camera driver.
    MXC_CSI2_GetImageDetails(&raw, &imgLen, &w, &h);

    MXC_TMR_SW_Start(MXC_TMR0);
    clear_serial_buffer();
    // snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
    //          "*IMG* %s %i %i %i", // Format img info into a string
    //          mipi_camera_get_pixel_format(STREAM_PIXEL_FORMAT), imgLen, w, h);
    snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
             "*IMG* %s %i %i %i", // Format img info into a string
             "BAYER", imgLen, w, h);
    send_msg(g_serial_buffer);

    clear_serial_buffer();
    MXC_UART_WriteBytes(Con_Uart, raw, imgLen);

    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (serial transmission took %i us)\n", elapsed);
}

void service_console()
{
    // Check for any incoming serial commands
    cmd_t cmd = CMD_UNKNOWN;
    if (recv_cmd(&cmd)) {
        // Process the received command...

        if (cmd == CMD_UNKNOWN) {
            printf("Uknown command '%s'\n", g_serial_buffer);
        } else if (cmd == CMD_HELP) {
            print_help();
        } else if (cmd == CMD_RESET) {
            // Issue a soft reset
            MXC_GCR->rst0 |= MXC_F_GCR_RST0_SYS;
        } else if (cmd == CMD_CAPTURE) {
            process_img();
        }
    }

    clear_serial_buffer();
}

volatile int buttonPressed = 0;
void buttonHandler()
{
    buttonPressed = 1;
}

int main(void)
{
    int csi2_dma_channel;
    int error;
    int id;
    mxc_csi2_req_t req;
    mxc_csi2_ctrl_cfg_t ctrl_cfg;
    mxc_csi2_vfifo_cfg_t vfifo_cfg;

    console_init();

    // Enable cache
    MXC_ICC_Enable(MXC_ICC0);

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("\n\n**** MIPI CSI-2 Example ****\n");
    printf("This example streams the image data through the COM port\n");
    printf("and a script running on the host pc converts the data into\n");
    printf("a .png image.\n");
    printf("\nGo into the pc_utility folder and run the script:\n");
    printf("python console.py [COM#]\n");
    printf("\nPress PB1 (SW4) or send the 'capture' command to trigger a frame capture.\n");

    // Initialize camera
    mipi_camera_init();

    // Confirm correct camera is connected
    mipi_camera_get_product_id(&id);
    printf("Camera ID = %x\n", id);
    if (id != CAMERA_ID) {
        printf("Incorrect camera.\n");
        LED_On(1);
        while (1) {}
    }

    mipi_camera_setup(IMAGE_WIDTH, IMAGE_HEIGHT, PIXEL_FORMAT, OUT_SEQ, MUX_CTRL);
    uint8_t sc_pll_ctrl0 = 0;
    mipi_camera_read_reg(0x3034, &sc_pll_ctrl0);
    printf("sc_pll_ctrl0: 0x%x\n", sc_pll_ctrl0);
    mipi_camera_write_reg(0x3034, (sc_pll_ctrl0 & 0xF0) | 0x8);
    mipi_camera_read_reg(0x3034, &sc_pll_ctrl0);
    printf("sc_pll_ctrl0: 0x%x\n", sc_pll_ctrl0);

    // Configure RX Controller and PPI (D-PHY)
    ctrl_cfg.invert_ppi_clk = MXC_CSI2_PPI_NO_INVERT;
    ctrl_cfg.num_lanes = NUM_DATA_LANES;
    ctrl_cfg.payload0 = PAYLOAD0_DATA_TYPE;
    ctrl_cfg.payload1 = PAYLOAD1_DATA_TYPE;
    ctrl_cfg.flush_cnt = FLUSH_COUNT;

    ctrl_cfg.lane_src.d0_swap_sel = MXC_CSI2_PAD_CDRX_PN_L0;
    ctrl_cfg.lane_src.d1_swap_sel = MXC_CSI2_PAD_CDRX_PN_L1;
    ctrl_cfg.lane_src.d2_swap_sel = MXC_CSI2_PAD_CDRX_PN_L2;
    ctrl_cfg.lane_src.d3_swap_sel = MXC_CSI2_PAD_CDRX_PN_L3;
    ctrl_cfg.lane_src.c0_swap_sel = MXC_CSI2_PAD_CDRX_PN_L4;

    // Image Data
    req.img_addr = IMAGE;
    req.pixels_per_line = IMAGE_WIDTH;
    req.lines_per_frame = IMAGE_HEIGHT;
    req.bits_per_pixel_odd = BITS_PER_PIXEL_ODD;
    req.bits_per_pixel_even = BITS_PER_PIXEL_EVEN;
    req.frame_num = 1;

    // Convert RAW to RGB
    req.process_raw_to_rgb = false;
    req.rgb_type = RGB_TYPE;
    req.raw_format = RAW_FORMAT;
    req.autoflush = MXC_CSI2_AUTOFLUSH_ENABLE;
    req.raw_buf0_addr = (uint32_t)RAW_ADDR0;
    req.raw_buf1_addr = (uint32_t)RAW_ADDR1;
    req.line_handler = CSI2_line_handler;
    // req.callback = CSI2_Callback;

    // Configure VFIFO
    vfifo_cfg.virtual_channel = VIRTUAL_CHANNEL;
    vfifo_cfg.rx_thd = RX_THRESHOLD;
    vfifo_cfg.wait_cyc = WAIT_CYCLE;
    vfifo_cfg.flow_ctrl = FLOW_CTRL;
    vfifo_cfg.err_det_en = MXC_CSI2_ERR_DETECT_DISABLE;
    vfifo_cfg.fifo_rd_mode = MXC_CSI2_READ_ONE_BY_ONE;
    // vfifo_cfg.dma_whole_frame = MXC_CSI2_DMA_WHOLE_FRAME;
    vfifo_cfg.dma_whole_frame = MXC_CSI2_DMA_LINE_BY_LINE;
    vfifo_cfg.dma_mode = MXC_CSI2_DMA_FIFO_ABV_THD;
    vfifo_cfg.bandwidth_mode = MXC_CSI2_NORMAL_BW;
    vfifo_cfg.wait_en = MXC_CSI2_AHBWAIT_ENABLE;

    error = MXC_CSI2_Init(&req, &ctrl_cfg, &vfifo_cfg);
    if (error != E_NO_ERROR) {
        printf("Error Initializating.\n\n");
        while (1) {}
    }

    csi2_dma_channel = MXC_CSI2_DMA_GetChannel();
    MXC_NVIC_SetVector(DMA0_IRQn + csi2_dma_channel, DMA_Handler);
    // MXC_NVIC_SetVector(CSI2_IRQn, CSI2_Handler);

    PB_RegisterCallback(0, (pb_callback)buttonHandler);
    buttonPressed = 0;

    while (1) {
        LED_On(0);

        MXC_Delay(MXC_DELAY_MSEC(100));
        // ^ Slow down main processing loop to work around some timing issues
        // with the console at extreme speeds.  1000 checks/sec is plenty
        service_console();

        if (buttonPressed) {
            process_img();
            LED_Off(0);

            buttonPressed = 0;
        }
    }
}

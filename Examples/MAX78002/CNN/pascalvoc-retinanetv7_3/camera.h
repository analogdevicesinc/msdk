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
#include "gcr_regs.h"
#include "mcr_regs.h"
#ifdef CONSOLE
#include "console.h"
#endif
#include "aps6404.h"

#ifndef CAMERA_H
#define CAMERA_H

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 256

// #define RAW


// Check CSI-2 Standard and your color format for these values.
#ifdef RAW
#define BITS_PER_PIXEL_ODD 8 // e.g. RAW8
#define BITS_PER_PIXEL_EVEN 8
#else
#define BITS_PER_PIXEL_ODD 16 // e.g. RGB565
#define BITS_PER_PIXEL_EVEN 16
#endif
#define BYTES_PER_PIXEL_ODD (BITS_PER_PIXEL_ODD >> 3)
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
#ifdef RAW
#define PIXEL_FORMAT MIPI_PIXFORMAT_RAW
#define OUT_SEQ 0x0 // BGBG GRGR
#else
#define PIXEL_FORMAT MIPI_PIXFORMAT_RGB565
#define OUT_SEQ 0x1
// RGB565 ({r[4:0]g[5:3]},{g[2:0],b[4:0]})
#endif
#define MUX_CTRL 1 // ISP RGB

// Streaming pixel format may not match pixel format the camera originally processed.
//    For example, the camera can processes RAW then converts to RGB888.
// #define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RGB888
//#define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RGB565
#ifdef RAW
#define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RAW
#else
#define STREAM_PIXEL_FORMAT MIPI_PIXFORMAT_RGB565
#endif

// Select RGB Type and RAW Format for the CSI2 Peripheral.
#define RGB_TYPE MXC_CSI2_TYPE_RGB888
#define RAW_FORMAT MXC_CSI2_FORMAT_RGRG_GBGB

// Select corresponding Payload data. Only one type can be selected across payload 0 and 1.
#ifdef RAW
#define PAYLOAD0_DATA_TYPE MXC_CSI2_PL0_RAW8
#else
#define PAYLOAD0_DATA_TYPE MXC_CSI2_PL0_RGB565
#endif
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

// RAW Line Buffers
// These buffers are used by the CSI2 hardware for debayering.
// The size of these is hard-coded to 2048 because there appears
// to be some instability in the CSI2 hardware if they are scaled
// based on the image dimensions
// __attribute__((section(".csi2_buff_raw0"))) uint32_t RAW_ADDR0[2048];
// __attribute__((section(".csi2_buff_raw1"))) uint32_t RAW_ADDR1[2048];

// Buffer for processed image
// __attribute__((section(".csi2_img_buff"))) uint8_t IMAGE[IMAGE_SIZE] = { 0 };

/***** Functions *****/

void process_img(void);

#ifdef CONSOLE
void service_console(cmd_t cmd);
#endif

bool camera_init();

#endif

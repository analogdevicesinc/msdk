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

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_CAMERA_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_CAMERA_H_

// includes
#include <stdio.h>
#include <stdint.h>

#if defined(CAMERA_HM01B0)
#include "hm01b0_regs.h"
#elif defined(CAMERA_HM0360_MONO)
#include "hm0360_regs.h"
#elif defined(CAMERA_HM0360_COLOR)
#include "hm0360_regs.h"
#elif defined(CAMERA_OV5642)
#include "ov5642_regs.h"
#elif defined(CAMERA_OV7692)
#include "ov7692_regs.h"
#elif defined(CAMERA_PAG7920)
#include "pag7920_regs.h"
#endif

#include "tmr_regs.h"

#define STATUS_OK (0)
#define STATUS_ERROR (-1)
#define STATUS_ERROR_ALLOCATING (-10)

/******************************** Type Definitions ***************************/
typedef enum {
    PIXFORMAT_INVALID = 0,
    PIXFORMAT_GRAYSCALE, // 2BPP/GRAYSCALE
    PIXFORMAT_RGB444, // 2BPP/RGB444
    PIXFORMAT_RGB565, // 2BPP/RGB565
    PIXFORMAT_RGB888, // 3BPP/RGB888 (expanded from rgb565 by camera interface hardware)
    PIXFORMAT_YUV422, // 2BPP/YUV422
    PIXFORMAT_BAYER, // 1BPP/RAW
} pixformat_t;

typedef enum { BIT_EXPAND_OFF, BIT_EXPAND_565_TO_888 } expandbitmode_t;

typedef enum {
    FIFO_THREE_BYTE = 0,
    FIFO_FOUR_BYTE,
} fifomode_t;

typedef enum { NO_DMA = 0, USE_DMA, STREAMING_DMA } dmamode_t;

typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X,
    GAINCEILING_64X,
    GAINCEILING_128X,
} gainceiling_t;

typedef struct _camera {
    // Sensor function pointers
    int (*init)(void);
    int (*get_slave_address)(void);
    int (*get_product_id)(int *id);
    int (*get_manufacture_id)(int *id);
    int (*dump_registers)(void);
    int (*reset)(void);
    int (*sleep)(int enable);
#if defined(CAMERA_HM01B0) || (CAMERA_HM0360_MONO) || (CAMERA_HM0360_COLOR) || \
    defined(CAMERA_OV5642)
    int (*read_reg)(uint16_t reg_addr, uint8_t *reg_data);
    int (*write_reg)(uint16_t reg_addr, uint8_t reg_data);
#else //(CAMERA_OV7692) || (CAMERA_PAG7920)
    int (*read_reg)(uint8_t reg_addr, uint8_t *reg_data);
    int (*write_reg)(uint8_t reg_addr, uint8_t reg_data);
#endif
    int (*set_pixformat)(pixformat_t pixformat);
    int (*get_pixformat)(pixformat_t *pixformat);
    int (*set_framesize)(int width, int height);
    int (*set_windowing)(int width, int height, int hsize, int vsize);
    int (*set_contrast)(int level);
    int (*set_brightness)(int level);
    int (*set_saturation)(int level);
    int (*set_gainceiling)(gainceiling_t gainceiling);
    int (*set_colorbar)(int enable);
    int (*set_hmirror)(int enable);
    int (*set_vflip)(int enable);
    int (*set_negateimage)(int enable);
    int (*get_luminance)(int *lum);
} camera_t;

typedef struct _stream_stat {
    uint32_t dma_transfer_count;
    uint32_t overflow_count;
} stream_stat_t;

/******************************** Public Functions ***************************/
// Initialize the sensor hardware and probe the image sensor.
int camera_init(uint32_t freq);

// Return sensor i2c slave address.
int camera_get_slave_address();

// Return sensor Product ID.
int camera_get_product_id(int *id);

// Return sensor Manufacturer ID.
int camera_get_manufacture_id(int *id);

// dump all registers of camera
int camera_dump_registers();

// Reset the sensor to its default state.
int camera_reset();

// Sleep mode.
int camera_sleep(int enable);

// Shutdown mode.
int camera_shutdown(int enable);

#ifdef CAMERA_BAYER
/**
* @brief Formulate a bayer "passthrough" image that splits an HM0360 bayer pattern into its RGB channels while preserving the bayer pattern.  Useful for debugging and demosaicing algorithm development.
* @param[in] srcimg Pointer to the raw bayer pattern
* @param[in] w Width of the bayer pattern (in pixels)
* @param[in] h Height of the bayer pattern (in pixels)
* @param[out] dstimg Output pointer for converted RGB565 image.
****************************************************************************/
void bayer_passthrough(uint8_t *srcimg, uint32_t w, uint32_t h, uint16_t *dstimg);

/**
* @brief Color-correct and demosaic a raw HM0360 bayer-patterned image array and convert to RGB565.
* @param[in] srcimg Pointer to the raw bayer pattern
* @param[in] w Width of the bayer pattern (in pixels)
* @param[in] h Height of the bayer pattern (in pixels)
* @param[out] dstimg Output pointer for converted RGB565 image.
****************************************************************************/
void bayer_bilinear_demosaicing(uint8_t *srcimg, uint32_t w, uint32_t h, uint16_t *dstimg);
#endif

#if defined(CAMERA_HM01B0) || (CAMERA_HM0360_MONO) || (CAMERA_HM0360_COLOR) || \
    defined(CAMERA_OV5642)
// Write a sensor register.
int camera_write_reg(uint16_t reg_addr, uint8_t reg_data);
int camera_read_reg(uint16_t reg_addr, uint8_t *reg_data);
#else //(CAMERA_OV7692) || (CAMERA_PAG7920)
// Write a sensor register.
int camera_write_reg(uint8_t reg_addr, uint8_t reg_data);
// Read a sensor register.
int camera_read_reg(uint8_t reg_addr, uint8_t *reg_data);
#endif

// Set the sensor frame size and pixel format.
int camera_set_frame_info(int width, int height, pixformat_t pixformat);

// Setup the camera resolution, pixel format, expand bits option, fifo byte mode and dma option.
int camera_setup(int xres, int yres, pixformat_t pixformat, fifomode_t fifo_mode,
                 dmamode_t dma_mode, int dma_channel);

// Setup the camera resolution, pixel format, expand bits option, fifo byte mode and dma option and stream to TFT
int camera_setup_tft(int xres, int yres, pixformat_t pixformat, fifomode_t fifo_mode,
                     dmamode_t dma_mode, int dma_channel);

// Set the sensor contrast level (from -2 to +2).
int camera_set_contrast(int level);

// Set the sensor brightness level (from -2 to +2).
int camera_set_brightness(int level);

// Set the sensor saturation level (from -2 to +2).
int camera_set_saturation(int level);

// Set the sensor AGC gain ceiling.
// Note: This function has no effect when AGC (Automatic Gain Control) is disabled.
int camera_set_gainceiling(gainceiling_t gainceiling);

// Enable/disable the colorbar mode.
int camera_set_colorbar(int enable);

// Enable/disable the hmirror mode.
int camera_set_hmirror(int enable);

// Enable/disable the vflip mode.
int camera_set_vflip(int enable);

// start to capture image
int camera_start_capture_image(void);

// start to capture image
int camera_start_capture_image_tft(void);

// check whether all image data rcv or not
int camera_is_image_rcv(void);

// Retrieve the camera pixel format of the camera.
uint8_t *camera_get_pixel_format(void);

// Get a pointer to the camera frame buffer, also get the image length and resolution.
void camera_get_image(uint8_t **img, uint32_t *imgLen, uint32_t *w, uint32_t *h);

// Get luminance level from camera.
int camera_get_luminance_level(int *lum);

// Get camera streaming buffer.
uint8_t *get_camera_stream_buffer(void);

// Get camera streaming buffer size (in bytes)
int camera_get_stream_buffer_size(void);

// Release camera streaming buffer.
void release_camera_stream_buffer(void);

// Get statistics of DMA streaming mode.
stream_stat_t *get_camera_stream_statistic(void);

#endif // LIBRARIES_MISCDRIVERS_CAMERA_CAMERA_H_

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_CAMERA_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_CAMERA_H_

// includes
#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CAMERA_HM01B0)
#include "hm01b0_regs.h"
#elif defined(CAMERA_HM0360_MONO)
#include "hm0360_regs.h"
#elif defined(CAMERA_HM0360_COLOR)
#include "hm0360_regs.h"
#elif defined(CAMERA_OV5642)
#include "ov5642_regs.h"
#elif defined(CAMERA_OV5640)
#include "ov5640_regs.h"
#elif defined(CAMERA_OV7692)
#include "ov7692_regs.h"
#elif defined(CAMERA_PAG7920)
#include "pag7920_regs.h"
#endif

#include "debayering.h"

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
    defined(CAMERA_OV5642) || defined(CAMERA_OV5640)
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
/**
 * @brief Initialize the sensor hardware and probe the image sensor.
 *
 * @param freq Frequency for sensor initialization.
 * @return 0 on success, otherwise an error code.
 */
int camera_init(uint32_t freq);

/**
 * @brief Return sensor I2C slave address.
 *
 * @return Sensor I2C slave address.
 */
int camera_get_slave_address();

/**
 * @brief Read the camera sensor's Product ID.
 *
 * @param id Pointer to store the Product ID.
 * @return 0 on success, otherwise an error code.
 */
int camera_get_product_id(int *id);

/**
 * @brief Read the camera sensor's Manufacturer ID.
 *
 * @param id Pointer to store the Manufacturer ID.
 * @return 0 on success, otherwise an error code.
 */
int camera_get_manufacture_id(int *id);

/**
 * @brief Dump all registers of the camera to 'printf'.
 *
 * @return 0 on success, otherwise an error code.
 */
int camera_dump_registers();

/**
 * @brief Reset the sensor to its default state.
 *
 * @return 0 on success, otherwise an error code.
 */
int camera_reset();

/**
 * @brief Set the sensor to sleep mode or wake it up.
 *
 * @param enable 1 to enable sleep mode, 0 to disable sleep mode.
 * @return 0 on success, otherwise an error code.
 */
int camera_sleep(int enable);

/**
 * @brief Shutdown the camera completely.  Typically, this involves shutting down its power supply.
 * All registers will be lost when.
 *
 * @param enable 1 to enable shutdown mode, 0 to disable.
 * @return 0 on success, otherwise an error code.
 */
int camera_shutdown(int enable);

#if defined(CAMERA_HM01B0) || (CAMERA_HM0360_MONO) || (CAMERA_HM0360_COLOR) || \
    defined(CAMERA_OV5642) || defined(CAMERA_OV5640)
/**
 * @brief Write a sensor register.
 *
 * @param reg_addr Register address to write.
 * @param reg_data Data to write to the register.
 * @return 0 on success, otherwise an error code.
 */
int camera_write_reg(uint16_t reg_addr, uint8_t reg_data);

/**
 * @brief Read a sensor register.
 *
 * @param reg_addr Register address to read.
 * @param reg_data Pointer to store the read data.
 * @return 0 on success, otherwise an error code.
 */
int camera_read_reg(uint16_t reg_addr, uint8_t *reg_data);
#else //(CAMERA_OV7692) || (CAMERA_PAG7920)
/**
 * @brief Write a sensor register.
 *
 * @param reg_addr Register address to write.
 * @param reg_data Data to write to the register.
 * @return 0 on success, otherwise an error code.
 */
int camera_write_reg(uint8_t reg_addr, uint8_t reg_data);

/**
 * @brief Read a sensor register.
 *
 * @param reg_addr Register address to read.
 * @param reg_data Pointer to store the read data.
 * @return 0 on success, otherwise an error code.
 */
int camera_read_reg(uint8_t reg_addr, uint8_t *reg_data);
#endif

/**
 * @brief Set the sensor frame size and pixel format.
 *
 * @param width Frame width.
 * @param height Frame height.
 * @param pixformat Pixel format.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_frame_info(int width, int height, pixformat_t pixformat);

/**
 * @brief Setup the camera parameters for capturing.
 *
 * @param xres X resolution.
 * @param yres Y resolution.
 * @param pixformat Pixel format.
 * @param fifo_mode FIFO mode.
 * @param dma_mode DMA mode.
 * @param dma_channel DMA channel.
 * @return 0 on success, otherwise an error code.
 */
int camera_setup(int xres, int yres, pixformat_t pixformat, fifomode_t fifo_mode,
                 dmamode_t dma_mode, int dma_channel);

/**
 * @brief Setup the camera parameters for capturing and stream to TFT.
 *
 * @param xres X resolution.
 * @param yres Y resolution.
 * @param pixformat Pixel format.
 * @param fifo_mode FIFO mode.
 * @param dma_mode DMA mode.
 * @param dma_channel DMA channel.
 * @return 0 on success, otherwise an error code.
 */
int camera_setup_tft(int xres, int yres, pixformat_t pixformat, fifomode_t fifo_mode,
                     dmamode_t dma_mode, int dma_channel);

/**
 * @brief Set the sensor contrast level.
 *
 * @param level Contrast level.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_contrast(int level);

/**
 * @brief Set the sensor brightness level.
 *
 * @param level Brightness level.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_brightness(int level);

/**
 * @brief Set the sensor saturation level.
 *
 * @param level Saturation level.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_saturation(int level);

/**
 * @brief Set the sensor AGC gain ceiling.
 *
 * @param gainceiling AGC gain ceiling.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_gainceiling(gainceiling_t gainceiling);

/**
 * @brief Enable/disable the colorbar mode.
 *
 * @param enable 1 to enable colorbar, 0 to disable.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_colorbar(int enable);

/**
 * @brief Enable/disable the horizontal mirror mode.
 *
 * @param enable 1 to enable horizontal mirror, 0 to disable.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_hmirror(int enable);

/**
 * @brief Enable/disable the vertical flip mode.
 *
 * @param enable 1 to enable vertical flip, 0 to disable.
 * @return 0 on success, otherwise an error code.
 */
int camera_set_vflip(int enable);

/**
 * @brief Start capturing an image.
 *
 * @return 0 on success, otherwise an error code.
 */
int camera_start_capture_image(void);

/**
 * @brief Start capturing an image and stream to TFT.
 *
 * @return 0 on success, otherwise an error code.
 */
int camera_start_capture_image_tft(void);

/**
 * @brief Check whether all image data has been received.
 *
 * @return 1 if image data received, 0 otherwise.
 */
int camera_is_image_rcv(void);

/**
 * @brief Retrieve the camera pixel format.
 *
 * @return Pointer to the camera pixel format.
 */
uint8_t *camera_get_pixel_format(void);

/**
 * @brief Get a pointer to the camera frame buffer, image length, and resolution.
 *
 * @param img Pointer to store the image buffer.
 * @param imgLen Pointer to store the image length.
 * @param w Pointer to store the image width.
 * @param h Pointer to store the image height.
 */
void camera_get_image(uint8_t **img, uint32_t *imgLen, uint32_t *w, uint32_t *h);

/**
 * @brief Get luminance level from the camera.
 *
 * @param lum Pointer to store the luminance level.
 * @return 0 on success, otherwise an error code.
 */
int camera_get_luminance_level(int *lum);

/**
 * @brief Get a pointer to the camera streaming buffer.
 *
 * @return Pointer to the camera streaming buffer.
 */
uint8_t *get_camera_stream_buffer(void);

/**
 * @brief Get the size of the camera streaming buffer (in bytes).
 *
 * @return Size of the camera streaming buffer.
 */
int camera_get_stream_buffer_size(void);

/**
 * @brief Release the camera streaming buffer.
 */
void release_camera_stream_buffer(void);

/**
 * @brief Get statistics of DMA streaming mode.
 *
 * @return Pointer to the structure containing DMA streaming statistics.
 */
stream_stat_t *get_camera_stream_statistic(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_CAMERA_CAMERA_H_

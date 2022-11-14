/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
#ifndef EXAMPLES_MAX32570_CAMERAIF_INCLUDE_CAMERA_H_
#define EXAMPLES_MAX32570_CAMERAIF_INCLUDE_CAMERA_H_

// includes
#include <stdio.h>
#include <stdint.h>

// Defines
#define CAM_OV7670 (1)
#define CAM_OV7725 (2)
#define CAM_OV2640 (3)
#define CAM_GC0308 (4) // implementation not finalized yet

#define ACTIVE_CAMERA CAM_OV7670
#define CAMERA_FREQ (15 * 1000 * 1000)

#include "ov7670_regs.h"
#include "ov7725_regs.h"
#include "ov2640_regs.h"
#include "gc0308_regs.h"

/******************************** Type Definitions ***************************/
typedef enum {
    PIXFORMAT_INVALID = 0,
    PIXFORMAT_BINARY, // 1BPP/BINARY
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_RGB565, // 2BPP/RGB565
    PIXFORMAT_YUV422, // 2BPP/YUV422
    PIXFORMAT_BAYER, // 1BPP/RAW
} pixformat_t;

typedef enum {
    // CIF Resolutions
    FRAMESIZE_QCIF, // 176x144
    FRAMESIZE_CIF, // 352x288
    // VGA Resolutions
    FRAMESIZE_QQVGA, // 160x120
    FRAMESIZE_QVGA, // 320x240
    FRAMESIZE_VGA, // 640x480
} framesize_t;

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
    int (*get_id)(void);
    int (*dump_registers)(void);
    int (*reset)(void);
    int (*sleep)(int enable);
    int (*read_reg)(uint16_t reg_addr);
    int (*write_reg)(uint16_t reg_addr, uint16_t reg_data);
    int (*set_pixformat)(pixformat_t pixformat);
    int (*set_framesize)(framesize_t framesize);
    int (*set_contrast)(int level);
    int (*set_brightness)(int level);
    int (*set_saturation)(int level);
    int (*set_gainceiling)(gainceiling_t gainceiling);
    int (*set_colorbar)(int enable);
    int (*set_hmirror)(int enable);
    int (*set_vflip)(int enable);
} camera_t;

/******************************** Public Functions ***************************/
// Initialize the sensor hardware and probe the image sensor.
int camera_init();

// Return sensor PID.
int camera_get_id();

// dump all registers of camera
int camera_dump_registers();

// Reset the sensor to its default state.
int camera_reset();

// Sleep mode.
int camera_sleep(int enable);

// Shutdown mode.
int camera_shutdown(int enable);

// Read a sensor register.
int camera_read_reg(uint16_t reg_addr);

// Write a sensor register.
int camera_write_reg(uint16_t reg_addr, uint16_t reg_data);

// Set the sensor pixel format.
int camera_set_pixformat(pixformat_t pixformat);

// Set the sensor frame size.
int camera_set_framesize(framesize_t framesize);

// Set the sensor contrast level (from -3 to +3).
int camera_set_contrast(int level);

// Set the sensor brightness level (from -3 to +3).
int camera_set_brightness(int level);

// Set the sensor saturation level (from -3 to +3).
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
int camera_start_campture_image(void);

// check whether all image data rcv or not
int camera_is_image_rcv(void);

//
uint8_t *camera_get_pixel_format(void);

//
void camera_get_image(uint8_t **img, uint32_t *imgLen, uint32_t *w, uint32_t *h);

#endif // EXAMPLES_MAX32570_CAMERAIF_INCLUDE_CAMERA_H_

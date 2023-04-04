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

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_MIPI_CAMERA_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_MIPI_CAMERA_H_

#include <stdint.h>

typedef enum {
    MIPI_PIXFORMAT_RAW = 0x0,
    MIPI_PIXFORMAT_Y8 = 0x1,
    MIPI_PIXFORMAT_RGB888 = 0x2, // or YUV444
    MIPI_PIXFORMAT_YUV422 = 0x3,
    MIPI_PIXFORMAT_YUV420 = 0x4,
    MIPI_PIXFORMAT_YUV420M = 0x5, // MIPI only
    MIPI_PIXFORMAT_RGB565 = 0x6,
    MIPI_PIXFORMAT_RGB555_1 = 0x7, // Format 1
    MIPI_PIXFORMAT_RGB555_2 = 0x8, // Format 2
    MIPI_PIXFORMAT_RGB444_1 = 0x9, // Format 1
    MIPI_PIXFORMAT_RGB444_2 = 0xa, // Format 2
    MIPI_PIXFORMAT_BYPASS = 0xf,
} mipi_pixformat_t;

typedef enum {
    MIPI_GAINCEILING_2X,
    MIPI_GAINCEILING_4X,
    MIPI_GAINCEILING_8X,
    MIPI_GAINCEILING_16X,
    MIPI_GAINCEILING_32X,
    MIPI_GAINCEILING_64X,
    MIPI_GAINCEILING_128X,
} mipi_gainceiling_t;

struct camera_reg {
    uint32_t addr;
    uint8_t val;
};

typedef struct _mipi_camera {
    // Sensor function pointers
    int (*init)(void);
    int (*get_slave_address)(void);
    int (*get_product_id)(int *id);
    int (*get_manufacture_id)(int *id);
    int (*dump_registers)(void);
    int (*reset)(void);
    int (*sleep)(int enable);
    int (*read_reg)(uint16_t reg_addr, uint8_t *reg_data);
    int (*write_reg)(uint16_t reg_addr, uint8_t reg_data);
    int (*set_pixformat)(mipi_pixformat_t pixformat, uint32_t out_seq, int mux_ctrl);
    int (*get_pixformat)(mipi_pixformat_t *pixformat);
    int (*set_framesize)(int width, int height);
    int (*set_windowing)(int width, int height, int start_x, int start_y, int hsize, int vsize);
    int (*set_contrast)(int level);
    int (*set_brightness)(int level);
    int (*set_saturation)(int level);
    int (*set_hue)(int degree);
    int (*set_gainceiling)(mipi_gainceiling_t gainceiling);
    int (*set_colorbar)(int enable);
    int (*set_hmirror)(int enable);
    int (*set_vflip)(int enable);
    int (*set_negateimage)(int enable);
    int (*get_luminance)(int *lum);
} mipi_camera_t;

int mipi_camera_reset(void);
int mipi_camera_init(void);
int mipi_camera_write_reg(uint16_t reg_addr, uint8_t reg_data);
int mipi_camera_read_reg(uint16_t reg_addr, uint8_t *reg_data);
int mipi_camera_get_slave_address(void);
int mipi_camera_get_product_id(int *id);
int mipi_camera_get_manufacture_id(int *id);
int mipi_camera_setup(int xres, int yres, mipi_pixformat_t pixformat, int out_seq, int mux_ctrl);
int mipi_camera_sleep(int sleep);
uint8_t *mipi_camera_get_pixel_format(mipi_pixformat_t pix_format);

#endif // LIBRARIES_MISCDRIVERS_CAMERA_MIPI_CAMERA_H_

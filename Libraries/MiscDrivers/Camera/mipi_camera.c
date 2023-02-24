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

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mipi_camera.h"
#include "ov5640_regs.h"
#include "sccb.h"
#include "gpio.h"

extern int mipi_sensor_register(mipi_camera_t *camera);

static mipi_camera_t camera;

static int g_pixel_format = MIPI_PIXFORMAT_RGB888;
static int g_framesize_width = 32;
static int g_framesize_height = 32;

int mipi_camera_init(void)
{
    int ret;

    sccb_init();

    mipi_sensor_register(&camera);

    ret = camera.init();
    ret |= camera.reset();

    return ret;
}

int mipi_camera_get_slave_address(void)
{
    return camera.get_slave_address();
}

int mipi_camera_get_product_id(int *id)
{
    return camera.get_product_id(id);
}

int mipi_camera_get_manufacture_id(int *id)
{
    return camera.get_manufacture_id(id);
}

int mipi_camera_reset(void)
{
    return camera.reset();
}

int mipi_camera_setup(int xres, int yres, mipi_pixformat_t pixformat, int out_seq, int mux_ctrl)
{
    g_pixel_format = pixformat;

    // Setup camera resolution and allocate a camera frame buffer.
    g_framesize_width = xres;
    g_framesize_height = yres;

    camera.set_pixformat(pixformat, out_seq, mux_ctrl);
    camera.set_framesize(g_framesize_width, g_framesize_height);

    return 0;
}

uint8_t *mipi_camera_get_pixel_format(mipi_pixformat_t pix_format)
{
    if (pix_format == MIPI_PIXFORMAT_RGB444_1 || pix_format == MIPI_PIXFORMAT_RGB444_2) {
        return (uint8_t *)"RGB444";
    } else if (pix_format == MIPI_PIXFORMAT_RGB555_1 || pix_format == MIPI_PIXFORMAT_RGB555_2) {
        return (uint8_t *)"RGB555";
    } else if (pix_format == MIPI_PIXFORMAT_RGB565) {
        return (uint8_t *)"RGB565";
    } else if (pix_format == MIPI_PIXFORMAT_RGB888) {
        return (uint8_t *)"RGB888";
    } else if (pix_format == MIPI_PIXFORMAT_YUV422) {
        return (uint8_t *)"YUV422";
    }

    return (uint8_t *)"INVALID";
}

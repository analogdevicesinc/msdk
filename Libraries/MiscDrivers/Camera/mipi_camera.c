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

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mipi_camera.h"
#include "ov5640_regs.h"
#include "sccb.h"
#include "gpio.h"
#include "csi2.h"
#include "csi2_regs.h"

// CSI-2 Peripheral Configuration.  A lot of these settings have been
// found empirically after sheer trial and error.
#define NUM_DATA_LANES 2
#define FLUSH_COUNT 3
#define VIRTUAL_CHANNEL 0x00
#define RX_THRESHOLD 0x10
#define WAIT_CYCLE 0x2000
#define FLOW_CTRL                                                                  \
    (MXC_F_CSI2_VFIFO_CFG1_WAIT_FIRST_FS | MXC_F_CSI2_VFIFO_CFG1_ACCU_FRAME_CTRL | \
     MXC_F_CSI2_VFIFO_CFG1_ACCU_LINE_CNT | MXC_F_CSI2_VFIFO_CFG1_ACCU_PIXEL_CNT)

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

extern int mipi_sensor_register(mipi_camera_t *camera);

static mipi_camera_t camera;

static mxc_csi2_req_t g_req;
static mxc_csi2_ctrl_cfg_t g_ctrl_cfg;
static mxc_csi2_vfifo_cfg_t g_vfifo_cfg;

static mipi_camera_settings_t g_camera_settings;
static char g_image_header[256];

/**
 * @brief       Convert the specified pixel format to payload types for
 *              the CSI2 hardware @ref mxc_csi2_payload0_t and
 *              and @ref mxc_csi2_payload1_t values
 * @param[in]   pixel_format The desired pixel format
 * @param[out]  payload0 Output pointer to store the calculated payload0 value
 * @param[out] payload1 Output pointer to store the calculated payload1 value
 * @note        MAINTAINERS:  This function provides a translation layer
 *              between the user's desired pixel format and our hardware
 *              payload types.  At the moment, our debayering engine is
 *              disabled.  Additionally, user-defined payload types and
 *              10-bit YUV formats are not well supported. 
 *              At the point where we do want to enable those features,
 *              the additional logic should be handled in this function.
 * @return     #E_NO_ERROR if everything is successful.
 */
inline int _pixel_format_to_payload(pixel_format_t pixel_format, mxc_csi2_payload0_t *payload0,
                                    mxc_csi2_payload1_t *payload1)
{
    // payload1 enables user-defined payload types.  If these are enabled,
    // payload0 should be disabled.  PL0 and PL1 can not be active simultaneously.
    *payload1 = MXC_CSI2_PL1_DISABLE_ALL;

    switch (pixel_format) {
    case PIXEL_FORMAT_YUV420:
        *payload0 = MXC_CSI2_PL0_YUV420_8BIT;
        break;
    case PIXEL_FORMAT_YUV422:
        *payload0 = MXC_CSI2_PL0_YUV420_8BIT;
        break;
    case PIXEL_FORMAT_RGB444:
        *payload0 = MXC_CSI2_PL0_RGB444;
        break;
    case PIXEL_FORMAT_RGB555:
        *payload0 = MXC_CSI2_PL0_RGB555;
        break;
    case PIXEL_FORMAT_RGB565:
        *payload0 = MXC_CSI2_PL0_RGB565;
        break;
    case PIXEL_FORMAT_RGB666:
        *payload0 = MXC_CSI2_PL0_RGB666;
        break;
    case PIXEL_FORMAT_RGB888:
        *payload0 = MXC_CSI2_PL0_RGB888;
        break;
    case PIXEL_FORMAT_RAW6:
        *payload0 = MXC_CSI2_PL0_RAW6;
        break;
    case PIXEL_FORMAT_RAW7:
        *payload0 = MXC_CSI2_PL0_RAW7;
        break;
    case PIXEL_FORMAT_RAW8:
        *payload0 = MXC_CSI2_PL0_RAW8;
        break;
    case PIXEL_FORMAT_RAW10:
        *payload0 = MXC_CSI2_PL0_RAW10;
        break;
    case PIXEL_FORMAT_RAW12:
        *payload0 = MXC_CSI2_PL0_RAW12;
        break;
    case PIXEL_FORMAT_RAW14:
        *payload0 = MXC_CSI2_PL0_RAW14;
        break;
    case PIXEL_FORMAT_RAW16:
        *payload0 = MXC_CSI2_PL0_RAW16;
        break;
    case PIXEL_FORMAT_RAW20:
        *payload0 = MXC_CSI2_PL0_RAW20;
        break;
    case PIXEL_FORMAT_BYPASS:
        // For bypass mode, we will select a null payload type.
        // See section 5.10.6.1 of our CSI2 IP spec for more details.
        *payload0 = MXC_CSI2_PL0_NULL;
        break;
    default:
        return E_NOT_SUPPORTED;
    }

    return E_NO_ERROR;
}

inline unsigned int _bits_per_pixel(pixel_format_t pixel_format)
{
    switch (pixel_format) {
    case PIXEL_FORMAT_YUV420:
        return 12;
    case PIXEL_FORMAT_YUV422:
        return 16;
    case PIXEL_FORMAT_RGB444:
        return 12;
    case PIXEL_FORMAT_RGB555:
        return 15;
    case PIXEL_FORMAT_RGB565:
        return 16;
    case PIXEL_FORMAT_RGB666:
        return 18;
    case PIXEL_FORMAT_RGB888:
        return 24;
    case PIXEL_FORMAT_RAW6:
        return 6;
    case PIXEL_FORMAT_RAW7:
        return 7;
    case PIXEL_FORMAT_RAW8:
        return 8;
    case PIXEL_FORMAT_RAW10:
        return 10;
    case PIXEL_FORMAT_RAW12:
        return 12;
    case PIXEL_FORMAT_RAW14:
        return 14;
    case PIXEL_FORMAT_RAW16:
        return 16;
    case PIXEL_FORMAT_RAW20:
        return 20;
    case PIXEL_FORMAT_BYPASS:
        // Section 5.10.6.1 of our CSI2 IP spec says that general/arbitrary
        // data types are all byte data types.
        return 8;
    default:
        return 8;
    }
}

int mipi_camera_init(mipi_camera_settings_t camera_settings)
{
    int error = 0;

    // Store camera settings to global scope
    g_camera_settings = camera_settings;

    // Initialize sensor I2C configuration interface
    error = sccb_init();
    if (error)
        return error;

    // Link function pointers from camera drivers
    mipi_sensor_register(&camera);

    error = camera.init();
    if (error)
        return error;

    // Note: There are some register writes that are causing the
    // reset to return an error. We will ignore the error for now.
    camera.reset();

    error = camera.set_pixformat(camera_settings.camera_format);
    if (error)
        return error;

    error = camera.set_framesize(camera_settings.width, camera_settings.height);
    if (error)
        return error;

    // Configure RX Controller and PPI (D-PHY)
    g_ctrl_cfg.invert_ppi_clk = MXC_CSI2_PPI_NO_INVERT;
    g_ctrl_cfg.num_lanes = NUM_DATA_LANES;
    g_ctrl_cfg.flush_cnt = FLUSH_COUNT;
    _pixel_format_to_payload(camera_settings.camera_format.pixel_format, &g_ctrl_cfg.payload0,
                             &g_ctrl_cfg.payload1);

    g_ctrl_cfg.lane_src.d0_swap_sel = MXC_CSI2_PAD_CDRX_PN_L0;
    g_ctrl_cfg.lane_src.d1_swap_sel = MXC_CSI2_PAD_CDRX_PN_L1;
    g_ctrl_cfg.lane_src.d2_swap_sel = MXC_CSI2_PAD_CDRX_PN_L2;
    g_ctrl_cfg.lane_src.d3_swap_sel = MXC_CSI2_PAD_CDRX_PN_L3;
    g_ctrl_cfg.lane_src.c0_swap_sel = MXC_CSI2_PAD_CDRX_PN_L4;

    // Configure CSI2 request options
    g_req.pixels_per_line = camera_settings.width;
    g_req.lines_per_frame = camera_settings.height;
    unsigned int bits_per_pixel = _bits_per_pixel(camera_settings.camera_format.pixel_format);
    g_req.bits_per_pixel_odd = bits_per_pixel;
    g_req.bits_per_pixel_even = bits_per_pixel; // TODO(Jake): Support uneven bit lengths
    g_req.frame_num = 1; // TODO(Jake): Support multi-frame exposures

    // Disable RAW to RGB conversions.  AI87 RevA hardware has major issues with its built-in debayering hardware.
    g_req.process_raw_to_rgb = false;
    // The following options are only used when the debayering engine
    // is enabled.
    // g_req.raw_format = RAW_FORMAT; // Specify the input format received from the sensor
    // g_req.rgb_type = RGB_TYPE; // Specify the desired output format
    g_req.autoflush = MXC_CSI2_AUTOFLUSH_ENABLE;

    // Link callback functions
    g_req.line_handler = camera_settings.line_handler;
    // g_req.callback = CSI2_Callback;

    // Configure VFIFO
    g_vfifo_cfg.virtual_channel = VIRTUAL_CHANNEL;
    g_vfifo_cfg.rx_thd = RX_THRESHOLD;
    g_vfifo_cfg.wait_cyc = WAIT_CYCLE;
    g_vfifo_cfg.flow_ctrl = FLOW_CTRL;
    g_vfifo_cfg.err_det_en = MXC_CSI2_ERR_DETECT_DISABLE;
    g_vfifo_cfg.fifo_rd_mode = MXC_CSI2_READ_ONE_BY_ONE;
    g_vfifo_cfg.dma_whole_frame = MXC_CSI2_DMA_LINE_BY_LINE;
    g_vfifo_cfg.dma_mode = MXC_CSI2_DMA_FIFO_ABV_THD;
    g_vfifo_cfg.bandwidth_mode = MXC_CSI2_NORMAL_BW;
    g_vfifo_cfg.wait_en = MXC_CSI2_AHBWAIT_ENABLE;

    error = MXC_CSI2_Init(&g_req, &g_ctrl_cfg, &g_vfifo_cfg);
    if (error != E_NO_ERROR) {
        return error;
    }

    return error;
}

int mipi_camera_write_reg(uint16_t reg_addr, uint8_t reg_data)
{
    return camera.write_reg(reg_addr, reg_data);
}

int mipi_camera_read_reg(uint16_t reg_addr, uint8_t *reg_data)
{
    return camera.read_reg(reg_addr, reg_data);
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

int mipi_camera_sleep(int sleep)
{
    return camera.sleep(sleep);
}

int mipi_camera_capture()
{
    return MXC_CSI2_CaptureFrameDMA();
}

mipi_camera_settings_t mipi_camera_get_camera_settings()
{
    return g_camera_settings;
}

mxc_csi2_capture_stats_t mipi_camera_get_capture_stats(void)
{
    return MXC_CSI2_GetCaptureStats();
}

// *IMG* [PIXEL_FORMAT] [LENGTH (in bytes)] [WIDTH (in pixels)] [HEIGHT (in pixels)]
char *mipi_camera_get_image_header(void)
{
    memset(g_image_header, '\0', sizeof(g_image_header));
    uint32_t img_len, width, height;

    MXC_CSI2_GetImageDetails(&img_len, &width, &height);

    char *format_str;
    pixel_format_t pixel_format = g_camera_settings.camera_format.pixel_format;
    if (pixel_format == PIXEL_FORMAT_RGB444) {
        format_str = "RGB444";
    } else if (pixel_format == PIXEL_FORMAT_RGB555) {
        format_str = "RGB555";
    } else if (pixel_format == PIXEL_FORMAT_RGB565) {
        format_str = "RGB565";
    } else if (pixel_format == PIXEL_FORMAT_RGB888) {
        format_str = "RGB888";
    } else if (pixel_format == PIXEL_FORMAT_YUV422) {
        format_str = "YUV422";
    } else if (pixel_format == PIXEL_FORMAT_RAW8) {
        format_str = "BAYER";
    } else {
        format_str = "INVALID";
    }

    snprintf(g_image_header, sizeof(g_image_header), "*IMG* %s %i %i %i",
             format_str, // PIXEL_FORMAT
             img_len, // LENGTH (in bytes)
             width, // WIDTH (in pixels)
             height); // HEIGHT (in pixels)

    return g_image_header;
}

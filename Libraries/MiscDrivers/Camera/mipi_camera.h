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
#include "csi2.h"

/**
 * @brief Pixel format enumerations.
*/
typedef enum pixel_format {
    PIXEL_FORMAT_BYPASS,
    PIXEL_FORMAT_YUV420,
    PIXEL_FORMAT_YUV422,
    PIXEL_FORMAT_RGB444,
    PIXEL_FORMAT_RGB555,
    PIXEL_FORMAT_RGB565,
    PIXEL_FORMAT_RGB666,
    PIXEL_FORMAT_RGB888,
    PIXEL_FORMAT_RAW6,
    PIXEL_FORMAT_RAW7,
    PIXEL_FORMAT_RAW8,
    PIXEL_FORMAT_RAW10,
    PIXEL_FORMAT_RAW12,
    PIXEL_FORMAT_RAW14,
    PIXEL_FORMAT_RAW16,
    PIXEL_FORMAT_RAW20
} pixel_format_t;

/**
 * @brief Pixel format bit order enumerations
*/
typedef enum pixel_order {
    PIXEL_ORDER_DEFAULT,
    PIXEL_ORDER_RAW_BGGR,
    PIXEL_ORDER_RAW_GBRG,
    PIXEL_ORDER_RAW_GRBG,
    PIXEL_ORDER_RAW_RGGB,
    PIXEL_ORDER_RGB565_BGR,
    PIXEL_ORDER_RGB565_RGB,
    PIXEL_ORDER_RGB565_GRB,
    PIXEL_ORDER_RGB565_BRG,
    PIXEL_ORDER_RGB565_GBR,
    PIXEL_ORDER_RGB565_RBG,
    // TODO(Jake): Support other pixel orders
} pixel_order_t;

/**
 * @brief Camera format struct describing the desired pixel format and ordering
*/
typedef struct _mipi_camera_format {
    pixel_format_t pixel_format;
    pixel_order_t pixel_order;
} mipi_camera_format_t;

/**
 * @brief Camera settings struct.  This is the "top-level" configuration struct
 * encapsulating all camera settings.  Pass this in to the @ref mipi_camera_init function.
*/
typedef struct _mipi_camera_settings_t {
    unsigned int width; /**< Image width in pixels */
    unsigned int height; /**< Image height in pixels */
    mipi_camera_format_t camera_format; /**< Image format */
    mxc_csi2_line_handler_cb_t
        line_handler; /**< Line handler provided by application to process incoming camera data */
} mipi_camera_settings_t;

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
    int (*set_pixformat)(mipi_camera_format_t camera_format);
    int (*get_pixformat)(mipi_camera_format_t *camera_format);
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

/**
 * @brief Initialize the MIPI camera with the specified settings.
 * @param[in] camera_settings Camera settings to use.
 * @return E_NO_ERROR (0) on success, non-zero @ref MXC_Error_Codes on failure
 */
int mipi_camera_init(mipi_camera_settings_t camera_settings);

/**
 * @brief Write a value to a MIPI camera register
 * @param[in] reg_addr Register to write to
 * @param[in] reg_data Value to write
 * @return 0 on success, non-zero on failure
 */
int mipi_camera_write_reg(uint16_t reg_addr, uint8_t reg_data);

/**
 * @brief Read a value from a MIPI camera register
 * @param[in] reg_addr Register to read from
 * @param[out] reg_data Output location for the read data
 * @return 0 on success, non-zero on failure
 */
int mipi_camera_read_reg(uint16_t reg_addr, uint8_t *reg_data);

/**
 * @brief Get the slave address of the MIPI camera
 * @return The slave address of the camera
 */
int mipi_camera_get_slave_address(void);

/**
 * @brief Get the product ID of the MIPI camera.
 * @param[out] id Output location for the product ID
 * @return E_NO_ERROR (0) on success, non-zero @ref MXC_Error_Codes on failure
 */
int mipi_camera_get_product_id(int *id);

/**
 * @brief Get the manufacture ID of the MIPI camera
 * @param[out] id Output location for the manufacture ID
 * @return E_NO_ERROR (0) on success, non-zero @ref MXC_Error_Codes on failure
 */
int mipi_camera_get_manufacture_id(int *id);

/**
 * @brief Put the MIPI camera to sleep or wake it up
 * @param[in] sleep 1 to sleep, 0 to wake up
 * @return E_NO_ERROR (0) on success, non-zero @ref MXC_Error_Codes on failure
 */
int mipi_camera_sleep(int sleep);

/**
 * @brief Capture an image from the MIPI camera.  This function will block
 * until the frame is complete or an error is received.  During the capture,
 * the line handler function passed to @ref mipi_camera_init is expected to unload
 * the received data in time for each new row.
 * @return 0 (E_NO_ERROR) on a successful capture of a full frame, non-zero @ref MXC_Error_Codes on failure.
 */
int mipi_camera_capture();

/**
 * @brief Retrieve the current camera settings
 * @return The current camera settings
 */
mipi_camera_settings_t mipi_camera_get_camera_settings(void);

/**
 * @brief Get detailed statistics about the last image capture and any errors that occurred.
 * @return A @ref mxc_csi2_capture_stats_t struct
 */
mxc_csi2_capture_stats_t mipi_camera_get_capture_stats(void);

/**
 * @brief Parse a string containing information about the last captured image.
 * This is used with console.py utilities.
 * @details The string will have the following format (ignore quotes ""):
 * 
 * "*IMG* [PIXEL_FORMAT] [LENGTH (in bytes)] [WIDTH (in pixels)] [HEIGHT (in pixels)]"
 * 
 * @return The image header of the last captured image
 */
char *mipi_camera_get_image_header(void);

#endif // LIBRARIES_MISCDRIVERS_CAMERA_MIPI_CAMERA_H_

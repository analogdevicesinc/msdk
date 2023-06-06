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

#include <stdio.h>
#include <stdlib.h>
#include "mxc_delay.h"
#include "aps6404.h"
#include "led.h"
#include "camera.h"
#include "fastspi.h"
#include "tft_st7789v.h"
#include "tmr.h"

#define CONVERSION_BUFFER_SIZE (IMAGE_WIDTH * 2)
// ^ 2 bytes per pixel for RGB565.

unsigned int g_index = 0;
int error;
int id;

int CSI2_line_handler(uint8_t *data, unsigned int len)
{
    ram_write_quad(g_index, data, len);
    g_index += len;
    return E_NO_ERROR;
}

void camera_capture(void)
{
    printf("Capturing image...\n");
    spi_init();
    ram_enter_quadmode();

    g_index = 0;
    MXC_TMR_SW_Start(MXC_TMR1);
    int error = mipi_camera_capture();
    unsigned int elapsed = MXC_TMR_SW_Stop(MXC_TMR1);

    if (error) {
        printf("Failed!\n");
        mxc_csi2_capture_stats_t stats = mipi_camera_get_capture_stats();
        printf(
            "Failed (%u/%u bytes received)!\nCTRL Error flags: 0x%x\tPPI Error flags: 0x%x\tVFIFO Error flags: 0x%x\n",
            stats.bytes_captured, stats.frame_size, stats.ctrl_err, stats.ppi_err, stats.vfifo_err);
        return;
    }

    printf("Done! (took %i us)\n", elapsed);
}

void camera_capture_and_load_cnn(void)
{
    camera_capture();
    mxc_csi2_capture_stats_t stats = mipi_camera_get_capture_stats();

    /*
    The CNN model needs RGB888, but the camera outputs RGB565.  Additionally,
    the camera data is buffered in the external QSPI SRAM.  So we require an
    intermediate buffer to perform the RGB565 -> RGB888 conversion before passing
    to the CNN FIFO.
    */
    uint8_t rgb565_buffer[CONVERSION_BUFFER_SIZE];
    uint8_t r5 = 0, g6 = 0, b5 = 0;
    uint8_t r = 0, g = 0, b = 0;
    uint32_t packed = 0;

    printf("Loading CNN...\n");
    MXC_TMR_SW_Start(MXC_TMR1);
    /* 
    The QSPI SRAM shares the same bus as the TFT display.  SPI is reinitialized here for QSPI
    operation in case the TFT display was being used previously.
    */
    spi_init();
    ram_enter_quadmode();

    for (int i = SRAM_ADDRESS; i < stats.frame_size;
         i += CONVERSION_BUFFER_SIZE) { // for every SRAM chunk
        ram_read_quad(i, rgb565_buffer, CONVERSION_BUFFER_SIZE);
        for (int j = 0; j < CONVERSION_BUFFER_SIZE; j += 2) { // for each RGB565 byte pair
            // Decode RGB565
            r5 = (rgb565_buffer[j] & 0b11111000) >> 3;
            g6 = ((rgb565_buffer[j] & 0b111) << 3) | ((rgb565_buffer[j + 1] & 0b11100000) >> 5);
            b5 = ((rgb565_buffer[j + 1]) & 0b11111);

            // RGB565 -> RGB888 (signed)
            r = (r5 << 3) - 128;
            g = (g6 << 2) - 128;
            b = (b5 << 3) - 128;

            // Pack into 32-bit word (0x00BBGGRR)
            packed = r | (g << 8) | (b << 16);

            // Remove the following line if there is no risk that the source would overrun the FIFO:
            while (((*((volatile uint32_t *)0x50000004) & 1)) != 0) {}
            // Wait for FIFO 0
            *((volatile uint32_t *)0x50000008) = packed; // Write FIFO 0
        }
    }
    int elapsed = MXC_TMR_SW_Stop(MXC_TMR1);
    printf("Done! (took %i us)\n", elapsed);
}

void camera_display_last_image(void)
{
    /* 
    This function loads the last image captured by the camera into the TFT display row by row.
    The QSPI SRAM shares the same bus as the TFT display, but requires different
    bus settings and control schemes.  Therefore SPI is reinitialized on the fly so that both 
    can be operated simultaneously.
    */
    uint8_t tft_buffer[IMAGE_WIDTH * 2];
    unsigned int address = SRAM_ADDRESS;
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        spi_init();
        ram_enter_quadmode();
        ram_read_quad(address, tft_buffer, IMAGE_WIDTH * 2);
        TFT_SPI_Init();
        MXC_TFT_WriteBufferRGB565(0, y, tft_buffer, IMAGE_WIDTH, 1);
        address += IMAGE_WIDTH * 2;
    }
}

bool camera_init()
{
    mipi_camera_settings_t camera_settings = { .width = IMAGE_WIDTH,
                                               .height = IMAGE_HEIGHT,
                                               .camera_format = { .pixel_format = PIXEL_FORMAT,
                                                                  .pixel_order = PIXEL_ORDER },
                                               .line_handler = CSI2_line_handler };

    mipi_camera_init(camera_settings);

    mipi_camera_get_product_id(&id);
    printf("Camera ID = %x\n", id);
    if (id != CAMERA_ID) {
        printf("Incorrect camera.\n");
        LED_On(1);
        return false;
    }

    printf("Initializing SRAM...\n");
    error = ram_init();
    if (error) {
        printf("Failed to initialize SRAM with error %i\n", error);
        return false;
    }

    ram_id_t ram_id;
    error = ram_read_id(&ram_id);
    if (error) {
        printf("Failed to read expected SRAM ID!\n");
        return false;
    }
    printf("RAM ID:\n\tMFID: 0x%.2x\n\tKGD: 0x%.2x\n\tDensity: 0x%.2x\n\tEID: 0x%x\n", ram_id.MFID,
           ram_id.KGD, ram_id.density, ram_id.EID);

    return true;
}
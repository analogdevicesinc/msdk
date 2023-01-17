/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
/**
 * @file    main.c
 * @brief   Example and utility for capturing an image using the PCIF interface and Camera drivers.
 *          The captured image can then be streamed over UART, or saved to an SD card.
 *          This firmware should be paired with the "pc_utility/console.py" program, which
 *          drives this firmware via a set of commands the firmware offers.
 *
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "uart.h"
#include "led.h"
#include "board.h"

#include "camera.h"
#include "cameraif.h"
#include "dma.h"
#include "example_config.h"
#include "cnn_memutils.h"

#ifndef ENABLE_TFT
#include "console.h"
#endif

// This describes a complete image for a standard blocking capture
typedef struct {
    uint8_t *raw; // Pointer to raw img data in SRAM.
    uint32_t imglen; // Length of img data (in bytes)
    uint32_t w; // Width of the image (in pixels)
    uint32_t h; // Height of the image (in pixels)
    uint8_t *pixel_format; // Pixel format string
} img_data_t;

// This describes a complete image for a streaming capture saved
// to CNN data SRAM
typedef struct {
    uint32_t *raw; // Pointer to raw img data in CNN data SRAM.
    uint32_t imglen; // Length of img data (in bytes)
    uint32_t w; // Width of the image (in pixels)
    uint32_t h; // Height of the image (in pixels)
    uint8_t *pixel_format; // Pixel format string
} cnn_img_data_t;

typedef enum { BAYER_FUNCTION_PASSTHROUGH = 0, BAYER_FUNCTION_BILINEAR } bayer_function_t;

// This contains global application settings
typedef struct {
    int dma_channel;
    dmamode_t dma_mode;
    unsigned int imgres_w;
    unsigned int imgres_h;
    pixformat_t pixel_format;
    bayer_function_t bayer_function;
} app_settings_t;

app_settings_t g_app_settings;

/**
* @brief Capture an image using a standard blocking PCIF capture.
* @param[in] w Set the width of the image (in pixels)
* @param[in] h Set the height of the image (in pixels)
* @param[in] pixel_format Set the pixel format.  See 'pixformat_t' in 'camera.h'
* @param[in] dma_mode Set the dma mode format.  Should be either "USE_DMA" or "NO_DMA" for standard captures.
* @param[in] dma_channel DMA channel to use if DMA mode is "USE_DMA".  Must be acquired by the application first.
* @return "img_data_t" struct describing the captured image.  If the "raw" struct member is NULL, the image capture
* failed.
* @details 
    An SRAM buffer is allocated to hold the entire image, limiting the max resolution to the available memory on the device.
    This function contains the full setup sequence for capturing an image.  
    In practice, step 1 can be broken out into a separate initialization 
    sequence if you're not reconfiguring the camera settings on the fly.
****************************************************************************/
img_data_t capture_img(uint32_t w, uint32_t h, pixformat_t pixel_format, dmamode_t dma_mode,
                       int dma_channel)
{
    img_data_t img_data;

    // 1. Configure the camera with the 'camera_setup' function.
    // Image dimensions should fall within the limitations
    // of the camera hardware and MCU SRAM limits.  In this simple capture mode the
    // camera.h drivers will allocate an SRAM buffer whose size is equal to
    // width * height * bytes_per_pixel.  See camera.c for implementation details.
    printf("Configuring camera\n");
    int ret = camera_setup(w, // width
                           h, // height
                           pixel_format, // pixel format
                           FIFO_FOUR_BYTE, // FIFO mode (four bytes is suitable for most cases)
                           dma_mode, // DMA (enabling DMA will drastically decrease capture time)
                           dma_channel); // Allocate the DMA channel retrieved in initialization

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
        img_data.raw = NULL;
        return img_data;
    }

    // 2. Start the camera capture with the 'camera_start_capture_image()' function.
    // This will 'launch' the image capture, but is non-blocking.  The CameraIF peripheral's
    // hardware handles the capture in the background.
    printf("Capturing image\n");
    MXC_TMR_SW_Start(MXC_TMR0);
    camera_start_capture_image();

    // 3. Wait until the image is fully received.
    while (!camera_is_image_rcv()) {}
    int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %i us)\n", elapsed_us);

    // 4. Retrieve details of the captured image from the camera driver with the
    // 'camera_get_image' function.  We don't need to copy any image data here, we'll
    // just retrieve a pointer to the camera driver's internal SRAM buffer.
    img_data.pixel_format = camera_get_pixel_format(); // Retrieve the pixel format of the image
    camera_get_image(&img_data.raw, &img_data.imglen, &img_data.w,
                     &img_data.h); // Retrieve info using driver function.
    printf("Captured %ux%u %s image to buffer location 0x%x (%i bytes)\n", img_data.w, img_data.h,
           img_data.pixel_format, img_data.raw, img_data.imglen);

    // 5. At this point, "img_data.raw" is pointing to the fully captured
    // image data, and all the info needed to decode it has been collected.
    return img_data;
}

/**
* @brief Transmit an image that has been captured with 'capture_img' over UART.
* @param[in] img_data The cnn_img_data_t struct describing the stored image.  This
* struct will be provided from a call to 'stream_img'.
* @details
    This function will send a header first, then the raw image data.  The header is
    of the following format:
    *IMG* [PIXEL FORMAT] [LENGTH (in bytes)] [WIDTH (in pixels)] [HEIGHT (in pixels)]

    When the console receives a header of this format, it enters into a "receive raw bytes"
    mode that listens for LENGTH bytes.  
    
    This is necessary because the raw image data has a non-zero probability to contain any 
    sequence of ASCII characters, including '\n', '\r\n', etc.
    So the console cannot rely on the same 'readline()' polling as it does for standard
    commands.
****************************************************************************/
void transmit_capture_uart(img_data_t img_data)
{
    if (img_data.raw != NULL) {
        // Send the image data over the serial port...
        MXC_TMR_SW_Start(MXC_TMR0);

        // First, tell the host that we're about to send the image.
        clear_serial_buffer();
        snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
                 "*IMG* %s %i %i %i", // Format img info into a string
                 img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);
        send_msg(g_serial_buffer);

        // The console should now be expecting to receive 'imglen' bytes.

        // Since standard image captures are buffered into SRAM, sending them
        // over the serial port is straightforward...
        clear_serial_buffer();
        MXC_UART_WriteBytes(Con_Uart, img_data.raw, img_data.imglen);

        int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
        printf("Done! (serial transmission took %i us)\n", elapsed);
    }
}

/**
* @brief Capture an image using DMA streaming mode.  This method only requires two
* rows worth of SRAM, allowing for high image resolutions.
* @param[in] w Set the width of the image (in pixels)
* @param[in] h Set the height of the image (in pixels)
* @param[in] pixel_format Set the pixel format.  See 'pixformat_t' in 'camera.h'
* @param[in] dma_channel DMA channel to use if DMA mode is "USE_DMA".  Must be acquired by the application first.
* @return "img_data_t" struct describing the captured image.  If the "raw" struct member is NULL, the image capture
* failed.
* @details 
    This method has a much lower memory footprint, but dramatically increased 
    timing requirements for higher resolutions.  Image data must be streamed to
    a destination with a high enough "sink" datarate to deal with the massive "source" 
    datarate coming from the PCIF.

    For the sake of this example, image data is streamed directly into the CNN
    accelerator's data SRAM.  **This is not the correct way to load data for inferences**
    The CNN accelerator is the only device with enough memory to store high resolution
    images, so this example essentially turns it into a giant SRAM buffer.
    This method allows for collecting high resolution images at full speed to send
    to any arbitrary lower bandwidth output destination.
****************************************************************************/
cnn_img_data_t stream_img(uint32_t w, uint32_t h, pixformat_t pixel_format, int dma_channel)
{
    // Enable CNN accelerator memory.
    // CNN clock: APB (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_init();

    cnn_img_data_t img_data;

    // Resolution check.  This method only supports resolutions that are multiples of 32.
    // Additionally, resolutions beyond 352x352 may result in image artifacts.
    if ((w * h) % 32 != 0) {
        img_data.raw = NULL;
        printf("Failed to stream!  Image resolutions must be multiples of 32.\n");
        return img_data;
    }

    // 1. Configure the camera.  This is the same as the standard blocking capture, except
    // the DMA mode is set to "STREAMING_DMA".
    printf("Configuring camera\n");
    int ret = camera_setup(w, // width
                           h, // height
                           pixel_format, // pixel format
                           FIFO_FOUR_BYTE, // FIFO mode
                           STREAMING_DMA, // Set streaming mode
                           dma_channel); // Allocate the DMA channel retrieved in initialization

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
        img_data.raw = NULL;
        return img_data;
    }

    // 2. Retrieve image format and info.
    img_data.pixel_format = camera_get_pixel_format(); // Retrieve the pixel format of the image
    camera_get_image(NULL, &img_data.imglen, &img_data.w,
                     &img_data.h); // Retrieve info using driver function.
    img_data.raw = (uint32_t *)
        CNN_QUAD0_DSRAM_START; // Manually save the destination address at the first quadrant of CNN data SRAM

    printf("Starting streaming capture...\n");
    MXC_TMR_SW_Start(MXC_TMR0);

    // 3. Start streaming
    camera_start_capture_image();

    uint8_t *data = NULL;
    int buffer_size = camera_get_stream_buffer_size();
    uint32_t *cnn_addr = img_data.raw; // Hard-coded to Quadrant 0 starting address

    // 4. Process the incoming stream data.
    while (!camera_is_image_rcv()) {
        if ((data = get_camera_stream_buffer()) !=
            NULL) { // The stream buffer will return 'NULL' until an image row is received.
            // 5. Unload buffer
            cnn_addr = write_bytes_to_cnn_sram(data, buffer_size, cnn_addr);
            // 6. Release buffer in time for next row
            release_camera_stream_buffer();
        }
    }

    int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %i us)\n", elapsed_us);

    // 7. Check for any overflow
    stream_stat_t *stat = get_camera_stream_statistic();
    printf("DMA transfer count = %d\n", stat->dma_transfer_count);
    printf("OVERFLOW = %d\n", stat->overflow_count);

    return img_data;
}

/**
* @brief Transmit an image that has been captured with 'stream_img' over UART.
* @param[in] img_data The cnn_img_data_t struct describing the stored image.  This
* struct will be provided from a call to 'stream_img'.
* @details
    This function will send a header first, then the raw image data.  The header is
    of the following format:
    *IMG* [PIXEL FORMAT] [LENGTH (in bytes)] [WIDTH (in pixels)] [HEIGHT (in pixels)]

    When the console receives a header of this format, it enters into a "receive raw bytes"
    mode that listens for LENGTH bytes.  
    
    This is necessary because the raw image data has a non-zero probability to contain any 
    sequence of ASCII characters, including '\n', '\r\n', etc.
    So the console cannot rely on the same 'readline()' polling as it does for standard
    commands.
****************************************************************************/
void transmit_stream_uart(cnn_img_data_t img_data)
{
    if (img_data.raw !=
        NULL) { // If img_data.raw is NULL, then there was an error collecting the image.
        printf("Transmitting image data over UART...\n");
        MXC_TMR_SW_Start(MXC_TMR0);

        // Tell the host console we're about to send an image.
        clear_serial_buffer();
        snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
                 "*IMG* %s %i %i %i", // Format img info into a string
                 img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);
        send_msg(g_serial_buffer);

        // Console should be ready to receive raw bytes now.

        clear_serial_buffer();
        int transfer_len = SERIAL_BUFFER_SIZE;
        uint32_t *cnn_addr = img_data.raw;

        // Transfer the bytes out of CNN memory and into the serial buffer, then write.
        // Since the CNN data SRAM is non-contiguous, some pointer manipulation at the
        // quadrant boundaries is required.
        for (int i = 0; i < img_data.imglen; i += transfer_len) {
            cnn_addr = read_bytes_from_cnn_sram((uint8_t *)g_serial_buffer, transfer_len, cnn_addr);
            MXC_UART_WriteBytes(Con_Uart, (uint8_t *)g_serial_buffer, transfer_len);
        }

        int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
        printf("Done! (serial transmission took %i us)\n", elapsed);
    }
}

#ifdef SD
/**
* @brief Save an image that has been captured with 'stream_img' to an SD card.
* @param[in] img_data The cnn_img_data_t struct describing the stored image.  This
* struct will be provided from a call to 'stream_img'.
* @param[in] file Filename to save to.
* @details
    The same image info header used for UART streaming will also be written to the
    first line of the raw image file.  Then, the raw image data will be written.
    This allows the raw image data to be decoded later.  
    
    It also allows you to quickly verify that the image has been saved to the SD card 
    with the "cat" command, which prints out the contents of a file.  The 
    Python console is always checking for the *IMG* ... header format.  So "cat"
    will print out the image header followed by the raw image data, just like
    'transmit_stream_uart'.  As a result, this can be used to read an image file off
    the SD card and the console will save it as a .png on the host PC.
****************************************************************************/
void save_stream_sd(cnn_img_data_t img_data, char *file)
{
    if (img_data.raw != NULL) { // Image data will be NULL if something went wrong during streaming
        // If file is NULL, find the next available file to save to.
        if (file == NULL) {
            int i = 0;

            for (;;) {
                // We'll use the global sd_filename buffer for this and
                // try to find /raw/imgN while incrementing N.
                memset(sd_filename, '\0', sizeof(sd_filename));
                snprintf(sd_filename, sizeof(sd_filename), "/raw/img%u", i++);
                sd_err = f_stat(sd_filename, &sd_fno);
                if (sd_err == FR_NO_FILE) {
                    file = sd_filename; // Point 'file' to the available path string
                    break;
                } else if (sd_err != FR_OK) {
                    printf("Error while searching for next available file: %s\n",
                           FR_ERRORS[sd_err]);
                    break;
                }
            }
        }

        sd_err = f_open(&sd_file, (const TCHAR *)file, FA_WRITE | FA_CREATE_NEW);

        if (sd_err != FR_OK) {
            printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
        } else {
            printf("Saving image to %s\n", file);

            MXC_TMR_SW_Start(MXC_TMR0);

            // Write image info as the first line of the file.
            clear_serial_buffer();
            snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
                     "*IMG* %s %i %i %i\n", // Format img info into a new-line terminated string
                     img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);

            unsigned int wrote = 0;
            sd_err = f_write(&sd_file, g_serial_buffer, strlen(g_serial_buffer), &wrote);
            if (sd_err != FR_OK || wrote != strlen(g_serial_buffer)) {
                printf("Failed to write header to file: %s\n", FR_ERRORS[sd_err]);
            }
            clear_serial_buffer();

            // Similar to streaming over UART, a secondary buffer is needed to
            // save the raw data to the SD card since the CNN data SRAM is non-contiguous.
            // Raw image data is written row by row.
            uint32_t *cnn_addr = img_data.raw;
            uint8_t *buffer = (uint8_t *)malloc(img_data.w);
            for (int i = 0; i < img_data.imglen; i += img_data.w) {
                cnn_addr = read_bytes_from_cnn_sram(buffer, img_data.w, cnn_addr);
                sd_err = f_write(&sd_file, buffer, img_data.w, &wrote);

                if (sd_err != FR_OK || wrote != img_data.w) {
                    printf("Failed to image data to file: %s\n", FR_ERRORS[sd_err]);
                }

                // Print progress %
                if (i % (img_data.w * 32) == 0) {
                    printf("%.1f%%\n", ((float)i / img_data.imglen) * 100.0f);
                }
            }
            free(buffer);
            int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
            printf("Finished (took %ius)\n", elapsed);
        }
        f_close(&sd_file);
    }
}
#endif

#ifdef CONSOLE
/**
* @brief Receive and service any received console commands.
****************************************************************************/
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
        } else if (cmd == CMD_IMGRES) {
            sscanf(g_serial_buffer, "imgres %u %u", &g_app_settings.imgres_w,
                   &g_app_settings.imgres_h);
            printf("Set image resolution to width %u, height %u\n", g_app_settings.imgres_w,
                   g_app_settings.imgres_h);
        } else if (cmd == CMD_CAPTURE) {
            // Perform a blocking image capture with the current camera settings.
            img_data_t img_data = capture_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                                              g_app_settings.pixel_format, g_app_settings.dma_mode,
                                              g_app_settings.dma_channel);

#ifdef CAMERA_BAYER
            uint8_t *bayer_data = (uint8_t *)malloc(img_data.w * img_data.h * 2);
            if (bayer_data != NULL) {
                MXC_TMR_SW_Start(MXC_TMR0);
                if (g_app_settings.bayer_function == BAYER_FUNCTION_PASSTHROUGH) {
                    bayer_passthrough(img_data.raw, img_data.w, img_data.h, (uint16_t *)bayer_data);
                } else if (g_app_settings.bayer_function == BAYER_FUNCTION_BILINEAR) {
                    bayer_bilinear_demosaicing(img_data.raw, img_data.w, img_data.h,
                                               (uint16_t *)bayer_data);
                }

                // Point img_data to the new debayered RGB565 array, which is double
                // the size of the original
                img_data.raw = bayer_data;
                img_data.imglen *= 2;
                img_data.pixel_format = (uint8_t *)"RGB565";
                unsigned int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
                printf("Debayering complete. (Took %u us)\n", elapsed_us);
            } else {
                printf("Failed to allocate memory for debayering!\n");
                return;
            }
#endif

            transmit_capture_uart(img_data);

#ifdef CAMERA_BAYER
            memset(bayer_data, 0, img_data.imglen);
            free(bayer_data);
#endif
        } else if (cmd == CMD_STREAM) {
            // Perform a streaming image capture with the current camera settings.
            cnn_img_data_t img_data = stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                                                 g_app_settings.pixel_format,
                                                 g_app_settings.dma_channel);

            transmit_stream_uart(img_data);

            // Disable the CNN when unused to preserve power.
            cnn_disable();
        } else if (cmd == CMD_SETREG) {
            // Set a camera register
            unsigned int reg;
            unsigned int val;
            // ^ Declaring these as unsigned ints instead of uint8_t
            // avoids some issues caused by type-casting inside of sscanf.

            sscanf(g_serial_buffer, "%s %u %u", cmd_table[cmd], &reg, &val);
            printf("Writing 0x%x to camera reg 0x%x\n", val, reg);
            camera_write_reg(reg, val);
        } else if (cmd == CMD_GETREG) {
            // Read a camera register
            unsigned int reg;
            uint8_t val;
            sscanf(g_serial_buffer, "%s %u", cmd_table[cmd], &reg);
            camera_read_reg(reg, &val);
            snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE, "Camera reg 0x%x=0x%x", reg, val);
            send_msg(g_serial_buffer);
#ifdef CAMERA_BAYER
        } else if (cmd == CMD_SETDEBAYER) {
            char buffer[20] = "\0";
            sscanf(g_serial_buffer, "%s %s", cmd_table[cmd], buffer);
            if (!strcmp("passthrough", buffer)) {
                g_app_settings.bayer_function = BAYER_FUNCTION_PASSTHROUGH;
                printf("Set %s\n", buffer);
            } else if (!strcmp("bilinear", buffer)) {
                g_app_settings.bayer_function = BAYER_FUNCTION_BILINEAR;
                printf("Set %s\n", buffer);
            } else {
                printf("Unknown debayering function '%s'\n", buffer);
            }
#endif
#ifdef SD
        } else if (cmd == CMD_SD_MOUNT) {
            // Mount the SD card
            sd_err = sd_mount();
            if (sd_err == FR_OK) {
                sd_get_size();

                printf("Disk Size: %u bytes\n", sd_sectors_total / 2);
                printf("Available: %u bytes\n", sd_sectors_free / 2);
            }
        } else if (cmd == CMD_SD_UNMOUNT) {
            sd_unmount();
        } else if (cmd == CMD_SD_CWD) {
            // Get the current working directory
            sd_err = sd_get_cwd();
            if (sd_err == FR_OK) {
                printf("%s\n", sd_cwd);
            }
        } else if (cmd == CMD_SD_CD) {
            // Change the current working directory
            char *b = (char *)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "cd %s", b); // Parse the target directory
            sd_cd(b);
            free(b);
        } else if (cmd == CMD_SD_LS) {
            sd_ls();
        } else if (cmd == CMD_SD_MKDIR) {
            // Make a directory
            char *b = (char *)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "mkdir %s", b); // Parse the directory name
            sd_mkdir(b);
            free(b);
        } else if (cmd == CMD_SD_RM) {
            // Remove an item
            char *b = (char *)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "rm %s", b); // Parse the item name
            sd_rm(b);
            free(b);
        } else if (cmd == CMD_SD_TOUCH) {
            // Create a new empty file
            char *b = (char *)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "touch %s", b); // Parse the filepath
            sd_touch(b);
            free(b);
        } else if (cmd == CMD_SD_WRITE) {
            // Write a string to a file
            char *b = (char *)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "write %s \"%[^\"]", sd_filename,
                   b); // \"$[^\"] captures everything between two quotes ""
            sd_write_string(sd_filename, b);
            free(b);
        } else if (cmd == CMD_SD_CAT) {
            // Print the contents of a file
            sscanf(g_serial_buffer, "cat %s", (char *)sd_filename); // Parse the target file
            sd_cat(sd_filename);
        } else if (cmd == CMD_SD_SNAP) {
            // Stream and save an image to the SD card
            int args = sscanf(g_serial_buffer, "snap %s", (char *)sd_filename);

            cnn_img_data_t img_data = stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                                                 g_app_settings.pixel_format,
                                                 g_app_settings.dma_channel);

            if (args == 1) {
                save_stream_sd(img_data, sd_filename);
            } else {
                save_stream_sd(img_data, NULL);
            }

            cnn_disable();
#endif // #ifdef SD
        }

        // Clear the serial buffer for the next command
        clear_serial_buffer();
    }
}
#endif // #ifdef CONSOLE

// *****************************************************************************
int main(void)
{
    // Initialization...
    int ret = 0;
    int slaveAddress;
    int id;
    g_app_settings.dma_mode = USE_DMA;
    g_app_settings.imgres_w = IMAGE_XRES;
    g_app_settings.imgres_h = IMAGE_YRES;
    g_app_settings.pixel_format = PIXFORMAT_RGB565; // This default may change during initialization

#if defined(CAMERA_MONO)
    g_app_settings.pixel_format = PIXFORMAT_BAYER;
#endif

#if defined(CAMERA_BAYER)
    g_app_settings.pixel_format = PIXFORMAT_BAYER;
    g_app_settings.bayer_function = BAYER_FUNCTION_BILINEAR;
#endif

    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

#ifdef CONSOLE
    console_init();
#endif

#ifdef SD
    sd_mount();
#endif

    // Initialize DMA and acquire a channel for the camera interface to use
    printf("Initializing DMA\n");
    MXC_DMA_Init();
    g_app_settings.dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    printf("Initializing camera\n");
    camera_init(CAMERA_FREQ);

    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    ret = camera_get_manufacture_id(&id);
    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }
    printf("Camera ID detected: %04x\n", id);

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360_MONO) || defined(CAMERA_HM0360_COLOR) || \
    defined(CAMERA_OV5642)
    camera_set_hmirror(0);
    camera_set_vflip(0);
#endif

#ifdef CAMERA_HM0360_COLOR
    camera_write_reg(0x3024, 0); // Select context A (320x240)
#endif

    // *********************************END CLEANME*************************************

    printf("Ready!\n");

    // Main processing loop.
    while (1) {
#ifdef CONSOLE
        // Serial console is enabled, service commands
        service_console();
#endif

#ifdef SD
        // SD card is enabled.
        if (PB_Get(0)) { // Enable image capture by pushing pushbutton 0
            cnn_img_data_t img_data = stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                                                 g_app_settings.pixel_format,
                                                 g_app_settings.dma_channel);
            save_stream_sd(img_data, NULL);
        }
#endif
    }
}

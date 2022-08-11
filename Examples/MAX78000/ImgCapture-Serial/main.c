/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @brief   Example and utility for capturing an image using the PCIF interface and Camera drivers
 *          then streaming image data over a serial port.  Should be paired with the 
 *          "pc_utility/console.py" program.
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
    uint8_t* raw;          // Pointer to raw img data in SRAM.
    uint32_t imglen;       // Length of img data (in bytes)
    uint32_t w;            // Width of the image (in pixels)
    uint32_t h;            // Height of the image (in pixels)
    uint8_t* pixel_format; // Pixel format string
} img_data_t;

// This describes a complete image for a streaming capture saved
// to CNN data SRAM
typedef struct {
    uint32_t* raw;         // Pointer to raw img data in CNN data SRAM.
    uint32_t imglen;       // Length of img data (in bytes)
    uint32_t w;            // Width of the image (in pixels)
    uint32_t h;            // Height of the image (in pixels)
    uint8_t* pixel_format; // Pixel format string
} cnn_img_data_t;

// This struct contains global application settings
typedef struct {
    int dma_channel;
    dmamode_t dma_mode;
    unsigned int imgres_w;
    unsigned int imgres_h;
    pixformat_t pixel_format;
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
    int ret = camera_setup(w,              // width
                           h,              // height
                           pixel_format,   // pixel format
                           FIFO_FOUR_BYTE, // FIFO mode (four bytes is suitable for most cases)
                           dma_mode,   // DMA (enabling DMA will drastically decrease capture time)
                           dma_channel // Allocate the DMA channel retrieved in initialization
    );

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

    // 3. Wait until the image is fully received.  The camera drivers
    // will populate an SRAM buffer with the received camera data.  If the
    // buffer is accessed before the image is fully received, the image might
    // contain artifacts or partial captures.
    while (!camera_is_image_rcv())
        ;
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
    int ret = camera_setup(w,              // width
                           h,              // height
                           pixel_format,   // pixel format
                           FIFO_FOUR_BYTE, // FIFO mode
                           STREAMING_DMA,  // Set streaming mode
                           dma_channel     // Allocate the DMA channel retrieved in initialization
    );

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
    img_data.raw = (uint32_t*)
        CNN_QUAD0_DSRAM_START; // Manually save the destination address at the first quadrant of CNN data SRAM

    printf("Starting streaming capture...\n");
    MXC_TMR_SW_Start(MXC_TMR0);

    // 3. Start streaming
    camera_start_capture_image();

    uint8_t* data      = NULL;
    int buffer_size    = camera_get_stream_buffer_size();
    uint32_t* cnn_addr = img_data.raw; // Hard-coded to Quadrant 0 starting address

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
    stream_stat_t* stat = get_camera_stream_statistic();
    printf("DMA transfer count = %d\n", stat->dma_transfer_count);
    printf("OVERFLOW = %d\n", stat->overflow_count);

    return img_data;
}

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
        }

        else if (cmd == CMD_RESET) {
            // Issue a soft reset
            MXC_GCR->rst0 |= MXC_F_GCR_RST0_SYS;
        }

        else if (cmd == CMD_IMGRES) {
            sscanf(g_serial_buffer, "imgres %u %u", &g_app_settings.imgres_w,
                   &g_app_settings.imgres_h);
            printf("Set image resolution to width %u, height %u\n", g_app_settings.imgres_w,
                   g_app_settings.imgres_h);
        }

        else if (cmd == CMD_CAPTURE) {
            // Perform a blocking image capture with the current camera settings.
            img_data_t img_data = capture_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                                              g_app_settings.pixel_format, g_app_settings.dma_mode,
                                              g_app_settings.dma_channel);

            if (img_data.raw != NULL) {
                // Send the image data over the serial port...
                MXC_TMR_SW_Start(MXC_TMR0);

                // First, tell the host that we're about to send the image.
                clear_serial_buffer();
                snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
                         "*IMG* %s %i %i %i", // Format img info into a string
                         img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);
                send_msg(g_serial_buffer); // Send the img info to the host

                // Now, send the image data.  The host should now be expecting
                // to receive 'imglen' bytes.
                clear_serial_buffer();
                MXC_UART_Write(Con_Uart, img_data.raw,
                               (int*)&img_data.imglen); // Send the raw data over the serial port.

                int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
                printf("Done! (serial transmission took %i us)\n", elapsed);
            }
        }

        else if (cmd == CMD_STREAM) {
            // Enable CNN acelerator.  In this example, it's used alongside streaming mode
            // to allow the capture of high-res images.
            // CNN clock: APB (50 MHz) div 1
            cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
            cnn_init();

            // Perform a streaming image capture with the current camera settings.
            cnn_img_data_t img_data =
                stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                           g_app_settings.pixel_format, g_app_settings.dma_channel);

            // Transmit the received image over UART.
            if (img_data.raw != NULL) {
                printf("Transmitting image data over UART...\n");
                MXC_TMR_SW_Start(MXC_TMR0);

                // Tell the host console we're about to send an image.
                clear_serial_buffer();
                snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
                         "*IMG* %s %i %i %i", // Format img info into a string
                         img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);
                send_msg(g_serial_buffer); // Send the img info to the host

                // Send the raw image data
                // We'll need to do some unpacking from the CNN
                // data SRAM in this case.
                clear_serial_buffer();
                int transfer_len   = SERIAL_BUFFER_SIZE;                
                uint32_t* cnn_addr = img_data.raw;
                for (int i = 0; i < img_data.imglen; i += transfer_len) {
                    cnn_addr = read_bytes_from_cnn_sram((uint8_t*)g_serial_buffer, transfer_len, cnn_addr);
                    MXC_UART_Write(Con_Uart, (uint8_t*)g_serial_buffer, &transfer_len);
                }

                int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
                printf("Done! (serial transmission took %i us)\n", elapsed);
            }

            // Disable the CNN when unused to preserve power.
            cnn_disable();
        }

        else if (cmd == CMD_SETREG) {
            // Set a camera register
            unsigned int reg;
            unsigned int val;
            // ^ Declaring these as unsigned ints instead of uintX_t
            // avoids some issues caused by type-casting inside of sscanf.

            sscanf(g_serial_buffer, "%s %u %u", cmd_table[cmd], &reg, &val);
            printf("Writing 0x%x to reg 0x%x\n", val, reg);
            camera_write_reg((uint8_t)reg, (uint8_t)val);
        }

        else if (cmd == CMD_GETREG) {
            // Read a camera register
            unsigned int reg;
            uint8_t val;
            sscanf(g_serial_buffer, "%s %u", cmd_table[cmd], &reg);
            camera_read_reg((uint8_t)reg, &val);
            snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE, "Register 0x%x=0x%x", reg, val);
            send_msg(g_serial_buffer);
        }

#ifdef FF_DEFINED
        else if (cmd == CMD_SD_MOUNT) {
            // Mount the SD card
            sd_err = sd_mount();
            if (sd_err == FR_OK) {
                sd_get_size();

                printf("Disk Size: %u bytes\n", sd_sectors_total / 2);
                printf("Available: %u bytes\n", sd_sectors_free / 2);
            }
        }

        else if (cmd == CMD_SD_UNMOUNT) {
            sd_err = f_unmount("");
            if (sd_err != FR_OK) {
                printf("Error unmounting SD card: %s\n", FR_ERRORS[sd_err]);
            }
            printf("SD card unmounted.\n");
        }

        else if (cmd == CMD_SD_CWD) {
            // Get the current working directory
            sd_err = f_getcwd(sd_cwd, MAXLEN);
            if (sd_err != FR_OK) {
                printf("Error getting cwd: %s\n", FR_ERRORS[sd_err]);
            } else {
                printf("%s\n", sd_cwd);
            }
        }

        else if (cmd == CMD_SD_CD) {
            char* b = (char*)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "cd %s", b);
            sd_err = f_chdir((const TCHAR*)b);
            if (sd_err != FR_OK) {
                printf("Error changing directory: %s\n", FR_ERRORS[sd_err]);
            } else {
                printf("\n");
            }
            free(b);

            // Refresh CWD
            sd_err = f_getcwd(sd_cwd, MAXLEN);
            if (sd_err != FR_OK) {
                printf("Error getting cwd: %s\n", FR_ERRORS[sd_err]);
            } else {
                printf("%s\n", sd_cwd);
            }
        }

        else if (cmd == CMD_SD_LS) {
            // List the contents of the current directory
            sd_err = f_opendir(&sd_dir, sd_cwd);
            if (sd_err != FR_OK) {
                printf("Error opening directory: %s\n", FR_ERRORS[sd_err]);
            } else {
                printf("\n");
                for (;;) {
                    sd_err = f_readdir(&sd_dir, &sd_fno);
                    if (sd_err != FR_OK || sd_fno.fname[0] == 0) break;
                    if (sd_fno.fattrib & AM_DIR) {
                        printf("%s/\n", sd_fno.fname);
                    } else {
                        printf("%s\n", sd_fno.fname);
                    }
                }
                f_closedir(&sd_dir);
            }
        }

        else if (cmd == CMD_SD_MKDIR) {
            // Make a directory
            char* b = (char*)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "mkdir %s", b);
            sd_err = f_mkdir((const TCHAR*)b);
            if (sd_err != FR_OK) {
                printf("Error creating directory: %i\n", FR_ERRORS[sd_err]);
            } else {
                printf("\n");
            }
            free(b);
        }

        else if (cmd == CMD_SD_RM) {
            // Remove an item
            char* b = (char*)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "rm %s", b);
            sd_err = f_unlink((const TCHAR*)b);
            if (sd_err != FR_OK) {
                printf("Error while deleting: %s\n", FR_ERRORS[sd_err]);
            } else {
                printf("\n");
            }
            free(b);
        }

        else if (cmd == CMD_SD_TOUCH) {
            char* b = (char*)malloc(SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "touch %s", b);
            sd_err = f_open(&sd_file, (const TCHAR*)b, FA_CREATE_NEW);
            if (sd_err != FR_OK) {
                printf("Error creating file: %s\n", FR_ERRORS[sd_err]);
            } else {
                printf("\n");
            }
            f_close(&sd_file);

            free(b);
        }

        else if (cmd == CMD_SD_WRITE) {
            char* b = (char*)malloc(SERIAL_BUFFER_SIZE);
            memset(b, '\0', SERIAL_BUFFER_SIZE);
            sscanf(g_serial_buffer, "write %s \"%[^\"]", sd_filename, b); // \"$[^\"] captures everything between two quotes ""
            printf("Writing %s\n", b);
            int len = strlen(b);
            UINT wrote = 0;
            sd_err = f_open(&sd_file, sd_filename, FA_CREATE_NEW | FA_WRITE);
            if (sd_err != FR_OK) {
                printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
            } else {
                sd_err = f_write(&sd_file, b, len, &wrote);
                if (sd_err != FR_OK || wrote != len) {
                    printf("Failed to write to file: %s\n", FR_ERRORS[sd_err]);
                }
            }
            f_close(&sd_file);
            printf("\n");
            free(b);
        }

        else if (cmd == CMD_SD_CAT) {
            sscanf(g_serial_buffer, "cat %s", (char*)sd_filename);
            sd_err = f_open(&sd_file, sd_filename, FA_READ);
            if (sd_err != FR_OK) {
                printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
            } else {
                while(sd_err == FR_OK && !f_eof(&sd_file)) {
                    sd_err = f_forward(&sd_file, out_stream, SERIAL_BUFFER_SIZE, NULL);
                }
                printf("\n");
            }
            f_close(&sd_file);
        }

        else if (cmd == CMD_SD_SNAP) {
            // Snap and save an image to the SD card
            sscanf(g_serial_buffer, "snap %s", (char*)sd_filename);

            // Enable CNN accelerator
            cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
            cnn_init();

            cnn_img_data_t img_data =
                stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                           g_app_settings.pixel_format, g_app_settings.dma_channel);
            
            if (img_data.raw != NULL) {
                MXC_TMR_SW_Start(MXC_TMR0);

                uint32_t* cnn_addr = img_data.raw;
                printf("Saving image to %s\n", sd_filename);
                sd_err = f_open(&sd_file, sd_filename, FA_WRITE | FA_CREATE_ALWAYS);

                uint8_t* buffer = (uint8_t*)malloc(img_data.w);
                if (sd_err != FR_OK) {
                    printf("Failed to open file: %s\n", FR_ERRORS[sd_err]);
                } else {
                    for (int i = 0; i < img_data.imglen; i += img_data.w) {
                        cnn_addr = read_bytes_from_cnn_sram(buffer, img_data.w, cnn_addr);
                        sd_err = f_write(&sd_file, buffer, img_data.w, NULL);
                        if (sd_err != FR_OK) {
                            printf("Failed to write bytes to file: %s\n", FR_ERRORS[sd_err]);
                            break;
                        }
                        if (i % (img_data.w * 32) == 0) {
                            printf("%.1f%%\n", ((float)i/img_data.imglen) * 100.0f);
                        }
                    }
                }
                free(buffer);

                f_close(&sd_file);
                int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
                printf("Finished (took %ius)\n", elapsed);
            }

            cnn_disable();
        }
#endif

        // Clear the serial buffer for the next command
        clear_serial_buffer();
    }
}

// *****************************************************************************
int main(void)
{
    // Initialization...
    int ret = 0;
    int slaveAddress;
    int id;
    g_app_settings.dma_mode     = USE_DMA;
    g_app_settings.imgres_w     = 64;
    g_app_settings.imgres_h     = 64;
    g_app_settings.pixel_format = PIXFORMAT_RGB565; // This default may change during initialization

#ifdef CAMERA_MONO
    g_app_settings.pixel_format = PIXFORMAT_BAYER;
#endif

    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    console_init();

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

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360) || defined(CAMERA_HM0360_MONO) || \
    defined(CAMERA_OV5642)
    camera_set_hmirror(0);
    camera_set_vflip(0);
#endif

    // *********************************END CLEANME*************************************

    printf("Ready!\n");

    // Main processing loop.
    while (1) {
        service_console();
    }
}

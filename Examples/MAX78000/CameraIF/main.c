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
#include "console.h"


#define CAMERA_FREQ 10000000

// Convenience struct for describing a complete captured image.
typedef struct {
    uint8_t* raw;           // Pointer to raw img data in SRAM.
    uint32_t imglen;        // Length of img data (in bytes)
    uint32_t w;             // Width of the image (in pixels)
    uint32_t h;             // Height of the image (in pixels)
    uint8_t* pixel_format;  // Pixel format string
} img_data_t;

img_data_t capture_img(uint32_t w, uint32_t h, pixformat_t pixel_format, dmamode_t dma_mode, int dma_channel) {
    // This demonstrates the most basic blocking capture of a single image.
    // The raw data for the entire image will be saved into an SRAM buffer.
    // For the purposes of demonstration this function contains the full setup
    // sequence for capturing an image.  In practice, 'camera_setup' can be 
    // broken out into a separate initialization sequence if you're not
    // reconfiguring the camera settings on the fly.

    img_data_t img_data;

    // 1. Configure the camera with the 'camera_setup' function.  
    // Image dimensions should fall within the limitations 
    // of the camera hardware and MCU SRAM limits.  In this simple capture mode the
    // camera.h drivers will allocate an SRAM buffer whose size is equal to
    // width * height * bytes_per_pixel.  See camera.c for implementation details.
    printf("Configuring camera\n");
    int ret = camera_setup(
        w, // width
        h, // height
        pixel_format, // pixel format
        FIFO_FOUR_BYTE, // FIFO mode (four bytes is suitable for most cases)
        dma_mode, // DMA (enabling DMA will drastically decrease capture time)
        dma_channel // Allocate the DMA channel retrieved in initialization
    );

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
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
    while(!camera_is_image_rcv()); 
    int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %ius)\n", elapsed_us);

    // 4. Retrieve details of the captured image from the camera driver with the
    // 'camera_get_image' function.  We don't need to copy any image data here, we'll
    // just retrieve a pointer to the camera driver's internal SRAM buffer.
    img_data.pixel_format = camera_get_pixel_format(); // Retrieve the pixel format of the image
    camera_get_image(&img_data.raw, &img_data.imglen, &img_data.w, &img_data.h); // Retrieve info using driver function.
    printf(
        "Captured %ux%u %s image to buffer location 0x%x (%i bytes)\n", 
        img_data.w, img_data.h, img_data.pixel_format, img_data.raw, img_data.imglen
    );

    // 5. At this point, the "raw" byte pointer is pointing to the fully captured raw
    // image data, which is stored in a byte array with length 'imgLen' and all the info
    // needed to decode the image data has been collected.

    return img_data;
}

void stream_img(uint32_t w, uint32_t h, pixformat_t pixel_format, int dma_channel) {
    // This demonstrates a more advanced streaming-based image capture.
    // With this method, the camera drivers don't need to allocate space for the entire
    // image in SRAM.  Instead, space for 2 rows is allocated for a DMA-enabled
    // swapping buffer.  This theoretically allows for higher image resolutions, but
    // comes at the cost of stricter timing requirements.  Each buffered line must be
    // serviced in time, otherwise the buffer will overflow.
    img_data_t img_data;

#ifdef CAMERA_OV7692
    camera_write_reg(0x11, 0x6); // set camera clock prescaller to prevent streaming overflow for QVGA
#endif
    
    // 1. Configure the camera.  This is the same as the standard blocking capture, except
    // the DMA mode is set to "STREAMING_DMA".
    printf("Configuring camera\n");
    int ret = camera_setup(
        w, // width
        h, // height
        pixel_format, // pixel format
        FIFO_FOUR_BYTE, // FIFO mode (four bytes is suitable for most cases)
        STREAMING_DMA, // Set streaming mode
        dma_channel // Allocate the DMA channel retrieved in initialization
    );

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
        return;
    }

    // 2. Retrieve image format and info.
    img_data.pixel_format = camera_get_pixel_format(); // Retrieve the pixel format of the image
    camera_get_image(&img_data.raw, &img_data.imglen, &img_data.w, &img_data.h); // Retrieve info using driver function.

    // 3. Tell the host that we're about to send an image.
    clear_serial_buffer();
    snprintf(
        g_serial_buffer,
        SERIAL_BUFFER_SIZE, 
        "*IMG* %s %i %i %i", // Format img info into a string
        img_data.pixel_format, 
        img_data.imglen,
        img_data.w,
        img_data.h
    );
    send_msg(g_serial_buffer); // Send the img info to the host
    
    // 4. Start the image capture.  Image data will immediately start streaming
    // into the streaming buffer.
    camera_start_capture_image();
    
    // 5. Transmit the image data line-by-line over UART.
    uint8_t* data = NULL;
    int row_len = w * 2; // Each row will contain 2 bytes per pixel
    for (int i = 0; i < h; i++) {
        // Wait until camera streaming buffer is full
        while ((data = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        };

        MXC_UART_Write(Con_Uart, data, &row_len);
        // utils_stream_image_row_to_pc(data, row_len);

        // Release stream buffer
        release_camera_stream_buffer();
    }

    stream_stat_t* stat = get_camera_stream_statistic();

    printf("DMA transfer count = %d\n", stat->dma_transfer_count);
    printf("OVERFLOW = %d\n", stat->overflow_count);
}

// *****************************************************************************
int main(void)
{
    // Initialization...
    int ret = 0;
    int slaveAddress;
    int id;
    int dma_channel;
    dmamode_t dma_mode = USE_DMA;
    unsigned int imgres_w = 64;
    unsigned int imgres_h = 64;

    console_init();

    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    if ((ret = MXC_UART_Init(Con_Uart, CON_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return ret;
    }

    // Wait until the string "*SYNC*" is echoed back over the serial port before starting the example
    printf("Establishing communication with host...\n");
    char* sync = "*SYNC*";
    while(1) {
        // Transmit sync string
        send_msg(sync);
        LED_Toggle(LED1);
        MXC_Delay(MXC_DELAY_MSEC(500));

        int available = MXC_UART_GetRXFIFOAvailable(Con_Uart);
        if (available > 0) {
            char* buffer = (char*)malloc(available);
            MXC_UART_Read(Con_Uart, (uint8_t*)buffer, &available);
            if (strcmp(buffer, sync) == 0) {
                // Received sync string back, break the loop.
                LED_On(LED1);
                break;
            }
            free(buffer);
        }
    }

    printf("Established communications with host!\n");
    print_help();

    printf("Initializing DMA\n");

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    camera_init(CAMERA_FREQ);
    printf("Initializing camera\n");

    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    // Obtain the manufacturer ID of the camera.
    ret = camera_get_manufacture_id(&id);

    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    printf("Camera ID detected: %04x\n", id);

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360) || defined(CAMERA_OV5642)
    camera_set_hmirror(0);
    camera_set_vflip(0);
#endif

    printf("Awaiting command from host\n");
    // Main processing loop.
    while (1) {

        // Check for any incoming serial commands
        cmd_t cmd = CMD_UNKNOWN;
        if (recv_cmd(&cmd)) {
            // Process the received command...

            if (cmd == CMD_UNKNOWN) {
                printf("Uknown command '%s'\n", g_serial_buffer);

            }

            else if (cmd == CMD_IMGRES) {
                sscanf(g_serial_buffer, "imgres %u %u", &imgres_w, &imgres_h);
                printf("Set image resolution to width %u, height %u\n", imgres_w, imgres_h);
            }

            else if (cmd == CMD_ENABLE_DMA) {
                dma_mode = USE_DMA;
                printf("Enabled DMA on channel %i\n", dma_channel);
                // TODO: Disabling DMA and then re-enabling it does not seem to work.
            }

            else if (cmd == CMD_DISABLE_DMA) {
                dma_mode = NO_DMA;
                printf("Disabled DMA\n");
            }

            else if (cmd == CMD_CAPTURE) {
                // Perform a blocking image capture with the current camera settings.
                // See the 'capture_img' function and 'img_data_t' struct for more details.
                img_data_t img_data = capture_img(
                    imgres_w,
                    imgres_h,
                    PIXFORMAT_RGB565,
                    dma_mode,
                    dma_channel
                );

                // Send the image data over the serial port...
                // First, tell the host that we're about to send the image.
                clear_serial_buffer();
                snprintf(
                    g_serial_buffer,
                    SERIAL_BUFFER_SIZE, 
                    "*IMG* %s %i %i %i", // Format img info into a string
                    img_data.pixel_format, 
                    img_data.imglen, 
                    img_data.w, 
                    img_data.h
                );
                send_msg(g_serial_buffer); // Send the img info to the host

                // Now, send the image data.  The host should now be expecting
                // to receive 'imglen' bytes.
                clear_serial_buffer();
                MXC_UART_Write(Con_Uart, img_data.raw, (int*)&img_data.imglen); // Blast the raw data over the serial port.
            }

            else if (cmd == CMD_STREAM) {
                // Perform a streaming image capture with the current camera settings.
                stream_img(
                    imgres_w,
                    imgres_h,
                    PIXFORMAT_RGB565,
                    dma_channel
                );
            }

            // Clear the serial buffer for the next command
            clear_serial_buffer();
        }
    }
}

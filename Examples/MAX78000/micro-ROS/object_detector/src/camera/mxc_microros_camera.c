/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

#include "mxc_microros_camera.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "camera.h"
#include "led.h"
#include "dma.h"
#include "cnn.h"
#include "post_process.h"

#include "app_config.h"

volatile uint32_t cnn_time; // Stopwatch
int g_camera_dma_channel = -1;
unsigned int g_img_no = 0;

unsigned int g_width = 0;
unsigned int g_height = 0;
const char* g_encoding = NULL;

int mxc_microros_camera_init(unsigned int width, unsigned int height, const char* encoding)
{
    MXC_DMA_Init();
    g_camera_dma_channel = MXC_DMA_AcquireChannel();

    printf("Initializing camera\n");
    // Initialize the camera driver.
    camera_init(10000000);

    int slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    // Obtain product ID of the camera.
    int id;
    int ret = camera_get_product_id(&id);
    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return ret;
    }
    printf("Camera ID detected: %04x\n", id);

    ret = camera_setup(width, height, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, STREAMING_DMA, g_camera_dma_channel);
    if (ret) {
        return ret;
    }
    g_width = width;
    g_height = height;
    g_encoding = encoding;

    return E_NO_ERROR;
}

int mxc_microros_camera_capture(sensor_msgs__msg__Image *out_img)
{
    uint8_t *row_data = NULL;
    unsigned int row_buffer_size = camera_get_stream_buffer_size();
    unsigned int j = 0;

	camera_start_capture_image();
	LED_On(0);

	while (!camera_is_image_rcv()) {
        if ((row_data = get_camera_stream_buffer()) != NULL) {
            for (int i = 0; i < row_buffer_size; i += 2) {
                // RGB565 to packed 24-bit RGB
                out_img->data.data[j + 2] = (*(row_data + i) & 0xF8); // Red
                out_img->data.data[j + 1] = (*(row_data + i) << 5) | ((*((row_data + i) + 1) & 0xE0) >> 3); // Green
                out_img->data.data[j] = (*((row_data + i) + 1) << 3); // Blue
				j += 3;
            }
            // Release buffer in time for next row
            release_camera_stream_buffer();
        }
    }
	LED_Off(0);

    stream_stat_t *stat = get_camera_stream_statistic();    

    if (stat->overflow_count > 0) {
        printf("DMA transfer count = %d\n", stat->dma_transfer_count);
        printf("OVERFLOW = %d\n", stat->overflow_count);
        return E_OVERFLOW;
    }

    // Fill the message header
    //  - Timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out_img->header.stamp.sec = ts.tv_sec;
    out_img->header.stamp.nanosec = ts.tv_nsec;
    //  - Frame ID = (g_img_no)_(device_id)
    snprintf(out_img->header.frame_id.data, STRING_BUFFER_LEN, "%d_%d", g_img_no, DEVICE_ID);
    out_img->header.frame_id.size = strlen(out_img->header.frame_id.data);

    // Fill image info
    strncpy(out_img->encoding.data, g_encoding, STRING_BUFFER_LEN);
    out_img->width = g_width;
    out_img->height = g_height;
    out_img->step = g_width * 3;

    return E_NO_ERROR;
}

typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
} bounding_box_t;

int mxc_microros_camera_run_cnn(sensor_msgs__msg__Image *input, sensor_msgs__msg__RegionOfInterest *output)
{
    // Union for in-place conversion from bytes to 32-bit word
    union {
        uint32_t w;
        uint8_t b[4];
    } m;

    printf("\n*** CNN Inference Test qrcode_tinierssd_nobias_ds_input ***\n");
    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: APB (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state

    cnn_start(); // Start CNN processing

    unsigned int i = 0;
    for (int row = 0; row < (input->height * input->step); row += input->step) {
        for (int col = 0; col < input->step; col += 3) {
            i = row + col;
            // CNN expectes 0x00BBGGRR
            // sensors_msgs/Image packs rgb8 as 0xRRGGBB
            m.b[0] = input->data.data[i + 2]; // Red
            m.b[1] = input->data.data[i + 1]; // Green
            m.b[2] = input->data.data[i];     // Blue
            m.b[3] = 0;

            // Remove the following line if there is no risk that the source would overrun the FIFO:
            while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0
            *((volatile uint32_t *) 0x50000008) = m.w ^ 0x00808080U; // Write FIFO 0
            // Note: XOR is to normalize uint8_t to signed int8_t
        }
    }

    LED_On(0);

    while (cnn_time == 0) {}
#ifdef CNN_INFERENCE_TIMER
    printf("Approximate data loading and inference time: %u us\n\n", cnn_time);
#endif    

    get_priors();
    nms();

    bounding_box_t bb;
    bb.x1 = 0;
    bb.x2 = 0;
    bb.y1 = 0;
    bb.y2 = 0;
    print_detected_boxes(&bb.x1, &bb.y1, &bb.x2, &bb.y2);
    output->x_offset = (uint32_t)(bb.x1 * input->width);
    output->y_offset = (uint32_t)(bb.y1 * input->width);
    output->width = (uint32_t)((bb.x2 - bb.x1) * input->height);
    output->height = (uint32_t)((bb.y2 - bb.y1) * input->height);
    output->do_rectify = false;

    cnn_disable();
    LED_Off(0);

    return E_NO_ERROR;
}
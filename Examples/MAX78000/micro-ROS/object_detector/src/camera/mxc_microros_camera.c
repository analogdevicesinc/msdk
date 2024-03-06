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

    camera_set_vflip(1);
    camera_set_hmirror(1);

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

    // Fill the message header
    //  - Timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out_img->header.stamp.sec = ts.tv_sec;
    out_img->header.stamp.nanosec = ts.tv_nsec;
    //  - Frame ID = (g_img_no)_(device_id)
    snprintf(out_img->header.frame_id.data, STRING_BUFFER_LEN, "%d-%d_%s", g_img_no, ts.tv_sec, DEVICE_ID);
    g_img_no++;
    out_img->header.frame_id.size = strlen(out_img->header.frame_id.data);

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

int mxc_microros_camera_run_cnn(sensor_msgs__msg__Image* input_image, sensor_msgs__msg__RegionOfInterest *output_roi, geometry_msgs__msg__PolygonStamped *output_polygon)
{
    // Union for in-place conversion from bytes to 32-bit word
    union {
        uint32_t w;
        uint8_t b[4];
    } m;
    m.b[3] = 0;

    // Fill the message header
    //  - Timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    output_polygon->header.stamp.sec = ts.tv_sec;
    output_polygon->header.stamp.nanosec = ts.tv_nsec;
    //  - Copy same frame ID as input image so we can match them later
    output_polygon->header = input_image->header;

    // unsigned int row_buffer_size = camera_get_stream_buffer_size();
    // uint8_t *row_data = NULL;
    // uint32_t img_length, width, height;
    // camera_get_image(NULL, &img_length, &width, &height);

    // LED_On(0);

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: APB (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state

    cnn_start(); // Start CNN processing
    // camera_start_capture_image();
    for (int i = 0; i < input_image->data.size; i += 3) {
        m.b[0] = input_image->data.data[i + 2]; // Red
        m.b[1] = input_image->data.data[i + 1]; // Green
        m.b[2] = input_image->data.data[i];     // Blue

        // Remove the following line if there is no risk that the source would overrun the FIFO:
        while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0
        *((volatile uint32_t *) 0x50000008) = m.w ^ 0x00808080U; // Write FIFO 0
        // Note: XOR is to normalize uint8_t to signed int8_t
    }

    // while (!camera_is_image_rcv()) {
    //     if ((row_data = get_camera_stream_buffer()) != NULL) {
    //         for (int i = 0; i < row_buffer_size; i += 2) {
    //             // RGB565 to packed 24-bit RGB
    //             m.b[0] = (*(row_data + i) & 0xF8); // Red
    //             m.b[1] = (*(row_data + i) << 5) | ((*((row_data + i) + 1) & 0xE0) >> 3); // Green
    //             m.b[2] = (*((row_data + i) + 1) << 3); // Blue

    //             // Remove the following line if there is no risk that the source would overrun the FIFO:
    //             while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0
    //             *((volatile uint32_t *) 0x50000008) = m.w ^ 0x00808080U; // Write FIFO 0
    //             // Note: XOR is to normalize uint8_t to signed int8_t
    //         }
    //         // Release buffer in time for next row
    //         release_camera_stream_buffer();
    //     }
    // }    

    while (cnn_time == 0) {}
// #ifdef CNN_INFERENCE_TIMER
//     printf("Approximate data loading and inference time: %u us\n\n", cnn_time);
// #endif

    get_priors();
    nms();

    float xy[4];
    memset(xy, 0, 4 * sizeof(float));
    float kpts[8];
    memset(kpts, 0, 8 * sizeof(float));
    print_detected_boxes(xy, kpts);
    output_roi->x_offset = (uint32_t)(xy[0] * input_image->width);
    output_roi->y_offset = (uint32_t)(xy[1] * input_image->width);
    output_roi->width = (uint32_t)((xy[2] - xy[0]) * input_image->height);
    output_roi->height = (uint32_t)((xy[3] - xy[1]) * input_image->height);
    output_roi->do_rectify = false;

    for (int i = 0; i < output_polygon->polygon.points.size; i++) {
        output_polygon->polygon.points.data[i].x = kpts[(2*i)] * input_image->width;
        output_polygon->polygon.points.data[i].y = kpts[(2*i) + 1] * input_image->height;
        output_polygon->polygon.points.data[i].z = 0;
    }

    cnn_disable();
    // LED_Off(0);

    return E_NO_ERROR;
}
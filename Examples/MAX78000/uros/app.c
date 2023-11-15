#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/region_of_interest.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "led.h"
#include "camera.h"
#include "dma.h"
#include "cnn.h"
#include "post_process.h"
#include "tmr.h"

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

rcl_publisher_t box_publisher;

std_msgs__msg__Header incoming_ping;
std_msgs__msg__Header outcoming_ping;
std_msgs__msg__Header incoming_pong;

// sensor_msgs__msg__Image outgoing_image;
// uint8_t image_data_buffer[160 * 120 * 3];
sensor_msgs__msg__RegionOfInterest outgoing_roi;
typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
} bounding_box_t;

int device_id;
int seq_no;
int pong_count;
int g_camera_dma_channel;
volatile uint32_t cnn_time; // Stopwatch

void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {

		seq_no = rand();
		sprintf(outcoming_ping.frame_id.data, "%d_%d", seq_no, device_id);
		outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_ping.stamp.sec = ts.tv_sec;
		outcoming_ping.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the ping message
		pong_count = 0;
		if (rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL) == RCL_RET_OK) {
            printf("Ping send seq %s\n", outcoming_ping.frame_id.data);
        } else {
            printf("Ping send req error\n");
        }

        LED_Toggle(0);
	}
}

void ping_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

    int incoming_id = 0, incoming_seq_no = 0;
    sscanf(msg->frame_id.data, "%i_%i", &incoming_seq_no, &incoming_id);

	// Dont pong my own pings
	if(incoming_id != device_id) {
		printf("Ping received with seq %s. Answering.\n", msg->frame_id.data);
		if (rcl_publish(&pong_publisher, (const void*)msg, NULL) == RCL_RET_OK) {
            LED_Toggle(1);
        } else {
            printf("Pong send req error\n");
        }
	}
}


void pong_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

	if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) == 0) {
		pong_count++;
		printf("Pong for seq %s (%d)\n", msg->frame_id.data, pong_count);
	}
}

int stream_img_to_cnn(uint32_t w, uint32_t h, pixformat_t pixel_format, int dma_channel, sensor_msgs__msg__RegionOfInterest *out)
{

    union {
        uint32_t w;
        uint8_t b[4];
    } m;

    camera_write_reg(0x28, 0x30);

    printf("Configuring camera\n");
    fifomode_t fifo_mode = (pixel_format == PIXFORMAT_RGB888) ? FIFO_THREE_BYTE : FIFO_FOUR_BYTE;
    int ret = camera_setup(w, // width
                           h, // height
                           pixel_format, // pixel format
                           fifo_mode, // FIFO mode
                           STREAMING_DMA, // Set streaming mode
                           dma_channel); // Allocate the DMA channel retrieved in initialization

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
        return ret;
    }

    printf("\n*** CNN Inference Test qrcode_tinierssd_nobias_ds_input ***\n");
    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: APB (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state 

    printf("Starting streaming capture...\n");
    MXC_TMR_SW_Start(MXC_TMR0);
    LED_On(0);

    cnn_start(); // Start CNN processing
    camera_start_capture_image();

    uint8_t *data = NULL;

    while (!camera_is_image_rcv()) {
        if ((data = get_camera_stream_buffer()) != NULL) {
            LED_Toggle(1);
            for (int i = 0; i < camera_get_stream_buffer_size(); i += 2) {
                // RGB565 to packed 24-bit RGB
                m.b[0] = (*(data + i) & 0xF8); // Red
                m.b[1] = (*(data + i) << 5) | ((*((data + i) + 1) & 0xE0) >> 3); // Green
                m.b[2] = (*((data + i) + 1) << 3); // Blue

                // Remove the following line if there is no risk that the source would overrun the FIFO:
                while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0
                *((volatile uint32_t *) 0x50000008) = m.w ^ 0x00808080U; // Write FIFO 0
            }
            // Release buffer in time for next row
            release_camera_stream_buffer();
        }
    }
    LED_Off(0);

    int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %i us)\n", elapsed_us);

    // 7. Check for any overflow
    stream_stat_t *stat = get_camera_stream_statistic();
    printf("DMA transfer count = %d\n", stat->dma_transfer_count);
    printf("OVERFLOW = %d\n", stat->overflow_count);

    if (stat->overflow_count > 0) {
        return E_OVERFLOW;
    }

    while (cnn_time == 0) {}
#ifdef CNN_INFERENCE_TIMER
    printf("Approximate data loading and inference time: %u us\n\n", cnn_time);
#endif

    get_priors();
    nms();

    bounding_box_t bb;
    print_detected_boxes(&bb.x1, &bb.y1, &bb.x2, &bb.y2);

    out->x_offset = (int)(bb.x1 * w);
    out->y_offset = (int)(bb.y1 * w);
    out->width = (int)((bb.x2 - bb.x1) * h);
    out->height = (int)((bb.y2 - bb.y1) * h);

    if (bb.x1 !=0 && bb.y1 != 0 && bb.x2 != 0 && bb.y2 != 0) {
        LED_On(1);
    }

    cnn_disable();

    return E_NO_ERROR;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    // uint8_t *raw;
    // uint32_t imgLen;
    // uint32_t w, h;

    // camera_get_image(&raw, &imgLen, &w, &h);

    // uint8_t *data = NULL;
    // unsigned int row_buffer_size = camera_get_stream_buffer_size();
    // unsigned int j = 0;

	// camera_start_capture_image();
	// LED_On(0);
	// while (!camera_is_image_rcv()) {
    //     if ((data = get_camera_stream_buffer()) != NULL) {
	// 		LED_Toggle(1);
    //         for (int i = 0; i < camera_get_stream_buffer_size(); i += 2) {
    //             // RGB565 to packed 24-bit RGB
    //             image_data_buffer[j + 2] = (*(data + i) & 0xF8); // Red
    //             image_data_buffer[j + 1] = (*(data + i) << 5) | ((*((data + i) + 1) & 0xE0) >> 3); // Green
    //             image_data_buffer[j] = (*((data + i) + 1) << 3); // Blue
	// 			j += 3;
    //         }
    //         // Release buffer in time for next row
    //         release_camera_stream_buffer();
    //     }
    // }
	// LED_Off(0);    

    if (timer != NULL) {
        outgoing_roi.x_offset = 0;
        outgoing_roi.y_offset = 0;
        outgoing_roi.width = 0;
        outgoing_roi.height = 0;
        stream_img_to_cnn(320, 240, PIXFORMAT_RGB565, g_camera_dma_channel, &outgoing_roi);

        int error = rcl_publish(&box_publisher, (const void*)&outgoing_roi, NULL);

        if (error == RCL_RET_OK) {
            printf("roi send seq %s\n", outcoming_ping.frame_id.data);
        } else {
            printf("roi send req error\n");
        }
    }
}


void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

	// Create a reliable ping publisher
	RCCHECK(rclc_publisher_init_default(&ping_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

    // Create an image publisher
    // RCCHECK(rclc_publisher_init_default(&image_publisher, &node, 
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), "/microROS/image"));

    // Create a bounding box publisher
    RCCHECK(rclc_publisher_init_default(&box_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RegionOfInterest), "/microROS/box"));

	// Create a 2 seconds ping timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), timer_callback));


	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	// RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
	// 	&ping_subscription_callback, ON_NEW_DATA));
	// RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong,
	// 	&pong_subscription_callback, ON_NEW_DATA));

	// Create and allocate the pingpong messages
	char outcoming_ping_buffer[STRING_BUFFER_LEN];
	outcoming_ping.frame_id.data = outcoming_ping_buffer;
	outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

	// char incoming_ping_buffer[STRING_BUFFER_LEN];
	// incoming_ping.frame_id.data = incoming_ping_buffer;
	// incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

	// char incoming_pong_buffer[STRING_BUFFER_LEN];
	// incoming_pong.frame_id.data = incoming_pong_buffer;
	// incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

    // char outgoing_image_buffer[STRING_BUFFER_LEN];
    // outgoing_image.header.frame_id.data = outgoing_image_buffer;
    // outgoing_image.header.frame_id.capacity = STRING_BUFFER_LEN;

    // char outgoing_image_encoding_buffer[STRING_BUFFER_LEN];
    // outgoing_image.encoding.data = outgoing_image_encoding_buffer;
    // outgoing_image.encoding.capacity = STRING_BUFFER_LEN;
    // outgoing_image.data.capacity = 160 * 120 * 3;
    // outgoing_image.data.data = image_data_buffer;

	device_id = rand();

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
    }
    printf("Camera ID detected: %04x\n", id);

    ret = camera_setup(320, 240, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, STREAMING_DMA,
                       g_camera_dma_channel); // RGB565

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
	RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
	RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}

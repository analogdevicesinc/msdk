#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/image.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "led.h"
#include "camera.h"
#include "dma.h"

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

rcl_publisher_t image_publisher;

std_msgs__msg__Header incoming_ping;
std_msgs__msg__Header outcoming_ping;
std_msgs__msg__Header incoming_pong;

sensor_msgs__msg__Image outgoing_image;
// uint8_t image_data_buffer[160 * 120 * 3];

int device_id;
int seq_no;
int pong_count;

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

void image_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    camera_get_image(&raw, &imgLen, &w, &h);

    if (timer != NULL) {
        // Fill message header
        seq_no = rand();
		sprintf(outgoing_image.header.frame_id.data, "%d_%d", seq_no, device_id);
		outgoing_image.header.frame_id.size = strlen(outgoing_image.header.frame_id.data);

        // Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outgoing_image.header.stamp.sec = ts.tv_sec;
		outgoing_image.header.stamp.nanosec = ts.tv_nsec;

        // Image encoding
        outgoing_image.encoding.data = "rgb8";
        outgoing_image.encoding.size = strlen(outgoing_image.encoding.data);
        
        outgoing_image.width = 160;
        outgoing_image.height = 120;
        outgoing_image.step = outgoing_image.width * 3;
        outgoing_image.is_bigendian = 0;

        outgoing_image.data.data = raw;
        outgoing_image.data.size = w*h*2;

        int error = rcl_publish(&image_publisher, (const void*)&outgoing_image, NULL);

        if (error == RCL_RET_OK) {
            printf("image send seq %s\n", outcoming_ping.frame_id.data);
        } else {
            printf("image send req error\n");
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
    RCCHECK(rclc_publisher_init_default(&image_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), "/microROS/image"));

	// Create a 2 seconds ping timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), image_timer_callback));


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

    char outgoing_image_buffer[STRING_BUFFER_LEN];
    outgoing_image.header.frame_id.data = outgoing_image_buffer;
    outgoing_image.header.frame_id.capacity = STRING_BUFFER_LEN;

    char outgoing_image_encoding_buffer[STRING_BUFFER_LEN];
    outgoing_image.encoding.data = outgoing_image_encoding_buffer;
    outgoing_image.encoding.capacity = STRING_BUFFER_LEN;
    outgoing_image.data.capacity = 160 * 120 * 3;

	device_id = rand();

    MXC_DMA_Init();
    int camera_dma_channel = MXC_DMA_AcquireChannel();

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

    ret = camera_setup(160, 120, PIXFORMAT_RGB888, FIFO_THREE_BYTE, USE_DMA,
                       camera_dma_channel); // RGB565

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

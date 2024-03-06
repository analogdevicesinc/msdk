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

#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/region_of_interest.h>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <rcl_interfaces/msg/log.h>

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
#include "uart.h"

#include "mxc_microros.h"
#include "mxc_microros_camera.h"
#include "app_config.h"
#include "mxc_delay.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


rcl_publisher_t roi_publisher;
sensor_msgs__msg__RegionOfInterest outgoing_roi;

rcl_publisher_t polygon_publisher;
geometry_msgs__msg__PolygonStamped outgoing_polygon;


sensor_msgs__msg__Image outgoing_image;
uint8_t image_data_buffer[IMG_XRES * IMG_YRES * 3];
#ifdef PUBLISH_IMAGE
rcl_publisher_t image_publisher;
#endif

rcl_publisher_t log_publisher;
rcl_interfaces__msg__Log log_msg;

#define LOG_MSG(fmt, args...) {\
    struct timespec ts; \
    clock_gettime(CLOCK_REALTIME, &ts); \
    log_msg.stamp.sec = ts.tv_sec; \
    log_msg.stamp.nanosec = ts.tv_nsec; \
    snprintf(log_msg.msg.data, STRING_BUFFER_LEN, fmt, ##args);\
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));\
}

void error_loop() {
    int i = 0;
    while(i < 10) {
        LED_Toggle(0);
        MXC_Delay(MXC_DELAY_MSEC(500));
        i++;
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    int error;

    error = mxc_microros_camera_capture(&outgoing_image);
    if (error) {
        printf("\nFailed to capture image!\n");
        return;
    }

#ifdef PUBLISH_IMAGE
    error = rcl_publish(&image_publisher, &outgoing_image, NULL);
    if (error) {
        printf("\nImage send req error %i\n", error);
        error_loop();
        return;
    }
#endif

    error = mxc_microros_camera_run_cnn(&outgoing_image, &outgoing_roi, &outgoing_polygon);
    if (error) {
        printf("\nFailed to run CNN\n");
        return;
    }

    
    if (outgoing_roi.width > 0 && outgoing_roi.height > 0) {
        LED_On(1);
        error = rcl_publish(&roi_publisher, &outgoing_roi, NULL);
        error = rcl_publish(&polygon_publisher, &outgoing_polygon, NULL);
        if (error) {
            error_loop();
        }
    } else {
        LED_Off(1);
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

    // Create a bounding box publisher
    RCCHECK(rclc_publisher_init_best_effort(&roi_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RegionOfInterest), "/microROS/roi"));

    RCCHECK(rclc_publisher_init_best_effort(&polygon_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PolygonStamped), "/microROS/polygon"));

#ifdef PUBLISH_IMAGE
    // Create an image publisher
    // NOTE: Must use reliable publisher because best effort currently requires a duplicate 
    // internal buffer that is the entire size of the message
    RCCHECK(rclc_publisher_init_default(&image_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), "/microROS/image"));
#endif

    RCCHECK(rclc_publisher_init_default(&log_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), "/rosout"));

    // Create a 5 seconds ping timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(250), timer_callback));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    // Initialize buffers for the Image message
    // Frame ID string
    char outgoing_image_frameid_buffer[STRING_BUFFER_LEN];
    memset(outgoing_image_frameid_buffer, '\0', STRING_BUFFER_LEN);
    outgoing_image.header.frame_id.capacity = STRING_BUFFER_LEN;
    outgoing_image.header.frame_id.data = outgoing_image_frameid_buffer;

    // Image encoding string
    char outgoing_image_encoding_buffer[STRING_BUFFER_LEN];
    memset(outgoing_image_encoding_buffer, '\0', STRING_BUFFER_LEN);
    outgoing_image.encoding.data = outgoing_image_encoding_buffer;
    
    // Image data buffer
    outgoing_image.encoding.capacity = STRING_BUFFER_LEN;
    outgoing_image.data.capacity = IMG_XRES * IMG_YRES * 3;
    outgoing_image.data.data = image_data_buffer;
    outgoing_image.data.size = outgoing_image.data.capacity;

    // Initialize Polygon
    char polygon_header_buffer[STRING_BUFFER_LEN];
    memset(polygon_header_buffer, '\0', STRING_BUFFER_LEN);
    outgoing_polygon.header.frame_id.data = polygon_header_buffer;
    outgoing_polygon.header.frame_id.size = STRING_BUFFER_LEN;
    geometry_msgs__msg__Point32 point_array[4];
    geometry_msgs__msg__Point32__Sequence points = {
        .capacity = 4,
        .data = point_array,
        .size = 4
    };
    outgoing_polygon.polygon.points = points;

    // Initialize logger buffers
    char log_name_buffer[STRING_BUFFER_LEN];
    memset(log_name_buffer, '\0', STRING_BUFFER_LEN);
    log_msg.name.capacity = STRING_BUFFER_LEN;
    log_msg.name.data = log_name_buffer;
    log_msg.name.size = STRING_BUFFER_LEN;
    snprintf(log_name_buffer, STRING_BUFFER_LEN, DEVICE_ID);

    char log_msg_buffer[STRING_BUFFER_LEN];
    memset(log_msg_buffer, '\0', STRING_BUFFER_LEN);
    log_msg.msg.capacity = STRING_BUFFER_LEN;
    log_msg.msg.data = log_msg_buffer;
    log_msg.msg.size = STRING_BUFFER_LEN;

    log_msg.level = rcl_interfaces__msg__Log__INFO;

    LOG_MSG("Hello logger!");

    LOG_MSG("micro-ROS successfully initialized.");
    LOG_MSG("Initializing camera...");
    // Initialize camera
    RCCHECK(mxc_microros_camera_init(IMG_XRES, IMG_YRES, "rgb8"));

    LOG_MSG("Starting executor...");
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);
    }

    // Free resources
    RCCHECK(rcl_node_fini(&node));
}

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <unistd.h>

#include <rmw_microros/rmw_microros.h>

#include <rosidl_runtime_c/string_functions.h>

#define RCCHECK(fn) \
{\
    rcl_ret_t temp_rc = fn;\
    if (RCL_RET_OK != temp_rc) {\
        printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);\
        return 1;\
    }\
}

#define RCSOFTCHECK(fn) \
{\
    rcl_ret_t temp_rc = fn;\
    if(RCL_RET_OK != temp_rc) {\
        printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);\
    }\
}

rcl_publisher_t publisher;
std_msgs__msg__String msg;

int usage()
{
    printf("Usage: ping_uros_agent <mode>\nModes:\n");
    printf("\t* 'basic': checks that the micro-ROS Agent is up or down, and exits\n");
    printf("\t* 'interactive': starts a basic micro-ROS application ");
    printf("if the Agent is up, or waits for the user to start it in a loop\n");
    return 1;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (NULL != timer) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        printf("Sent: '%s'\n", msg.data.data);

        /**
         * If agent suddently goes down, we will exit the application.
         * This proves that `rmw_uros_ping_agent` functionality can be used
         * also when a micro-ROS node is already up and running (and hence, the
         * transport has been initialized in the RMW context).
         */
        printf("\nPinging agent again...\n");
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
            printf("micro-ROS agent has stopped. Exiting...\n");
            exit(1);
        } else {
            printf("Agent is still up!\n\n");
        }
    }
}

int main(int argc, char ** argv)
{
    /**
     * Parse arguments
     */
    if (2 != argc) {
        return usage();
    }

    const char * mode = argv[1];

    // Init data
    msg.data.data = "This is a message sent after pinging micro-ROS Agent";
    msg.data.size = strlen(msg.data.data);
    msg.data.capacity = msg.data.size + 1;

    /**
     * Basic mode
     */
    if (0 == strcmp("basic", mode)) {
        /**
         * Ping should work even without a micro-ROS node active.
         */
        rmw_ret_t ping_result = rmw_uros_ping_agent(1000, 5);

        if (RMW_RET_OK == ping_result) {
            printf("Success! micro-ROS Agent is up.\n");
            return 0;
        } else {
            printf("Sorry, the micro-ROS Agent is not available.\n");
            return 1;
        }
    } else if (0 == strcmp("interactive", mode)) {
        /**
         * Loop until micro-ROS Agent is up
         */
        while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1)) {
            printf("Please, start your micro-ROS Agent first\n");
        }

        /**
         * Start the micro-ROS basic application
         */
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        // Create init options
        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

        // Create node
        rcl_node_t node;
        RCCHECK(rclc_node_init_default(
            &node, "ping_uros_agent_publisher", "", &support));

        // Create publisher
        RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "ping_uros_agent_topic"));

        // Create timer
        rcl_timer_t timer;
        const unsigned int timer_timeout = 1000;
        RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

        // Create executor
        rclc_executor_t executor;
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    	RCCHECK(rclc_executor_add_timer(&executor, &timer));

        // Spin executor
        rclc_executor_spin(&executor);

        (void)! rcl_publisher_fini(&publisher, &node);
        (void)! rcl_timer_fini(&timer);
        (void)! rclc_executor_fini(&executor);
        (void)! rcl_node_fini(&node);
        (void)! rclc_support_fini(&support);

    } else {
        return usage();
    }

    return 0;
}

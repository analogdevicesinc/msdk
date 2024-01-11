#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_1;
rcl_subscription_t subscriber_1;
std_msgs__msg__Int32 send_msg_1;
std_msgs__msg__Int32 recv_msg_1;

rcl_publisher_t publisher_2;
rcl_subscription_t subscriber_2;
std_msgs__msg__Int32 send_msg_2;
std_msgs__msg__Int32 recv_msg_2;

void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time)
{
 	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher_1, &send_msg_1, NULL));
		printf("Node 1 -- Sent: %d\n", send_msg_1.data);
		send_msg_1.data++;
	}
}

void subscription_callback_1(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Node 1 --Received: %d\n", msg->data);
}

void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher_2, &send_msg_2, NULL));
		printf("Node 2 -- Sent: %d\n", send_msg_2.data);
		send_msg_2.data++;
	}
}

void subscription_callback_2(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Node 2 --Received: %d\n", msg->data);
}


int main(int argc, const char * const * argv)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, argc, argv, &allocator));

	// ----- NODE 1 -----
	// create node
	rcl_node_t node_1;
	RCCHECK(rclc_node_init_default(&node_1, "int32_node_1", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_1,
		&node_1,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"node_1_to_2"));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber_1,
		&node_1,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"node_2_to_1"));

	// create timer,
	rcl_timer_t timer_1;
	RCCHECK(rclc_timer_init_default(
		&timer_1,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback_1));

	// create executor
	rclc_executor_t executor_1 = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor_1, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor_1, &timer_1));
	RCCHECK(rclc_executor_add_subscription(&executor_1, &subscriber_1, &recv_msg_1, &subscription_callback_1, ON_NEW_DATA));

	// ----- NODE 2 -----
	// create node
	rcl_node_t node_2;
	RCCHECK(rclc_node_init_default(&node_2, "int32_node_2", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_2,
		&node_2,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"node_2_to_1"));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber_2,
		&node_2,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"node_1_to_2"));

	// create timer,
	rcl_timer_t timer_2;
	RCCHECK(rclc_timer_init_default(
		&timer_2,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback_2));

	// create executor
	rclc_executor_t executor_2 = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor_2, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor_2, &timer_2));
	RCCHECK(rclc_executor_add_subscription(&executor_2, &subscriber_2, &recv_msg_2, &subscription_callback_2, ON_NEW_DATA));

	// ----- MAIN LOOP -----
	send_msg_1.data = 0;
	send_msg_2.data = 100;

	while (1)
	{
		rclc_executor_spin_some(&executor_1, RCL_MS_TO_NS(100));
		rclc_executor_spin_some(&executor_2, RCL_MS_TO_NS(100));
	}
}

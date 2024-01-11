#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>

#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		printf("Sent: %d\n", msg.data);
		msg.data++;
	}
}

int main(int argc, char * const argv[])
{
	if (argc < 3 || argc > 4)
	{
		printf("Usage: configured_publisher <IP> <port> <DomainID (default: 0)>\n");
		return 1;
	}

  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	printf("Connecting to agent %s:%d\n",argv[1],atoi(argv[2]));
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_udp_address(argv[1], argv[2], rmw_options))
	RCCHECK(rmw_uros_options_set_client_key(0xCAFEBABA, rmw_options))

	size_t domain_id = (size_t)(argc == 4 ? atoi(argv[3]) : 0);
	const char * node_name = "int32_configured_publisher_rclc";

	RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
	printf("Initializing RCL '%s' with ROS Domain ID %ld...\n", node_name, domain_id);

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"std_msgs_msg_Int32"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.data = 0;

  	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	return 0;
}

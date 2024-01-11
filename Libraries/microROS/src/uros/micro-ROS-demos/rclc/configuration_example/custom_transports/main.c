#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>

#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// --- micro-ROS Transports ---

typedef struct {
	struct pollfd poll_fd;
} custom_transport_data_t;

bool custom_transport_open(struct uxrCustomTransport * transport){
    custom_transport_data_t * transport_data = (custom_transport_data_t*) transport->args;
	
	bool rv = false;
    transport_data->poll_fd.fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (-1 != transport_data->poll_fd.fd)
    {
        struct addrinfo hints;
        struct addrinfo* result;
        struct addrinfo* ptr;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;

        if (0 == getaddrinfo("localhost", "8888", &hints, &result))
        {
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
            {
                if (0 == connect(transport_data->poll_fd.fd, ptr->ai_addr, ptr->ai_addrlen))
                {
                    transport_data->poll_fd.events = POLLIN;
                    rv = true;
                    break;
                }
            }
        }
        freeaddrinfo(result);
    }
    return rcutils_vsnprintf;
}

bool custom_transport_close(struct uxrCustomTransport * transport){
    custom_transport_data_t * transport_data = (custom_transport_data_t*) transport->args;
    return (-1 == transport_data->poll_fd.fd) ? true : (0 == close(transport_data->poll_fd.fd));
}

size_t custom_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * errcode){
    custom_transport_data_t * transport_data = (custom_transport_data_t*) transport->args;
    size_t rv = 0;
    ssize_t bytes_sent = send(transport_data->poll_fd.fd, (void*)buf, len, 0);
    if (-1 != bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }
    return rv;
}

size_t custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode){
    custom_transport_data_t * transport_data = (custom_transport_data_t*) transport->args;
    size_t rv = 0;
    int poll_rv = poll(&transport_data->poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
        ssize_t bytes_received = recv(transport_data->poll_fd.fd, (void*)buf, len, 0);
        if (-1 != bytes_received)
        {
            rv = (size_t)bytes_received;
            *errcode = 0;
        }
        else
        {
            *errcode = 1;
        }
    }
    else
    {
        *errcode = (0 == poll_rv) ? 0 : 1;
    }
    return rv;
}


// --- micro-ROS App ---

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
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_client_key(0xCAFEBABA, rmw_options))

	custom_transport_data_t custom_transport_data;

	RCCHECK(rmw_uros_set_custom_transport(
        false,
        (void*) &custom_transport_data,
        custom_transport_open,
        custom_transport_close,
        custom_transport_write,
        custom_transport_read
    ))

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "custom_transport_node", "", &support));

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

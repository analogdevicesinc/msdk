#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <complex_msgs/msg/nested_msg_test.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


int main()
{	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "complex_message_node", "", &support));

	// create publisher
	rcl_publisher_t publisher;
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(complex_msgs, msg, NestedMsgTest),
		"complex_message")
	);

	complex_msgs__msg__NestedMsgTest msg;
	char buff1[30];
	msg.data14.data1.data = buff1;
	msg.data14.data1.size = 0;
	msg.data14.data1.capacity = sizeof(buff1);

	char buff2[30];
	msg.data14.data2.data = buff2;
	msg.data14.data2.size = 0;
	msg.data14.data2.capacity = sizeof(buff1);

	char buff3[30];
	msg.data14.data3.data = buff3;
	msg.data14.data3.size = 0;
	msg.data14.data3.capacity = sizeof(buff1);

	char buff4[30];
	msg.data14.data4.data = buff4;
	msg.data14.data4.size = 0;
	msg.data14.data4.capacity = sizeof(buff1);


	int num = 0;
	while (true) {
		msg.data1 = (bool) (num & 0x01);
		msg.data2 = (uint8_t) num;
		msg.data3 = (signed char) num;
		msg.data4 = (float) num;
		msg.data5 = (double) num;
		msg.data6 = (int8_t) num;
		msg.data7 = (uint8_t) num;
		msg.data8 = (int16_t) num;
		msg.data9 = (uint16_t) num;
		msg.data10 = (int32_t) num;
		msg.data11 = (uint32_t) num;
		msg.data12 = (int64_t) num;
		msg.data13 = (uint64_t) num;
		msg.data14.data1.size = snprintf(msg.data14.data1.data, msg.data14.data1.capacity, "Msg A - %i", num);
		msg.data14.data2.size = snprintf(msg.data14.data2.data, msg.data14.data2.capacity, "Msg B - %i", num);
		msg.data14.data3.size = snprintf(msg.data14.data3.data, msg.data14.data3.capacity, "Msg C - %i", num);
		msg.data14.data4.size = snprintf(msg.data14.data4.data, msg.data14.data4.capacity, "Msg D - %i", num);
		
		num++;

		rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);

		if (ret == RCL_RET_OK) {
			printf("Complex message sent:\n");
			printf("\tBool: %u\n", msg.data1);
			printf("\tuint8_t: %u\n", msg.data2);
			printf("\tsigned char: %u\n", msg.data3);
			printf("\tfloat: %f\n", msg.data4);
			printf("\tdouble: %lf\n", msg.data5);
			printf("\tint8_t: %i\n", msg.data6);
			printf("\tuint8_t: %u\n", msg.data7);
			printf("\tint16_t: %i\n", msg.data8);
			printf("\tuint16_t: %u\n", msg.data9);
			printf("\tint32_t: %i\n", msg.data10);
			printf("\tuint32_t: %u\n", msg.data11);
			printf("\tint64_t: %li\n", msg.data12);
			printf("\tuint64_t: %lu\n", msg.data13);
			printf("\tstring 1: %s\n", msg.data14.data1.data);
			printf("\tstring 2: %s\n", msg.data14.data2.data);
			printf("\tstring 3: %s\n", msg.data14.data3.data);
			printf("\tstring 4: %s\n", msg.data14.data4.data);
			printf("\n\n");
		}

		sleep(1);
	}

	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
}

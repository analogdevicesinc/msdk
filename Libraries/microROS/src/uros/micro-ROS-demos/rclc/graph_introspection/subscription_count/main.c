#include <rcl/rcl.h>
#include <rcl/graph.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl_interfaces/msg/parameter_event.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_node_t node;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  printf("Subscriptions count for publisher '/parameter_events':\n");
  size_t subscription_count = 0;
  RCSOFTCHECK(rcl_publisher_get_subscription_count(&publisher, &subscription_count));
  printf("    * Using 'rcl_publisher_get_subscription_count': %lu\n", subscription_count);
  RCSOFTCHECK(rcl_count_subscribers(&node, "/parameter_events", &subscription_count));
  printf("    * Using 'rcl_count_subscribers': %lu\n", subscription_count);
}

int main(int argc, const char * const * argv)
{
  printf("***Hint: test this example using 'ros2 run demo_nodes_cpp talker/listener' in another terminal***\n");
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create_node
  RCCHECK(rclc_node_init_default(&node, "subscription_count_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, ParameterEvent),
    "parameter_events"));

  // create timer
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_node_fini(&node));
}

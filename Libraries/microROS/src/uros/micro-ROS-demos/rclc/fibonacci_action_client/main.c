#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <example_interfaces/action/fibonacci.h>

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {printf( \
        "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {printf( \
        "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);}}

#define MAX_FIBONACCI_ORDER 500
bool goal_completed = false;
int goal_order = 10;

void goal_request_callback(rclc_action_goal_handle_t * goal_handle, bool accepted, void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;
  printf(
    "Goal request (order: %d): %s\n",
    request->goal.order,
    accepted ? "Accepted" : "Rejected"
  );

  if (!accepted) {
    goal_completed = true;
  }
}

void feedback_callback(rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal Feedback (order: %d) [",
    request->goal.order
  );

  example_interfaces__action__Fibonacci_FeedbackMessage * feedback =
    (example_interfaces__action__Fibonacci_FeedbackMessage *) ros_feedback;

  for (size_t i = 0; i < feedback->feedback.sequence.size; i++) {
    printf("%d ", feedback->feedback.sequence.data[i]);
  }
  printf("\b]\n");
}

void result_request_callback(
  rclc_action_goal_handle_t * goal_handle, void * ros_result_response,
  void * context)
{
  (void) context;

  static size_t goal_count = 1;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal Result (order: %d) [",
    request->goal.order
  );

  example_interfaces__action__Fibonacci_GetResult_Response * result =
    (example_interfaces__action__Fibonacci_GetResult_Response *) ros_result_response;

  if (result->status == GOAL_STATE_SUCCEEDED) {
    for (size_t i = 0; i < result->result.sequence.size; i++) {
      printf("%d ", result->result.sequence.data[i]);
    }
  } else if (result->status == GOAL_STATE_CANCELED) {
    printf("CANCELED ");
  } else {
    printf("ABORTED ");
  }

  printf("\b]\n");

  goal_completed = true;
}

void cancel_request_callback(
  rclc_action_goal_handle_t * goal_handle, bool cancelled,
  void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal cancel request (order: %d): %s\n",
    request->goal.order,
    cancelled ? "Accepted" : "Rejected");

  if (cancelled) {
    goal_completed = true;
  }
}

int main(int argc, const char * const * argv)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "fibonacci_action_client_rcl", "", &support));

  // Create action client
  rclc_action_client_t action_client;
  RCCHECK(
    rclc_action_client_init_default(
      &action_client,
      &node,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
      "fibonacci"
  ));

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  example_interfaces__action__Fibonacci_FeedbackMessage ros_feedback;
  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;
  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;

  // Allocate msg memory
  ros_feedback.feedback.sequence.capacity = MAX_FIBONACCI_ORDER;
  ros_feedback.feedback.sequence.size = 0;
  ros_feedback.feedback.sequence.data = (int32_t *) malloc(
    ros_feedback.feedback.sequence.capacity * sizeof(int32_t));

  ros_result_response.result.sequence.capacity = MAX_FIBONACCI_ORDER;
  ros_result_response.result.sequence.size = 0;
  ros_result_response.result.sequence.data = (int32_t *) malloc(
    ros_result_response.result.sequence.capacity * sizeof(int32_t));

  RCCHECK(
    rclc_executor_add_action_client(
      &executor,
      &action_client,
      10,
      &ros_result_response,
      &ros_feedback,
      goal_request_callback,
      feedback_callback,
      result_request_callback,
      cancel_request_callback,
      (void *) &action_client
  ));

  sleep(2);

  // Send goal request
  ros_goal_request.goal.order = goal_order;
  RCCHECK(rclc_action_send_goal_request(&action_client, &ros_goal_request, NULL));

  while (!goal_completed) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    usleep(100000);
  }

  RCCHECK(rclc_action_client_fini(&action_client, &node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}

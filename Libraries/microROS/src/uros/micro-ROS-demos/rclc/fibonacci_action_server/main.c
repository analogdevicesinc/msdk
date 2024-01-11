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

const char * goalResult[] =
{[GOAL_STATE_SUCCEEDED] = "succeeded", [GOAL_STATE_CANCELED] = "canceled",
  [GOAL_STATE_ABORTED] = "aborted"};

void * fibonacci_worker(void * args)
{
  (void) args;
  rclc_action_goal_handle_t * goal_handle = (rclc_action_goal_handle_t *) args;
  rcl_action_goal_state_t goal_state;

  example_interfaces__action__Fibonacci_SendGoal_Request * req =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  example_interfaces__action__Fibonacci_GetResult_Response response = {0};
  example_interfaces__action__Fibonacci_FeedbackMessage feedback;

  if (req->goal.order < 2) {
    goal_state = GOAL_STATE_ABORTED;
  } else {
    feedback.feedback.sequence.capacity = req->goal.order;
    feedback.feedback.sequence.size = 0;
    feedback.feedback.sequence.data =
      (int32_t *) malloc(feedback.feedback.sequence.capacity * sizeof(int32_t));

    feedback.feedback.sequence.data[0] = 0;
    feedback.feedback.sequence.data[1] = 1;
    feedback.feedback.sequence.size = 2;

    for (size_t i = 2; i < (size_t) req->goal.order && !goal_handle->goal_cancelled; i++) {
      feedback.feedback.sequence.data[i] = feedback.feedback.sequence.data[i - 1] +
        feedback.feedback.sequence.data[i - 2];
      feedback.feedback.sequence.size++;

      printf("Publishing feedback\n");
      rclc_action_publish_feedback(goal_handle, &feedback);
      usleep(500000);
    }

    if (!goal_handle->goal_cancelled) {
      response.result.sequence.capacity = feedback.feedback.sequence.capacity;
      response.result.sequence.size = feedback.feedback.sequence.size;
      response.result.sequence.data = feedback.feedback.sequence.data;
      goal_state = GOAL_STATE_SUCCEEDED;
    } else {
      goal_state = GOAL_STATE_CANCELED;
    }
  }

  printf("Goal %d %s, sending result array\n", req->goal.order, goalResult[goal_state]);

  rcl_ret_t rc;
  do {
    rc = rclc_action_send_result(goal_handle, goal_state, &response);
    usleep(1e6);
  } while (rc != RCL_RET_OK);

  free(feedback.feedback.sequence.data);
  pthread_exit(NULL);
}

rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * req =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  // Too big, rejecting
  if (req->goal.order > 200) {
    printf("Goal %d rejected\n", req->goal.order);
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  printf("Goal %d accepted\n", req->goal.order);

  pthread_t * thread_id = malloc(sizeof(pthread_t));
  pthread_create(thread_id, NULL, fibonacci_worker, goal_handle);
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

bool handle_cancel(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;
  (void) goal_handle;

  return true;
}

int main(int argc, const char * const * argv)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "fibonacci_action_server_rcl", "", &support));

  // Create action service
  rclc_action_server_t action_server;
  RCCHECK(
    rclc_action_server_init_default(
      &action_server,
      &node,
      &support,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
      "fibonacci"
  ));

  // Create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[10];

  RCCHECK(
    rclc_executor_add_action_server(
      &executor,
      &action_server,
      10,
      ros_goal_request,
      sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
      handle_goal,
      handle_cancel,
      (void *) &action_server));

  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    usleep(100000);
  }

  RCCHECK(rclc_action_server_fini(&action_server, &node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}

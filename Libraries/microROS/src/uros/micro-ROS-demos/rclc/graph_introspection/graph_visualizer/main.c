#include <rcl/rcl.h>
#include <rcl/graph.h>
#include <rcl/error_handling.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

int main(int argc, const char * const * argv)
{
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  RCCHECK(rcl_init_options_init(&options, allocator));

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context));

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "microros_graph_tester", "", &context, &node_ops));

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 0, 1, 0, 0, 0, 0, &context, allocator));

  const rcl_guard_condition_t * graph_guard_condition =
    rcl_node_get_graph_guard_condition(&node);
  if (NULL == graph_guard_condition) {
    printf("Error: node graph guard condition is invalid\n");
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_shutdown(&context));
    return 1;
  }

  while (true)
  {
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set));

    size_t index;
    RCCHECK(rcl_wait_set_add_guard_condition(
      &wait_set, graph_guard_condition, &index));

    rcl_ret_t unused = rcl_wait(&wait_set, RCL_MS_TO_NS(100));

    for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
      if (wait_set.guard_conditions[i]) {
        // Topic names and types
        rcl_names_and_types_t topic_names_and_types =
          rcl_get_zero_initialized_names_and_types();
        RCCHECK(rcl_get_topic_names_and_types(&node, &allocator, false, &topic_names_and_types));

        printf("\n---------------------------------------------------------------\n");
        printf("  'rcl_get_topic_names_and_types' result\n");
        printf("---------------------------------------------------------------\n");
        size_t topic_num = topic_names_and_types.names.size;
        printf("Current number of ROS 2 and micro-ROS topics: %lu\n", topic_num);

        for (size_t j = 0; j < topic_num; ++j) {
          rcutils_string_array_t * topic_types = &topic_names_and_types.types[j];
          printf("  %lu)  topic: '%s', types: ", j + 1, topic_names_and_types.names.data[j]);
          for (size_t k = 0; k < topic_types->size; ++k) {
            printf("%s", topic_types->data[k]);
            if (k < (topic_types->size - 1)) {
              printf(", ");
            } else {
              printf("\n");
            }
          }
          rcl_topic_endpoint_info_array_t publishers_info =
            rcl_get_zero_initialized_topic_endpoint_info_array();
          RCCHECK(rcl_get_publishers_info_by_topic(&node, &allocator,
            topic_names_and_types.names.data[j], false, &publishers_info));
          printf("        topic endpoint information:\n");
          for (size_t k = 0; k < publishers_info.size; ++k) {
            printf("          Node: '%s%s', type: '%s', '%s'\n",
              publishers_info.info_array[k].node_namespace,
              publishers_info.info_array[k].node_name,
              publishers_info.info_array[k].topic_type,
              RMW_ENDPOINT_PUBLISHER == publishers_info.info_array[k].endpoint_type ? "publisher" : "error");
          }
          rcl_topic_endpoint_info_array_fini(&publishers_info, &allocator);

          rcl_topic_endpoint_info_array_t subscriptions_info =
            rcl_get_zero_initialized_topic_endpoint_info_array();
          RCCHECK(rcl_get_subscriptions_info_by_topic(&node, &allocator,
            topic_names_and_types.names.data[j], false, &subscriptions_info));
          for (size_t k = 0; k < subscriptions_info.size; ++k) {
            printf("          Node: '%s%s', type: '%s', '%s'\n",
              subscriptions_info.info_array[k].node_namespace,
              subscriptions_info.info_array[k].node_name,
              subscriptions_info.info_array[k].topic_type,
              RMW_ENDPOINT_SUBSCRIPTION == subscriptions_info.info_array[k].endpoint_type ? "subscription" : "error");
          }
          rcl_topic_endpoint_info_array_fini(&subscriptions_info, &allocator);
        }

        RCSOFTCHECK(rcl_names_and_types_fini(&topic_names_and_types));

        // Service names and types
        rcl_names_and_types_t service_names_and_types =
          rcl_get_zero_initialized_names_and_types();
        RCCHECK(rcl_get_service_names_and_types(&node, &allocator, &service_names_and_types));

        printf("\n---------------------------------------------------------------\n");
        printf("  'rcl_get_service_names_and_types' result\n");
        printf("---------------------------------------------------------------\n");
        size_t service_num = service_names_and_types.names.size;
        printf("Current number of ROS 2 and micro-ROS services: %lu\n", service_num);

        for (size_t j = 0; j < service_num; ++j) {
          rcutils_string_array_t * service_types = &service_names_and_types.types[j];
          printf("  %lu)  service: '%s', types: ", j + 1, service_names_and_types.names.data[j]);
          for (size_t k = 0; k < service_types->size; ++k) {
            printf("%s", service_types->data[k]);
            if (k < (service_types->size - 1)) {
              printf(", ");
            } else {
              printf("\n");
            }
          }
        }

        RCSOFTCHECK(rcl_names_and_types_fini(&service_names_and_types));

        // Node names
        rcutils_string_array_t node_names = rcutils_get_zero_initialized_string_array();
        rcutils_string_array_t node_namespaces = rcutils_get_zero_initialized_string_array();
        RCCHECK(rcl_get_node_names(&node, allocator, &node_names, &node_namespaces));

        printf("\n---------------------------------------------------------------\n");
        printf("  'rcl_get_node_names' result\n");
        printf("---------------------------------------------------------------\n");
        size_t node_num = node_names.size;
        printf("Current number of ROS 2 and micro-ROS nodes: %lu\n", node_num);

        for (size_t j = 0; j < node_num; ++j) {
          printf("  %lu)  node: '%s%s'\n", j + 1, node_namespaces.data[j], node_names.data[j]);
          printf("    'rcl_get_publisher_names_and_types_by_node' result for this node:\n");
          rcl_names_and_types_t publisher_names_and_types = rcl_get_zero_initialized_names_and_types();
          RCCHECK(rcl_get_publisher_names_and_types_by_node(
            &node, &allocator, false, node_names.data[j], node_namespaces.data[j],
            &publisher_names_and_types));
          for (size_t k = 0; k < publisher_names_and_types.names.size; ++k) {
            printf("      %lu.%lu)  %s, with types: ", j + 1, k + 1,
              publisher_names_and_types.names.data[k]);
            for (size_t l = 0; l < publisher_names_and_types.types[k].size; ++l) {
              printf("%s", publisher_names_and_types.types[k].data[l]);
              if (l < (publisher_names_and_types.types[k].size - 1)) {
                printf(", ");
              } else {
                printf("\n");
              }
            }
          }
          RCSOFTCHECK(rcl_names_and_types_fini(&publisher_names_and_types));
        }

        if (RCUTILS_RET_OK != rcutils_string_array_fini(&node_names) ||
            RCUTILS_RET_OK != rcutils_string_array_fini(&node_namespaces)) {
          printf("Error while freeing rcutils resources\n");
          return 1;
        }
      }
    }
  }

  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rcl_shutdown(&context));
  return 0;
}

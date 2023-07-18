#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>

#include <stdio.h>

#define RCCHECK(fn) {\
  rcl_ret_t temp_rc = fn;\
  if ((temp_rc != RCL_RET_OK)) {\
    printf("Failed status on line %d: %d. Aborting.\n", \
        __LINE__, (int)temp_rc);\
    return 1;\
  } \
}
#define RCSOFTCHECK(fn) {\
  rcl_ret_t temp_rc = fn;\
  if ((temp_rc != RCL_RET_OK)) {\
    printf("Failed status on line %d: %d. Continuing.\n", \
        __LINE__, (int)temp_rc);\
  }\
}

rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;

void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Vector3 * msg =
      (const geometry_msgs__msg__Vector3 *)msgin;
  printf("Vector3: x: %f, y: %f, z: %f\n", msg->x, msg->y, msg->z);
}

int main()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(
      &node, "vector3_subscriber_rclc", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "geometry_msgs_msg_Vector3"));

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  rclc_executor_spin(&executor);

  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));
}

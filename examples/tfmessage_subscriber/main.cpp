#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <micro_ros_utilities/type_utilities.h>

#include <rosidl_runtime_c/string.h>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>

#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <tf2_msgs/msg/tf_message.h>

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
tf2_msgs__msg__TFMessage msg;

// TFMessage contains uninitialised sequence values
// https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
//
// msg.transforms.data is NULL or uninitialised
// msg.transforms.size is 0 or uninitialised
// msg.transforms.capacity is 0 or uninitialised

void subscription_callback(const void * msgin)
{
  const tf2_msgs__msg__TFMessage * msg =
      (const tf2_msgs__msg__TFMessage *)msgin;

  size_t size = msg->transforms.size;
  geometry_msgs__msg__TransformStamped * data = msg->transforms.data;

  printf("TFMessage: size: %zu\n", size);

  for (size_t i = 0; i < size; ++i) {
    const std_msgs__msg__Header * header = &data[i].header;
    const rosidl_runtime_c__String * frame_id = &header->frame_id;

    const rosidl_runtime_c__String * child_frame_id = &data[i].child_frame_id;
    const geometry_msgs__msg__Transform * transform = &data[i].transform;
    const geometry_msgs__msg__Vector3 * translation = &transform->translation;
    const geometry_msgs__msg__Quaternion * rotation = &transform->rotation;

    printf("frame_id[%zu]: %s, child_frame_id[%zu]: %s\n",
        i, frame_id->data, i, child_frame_id->data);
    printf("translation[%zu]: x: %f, y: %f, z: %f\n",
        i, translation->x, translation->y, translation->z);
    printf("rotation[%zu]:    x: %f, y: %f, z: %f, w: %f\n",
        i, rotation->x, rotation->y, rotation->z, rotation->w);
  }
  printf("\n");
}

int main()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "tf2_subscriber_rclc", "", &support));

  // memory management
  // static geometry_msgs__msg__TransformStamped buffer[100];
  // msg.transforms.data = buffer;
  // msg.transforms.size = 0;
  // msg.transforms.capacity = 100;

  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = 100;
  conf.max_basic_type_sequence_capacity = 100;

  bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    &msg,
    conf
  );

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf2_msgs_msg_TFMessage"));

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  rclc_executor_spin(&executor);

  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));
}

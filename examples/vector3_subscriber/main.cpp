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

class AP_UROS_Client
{
public:
    bool start();
    void main_loop();
    bool init();
    bool create();

#if AP_UROS_UDP_ENABLED
    // functions for udp transport
    bool urosUdpInit();
    static bool udp_transport_open(uxrCustomTransport* transport);
    static bool udp_transport_close(uxrCustomTransport* transport);
    static size_t udp_transport_write(uxrCustomTransport* transport,
            const uint8_t* buf, size_t len, uint8_t* error);
    static size_t udp_transport_read(uxrCustomTransport* transport,
            uint8_t* buf, size_t len, int timeout, uint8_t* error);

    struct {
        AP_Int32 port;
        // UDP endpoint
        const char* ip = "127.0.0.1";
        // UDP Allocation
        uxrCustomTransport transport;
        SocketAPM *socket;
    } udp;
#endif

private:
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
};

void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Vector3 * msg =
      (const geometry_msgs__msg__Vector3 *)msgin;
  printf("Vector3: x: %f, y: %f, z: %f\n", msg->x, msg->y, msg->z);
}

bool AP_UROS_Client::start()
{
    main_loop();
}

void AP_UROS_Client::main_loop()
{
    if (!init() || !create()) {
        printf("UROS: creation failed\n");
        return;
    }
    printf("UROS: initialization passed\n");

    // one-time actions


    // periodic actions
    // rclc_executor_spin(&executor);

    // RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    // RCSOFTCHECK(rcl_node_fini(&node));
}

bool AP_UROS_Client::init()
{
    // initialize transport
    bool initTransportStatus = true;

#if AP_UROS_UDP_ENABLED
    // fallback to UDP if available
    if (!initTransportStatus) {
        initTransportStatus = urosUdpInit();
    }
#endif

    if (initTransportStatus) {
        printf("UROS: transport initializated\n");
    }
    else {
        printf("UROS: transport initialization failed\n");
        return false;
    }

    // // create allocator
    // allocator = rcl_get_default_allocator();

    // // create init_options
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // // create node
    // RCCHECK(rclc_node_init_default(
    //     &node, "ardupilot_uros_rclc", "", &support));

    // // create executor
    // executor = rclc_executor_get_zero_initialized_executor();
    // RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    printf("UROS: init complete\n");

    return true;

}

bool AP_UROS_Client::create()
{
    // create subscriber
    // RCCHECK(rclc_subscription_init_default(
    //     &subscriber,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    //     "geometry_msgs_msg_Vector3"));

    // RCCHECK(rclc_executor_add_subscription(
    //     &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    printf("UROS: create complete\n");

    return true;
}

int main()
{
    AP_UROS_Client uros_client;
    uros_client.start();
}

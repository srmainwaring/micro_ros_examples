// micro-xrce-dds
#include "uxr/client/client.h"

// micro-ros
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>

#include <stdio.h>

// ardupilot
#include "Socket.h"

#define AP_UROS_UDP_ENABLED 1

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
    void start();
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
        uint16_t port {2019};
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

void AP_UROS_Client::start()
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
    rclc_executor_spin(&executor);

    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
}

bool AP_UROS_Client::init()
{
    // initialize transport
    bool initTransportStatus = false;

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

    // create allocator
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(
        &node, "ardupilot_uros_rclc", "", &support));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    printf("UROS: init complete\n");

    return true;
}

bool AP_UROS_Client::create()
{
    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "geometry_msgs_msg_Vector3"));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    printf("UROS: create complete\n");

    return true;
}

#if AP_UROS_UDP_ENABLED

#include <rmw_microros/custom_transport.h>

#include <errno.h>

/*
  open connection on UDP
 */
bool AP_UROS_Client::udp_transport_open(uxrCustomTransport *t)
{
    printf("udp_transport_open\n");

    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    auto *sock = new SocketAPM(true);
    if (sock == nullptr) {
        return false;
    }
    if (!sock->connect(uros->udp.ip, uros->udp.port)) {
        return false;
    }
    uros->udp.socket = sock;
    return true;
}

/*
  close UDP connection
 */
bool AP_UROS_Client::udp_transport_close(uxrCustomTransport *t)
{
    printf("udp_transport_close\n");

    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    delete uros->udp.socket;
    uros->udp.socket = nullptr;
    return true;
}

/*
  write on UDP
 */
size_t AP_UROS_Client::udp_transport_write(uxrCustomTransport *t,
    const uint8_t* buf, size_t len, uint8_t* error)
{
    printf("udp_transport_write: len: %zu\n", len);

    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    if (uros->udp.socket == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const ssize_t ret = uros->udp.socket->send(buf, len);
    if (ret <= 0) {
        *error = errno;
        return 0;
    }
    return ret;
}

/*
  read from UDP
 */
size_t AP_UROS_Client::udp_transport_read(uxrCustomTransport *t,
    uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    printf("udp_transport_read: len: %zu\n", len);

    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    if (uros->udp.socket == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const ssize_t ret = uros->udp.socket->recv(buf, len, timeout_ms);
    if (ret <= 0) {
        *error = errno;
        return 0;
    }
    return ret;
}

/*
  initialise UDP connection
 */
bool AP_UROS_Client::urosUdpInit()
{
    printf("urosUdpInit\n");

    // setup a non-framed transport for UDP
    rmw_ret_t rcl_ret = rmw_uros_set_custom_transport(
            false,
            (void*)this,
            udp_transport_open,
            udp_transport_close,
            udp_transport_write,
            udp_transport_read);

    return (rcl_ret == RCL_RET_OK);
}

#endif // AP_UROS_UDP_ENABLED

int main()
{
    AP_UROS_Client uros_client;
    uros_client.start();
}

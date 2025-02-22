#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/subscription.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>

// void cmd_vel_callback(const void *msg_in)
// {
//     const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
//     printf("Received Twist Message: Linear X: %f, Angular Z: %f\n", msg->linear.x, msg->angular.z);
// }

void cmd_vel_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    printf("Received Twist Message:\n");
    printf("  Linear X: %f\n", msg->linear.x);
    printf("  Angular Z: %f\n", msg->angular.z);
}


int main(int argc, char *argv[])
{
    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    
    // Initialize ROS 2
    rclc_support_t support;
    rc = rclc_support_init(&support, argc, (const char *const *)argv, &allocator);
    if (rc != RCL_RET_OK)
    {
        printf("Error initializing ROS support\n");
        return -1;
    }

    // Create a Node
    rcl_node_t node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "teleop_listener", "", &support);
    if (rc != RCL_RET_OK)
    {
        printf("Error creating node\n");
        return -1;
    }

    // Create a Subscription
    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    rc = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    if (rc != RCL_RET_OK)
    {
        printf("Error creating subscriber\n");
        return -1;
    }

    // Create Executor
    rclc_executor_t executor;
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (rc != RCL_RET_OK)
    {
        printf("Error creating executor\n");
        return -1;
    }

    geometry_msgs__msg__Twist msg;
    rc = rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        printf("Error adding subscription to executor\n");
        return -1;
    }

    // Spin
    printf("Listening for /cmd_vel messages...\n");
    while (rcl_context_is_valid(&support.context))
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup
    rc = rcl_subscription_fini(&subscriber, &node);
    rc = rcl_node_fini(&node);
    rclc_support_fini(&support);
    
    return 0;
}

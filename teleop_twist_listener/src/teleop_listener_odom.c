#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#define ODOM_TOPIC "/odom"
#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F
#define RIGHT_A_PIN 0
#define RIGHT_B_PIN 1
#define LEFT_A_PIN 3
#define LEFT_B_PIN 4

int right_step_count = 0;
int left_step_count = 0;
double right_speed = 0.0;
double left_speed = 0.0;

typedef struct {
    double x;
    double y;
    double theta;
} RobotPose;

RobotPose robot_pose = {0.0, 0.0, 0.0};

void enter_extended_function_mode() {
    outb(0x87, LPC_ADDR_PORT);
    outb(0x87, LPC_ADDR_PORT);
}

void leave_extended_function_mode() {
    outb(0xAA, LPC_ADDR_PORT);
}

int read_gpio_pin(int pin) {
    int bit_map;
    enter_extended_function_mode();
    outb(0x07, LPC_ADDR_PORT);
    bit_map = inb(LPC_DATA_PORT);
    leave_extended_function_mode();
    return (bit_map & (1 << pin)) ? 1 : 0;
}

void update_encoder() {
    static int prev_right_a = 0, prev_right_b = 0;
    static int prev_left_a = 0, prev_left_b = 0;
    static struct timespec prev_time = {0, 0};

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
                          (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

    int right_a = read_gpio_pin(RIGHT_A_PIN);
    int right_b = read_gpio_pin(RIGHT_B_PIN);
    int left_a = read_gpio_pin(LEFT_A_PIN);
    int left_b = read_gpio_pin(LEFT_B_PIN);

    if (right_a != prev_right_a || right_b != prev_right_b) {
        if (prev_right_a == 0 && right_a == 1) {
            if (right_b == 0) right_step_count++;
            else right_step_count--;
        }
    }
    if (left_a != prev_left_a || left_b != prev_left_b) {
        if (prev_left_a == 0 && left_a == 1) {
            if (left_b == 0) left_step_count++;
            else left_step_count--;
        }
    }

    right_speed = right_step_count / elapsed_time;
    left_speed = left_step_count / elapsed_time;

    prev_right_a = right_a;
    prev_right_b = right_b;
    prev_left_a = left_a;
    prev_left_b = left_b;
    prev_time = current_time;
    right_step_count = 0;
    left_step_count = 0;
}

void publish_odom(rcl_publisher_t *publisher) {
    nav_msgs__msg__Odometry odom_msg;
    odom_msg.twist.twist.linear.x = (right_speed + left_speed) / 2.0;
    odom_msg.twist.twist.angular.z = (right_speed - left_speed) / 0.5;
    rcl_publish(publisher, &odom_msg, NULL);
}

int main(int argc, char *argv[]) {
    if (iopl(3)) {
        perror("Failed to gain I/O privilege level");
        exit(1);
    }

    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rc = rclc_support_init(&support, argc, (const char *const *)argv, &allocator);
    if (rc != RCL_RET_OK) return -1;

    rcl_node_t node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "odom_publisher", "", &support);
    if (rc != RCL_RET_OK) return -1;

    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rc = rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC);
    if (rc != RCL_RET_OK) return -1;

    while (rcl_context_is_valid(&support.context)) {
        update_encoder();
        publish_odom(&publisher);
        usleep(100000);
    }

    return 0;
}

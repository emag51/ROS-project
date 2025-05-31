#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/subscription.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ROS 2 Subscription
#define CMD_VEL_TOPIC "/cmd_vel"

// LPC (Low Pin Count) communication
#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

// Motor control pins
#define RIGHT_INA_PIN 0
#define RIGHT_INB_PIN 1
#define RIGHT_PWM_PIN 2
#define LEFT_INA_PIN 3
#define LEFT_INB_PIN 4
#define LEFT_PWM_PIN 5

// Encoder pins
#define RIGHT_A_PIN 0
#define RIGHT_B_PIN 1
#define LEFT_A_PIN 3
#define LEFT_B_PIN 4

// Speed variables
int right_step_count = 0;
int left_step_count = 0;
double right_speed = 0.0;
double left_speed = 0.0;

// PID control variables (Optional tuning)
double Kp = 1.0, Ki = 0.0, Kd = 0.0; // PID gains
double prev_error = 0.0, integral = 0.0;

// ROS Message for Twist
geometry_msgs__msg__Twist received_msg;

// LPC Functions
void enter_extended_function_mode() {
    outb(0x87, LPC_ADDR_PORT);
    outb(0x87, LPC_ADDR_PORT);
}

void leave_extended_function_mode() {
    outb(0xAA, LPC_ADDR_PORT);
}

void lpc_reg_write(int reg, int val) {
    outb(reg, LPC_ADDR_PORT);
    outb(val, LPC_DATA_PORT);
}

int lpc_reg_read(int reg) {
    outb(reg, LPC_ADDR_PORT);
    return inb(LPC_DATA_PORT);
}

int read_gpio_pin(int pin) {
    int bit_map;
    enter_extended_function_mode();
    lpc_reg_write(0x07, 7);
    bit_map = lpc_reg_read(0xF5);
    leave_extended_function_mode();
    return (bit_map & (1 << pin)) ? 1 : 0;
}

void set_pin_level(int pin, int level) {
    enter_extended_function_mode();
    lpc_reg_write(0x07, 8);
    int bit_map = lpc_reg_read(0xE1);
    if (level)
        bit_map |= (1 << pin);
    else
        bit_map &= ~(1 << pin);
    lpc_reg_write(0xE1, bit_map);
    leave_extended_function_mode();
}

void set_pwm_level(int pin, int pwm_value) {
    set_pin_level(pin, 1);
    usleep(pwm_value * 10);
    set_pin_level(pin, 0);
    usleep((255 - pwm_value) * 10);
}

// Encoder update function
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

    printf("Right Speed: %.2f steps/sec, Left Speed: %.2f steps/sec\n", right_speed, left_speed);

    prev_right_a = right_a;
    prev_right_b = right_b;
    prev_left_a = left_a;
    prev_left_b = left_b;
    prev_time = current_time;
    right_step_count = 0;
    left_step_count = 0;
}

// ROS 2 Callback: Control robot based on /cmd_vel
void cmd_vel_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;

    double linear_x = msg->linear.x;  // Forward/backward speed
    double angular_z = msg->angular.z; // Turning speed

    // Convert to motor PWM values (example scaling)
    int right_pwm = (int)(linear_x * 100 + angular_z * 50);
    int left_pwm = (int)(linear_x * 100 - angular_z * 50);

    // Keep PWM within valid range
    if (right_pwm > 255) right_pwm = 255;
    if (left_pwm > 255) left_pwm = 255;
    if (right_pwm < 0) right_pwm = 0;
    if (left_pwm < 0) left_pwm = 0;

    // Drive the motors
    printf("Motor Command - Right PWM: %d, Left PWM: %d\n", right_pwm, left_pwm);
    set_pwm_level(RIGHT_PWM_PIN, right_pwm);
    set_pwm_level(LEFT_PWM_PIN, left_pwm);
}

// ROS 2 Main Function
int main(int argc, char *argv[]) {
    if (iopl(3)) {
        perror("Failed to gain I/O privilege level");
        exit(1);
    }

    // ROS 2 Setup
    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    
    rclc_support_t support;
    rc = rclc_support_init(&support, argc, (const char *const *)argv, &allocator);
    if (rc != RCL_RET_OK) {
        printf("Error initializing ROS support\n");
        return -1;
    }

    rcl_node_t node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "motor_controller", "", &support);
    if (rc != RCL_RET_OK) {
        printf("Error creating node\n");
        return -1;
    }

    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    rc = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    );

    if (rc != RCL_RET_OK) {
        printf("Error creating subscriber\n");
        return -1;
    }

    rclc_executor_t executor;
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    rc = rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &cmd_vel_callback, ON_NEW_DATA);

    printf("Listening for /cmd_vel messages...\n");
    while (rcl_context_is_valid(&support.context)) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        update_encoder();
    }

    return 0;
}

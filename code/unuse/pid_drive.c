// sudo gcc pid_drive.c -o pid-test -O2
// sudo ./pid-test
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>

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

int right_step_count = 0;
int left_step_count = 0;
double right_speed = 0.0;
double left_speed = 0.0;

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
    if (right_speed != 0.0 || left_speed != 0.0) {
        printf("Right Speed: %.2f steps/sec, Left Speed: %.2f steps/sec\n", right_speed, left_speed);
    }

    prev_right_a = right_a;
    prev_right_b = right_b;
    prev_left_a = left_a;
    prev_left_b = left_b;
    prev_time = current_time;
    right_step_count = 0;
    left_step_count = 0;
}

// void update_encoder() {
//     static int prev_right_a = 0, prev_right_b = 0;
//     static int prev_left_a = 0, prev_left_b = 0;
//     static struct timespec prev_time = {0, 0};

//     struct timespec current_time;
//     clock_gettime(CLOCK_MONOTONIC, &current_time);
//     double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
//                           (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

//     int right_a = read_gpio_pin(RIGHT_A_PIN);
//     int right_b = read_gpio_pin(RIGHT_B_PIN);
//     int left_a = read_gpio_pin(LEFT_A_PIN);
//     int left_b = read_gpio_pin(LEFT_B_PIN);

//     // Update step counts based on encoder inputs
//     if (right_a != prev_right_a || right_b != prev_right_b) {
//         if (prev_right_a == 0 && right_a == 1) {
//             if (right_b == 0) right_step_count++;
//             else right_step_count--;
//         }
//     }
//     if (left_a != prev_left_a || left_b != prev_left_b) {
//         if (prev_left_a == 0 && left_a == 1) {
//             if (left_b == 0) left_step_count++;
//             else left_step_count--;
//         }
//     }

//     // Calculate speed in steps per second
//     if (elapsed_time > 0.0) {
//         right_speed = right_step_count / elapsed_time;
//         left_speed = left_step_count / elapsed_time;

//         // Print the speed once per second or other criteria
//         if (elapsed_time >= 1.0) {
//             if (right_speed != 0.0 || left_speed != 0.0) {
//                 printf("Right Speed: %.2f steps/sec, Left Speed: %.2f steps/sec\n", right_speed, left_speed);
//             }

//             // Reset the step counts after printing speed
//             right_step_count = 0;
//             left_step_count = 0;
//             prev_time = current_time;  // Update time after print
//         }
//     }

//     prev_right_a = right_a;
//     prev_right_b = right_b;
//     prev_left_a = left_a;
//     prev_left_b = left_b;
// }

// void update_encoder() {
//     static int prev_right_a = 0, prev_right_b = 0;
//     static int prev_left_a = 0, prev_left_b = 0;
//     static struct timespec prev_time = {0, 0};

//     struct timespec current_time;
//     clock_gettime(CLOCK_MONOTONIC, &current_time);
//     double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
//                           (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

//     int right_a = read_gpio_pin(RIGHT_A_PIN);
//     int right_b = read_gpio_pin(RIGHT_B_PIN);
//     int left_a = read_gpio_pin(LEFT_A_PIN);
//     int left_b = read_gpio_pin(LEFT_B_PIN);

//     // Debug prints to check encoder pin states
//     printf("Right A: %d, Right B: %d, Left A: %d, Left B: %d\n", right_a, right_b, left_a, left_b);

//     // Update step counts based on encoder inputs
//     if (right_a != prev_right_a || right_b != prev_right_b) {
//         if (prev_right_a == 0 && right_a == 1) {
//             if (right_b == 0) right_step_count++;
//             else right_step_count--;
//         }
//     }
//     if (left_a != prev_left_a || left_b != prev_left_b) {
//         if (prev_left_a == 0 && left_a == 1) {
//             if (left_b == 0) left_step_count++;
//             else left_step_count--;
//         }
//     }

//     // Debug print for step counts
//     printf("Right Step Count: %d, Left Step Count: %d\n", right_step_count, left_step_count);

//     // Calculate speed in steps per second
//     if (elapsed_time > 0.0) {
//         right_speed = right_step_count / elapsed_time;
//         left_speed = left_step_count / elapsed_time;

//         // Print the speed once per second or other criteria
//         if (elapsed_time >= 1.0) {
//             if (right_speed != 0.0 || left_speed != 0.0) {
//                 printf("Right Speed: %.2f steps/sec, Left Speed: %.2f steps/sec\n", right_speed, left_speed);
//             }

//             // Reset the step counts after printing speed
//             right_step_count = 0;
//             left_step_count = 0;
//             prev_time = current_time;  // Update time after print
//         }
//     }

//     prev_right_a = right_a;
//     prev_right_b = right_b;
//     prev_left_a = left_a;
//     prev_left_b = left_b;
// }

void control_robot(int right_pwm, int left_pwm, int duration_sec) {
    printf("Running robot for %d seconds with PWM Right: %d, Left: %d\n", duration_sec, right_pwm, left_pwm);
    set_pin_level(RIGHT_INA_PIN, 0);
    set_pin_level(RIGHT_INB_PIN, 1);
    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current);
        double elapsed_time = (current.tv_sec - start.tv_sec) +
                              (current.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed_time >= duration_sec) break;

        set_pwm_level(RIGHT_PWM_PIN, right_pwm);
        set_pwm_level(LEFT_PWM_PIN, left_pwm);
        update_encoder();
        usleep(10);
    }

    printf("Stopping robot\n");
    set_pin_level(RIGHT_PWM_PIN, 0);
    set_pin_level(LEFT_PWM_PIN, 0);
}

void turn_left(int pwm_value, int duration_sec) {
    control_robot(80, 40, duration_sec);
}

void turn_right(int pwm_value, int duration_sec) {
    control_robot(40, 80, duration_sec);
}

int main() {
    if (iopl(3)) {
        perror("Failed to gain I/O privilege level");
        exit(1);
    }

    control_robot(50, 50, 15);
    control_robot(150, 150, 15);
    control_robot(250, 250, 15);
    // turn_left(80, 2);
    // turn_right(80, 2);

    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}


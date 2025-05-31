#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

// Motor control pins for LEFT motor
#define LEFT_INA_PIN 0
#define LEFT_INB_PIN 1
#define LEFT_PWM_PIN 2

// Encoder pins for LEFT motor
#define LEFT_A_PIN 3
// #define LEFT_B_PIN 3

#define PPR 1024
#define MEASURE_INTERVAL 0.4

// Global variables (fixed duplication)
int left_step_count = 0;
double left_rpm = 0.0;
double left_setpoint = 70.0;

double Kp = 1.0, Ki = 0.1, Kd = 0.01;
double left_integral = 0.0, prev_left_error = 0.0;

// LPC I/O
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
    enter_extended_function_mode();
    lpc_reg_write(0x07, 7);
    int bit_map = lpc_reg_read(0xF5);
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
    int pulse_width = (pwm_value / 255.0) * 800;
    set_pin_level(pin, 1);
    usleep(pulse_width);
    set_pin_level(pin, 0);
    usleep(800 - pulse_width);
}

// // Encoder logic (fixed initial state and removed duplicates)
// void update_left_encoder() {
//     static int prev_a_state = -1, prev_b_state = -1;
//     static struct timespec prev_time = {0, 0};

//     int a_state = read_gpio_pin(LEFT_A_PIN);
//     int b_state = read_gpio_pin(LEFT_B_PIN);

//     if (prev_a_state == -1) {
//         prev_a_state = a_state;
//         prev_b_state = b_state;
//         clock_gettime(CLOCK_MONOTONIC, &prev_time);
//         return;
//     }

//     if (a_state != prev_a_state) {
//         if (a_state == 1) {
//             left_step_count += (b_state == 0) ? 1 : -1;
//         } else {
//             left_step_count += (b_state == 1) ? 1 : -1;
//         }
//     }

//     prev_a_state = a_state;
//     prev_b_state = b_state;

//     struct timespec current_time;
//     clock_gettime(CLOCK_MONOTONIC, &current_time);
//     double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
//                           (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

//     if (elapsed_time >= MEASURE_INTERVAL) {
//         left_rpm = ((left_step_count / (double)PPR) / elapsed_time) * 60.0;
//         printf("Left RPM: %.2f\n", left_rpm);
//         left_step_count = 0;
//         prev_time = current_time;
//     }
// }

void update_left_encoder() {
    static int prev_a_state = 0;
    static struct timespec prev_time = {0, 0};

    int a_state = read_gpio_pin(LEFT_A_PIN);

    if (a_state == 1 && prev_a_state == 0) {  // Rising edge only
        left_step_count++;  // Count only forward (assumes motor moves in one direction)
    }

    prev_a_state = a_state;

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
                          (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

    if (elapsed_time >= MEASURE_INTERVAL) {
        left_rpm = ((left_step_count / (double)PPR) / MEASURE_INTERVAL) * 60.0;
        printf("Left RPM: %.2f\n", left_rpm);
        left_step_count = 0;
        prev_time = current_time;
    }
}

double pid_calculate(double setpoint, double current_rpm, double *integral, double *prev_error) {
    double error = setpoint - current_rpm;
    *integral += error;
    double derivative = error - *prev_error;
    double output = Kp * error + Ki * (*integral) + Kd * derivative;
    *prev_error = error;
    return output;
}

void control_motor_pid(double setpoint, int duration_sec) {
    printf("Running LEFT motor for %d seconds with PID control, Setpoint: %.2f\n", duration_sec, setpoint);

    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current);
        double elapsed_time = (current.tv_sec - start.tv_sec) +
                              (current.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed_time >= duration_sec) break;

        update_left_encoder();  // ✅ Fixed function name
        double pwm = pid_calculate(setpoint, left_rpm, &left_integral, &prev_left_error);

        if (pwm > 255) pwm = 255;
        if (pwm < 0) pwm = 0;

        set_pwm_level(LEFT_PWM_PIN, (int)pwm);
        usleep(10000);
    }

    printf("Stopping LEFT motor\n");
    set_pin_level(LEFT_PWM_PIN, 0);
}

int main() {
    if (iopl(3)) {
        perror("Failed to gain I/O privilege level");
        exit(1);
    }

    control_motor_pid(left_setpoint, 10);
    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}

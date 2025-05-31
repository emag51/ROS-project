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

// Encoder pins
#define RIGHT_A_PIN 0
#define RIGHT_B_PIN 1
#define PPR 500  // Pulses Per Revolution for HKT3001-500B
#define MEASURE_INTERVAL 0.1  // 400 ms (0.4 sec)

// Global variables
int right_step_count = 0;
double right_rpm = 0.0;
double right_setpoint = 50.0; // Target speed for the right motor

// PID parameters
double Kp = 1.0, Ki = 0.1, Kd = 0.01;
double right_integral = 0.0, prev_right_error = 0.0;

// LPC I/O Functions
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

// GPIO Read/Write
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

// Encoder Update Function
void update_encoder() {
    static int prev_right_a = 0, prev_right_b = 0;
    static struct timespec prev_time = {0, 0};

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
                          (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

    if (elapsed_time >= MEASURE_INTERVAL) {
        int right_a = read_gpio_pin(RIGHT_A_PIN);
        int right_b = read_gpio_pin(RIGHT_B_PIN);

        if (right_a != prev_right_a || right_b != prev_right_b) {
            if (prev_right_a == 0 && right_a == 1) {
                if (right_b == 0) right_step_count++;
                else right_step_count--;
            }
        }

        // Convert steps to RPM
        right_rpm = ((right_step_count / (double)PPR) / MEASURE_INTERVAL);

        printf("Right RPM: %.2f\n", right_rpm);

        prev_right_a = right_a;
        prev_right_b = right_b;
        prev_time = current_time;
        right_step_count = 0; // Reset pulse count every 400ms
    }
}

// PID Controller
double pid_calculate(double setpoint, double current_rpm, double *integral, double *prev_error) {
    double error = setpoint - current_rpm;
    *integral += error;
    double derivative = error - *prev_error;
    double output = Kp * error + Ki * (*integral) + Kd * derivative;
    *prev_error = error;
    return output;
}

// Motor Control with PID
void control_motor_pid(double right_setpoint, int duration_sec) {
    printf("Running motor for %d seconds with PID control, Setpoint: %.2f\n", duration_sec, right_setpoint);

    set_pin_level(RIGHT_INA_PIN, 0);
    set_pin_level(RIGHT_INB_PIN, 1);

    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current);
        double elapsed_time = (current.tv_sec - start.tv_sec) +
                              (current.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed_time >= duration_sec) break;

        update_encoder();
        double right_pwm = pid_calculate(right_setpoint, right_rpm, &right_integral, &prev_right_error);

        if (right_pwm > 255) right_pwm = 255;
        if (right_pwm < 0) right_pwm = 0;

        set_pwm_level(RIGHT_PWM_PIN, (int)right_pwm);
        usleep(10000);
    }

    printf("Stopping motor\n");
    set_pin_level(RIGHT_PWM_PIN, 0);
}

// Main Function
int main() {
    if (iopl(3)) {
        perror("Failed to gain I/O privilege level");
        exit(1);
    }

    control_motor_pid(right_setpoint, 10);
    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}

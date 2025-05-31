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
#define GPIO_A_PIN 0
#define GPIO_B_PIN 1

// PID parameters
float Kp = 1.5, Ki = 0.1, Kd = 0.05;
float target_speed = 100.0;
float integral = 0, previous_error = 0;

int step_count = 0; // Encoder steps

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
    set_pin_level(pin, 1);
    usleep(pwm_value * 10);
    set_pin_level(pin, 0);
    usleep((255 - pwm_value) * 10);
}

void update_encoder() {
    static int prev_a_state = 0;
    int a_state = read_gpio_pin(GPIO_A_PIN);
    int b_state = read_gpio_pin(GPIO_B_PIN);
    if (a_state != prev_a_state) {
        step_count += (a_state == b_state) ? 1 : -1;
    }
    prev_a_state = a_state;
}

float pid_control(float target, float actual) {
    float error = target - actual;
    integral += error;
    float derivative = error - previous_error;
    previous_error = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void control_motor(float target_speed) {
    float actual_speed = step_count; // Simplified speed estimation
    step_count = 0;
    float pid_output = pid_control(target_speed, actual_speed);
    int pwm_value = (int)pid_output;
    if (pwm_value > 255) pwm_value = 255;
    if (pwm_value < 0) pwm_value = 0;
    set_pwm_level(RIGHT_PWM_PIN, pwm_value);
    set_pwm_level(LEFT_PWM_PIN, pwm_value);
}

int main() {
    if (iopl(3)) {
        perror("Failed to gain I/O privileges");
        exit(1);
    }
    while (1) {
        update_encoder();
        control_motor(target_speed);
        usleep(50000);
    }
    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}

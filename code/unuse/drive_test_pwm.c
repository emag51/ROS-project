// sudo gcc drive_test_pwm.c -o drive_test_pwm -O2
// sudo ./drive_test_pwm
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <errno.h>
#include <time.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

// Right motor pins
#define RIGHT_INA_PIN 0 // GPO0
#define RIGHT_INB_PIN 1 // GPO1
#define RIGHT_PWM_PIN 2 // GPO2

// Left motor pins
#define LEFT_INA_PIN 3  // GPO3
#define LEFT_INB_PIN 4  // GPO4
#define LEFT_PWM_PIN 5  // GPO5

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

void set_pin_level(int pin, int level) {
    enter_extended_function_mode();

    lpc_reg_write(0x07, 8); // Select device 8
    int bit_map = lpc_reg_read(0xE1);

    if (level)
        bit_map |= (1 << pin);
    else
        bit_map &= ~(1 << pin);

    lpc_reg_write(0xE1, bit_map);
    leave_extended_function_mode();
}

void set_pwm_level(int pin, int pwm_value, int duration_ms) {
    // Simulate PWM by toggling the pin for the specified duration
    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current);
        long elapsed_ms = (current.tv_sec - start.tv_sec) * 1000 +
                          (current.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= duration_ms)
            break;

        // Simulate duty cycle
        set_pin_level(pin, 1);
        usleep(pwm_value * 10); // ON time proportional to PWM value
        set_pin_level(pin, 0);
        usleep((255 - pwm_value) * 10); // OFF time proportional to remaining cycle
    }
}

void control_robot_forward(int pwm_value, int duration_sec) {
    printf("Setting Robot to FORWARD with PWM = %d for %d seconds...\n", pwm_value, duration_sec);

    // Right motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(RIGHT_INA_PIN, 0);
    set_pin_level(RIGHT_INB_PIN, 1);

    // Left motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    int duration_ms = duration_sec * 1000;
    for (int i = 0; i < duration_ms / 20; i++) { // PWM cycle period = 20ms
        set_pwm_level(RIGHT_PWM_PIN, pwm_value, 20);
        set_pwm_level(LEFT_PWM_PIN, pwm_value, 20);
    }
}

void control_robot_left(int pwm_value, int duration_sec) {
    printf("Setting Robot to LEFT TURN with PWM = %d for %d seconds...\n", pwm_value, duration_sec);

    // Right motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(RIGHT_INA_PIN, 0);
    set_pin_level(RIGHT_INB_PIN, 1);

    // Left motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(LEFT_INA_PIN, 1);
    set_pin_level(LEFT_INB_PIN, 0);

    int duration_ms = duration_sec * 1000;
    for (int i = 0; i < duration_ms / 20; i++) { // PWM cycle period = 20ms
        set_pwm_level(RIGHT_PWM_PIN, pwm_value, 20);
        set_pwm_level(LEFT_PWM_PIN, pwm_value, 20);
    }
}

void control_robot_right(int pwm_value, int duration_sec) {
    printf("Setting Robot to RIGHT TURN with PWM = %d for %d seconds...\n", pwm_value, duration_sec);

    // Right motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(RIGHT_INA_PIN, 0);
    set_pin_level(RIGHT_INB_PIN, 0);

    // Left motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    int duration_ms = duration_sec * 1000;
    for (int i = 0; i < duration_ms / 20; i++) { // PWM cycle period = 20ms
        set_pwm_level(RIGHT_PWM_PIN, pwm_value, 20);
        set_pwm_level(LEFT_PWM_PIN, pwm_value, 20);
    }
}

void control_robot_backward(int pwm_value, int duration_sec) {
    printf("Setting Robot to BACKWARD with PWM = %d for %d seconds...\n", pwm_value, duration_sec);

    // Right motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(RIGHT_INA_PIN, 1);
    set_pin_level(RIGHT_INB_PIN, 0);

    // Left motor: INA = Low, INB = High, PWM = pwm_value
    set_pin_level(LEFT_INA_PIN, 1);
    set_pin_level(LEFT_INB_PIN, 0);

    int duration_ms = duration_sec * 1000;
    for (int i = 0; i < duration_ms / 20; i++) { // PWM cycle period = 20ms
        set_pwm_level(RIGHT_PWM_PIN, pwm_value, 20);
        set_pwm_level(LEFT_PWM_PIN, pwm_value, 20);
    }
}

void control_robot_stop(int duration_sec) {
    printf("Stopping Robot for %d seconds...\n", duration_sec);
    // Stop both motors by disabling PWM
    set_pin_level(RIGHT_PWM_PIN, 0);
    set_pin_level(LEFT_PWM_PIN, 0);
    sleep(duration_sec);
}

int main() {
    // Request I/O permissions
    if (iopl(3)) { // Request I/O privilege level 3
        perror("Failed to gain I/O privilege level");
        exit(1);
    }

    // while (1) {
	// int speed = 255;
    control_robot_forward(55, 2); // PWM = 50 for 10 seconds
    control_robot_stop(1);        // Stop for 5 seconds

        // control_robot_forward(100, 5); // PWM = 50 for 10 seconds
        // control_robot_stop(5);    
// 
        // control_robot_forward(150, 5); // PWM = 50 for 10 seconds
        // control_robot_stop(5);    

        // control_robot_forward(200, 5); // PWM = 50 for 10 seconds
        // control_robot_stop(5);    

        // control_robot_forward(250, 5); // PWM = 50 for 10 seconds
        // control_robot_stop(5);    
        // control_robot_left(100, 5);
        // control_robot_stop(5);        // Stop for 5 seconds
	
    control_robot_left(45, 1.8);
    control_robot_stop(1);        // Stop for 5 seconds

        // control_robot_backward(speed, 5);
        // control_robot_stop(5);        // Stop for 5 seconds

	// speed = 200;
	control_robot_forward(45, 1.5);
	// control_robot_stop(5);

    // }

    // Release I/O permissions
    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}


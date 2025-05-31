#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>
#include <errno.h>

// === GPIO Setup ===
#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

#define PIN_GPO0 0 // INA
#define PIN_GPO1 1 // INB
#define PIN_GPO2 2 // PWM

#define PIN_GPI3 11 // Encoder A
#define PIN_GPI4 12 // Encoder B

#define SAMPLE_INTERVAL_MS 100
#define MAX_PWM 255

// PID constants
#define KP 2.0
#define KI 0.5
#define KD 0.2

#define MAGIC_KEY 0x36D4EBAC

typedef struct {
    int capability;
    int mode;
} gpio_pin;

typedef struct {
    int signature;
    gpio_pin pins[16];
} gpio_controller;

gpio_controller* gpio_initialize();
int gpio_pin_set_level(gpio_controller *gc, int pin, int level);
int gpio_pin_get_level(gpio_controller *gc, int pin, int *level);

// === Encoder logic ===
int step_count = 0;

void monitor_encoder_step_once(gpio_controller* gpio, int* prev_a, int* prev_b) {
    int a, b;
    gpio_pin_get_level(gpio, PIN_GPI3, &a);
    gpio_pin_get_level(gpio, PIN_GPI4, &b);
    if (a != *prev_a) {
        if (a == 1 && b == 0) step_count++;
        else if (a == 1 && b == 1) step_count--;
        else if (a == 0 && b == 1) step_count++;
        else step_count--;
    }
    *prev_a = a;
    *prev_b = b;
}

void set_motor_forward(gpio_controller* gpio) {
    gpio_pin_set_level(gpio, PIN_GPO0, 0);
    gpio_pin_set_level(gpio, PIN_GPO1, 1);
}

void simulate_pwm(gpio_controller* gpio, int pin, int pwm_value, int duration_ms) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                       (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed >= duration_ms) break;

        gpio_pin_set_level(gpio, pin, 1);
        usleep(pwm_value * 10);
        gpio_pin_set_level(gpio, pin, 0);
        usleep((255 - pwm_value) * 10);
    }
}

// === Main ===
int main() {
    gpio_controller* gpio = gpio_initialize();
    if (!gpio) {
        fprintf(stderr, "GPIO init failed.\n");
        return 1;
    }

    int target_speed = 20; // steps per 100ms
    double integral = 0, prev_error = 0;
    int pwm = 0;

    int prev_a, prev_b;
    gpio_pin_get_level(gpio, PIN_GPI3, &prev_a);
    gpio_pin_get_level(gpio, PIN_GPI4, &prev_b);

    set_motor_forward(gpio);

    while (1) {
        step_count = 0;
        long interval_us = SAMPLE_INTERVAL_MS * 1000;

        struct timespec start, now;
        clock_gettime(CLOCK_MONOTONIC, &start);

        // Sample encoder
        while (1) {
            monitor_encoder_step_once(gpio, &prev_a, &prev_b);
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsed_us = (now.tv_sec - start.tv_sec) * 1000000 +
                              (now.tv_nsec - start.tv_nsec) / 1000;
            if (elapsed_us >= interval_us) break;
            usleep(50);
        }

        // PID logic
        double error = target_speed - step_count;
        integral += error;
        double derivative = error - prev_error;
        prev_error = error;

        double output = KP * error + KI * integral + KD * derivative;
        pwm += (int)output;
        if (pwm < 0) pwm = 0;
        if (pwm > MAX_PWM) pwm = MAX_PWM;

        printf("Steps: %d | PWM: %d | Error: %.2f\n", step_count, pwm, error);

        simulate_pwm(gpio, PIN_GPO2, pwm, SAMPLE_INTERVAL_MS);
    }

    return 0;
}

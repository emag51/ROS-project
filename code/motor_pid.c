// sudo gcc motor_pid_aonly.c -o motor_pid_aonly -O2 -lrt
// sudo ./motor_pid_aonly

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/io.h>
#include <errno.h>
#include <math.h>

// Dell LPC port
#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F
#define MAGIC_KEY 0x36D4EBAC

// Pin mapping (adjusted per your request)
#define LEFT_INA_PIN 0   // GPO0
#define LEFT_INB_PIN 1   // GPO1
#define LEFT_PWM_PIN 2   // GPO2
#define ENCODER_A_PIN 0  // GPI0

#define ENCODER_RESOLUTION 4096  // 1024 PPR × 4

// PID parameters (initial tuning)
#define KP 1.2
#define KI 0.05
#define KD 0.0

typedef struct {
    int signature;
} gpio_controller;

gpio_controller *gc = NULL;

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

int gpio_init() {
    gc = malloc(sizeof(*gc));
    if (!gc) return -1;
    if (ioperm(LPC_ADDR_PORT, 2, 1)) {
        perror("ioperm");
        free(gc);
        return -1;
    }
    setuid(getuid());
    gc->signature = MAGIC_KEY;
    return 0;
}

void gpio_cleanup() {
    if (!gc) return;
    ioperm(LPC_ADDR_PORT, 2, 0);
    free(gc);
}

void set_pin_level(int pin, int level) {
    enter_extended_function_mode();
    lpc_reg_write(0x07, 8); // Select GPO device
    int bitmap = lpc_reg_read(0xE1);
    if (level) bitmap |= (1 << pin);
    else bitmap &= ~(1 << pin);
    lpc_reg_write(0xE1, bitmap);
    leave_extended_function_mode();
}

int read_gpio_pin(int pin) {
    enter_extended_function_mode();
    lpc_reg_write(0x07, 7);  // Select GPI device
    int map = lpc_reg_read(0xF5);
    leave_extended_function_mode();
    return (map & (1 << pin)) ? 1 : 0;
}

void set_pwm(int pin, int pwm_value, int duration_ms) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= duration_ms) break;

        set_pin_level(pin, 1);
        usleep(pwm_value * 10);
        set_pin_level(pin, 0);
        usleep((255 - pwm_value) * 10);
    }
}

// Detect rising edge on A only
void update_step_count_signal_a_only(int *prev_a, int *step_count) {
    int a = read_gpio_pin(ENCODER_A_PIN);
    if (a == 1 && *prev_a == 0) {
        (*step_count)++;
    }
    *prev_a = a;
}

double get_rpm(int steps, double interval_sec) {
    return (steps * 60.0) / (ENCODER_RESOLUTION * interval_sec);
}

int main() {
    if (gpio_init() != 0) {
        fprintf(stderr, "GPIO init failed\n");
        return 1;
    }

    printf("Target: 70 RPM (Signal A Only)\n");

    // Motor direction: FORWARD
    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    int pwm = 0;
    double integral = 0, last_error = 0;
    int prev_a = read_gpio_pin(ENCODER_A_PIN);

    while (1) {
        int step_count = 0;
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        for (int t = 0; t < 100000; t += 50) {
            update_step_count_signal_a_only(&prev_a, &step_count);
            usleep(50);
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        double elapsed = (end.tv_sec - start.tv_sec) +
                         (end.tv_nsec - start.tv_nsec) / 1e9;

        double rpm = get_rpm(step_count, elapsed);
        double error = 70.0 - rpm;
        integral += error * elapsed;
        double derivative = (error - last_error) / elapsed;
        last_error = error;

        double control = KP * error + KI * integral + KD * derivative;
        pwm += (int)control;
        if (pwm < 0) pwm = 0;
        if (pwm > 255) pwm = 255;

        printf("RPM: %.2f, PWM: %d, Error: %.2f\n", rpm, pwm, error);

        set_pwm(LEFT_PWM_PIN, pwm, 100); // 100 ms window
    }

    gpio_cleanup();
    return 0;
}

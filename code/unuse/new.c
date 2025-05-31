#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include <fcntl.h>
#include <string.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

// === PIN DEFINITIONS ===
#define RIGHT_INA_PIN 0
#define RIGHT_INB_PIN 1
#define RIGHT_PWM_PIN 2
#define RIGHT_ENCODER_PIN 8 // GPI0

#define ENCODER_PPR 1024.0
#define PWM_MAX 255

// === GLOBALS ===
volatile int encoder_count = 0;
volatile int last_encoder_count = 0;
volatile double speed_steps_per_sec = 0.0;

pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

// PID variables
double kp = 1.5, ki = 0.5, kd = 0.2;
double integral = 0, last_error = 0;

// === GPIO HELPERS ===
void gpio_export(int gpio) {
    FILE *f = fopen("/sys/class/gpio/export", "w");
    if (f) {
        fprintf(f, "%d", gpio);
        fclose(f);
        usleep(100000);
    }
}

void gpio_set_direction(int gpio, const char* direction) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "%s", direction);
        fclose(f);
    }
}

void gpio_write(int gpio, int value) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "%d", value);
        fclose(f);
    }
}

int gpio_read(int gpio) {
    char path[64];
    char value_str[3];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    FILE *f = fopen(path, "r");
    if (f) {
        fgets(value_str, sizeof(value_str), f);
        fclose(f);
        return atoi(value_str);
    }
    return -1;
}

void gpio_set_level(int pin, int value) {
    gpio_write(pin, value);
}

void pwm_write(int pin, int duty) {
    static int last_pwm = -1;
    if (duty != last_pwm) {
        last_pwm = duty;
        printf("Set PWM pin %d to %d\n", pin, duty);
    }
    gpio_write(pin, duty > 128 ? 1 : 0); // crude PWM simulation
}

// === ENCODER PULSE DETECTION ===
void simulate_encoder_pulse() {
    static int last = 0;
    int current = gpio_read(RIGHT_ENCODER_PIN);

    if (current == 1 && last == 0) {
        pthread_mutex_lock(&lock);
        encoder_count++;
        pthread_mutex_unlock(&lock);
    }

    last = current;
}

// === TIME HELPER ===
double time_diff_ms(struct timeval t1, struct timeval t2) {
    return (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_usec - t1.tv_usec) / 1000.0;
}

// === SPEED MONITOR THREAD ===
void* update_speed(void* arg) {
    struct timeval prev, now;
    gettimeofday(&prev, NULL);

    while (1) {
        usleep(100000);
        gettimeofday(&now, NULL);
        double dt = time_diff_ms(prev, now) / 1000.0;
        prev = now;

        pthread_mutex_lock(&lock);
        int delta = encoder_count - last_encoder_count;
        last_encoder_count = encoder_count;
        pthread_mutex_unlock(&lock);

        speed_steps_per_sec = delta / dt;
        double rpm = (speed_steps_per_sec / ENCODER_PPR) * 60.0;

        printf("Speed: %.2f steps/sec | RPM: %.2f\n", speed_steps_per_sec, rpm);
    }
}

// === CLAMP FUNCTION ===
int clamp(int val, int min, int max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// === PID CONTROL THREAD ===
void* pid_control(void* arg) {
    double target_rpm = 130.0;
    double target_speed = (target_rpm / 60.0) * ENCODER_PPR;

    while (1) {
        usleep(100000);

        pthread_mutex_lock(&lock);
        double current_speed = speed_steps_per_sec;
        pthread_mutex_unlock(&lock);

        double error = target_speed - current_speed;
        integral += error;
        double derivative = error - last_error;
        last_error = error;

        double output = kp * error + ki * integral + kd * derivative;
        int pwm_value = clamp((int)output, 0, PWM_MAX);

        gpio_set_level(RIGHT_INA_PIN, 1);
        gpio_set_level(RIGHT_INB_PIN, 0);
        pwm_write(RIGHT_PWM_PIN, pwm_value);
    }
}

// === MAIN FUNCTION ===
int main() {
    // Export and set direction
    gpio_export(RIGHT_INA_PIN);
    gpio_export(RIGHT_INB_PIN);
    gpio_export(RIGHT_PWM_PIN);
    gpio_export(RIGHT_ENCODER_PIN);

    gpio_set_direction(RIGHT_INA_PIN, "out");
    gpio_set_direction(RIGHT_INB_PIN, "out");
    gpio_set_direction(RIGHT_PWM_PIN, "out");
    gpio_set_direction(RIGHT_ENCODER_PIN, "in");

    pthread_t speed_thread, control_thread;
    pthread_create(&speed_thread, NULL, update_speed, NULL);
    pthread_create(&control_thread, NULL, pid_control, NULL);

    while (1) {
        simulate_encoder_pulse();
        usleep(10000); // simulate encoder polling rate
    }

    pthread_join(speed_thread, NULL);
    pthread_join(control_thread, NULL);
    return 0;
}

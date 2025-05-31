#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>

// === PIN DEFINITIONS ===
// Motor control pins mapped to GPIO bits
#define RIGHT_INA_PIN 0
#define RIGHT_INB_PIN 1
#define RIGHT_PWM_PIN 2
#define LEFT_INA_PIN 3
#define LEFT_INB_PIN 4
#define LEFT_PWM_PIN 5

// Encoder pins mapped to GPIO bits
#define RIGHT_A_PIN 3   // GPIO36
#define LEFT_A_PIN 0    // GPIO39

// Only using A signal for encoder counting

// === CONSTANTS ===
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

// === MOCK GPIO FUNCTIONS (replace with LPC I/O functions) ===
void gpio_set_level(int pin, int value) {
    printf("GPIO %d = %d\n", pin, value);
}

void pwm_write(int pin, int duty) {
    printf("PWM on pin %d = %d\n", pin, duty);
}

// === ENCODER INTERRUPT SIMULATION ===
void simulate_encoder_pulse() {
    pthread_mutex_lock(&lock);
    encoder_count++;
    pthread_mutex_unlock(&lock);
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
        usleep(100000); // 100ms

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
        usleep(100000); // 100 ms

        pthread_mutex_lock(&lock);
        double current_speed = speed_steps_per_sec;
        pthread_mutex_unlock(&lock);

        double error = target_speed - current_speed;
        integral += error;
        double derivative = error - last_error;
        last_error = error;

        double output = kp * error + ki * integral + kd * derivative;
        int pwm_value = clamp((int)output, 0, PWM_MAX);

        // Set direction forward
        gpio_set_level(RIGHT_INA_PIN, 1);
        gpio_set_level(RIGHT_INB_PIN, 0);
        pwm_write(RIGHT_PWM_PIN, pwm_value);
    }
}

// === MAIN FUNCTION ===
int main() {
    pthread_t speed_thread, control_thread;

    pthread_create(&speed_thread, NULL, update_speed, NULL);
    pthread_create(&control_thread, NULL, pid_control, NULL);

    // Simulate encoder pulses (in real case, use interrupt on pin 36)
    while (1) {
        simulate_encoder_pulse();
        usleep(10000); // Simulate pulse every 10ms
    }

    pthread_join(speed_thread, NULL);
    pthread_join(control_thread, NULL);
    return 0;
}

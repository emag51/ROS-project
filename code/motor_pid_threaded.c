// sudo gcc motor_pid_threaded.c -o motor_pid_threaded -O2 -lrt -lpthread
// sudo ./motor_pid_threaded
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

#define LEFT_INA_PIN 0   // GPO0
#define LEFT_INB_PIN 1   // GPO1
#define LEFT_PWM_PIN 2   // GPO2
#define ENCODER_A_PIN 0  // GPI0

#define ENCODER_RESOLUTION 4096  // 1024 PPR * 4

// PID parameters
#define KP 1.2
#define KI 0.05
#define KD 0.0

volatile int pwm_duty = 0;
volatile int running = 1;
volatile int global_step_count = 0;

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
    int value = lpc_reg_read(0xF5);
    leave_extended_function_mode();
    return (value & (1 << pin)) ? 1 : 0;
}
// void set_pin_level(int pin, int level) {
//     enter_extended_function_mode();
//     lpc_reg_write(0x07, 8);
//     int bitmap = lpc_reg_read(0xE1);
//     if (level)
//         bitmap |= (1 << pin);
//     else
//         bitmap &= ~(1 << pin);
//     lpc_reg_write(0xE1, bitmap);
//     leave_extended_function_mode();
// }
void set_pin_level(int pin, int level) {
    enter_extended_function_mode();
    lpc_reg_write(0x07, 8); // Select device 8 (GPO)

    // Re-force direction: clear bit = output
    int dir = lpc_reg_read(0xE0);
    dir &= ~(1 << pin);             // Set this pin as output
    lpc_reg_write(0xE0, dir);       // Apply direction again

    // Set output value
    int val = lpc_reg_read(0xE1);
    if (level)
        val |= (1 << pin);
    else
        val &= ~(1 << pin);
    lpc_reg_write(0xE1, val);

    leave_extended_function_mode();
}

void *pwm_thread(void *arg) {
    const int period_us = 50000; // 50 Hz
    int count = 0;

    while (running) {
        // ✅ Get fresh duty cycle from main thread
        int duty = pwm_duty;

        int on_time = (duty * period_us) / 255;
        int off_time = period_us - on_time;

        set_pin_level(LEFT_PWM_PIN, 1);
        usleep(on_time);
        set_pin_level(LEFT_PWM_PIN, 0);
        usleep(off_time);

        if (++count % 100 == 0) {
            printf("[PWM] running... duty=%d\n", duty);
            enter_extended_function_mode();
            lpc_reg_write(0x07, 8);
            int val = lpc_reg_read(0xE1);
            leave_extended_function_mode();
            printf("[GPO2] Output register 0xE1 = 0x%02X\n", val);
        }
    }

    set_pin_level(LEFT_PWM_PIN, 0);
    return NULL;
}

// void *pwm_thread(void *arg) {
//     // const int period_us = 10000;
//     const int period_us = 20000;  // 20ms = 50 Hz

//     int duty = 180;
//     int count = 0;

//     while (running) {
//         int on_time = (duty * period_us) / 255;
//         int off_time = period_us - on_time;

//         set_pin_level(LEFT_PWM_PIN, 1);
//         usleep(on_time);
//         set_pin_level(LEFT_PWM_PIN, 0);
//         usleep(off_time);

//         if (++count % 100 == 0) {
//             printf("[PWM] running... duty=%d\n", duty);

//             // 🔍 DEBUG: Check GPO2 (0xE1 bit 2)
//             enter_extended_function_mode();
//             lpc_reg_write(0x07, 8);         // Select device 8 (GPO)
//             int val = lpc_reg_read(0xE1);   // Read GPO output register
//             leave_extended_function_mode();
//             printf("[GPO2] Output register 0xE1 = 0x%02X\n", val);
//         }
//     }

//     set_pin_level(LEFT_PWM_PIN, 0);
//     return NULL;
// }




// void *pwm_thread(void *arg) {
//     while (running) {
//         int on_time = pwm_duty * 10;
//         int off_time = (255 - pwm_duty) * 10;
//         if (on_time > 0) {
//             set_pin_level(LEFT_PWM_PIN, 1);
//             usleep(on_time);
//         }
//         if (off_time > 0) {
//             set_pin_level(LEFT_PWM_PIN, 0);
//             usleep(off_time);
//         }
//     }
//     // Ensure pin is low when stopping
//     set_pin_level(LEFT_PWM_PIN, 0);
//     return NULL;
// }

// void update_step_count_signal_a_only(int *prev_a, int *step_count) {
//     int a = read_gpio_pin(ENCODER_A_PIN);
//     if (a == 1 && *prev_a == 0) {
//         (*step_count)++;
//     }
//     *prev_a = a;
// }
void *encoder_thread(void *arg) {
    int prev_a = read_gpio_pin(ENCODER_A_PIN);
    int debug_count = 0;

    while (running) {
        int a = read_gpio_pin(ENCODER_A_PIN);
        if (a != prev_a) {
            __sync_fetch_and_add(&global_step_count, 1);
            usleep(20);  // debounce delay after edge
        }
        
        prev_a = a;

        if (++debug_count % 1000 == 0) {
            printf("[ENCODER] Still running. Last A: %d\n", a);
        }

        usleep(10);  // Faster polling: 10 µs
    }

    return NULL;
}

void update_step_count_signal_a_only(int *prev_a, int *step_count) {
    int a = read_gpio_pin(ENCODER_A_PIN);
    if (a != *prev_a) {
        (*step_count)++;
    }
    *prev_a = a;
}

double get_rpm(int steps, double interval_sec) {
    return (steps * 60.0) / (ENCODER_RESOLUTION * interval_sec);
}

int main() {
    if (ioperm(LPC_ADDR_PORT, 2, 1)) {
        perror("ioperm");
        return 1;
    }
    setuid(getuid());

    // Setup motor direction (forward)
    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    pthread_t pwm_tid, encoder_tid;

    // Start PWM thread
    if (pthread_create(&pwm_tid, NULL, pwm_thread, NULL) != 0) {
        perror("pthread_create (PWM)");
        return 1;
    }

    // Start Encoder thread
    if (pthread_create(&encoder_tid, NULL, encoder_thread, NULL) != 0) {
        perror("pthread_create (Encoder)");
        return 1;
    }

    // PID control variables
    double integral = 0, last_error = 0;

    printf("Target: 70 RPM (Multithreaded, Signal A only)\n");

    while (1) {
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        usleep(500000);  // Sample duration: 0.5 seconds

        clock_gettime(CLOCK_MONOTONIC, &end);

        int step_count = __sync_lock_test_and_set(&global_step_count, 0);


        double elapsed = (end.tv_sec - start.tv_sec) +
                         (end.tv_nsec - start.tv_nsec) / 1e9;

        double rpm = get_rpm(step_count, elapsed);
        double error = 70.0 - rpm;
        integral += error * elapsed;
        double derivative = (error - last_error) / elapsed;
        last_error = error;

        double control = KP * error + KI * integral + KD * derivative;
        int new_pwm = pwm_duty + (int)control;

        if (new_pwm < 0) new_pwm = 0;
        if (new_pwm > 255) new_pwm = 255;

        if (new_pwm < 60) new_pwm = 60;
        pwm_duty = new_pwm;

        printf("Step Count: %d\n", step_count);
        printf("RPM: %.2f, PWM: %d, Error: %.2f\n", rpm, pwm_duty, error);
    }

    // Cleanup (won't be reached unless you break loop)
    running = 0;
    pthread_join(pwm_tid, NULL);
    pthread_join(encoder_tid, NULL);
    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

#define GPIO_A_PIN 0  // Encoder A signal
#define GPIO_B_PIN 1  // Encoder B signal

int step_count = 0; // Step counter

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

int main() {
    if (iopl(3)) {
        perror("Failed to gain I/O privileges");
        exit(1);
    }

    int prev_a_state = read_gpio_pin(GPIO_A_PIN);
    int prev_b_state = read_gpio_pin(GPIO_B_PIN);

    printf("Starting encoder monitoring without Z signal...");

    while (1) {
        int a_state = read_gpio_pin(GPIO_A_PIN);
        int b_state = read_gpio_pin(GPIO_B_PIN);

        if (a_state != prev_a_state || b_state != prev_b_state) {
            if (prev_a_state == 0 && a_state == 1) {
                if (b_state == 0) {
                    step_count++;
                    printf("Motor moving Clockwise, Steps: %d\n", step_count);
                } else {
                    step_count--;
                    printf("Motor moving Counterclockwise, Steps: %d\n", step_count);
                }
            }
            prev_a_state = a_state;
            prev_b_state = b_state;
        }

        usleep(50);
    }
    return 0;
}


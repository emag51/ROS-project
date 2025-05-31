// sudo gcc encoder_test_ab.c -o encoder_test_ab -O2
// sudo ./encoder_test_ab

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

#define ENCODER_A_PIN 0  // GPI0
#define ENCODER_B_PIN 1  // GPI1

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
    lpc_reg_write(0x07, 7);  // Select GPI device
    int value = lpc_reg_read(0xF5);
    leave_extended_function_mode();
    return (value & (1 << pin)) ? 1 : 0;
}

int main() {
    if (ioperm(LPC_ADDR_PORT, 2, 1)) {
        perror("ioperm failed");
        return 1;
    }
    setuid(getuid());

    int step_a = 0, step_b = 0;
    int prev_a = read_gpio_pin(ENCODER_A_PIN);
    int prev_b = read_gpio_pin(ENCODER_B_PIN);

    printf("Monitoring encoder A (GPI0) and B (GPI1) signals...\n");

    while (1) {
        int a = read_gpio_pin(ENCODER_A_PIN);
        int b = read_gpio_pin(ENCODER_B_PIN);

        if (a == 1 && prev_a == 0) {
            step_a++;
            printf("[A ↑] Steps A = %d\n", step_a);
        }
        if (b == 1 && prev_b == 0) {
            step_b++;
            printf("[B ↑] Steps B = %d\n", step_b);
        }

        prev_a = a;
        prev_b = b;
        usleep(100); // 100 microseconds
    }

    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}

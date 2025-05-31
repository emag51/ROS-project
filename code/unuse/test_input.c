#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

#define GPI0_PIN 2  // Input Pin 0 (GPI0)

// Function to enter Extended Function Mode
void enter_extended_function_mode() {
    outb(0x87, LPC_ADDR_PORT);
    outb(0x87, LPC_ADDR_PORT);
}

// Function to leave Extended Function Mode
void leave_extended_function_mode() {
    outb(0xAA, LPC_ADDR_PORT);
}

// Function to write to an LPC register
void lpc_reg_write(int reg, int val) {
    outb(reg, LPC_ADDR_PORT);
    outb(val, LPC_DATA_PORT);
}

// Function to read from an LPC register
int lpc_reg_read(int reg) {
    outb(reg, LPC_ADDR_PORT);
    return inb(LPC_DATA_PORT);
}

// Function to read the state of GPI0
int read_gpi0() {
    int bit_map;

    enter_extended_function_mode();

    // Select virtual device 7 for input pins
    lpc_reg_write(0x07, 7);

    // Read levels from register 0xF5 (Input pins)
    bit_map = lpc_reg_read(0xF5);

    leave_extended_function_mode();

    // Check the level of GPI0 (bit 0)
    return (bit_map & (1 << GPI0_PIN)) ? 1 : 0;
}

int main() {
    // Request I/O privilege level
    if (iopl(3)) {  // Use iopl(3) for full I/O privilege
        perror("Failed to gain I/O privileges");
        exit(1);
    }

    while (1) {
        int gpi0_state = read_gpi0();

        if (gpi0_state) {
            printf("GPI0 is HIGH\n");
        } else {
            printf("GPI0 is LOW\n");
        }

        sleep(1);  // Check every second
    }

    return 0;
}


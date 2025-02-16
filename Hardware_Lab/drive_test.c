#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <errno.h>

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

// Right motor pin
#define INA_PIN 0     // GPO0
#define INB_PIN 1     // GPO1
#define PWM_PIN 2     // GPO2

// Left motor pin
//#define INA_PIN 3     // GPO3
//#define INB_PIN 4     // GPO4
//#define PWM_PIN 5     // GPO5

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

void control_robot_forward() {
    printf("Setting Robot to FORWARD...\n");
    set_pin_level(INA_PIN, 0);  // INA = High
    set_pin_level(INB_PIN, 1);  // INB = Low
    set_pin_level(PWM_PIN, 1);  // PWM = High (Enable)
}

void control_robot_stop() {
    printf("Stopping Robot...\n");
    set_pin_level(PWM_PIN, 0);  // PWM = Low (Disable)
}

int main() {
    // Request I/O permissions
    if (iopl(3)) { // Request I/O privilege level 3
	perror("Failed to gain I/O privilege level");
	exit(1);
    }

    while (1) {
        control_robot_forward();
        sleep(2);  // Move forward for 2 seconds

        control_robot_stop();
        sleep(2);  // Stop for 2 seconds
    }

    // Release I/O permissions
    ioperm(LPC_ADDR_PORT, 2, 0);
    return 0;
}

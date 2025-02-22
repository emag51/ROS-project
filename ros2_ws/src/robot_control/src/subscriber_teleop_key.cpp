#include <functional>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <sys/io.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define LPC_ADDR_PORT 0x2E
#define LPC_DATA_PORT 0x2F

#define RIGHT_INA_PIN 0
#define RIGHT_INB_PIN 1
#define RIGHT_PWM_PIN 2
#define LEFT_INA_PIN 3
#define LEFT_INB_PIN 4
#define LEFT_PWM_PIN 5

#define RIGHT_A_PIN 0
#define RIGHT_B_PIN 1
#define LEFT_A_PIN 3
#define LEFT_B_PIN 4

class TeleopMotorController : public rclcpp::Node
{
public:
  TeleopMotorController()
  : Node("teleop_motor_controller")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TeleopMotorController::topic_callback, this, std::placeholders::_1));
    
    if (iopl(3)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to gain I/O privilege level");
      exit(1);
    }
  }

  ~TeleopMotorController()
  {
    ioperm(LPC_ADDR_PORT, 2, 0);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  void topic_callback(const geometry_msgs::msg::Twist & msg)
  {
    double linear_x = msg.linear.x;
    double angular_z = msg.angular.z;

    int base_pwm = 100;  // Maximum PWM value
    int right_pwm, left_pwm;

    if (linear_x > 0) {  
      // Forward motion
      right_pwm = left_pwm = base_pwm * linear_x;
      control_robot(right_pwm, left_pwm, 1);
    } 
    else if (linear_x < 0) {  
      // Backward motion
      right_pwm = left_pwm = base_pwm * -linear_x;
      set_pin_level(RIGHT_INA_PIN, 1);
      set_pin_level(RIGHT_INB_PIN, 0);
      set_pin_level(LEFT_INA_PIN, 1);
      set_pin_level(LEFT_INB_PIN, 0);
      control_robot(right_pwm, left_pwm, 1);
    } 
    else if (angular_z > 0) {  
      // Turn left
      turn_left(base_pwm, 1);
    } 
    else if (angular_z < 0) {  
      // Turn right
      turn_right(base_pwm, 1);
    } 
    else {  
      // Stop
      set_pin_level(RIGHT_PWM_PIN, 0);
      set_pin_level(LEFT_PWM_PIN, 0);
    }
  }

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
    int bit_map = lpc_reg_read(0xF5);
    leave_extended_function_mode();
    return (bit_map & (1 << pin)) ? 1 : 0;
  }

  void set_pin_level(int pin, int level) {
    enter_extended_function_mode();
    lpc_reg_write(0x07, 8);
    int bit_map = lpc_reg_read(0xE1);
    if (level)
        bit_map |= (1 << pin);
    else
        bit_map &= ~(1 << pin);
    lpc_reg_write(0xE1, bit_map);
    leave_extended_function_mode();
  }

  void set_pwm_level(int pin, int pwm_value) {
    set_pin_level(pin, 1);
    usleep(pwm_value * 10);
    set_pin_level(pin, 0);
    usleep((255 - pwm_value) * 10);
  }

  void update_encoder() {
    static int prev_right_a = 0, prev_right_b = 0;
    static int prev_left_a = 0, prev_left_b = 0;
    static struct timespec prev_time = {0, 0};

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time = (current_time.tv_sec - prev_time.tv_sec) +
                          (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;

    int right_a = read_gpio_pin(RIGHT_A_PIN);
    int right_b = read_gpio_pin(RIGHT_B_PIN);
    int left_a = read_gpio_pin(LEFT_A_PIN);
    int left_b = read_gpio_pin(LEFT_B_PIN);

    prev_right_a = right_a;
    prev_right_b = right_b;
    prev_left_a = left_a;
    prev_left_b = left_b;
    prev_time = current_time;
  }

  void control_robot(int right_pwm, int left_pwm, int duration_sec) {
    set_pin_level(RIGHT_INA_PIN, 0);
    set_pin_level(RIGHT_INB_PIN, 1);
    set_pin_level(LEFT_INA_PIN, 0);
    set_pin_level(LEFT_INB_PIN, 1);

    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current);
        double elapsed_time = (current.tv_sec - start.tv_sec) +
                              (current.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed_time >= duration_sec) break;

        set_pwm_level(RIGHT_PWM_PIN, right_pwm);
        set_pwm_level(LEFT_PWM_PIN, left_pwm);
        update_encoder();
        usleep(10000);
    }

    set_pin_level(RIGHT_PWM_PIN, 0);
    set_pin_level(LEFT_PWM_PIN, 0);
  }

  void turn_left(int pwm_value, int duration_sec) {
    control_robot(80, 40, duration_sec);
  }

  void turn_right(int pwm_value, int duration_sec) {
    control_robot(40, 80, duration_sec);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopMotorController>());
  rclcpp::shutdown();
  return 0;
}

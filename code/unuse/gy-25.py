import rclpy
from rclpy.node import Node
import serial
import struct
from sensor_msgs.msg import Imu

class GY25IMUNode(Node):
    def __init__(self):
        super().__init__('gy25_imu_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.serial_port = serial.Serial(port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected to IMU on {port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to IMU: {e}')
            return
        
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.read_imu_data)  # 10 Hz
    
    def read_imu_data(self):
        try:
            self.serial_port.write(b'\xA5\x54\x01\xFD')  # Switch to binary mode
            # rclpy.sleep(1)  # Give it time to switch modes

            data = self.serial_port.read(11)
            self.get_logger().info(f'Received raw data: {data.hex()}')  # Print raw data

            if len(data) == 11 and data[0] == 0xAA and data[1] == 0x55:
                yaw, pitch, roll = struct.unpack('<hhh', data[2:8])
                yaw, pitch, roll = yaw / 100.0, pitch / 100.0, roll / 100.0
                self.get_logger().info(f'Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}')
        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}')


    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    imu_node = GY25IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

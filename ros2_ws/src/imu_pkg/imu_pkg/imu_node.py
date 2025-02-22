import rclpy
from rclpy.node import Node
import serial
import struct
import math
from sensor_msgs.msg import Imu

class GY25IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Connected to /dev/ttyUSB0 at 115200 baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.read_imu_data)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Convert Euler angles to quaternion (w, x, y, z) """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy
        return q_w, q_x, q_y, q_z

    def read_imu_data(self):
        if not self.ser.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        data = self.ser.readline().decode('utf-8', errors='ignore').strip()
        self.get_logger().info(f"IMU Raw Data: {data}")

        if not data:
            self.get_logger().warn("No data received from IMU.")
            return

        try:
            if '=' in data:
                # Text-based format (e.g., "Yaw=45.6 Pitch=-12.3 Roll=5.7")
                parts = data.split()
                yaw = float(parts[0].split('=')[1])
                pitch = float(parts[1].split('=')[1])
                roll = float(parts[2].split('=')[1])

            else:
                # Binary data (GY-25 default mode sends binary packets)
                data_bytes = self.ser.read(11)  # GY-25 sends 11-byte messages
                if len(data_bytes) == 11 and data_bytes[0] == 0xAA and data_bytes[1] == 0x55:
                    yaw, pitch, roll = struct.unpack('<hhh', data_bytes[2:8])
                    yaw /= 100.0
                    pitch /= 100.0
                    roll /= 100.0
                else:
                    self.get_logger().warn("Unknown data format received.")
                    return

            # Convert degrees to radians
            yaw = math.radians(yaw)
            pitch = math.radians(pitch)
            roll = math.radians(roll)

            q_w, q_x, q_y, q_z = self.euler_to_quaternion(roll, pitch, yaw)

            imu_msg = Imu()
            imu_msg.orientation.w = q_w
            imu_msg.orientation.x = q_x
            imu_msg.orientation.y = q_y
            imu_msg.orientation.z = q_z
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            self.imu_pub.publish(imu_msg)
            self.get_logger().info(f'Published IMU Data: Roll={roll}, Pitch={pitch}, Yaw={yaw}')

        except Exception as e:
            self.get_logger().error(f"Failed to parse IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GY25IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

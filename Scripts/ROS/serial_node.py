import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class GripperSerialNode(Node):
    def __init__(self):
        super().__init__('gripper_serial_node')

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Serial connected to /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f" Serial connection failed: {e}")
            exit(1)

        self.subscription = self.create_subscription(
            String,
            '/gripper_command',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f" Sending command over serial: {command}")

        if command in ["open", "close", "stop"]:
            try:
                self.ser.write((command + '\n').encode())
            except Exception as e:
                self.get_logger().error(f" Failed to write to serial: {e}")
        else:
            self.get_logger().warn(f" Unknown command received: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class GripperSerialNode(Node):
    def __init__(self):
        super().__init__('gripper_serial_node')

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info(" Serial connected to /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f" Serial connection failed: {e}")
            exit(1)

        self.subscription = self.create_subscription(
            String,
            '/gripper_command',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f" Sending command over serial: {command}")

        if command in ["open", "close", "stop"]:
            try:
                self.ser.write((command + '\n').encode())
            except Exception as e:
                self.get_logger().error(f" Failed to write to serial: {e}")
        else:
            self.get_logger().warn(f" Unknown command received: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

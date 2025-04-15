import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import serial
import serial.serialutil

class TestSub(Node):

    def __init__(self):
        super().__init__('serial_writer')
        self.subscription = self.create_subscription(
            String,
            '/drill_commands',
            self.subscriber_callback,
            10
        )
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.arduino = None

    def subscriber_callback(self, msg):
        if self.arduino is None:
            return
        self.get_logger().info("Heard " + msg.data)
        encoded_data = msg.data.encode() + b'\n'  # Add newline character
        self.arduino.write(encoded_data)
        self.arduino.write(encoded_data)
        self.arduino.write(encoded_data)
        try:
            line = self.arduino.readline().decode('utf-8').rstrip()
            self.get_logger().info(f"Received from Arduino: {line}")
        except serial.serialutil.SerialTimeoutException:
            self.get_logger().warn("Serial read timed out.")
        except UnicodeDecodeError:
            self.get_logger().warn("Could not decode serial data.")

def main(args=None):
    rclpy.init(args=args)
    testsub = TestSub()
    try:
        rclpy.spin(testsub)
    finally:
        if hasattr(testsub, 'arduino') and testsub.arduino is not None:
            testsub.arduino.close()
            testsub.get_logger().info("Serial port closed.")
        testsub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Temperature
import time
from datetime import datetime

class TemperatureController(Node):
    def __init__(self):
        super().__init__('temperature_controller')
        self.subscription = self.create_subscription(
            Temperature,
            '/temperature',
            self.temperature_callback,
            10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.last_method = ''
        self.publisher = self.create_publisher(String, '/command', qos_profile)

        self.lower_threshold = 24.0  # Adjust as needed
        self.upper_threshold = 25.0  # Adjust as needed
        self.last_publish_time = 0
        self.publish_interval = 30  # seconds

        self.get_logger().info('TemperatureController node initialized')

    def temperature_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_publish_time < self.publish_interval:
            return  # Skip if within the interval

        # Select appropriate action
        temperature = msg.temperature
        if temperature < self.lower_threshold:
            command = "on"
        elif temperature > self.upper_threshold:
            command = "off"
        else:
            # Do nothing if within the threshold range
            return

        # Do nothing if command is same as last time
        if command == self.last_method:
            return
        self.last_method = command

        # Publish message
        command_msg = String()
        command_msg.data = command
        self.publisher.publish(command_msg)
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Temperature: {temperature:.2f}°C -> Command: {command}')

        self.last_publish_time = current_time


def main(args=None):
    rclpy.init(args=args)

    TC = TemperatureController()
    rclpy.spin(TC)

    TC.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

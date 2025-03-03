import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Float32
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

        self.sleep_range_24hr = [18,6] #6pm-6am

        self.lower_threshold = 24.0  # Adjust as needed
        self.upper_threshold = 25.0  # Adjust as needed
        self.get_logger().info(f'TemperatureController lower_threshold set at {self.lower_threshold}')
        self.get_logger().info(f'TemperatureController upper_threshold set at {self.upper_threshold}')
        self.lower_sub = self.create_subscription(Float32, 'lower_threshold', self.setmin_cb, 10)
        self.upper_sub = self.create_subscription(Float32, 'upper_threshold', self.setmax_cb, 10)

        self.last_publish_time = 0
        self.publish_interval = 30  # seconds
        self.repeat_message_override_timeout = self.publish_interval * 10

        self.get_logger().info('TemperatureController node initialized')

    def setmin_cb(self, msg):
        self.get_logger().info(f'TemperatureController lower_threshold moved from {self.lower_threshold} to {msg.data}')
        self.lower_threshold = msg.data

    def setmax_cb(self, msg):
        self.get_logger().info(f'TemperatureController upper_threshold moved from {self.upper_threshold} to {msg.data}')
        self.upper_threshold = msg.data

    def temperature_callback(self, msg):
        current_time = time.time()
        time_since_publish = current_time - self.last_publish_time
        if time_since_publish < self.publish_interval:
            return  # Skip if within the interval

        # Skip at night
        now = datetime.now()
        if now.hour >= self.sleep_range_24hr[0] or now.hour < self.sleep_range_24hr[1]:
            return

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
            # Unless it has been a while
            if time_since_publish < self.repeat_message_override_timeout:
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

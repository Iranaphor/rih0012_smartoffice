import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta
from std_msgs.msg import String

class DailyCommandPublisher(Node):
    def __init__(self, schedule):
        super().__init__('daily_command_publisher')
        self.publisher_ = self.create_publisher(String, '/command', 10)
        self.schedule = schedule
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        now = datetime.now()
        current_time = now.strftime("%H:%M")

        if current_time in self.schedule:
            msg = String()
            msg.data = self.schedule[current_time]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published "{msg.data}" to /command at {now.strftime("%H:%M:%S")}')

def main(args=None):
    rclpy.init(args=args)

    schedule = {
        "06:05": "off",
        "17:55": "off"
        # Add more times and corresponding strings as needed
    }

    node = DailyCommandPublisher(schedule)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

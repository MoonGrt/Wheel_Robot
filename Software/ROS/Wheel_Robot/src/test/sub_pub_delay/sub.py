import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.time import Time

class TimestampSubscriber(Node):
    def __init__(self):
        super().__init__('timestamp_subscriber')
        self.subscription = self.create_subscription(
            Header,
            'timestamp_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        now = self.get_clock().now()
        sent_time = Time.from_msg(msg.stamp)
        latency = (now - sent_time).nanoseconds / 1e6  # ms
        self.get_logger().info(f'Message latency: {latency:.2f} ms')

def main(args=None):
    rclpy.init(args=args)
    node = TimestampSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

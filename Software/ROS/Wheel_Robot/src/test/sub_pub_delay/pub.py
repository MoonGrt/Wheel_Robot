import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class TimestampPublisher(Node):
    def __init__(self):
        super().__init__('timestamp_publisher')
        self.publisher_ = self.create_publisher(Header, 'timestamp_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Sent message with timestamp: %s' % msg.stamp)

def main(args=None):
    rclpy.init(args=args)
    node = TimestampPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

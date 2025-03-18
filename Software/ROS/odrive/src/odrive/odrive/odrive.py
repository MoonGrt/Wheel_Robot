import rclpy
from rclpy.node import Node
from serial import Serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import time

class ODriveOdometryNode(Node):
    def __init__(self):
        super().__init__('odrive_uart_odom_node')

        # 创建发布者
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 设置串口通信
        self.ser = Serial('/dev/odrive_uart', baudrate=115200, timeout=1)

        # 初始化
        self.get_logger().info('Odometry Node Started')

    def read_and_publish(self):
        # 读取串口数据
        if self.ser.in_waiting > 0:
            data = self.ser.readline()
            data_str = data.decode('utf-8').strip()
            # self.get_logger().info(f'Received: {data_str}')

            # 解析串口数据：四个浮点数
            try:
                pos0, vel0, pos1, vel1 = map(float, data_str.split())  # 格式为 "pos0 vel0 pos1 vel1"
            except ValueError:
                self.get_logger().warn('Invalid data format')
                return

            # 创建 Odometry 消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'

            # 设置位置估计（将 pos0 和 pos1 设置为位置）
            odom_msg.pose.pose.position.x = pos0  # 假设使用轴 0 的位置作为 x
            odom_msg.pose.pose.position.y = pos1  # 假设使用轴 1 的位置作为 y
            odom_msg.pose.pose.orientation.w = 1.0  # 默认四元数（这里没有方向信息）

            # 设置速度估计（将 vel0 和 vel1 设置为线性速度和角速度）
            odom_msg.twist.twist.linear.x = vel0  # 使用轴 0 的速度作为线性速度
            odom_msg.twist.twist.angular.z = vel1  # 使用轴 1 的速度作为角速度

            # 发布消息
            self.odom_pub.publish(odom_msg)
            # self.get_logger().info(f'Published Odometry: pos0={pos0}, vel0={vel0}, pos1={pos1}, vel1={vel1}')


def main(args=None):
    rclpy.init(args=args)

    odrive_node = ODriveOdometryNode()

    try:
        # 主循环：不断读取串口数据并发布
        while rclpy.ok():
            odrive_node.read_and_publish()
            rclpy.spin_once(odrive_node, timeout_sec=0.1)  # 在没有定时器的情况下, 仍需要调用spin_once来处理ROS 2消息队列
    except KeyboardInterrupt:
        pass
    finally:
        odrive_node.ser.close()  # 关闭串口
        rclpy.shutdown()


if __name__ == '__main__':
    main()

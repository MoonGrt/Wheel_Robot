import rclpy
from rclpy.node import Node
from serial import Serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time, math

class ODriveOdometryNode(Node):
    def __init__(self):
        super().__init__('odrive_test_node')

        # 创建发布者
        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 设置串口通信
        self.ser = Serial('/dev/odrive_uart', baudrate=115200, timeout=1)
        self.get_logger().info('Serial connection established')
        self.get_logger().info('Odometry Node Started')

        self.x = 0.0
        self.y = 0.0
        self.wheel_radius = 0.026  # 轮子半径 (meters)
        self.wheel_base = 0.174    # 车轮间距 (meters)

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

            # 计算机器人位置和姿态
            # 计算左右轮的线速度
            v_left = vel0 * self.wheel_radius
            v_right = vel1 * self.wheel_radius
            # 线速度和角速度
            linear_velocity = (v_left + v_right) / 2.0
            angular_velocity = (v_right - v_left) / self.wheel_base

            # 更新机器人的位置
            # 计算左右轮的线性行驶距离
            d_left = pos0 * 2 * math.pi * self.wheel_radius
            d_right = pos1 * 2 * math.pi * self.wheel_radius
            # 计算机器人行驶的距离和角度变化
            d = (d_left + d_right) / 2.0
            self.theta = (d_right - d_left) / self.wheel_base
            # 直接使用 pos0 和 pos1 计算位置
            self.x = d * math.cos(self.theta)
            self.y = d * math.sin(self.theta)


            # 发布TransformStamped消息
            odom_trans = TransformStamped()
            odom_trans.header.stamp = self.get_clock().now().to_msg()
            odom_trans.header.frame_id = self.odom_frame_id
            odom_trans.child_frame_id = self.child_frame_id
            # 设置位置和姿态
            odom_trans.transform.translation.x = self.x
            odom_trans.transform.translation.y = self.y
            odom_trans.transform.rotation.w = math.cos(self.theta / 2)
            odom_trans.transform.rotation.z = math.sin(self.theta / 2)


            # 创建Odometry消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.child_frame_id
            # 设置位置和姿态
            odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
            odom_msg.pose.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=math.sin(self.theta / 2), w=math.cos(self.theta / 2)
            )
            odom_msg.twist.twist.linear.x = linear_velocity
            odom_msg.twist.twist.angular.z = angular_velocity

            # 发布消息
            self.odom_pub.publish(odom_msg)
            # 发布 tf 变换
            self.odom_tf.sendTransform(odom_trans)
            # self.get_logger().info(f"x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}")


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

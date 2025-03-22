import rclpy, threading, math, time
from rclpy.node import Node
from serial import Serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ODriveOdometryNode(Node):
    def __init__(self, port_name, baudrate):
        super().__init__('odrive_node')

        # 创建发布者
        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        #
        self.x = 0.0
        self.y = 0.0
        self.wheel_radius = 0.026  # 轮子半径 (meters)
        self.wheel_base = 0.174    # 车轮间距 (meters)

        # 打开串口
        try:
            self.odrive = Serial(port=port_name, baudrate=baudrate, timeout=0.5)
            self.get_logger().info("\033[32mOdrive serial connection established...\033[0m")
            self.get_logger().info("\033[32mOdometry Node Started...\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            exit(0)

        # 启动 Odrive 驱动线程
        self.driver_thread = threading.Thread(target=self.driver_loop)
        self.driver_thread.start()

    def driver_loop(self):
        # 循环读取IMU数据
        while True:
            # 读取加速度计数据
            try:
                self.read_and_publish()
            except Exception as e:
                self.odrive.close()
                exit(0)

    def read_and_publish(self):
        # 读取串口数据
        if self.odrive.in_waiting > 0:
            data = self.odrive.readline()
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
    odrive_node = ODriveOdometryNode("/dev/odrive_uart", 921600)

    # 运行 ROS2 节点
    try:
        rclpy.spin(odrive_node)
    except KeyboardInterrupt:
        pass
    # 停止 ROS2 节点
    finally:
        odrive_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

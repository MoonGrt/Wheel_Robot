import rclpy, odrive, math, time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from odrive.enums import AxisState

class BalanceBot(Node):
    def __init__(self):
        super().__init__('balance_bot')

        # 初始化订阅
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 连接Odrive
        try:
            self.odrive = odrive.find_any(timeout=10)
            self.odrive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.odrive.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
        except Exception as e:
            self.get_logger().error(f"ODrive连接失败: {str(e)}")
            raise

        # 控制参数
        self.kp = 3.4
        self.ki = 0.0
        self.kd = 0.5
        self.integral = 0.0
        self.prev_error = 0.0
        self.target_linear_speed = 0.0
        self.target_angular_speed = 0.0

    def imu_callback(self, msg):
        pitch = math.atan2(msg.linear_acceleration.y, msg.linear_acceleration.z)
        error = pitch
        self.integral += error
        derivative = error - self.prev_error
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        left_speed = self.target_linear_speed - self.target_angular_speed - correction
        right_speed = self.target_linear_speed + self.target_angular_speed + correction
        self.set_motor_speeds(left_speed, right_speed)

    def cmd_vel_callback(self, msg):
        self.target_linear_speed = msg.linear.x
        self.target_angular_speed = msg.angular.z

    def set_motor_speeds(self, left_speed, right_speed):
        self.odrive.axis0.controller.input_vel = left_speed
        self.odrive.axis1.controller.input_vel = right_speed


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BalanceBot()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Node crashed: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

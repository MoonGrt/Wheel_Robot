import rclpy, math, time
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import tf_transformations as tf
from serial import Serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Point, Quaternion, TransformStamped
from odrive.enums import *

import numpy as np
import time
from scipy.linalg import solve_continuous_are
from sensor_msgs.msg import Imu  # ROS2 IMU 消息类型

class KalmanFilter:
    def __init__(self):
        self.dt = 0.005  # 采样时间
        # 状态变量 [角度, 角速度]
        self.x = np.array([[0], [0]])
        # 状态转移矩阵
        self.A = np.array([[1, -self.dt], [0, 1]])
        # 控制矩阵
        self.B = np.array([[self.dt], [1]])
        # 观测矩阵
        self.H = np.array([[1, 0]])
        # 过程噪声
        self.Q = np.array([[0.001, 0], [0, 0.003]])
        # 观测噪声
        self.R = np.array([[0.03]])
        # 误差协方差
        self.P = np.eye(2)

    def update(self, gyro, accel_angle):
        # 预测步骤
        u = np.array([[gyro]])  # 角速度输入
        x_pred = self.A @ self.x + self.B @ u
        P_pred = self.A @ self.P @ self.A.T + self.Q

        # 观测更新
        z = np.array([[accel_angle]])  # 加速度计计算的角度
        y = z - (self.H @ x_pred)  # 残差
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)  # 卡尔曼增益

        # 更新状态
        self.x = x_pred + K @ y
        self.P = (np.eye(2) - K @ self.H) @ P_pred

        return self.x[0, 0]  # 返回估计角度


class IMUProcessor:
    def __init__(self):
        self.alpha = 0.98  # 互补滤波系数
        self.roll = 0.0
        self.pitch = 0.0

    def update(self, accel, gyro, dt):
        # 加速度计计算角度（适用于低频、长期稳定）
        accel_roll = math.atan2(accel[1], accel[2])
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))

        # 陀螺仪积分计算角度（短期稳定，易漂移）
        gyro_roll = self.roll + gyro[0] * dt
        gyro_pitch = self.pitch + gyro[1] * dt

        # 互补滤波
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch


class ModorNode(Node):
    def __init__(self, port_name, baudrate):
        super().__init__('motor_node')
        self.motor = None

        # === 机器人物理参数（可配置） ===
        self.mass = 1.5                # 质量 kg
        self.wheel_radius = 0.026      # 轮子半径 m
        self.wheel_base = 0.174        # 轮距 m
        self.gravity = 9.81            # 重力加速度
        self.center_of_mass = 0.1      # 质心高度 m

        # === 推导惯性 ===
        self.inertia = self.mass * self.center_of_mass ** 2

        # === 控制目标 ===
        self.target_pitch = -0.02         # 平衡角
        self.target_linear = 0.0         # 期望线速度
        self.target_angular = 0.0        # 期望角速度

        # === 状态跟踪 ===
        self.last_time = None
        self.prev_pitch = 0.0

        # === 初始化 LQR 控制器 ===
        self.K = self.compute_lqr_gain()

        # 机器人物理参数
        self.x = 0.0
        self.y = 0.0

        self.max_speed = 25.0  # 电机最大转速 (rad/s)
        self.max_angle = 90.0  #
        self.min_angle = 0.0  #
        self.imu_processor = IMUProcessor()
        self.kf_roll = KalmanFilter()
        self.kf_pitch = KalmanFilter()

        # 连接 Motor
        try:
            self.motor = Serial(port=port_name, baudrate=baudrate, timeout=0.1)
            self.set_servo_angle(15, 15)
            self.get_logger().info("\033[32mMotor serial port connection established...\033[0m")
            self.motor_setup()
            self.get_logger().info("\033[32mMotor Setup Successed...\033[0m")
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            if self.motor:
                self.motor.close()
            exit(0)

        # 创建发布者
        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 初始化订阅
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg:Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        # self.get_logger().info(f"收到指令: 线速度={self.target_linear}m/s, 角速度={self.target_angular}rad/s")

    def compute_lqr_gain(self):
        # 状态空间模型（倒立摆线性化模型）
        A = np.array([
            [0, 1],
            [self.mass * self.gravity * self.center_of_mass / self.inertia, 0]
        ])
        B = np.array([
            [0],
            [1 / self.inertia]
        ])

        # 权重矩阵（可根据实际情况调整）
        Q = np.diag([500, 10])  # 强调姿态角度控制
        R = np.array([[10]])  # 控制能耗惩罚项

        # 求解 LQR 增益矩阵
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return K

    def imu_callback(self, msg: Imu):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.01

        # 获取传感器原始数据
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        # 滤波处理（假设你已经封装好 IMU 融合）
        roll, pitch = self.imu_processor.update(accel, gyro, dt)
        # roll = self.kf_roll.update(gyro[0], roll)
        # pitch = self.kf_pitch.update(gyro[1], pitch)
        
        # 计算角速度
        pitch_velocity = (pitch - self.prev_pitch) / dt

        # LQR 控制器输出
        print(f'pitch: {pitch:.8f}, pitch_velocity: {pitch_velocity:.8f}')
        left_speed, right_speed = self.balance_compute(pitch, pitch_velocity)
        print(f'ls: {left_speed:.4f}, rs: {right_speed:.4f}')

        # 更新状态
        self.prev_pitch = pitch
        self.last_time = current_time

        # # 安全角度限制
        # if abs(pitch - self.target_pitch) > 0.30:
        #     self.motor.write('v 0 0.0 0.0\nv 1 0.0 0.0\n'.encode())
        # else:
        #     self.set_motor_speeds(left_speed, right_speed, direction=1)

    def balance_compute(self, pitch, pitch_velocity):
        # 状态向量（单位：弧度 & 弧度/秒）
        x = np.array([[pitch], [pitch_velocity]])

        # 控制输出：u = -Kx
        correction = -self.K @ x  # scalar = correction[0, 0]

        # 基础速度映射
        linear_speed = self.target_linear / self.wheel_radius
        angular_diff = (self.target_angular * self.wheel_base) / (2 * self.wheel_radius)

        left_speed = linear_speed - angular_diff - correction[0, 0]
        right_speed = linear_speed + angular_diff + correction[0, 0]

        return left_speed, right_speed

    def motor_setup(self):
        while (True):
            # 切换到闭环控制
            self.motor_send_cmd(f'w axis0.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
            self.motor_send_cmd(f'w axis1.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
            errors = self.check_motor_error()
            if errors:
                self.handle_motor_error(errors)
                # 等待错误处理完成
                while True:
                    state0 = int(self.motor_send_cmd('r axis0.current_state'))
                    state1 = int(self.motor_send_cmd('r axis1.current_state'))
                    if state0 == AxisState.IDLE and state1 == AxisState.IDLE:
                        break
                    time.sleep(0.1)
            else:
                break

    def motor_send_cmd(self, cmd):
        self.motor.write((cmd + '\n').encode())
        return self.motor.readline().decode().strip()

    def motor_idle(self):
        self.motor_send_cmd(f'w axis0.requested_state {AxisState.IDLE}')
        self.motor_send_cmd(f'w axis1.requested_state {AxisState.IDLE}')
        self.motor_send_cmd('v 0 0.0 0.0')
        self.motor_send_cmd('v 1 0.0 0.0')

    def check_motor_error(self, clear=True, full=True):
        errors = []

        # ANSI颜色代码
        RED = "\033[91m"
        YELLOW = "\033[93m"
        RESET = "\033[0m"

        def decode_flags(val, enum_type):
            val = int(val, 0)
            msgs = []
            for bit in range(64):
                flag = 1 << bit
                if val & flag:
                    try:
                        name = enum_type(flag).name
                        msgs.append(f"{enum_type.__name__}.{name}")
                    except ValueError:
                        msgs.append(f"Unknown: 0x{flag:08X}")
            return msgs

        def check_path_error(indent, name, obj, path, enum_type):
            val = self.motor_send_cmd(f"r {path}")
            if val == '':
                return
            decoded = decode_flags(val, enum_type)

            show = full or bool(decoded)
            if show:
                color = RED if decoded else RESET
                print(f"{indent}{color}{obj} = {val}{RESET}")
                for msg in decoded:
                    print(f"{indent}  {YELLOW}⮡ {msg}{RESET}")

            if decoded:
                errors.append({
                    "name": name,
                    "obj": obj,
                    "path": path,
                    "type": enum_type,
                    "raw": val,
                    "decoded": decoded
                })

        check_path_error("", "system", "system.error", "error", LegacyODriveError)

        for axis in [0, 1]:
            name = f"axis{axis}"
            print(f"{name}:")
            check_path_error("  ", name, "axis.error", f"{name}.error", AxisError)
            check_path_error("  ", name, "motor.error", f"{name}.motor.error", MotorError)
            check_path_error("  ", name, "encoder.error", f"{name}.encoder.error", EncoderError)
            check_path_error("  ", name, "controller.error", f"{name}.controller.error", ControllerError)

        if clear:
            self.motor_send_cmd("sc")

        return errors

    def handle_motor_error(self, errors):
        for err in errors:
            name = err["name"]
            obj = err["obj"]
            type = err["type"]
            raw = err["raw"]
            decoded = err["decoded"]

            # 根据错误代码执行相应的错误处理函数
            if type == AxisError:
                pass
            elif type == MotorError:
                for code in decoded:
                    # 示例：根据错误类型分类处理
                    if "DRV_FAULT" in code:
                        self.motor_send_cmd('sc')
                    elif "OVER_TEMP" in code:
                        pass
                    elif "UNKNOWN_PHASE_ESTIMATE" in code:
                        self.motor_calibration(name)
            elif type == EncoderError:
                self.motor_calibration(name)
            elif type == ControllerError:
                pass

    def motor_calibration(self, name):
        # 启动编码器偏移校准
        if name == 'system':
            return
        self.motor_send_cmd(f'w {name}.requested_state {AxisState.ENCODER_OFFSET_CALIBRATION}')

    def read_and_publish(self):
        # 解析串口数据：四个浮点数
        try:
            pos0, pos1 = map(float, self.motor_send_cmd('r p').split())
            vel0, vel1 = map(float, self.motor_send_cmd('r v').split())
            # print(f"pos0={-pos0}, vel0={-vel0}, pos1={pos1}, vel1={vel1}")
        except ValueError:
            self.get_logger().warn('Invalid data format')
            return

        # 计算机器人位置和姿态
        # 计算左右轮的线速度
        v_left = (-vel0) * self.wheel_radius
        v_right = vel1 * self.wheel_radius
        # 线速度和角速度
        linear_velocity = (v_left + v_right) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_base

        # 更新机器人的位置
        # 计算左右轮的线性行驶距离
        d_left = (-pos0) * 2 * math.pi * self.wheel_radius
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
        self.odom_tf.sendTransform(odom_trans)

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
        self.odom_pub.publish(odom_msg)

        # 发布 JointStates
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_msg.position = [pos0 * 2 * math.pi, pos1 * 2 * math.pi]  # 轮子旋转角度
        joint_msg.velocity = [vel0, vel1]  # 轮子速度
        self.joint_pub.publish(joint_msg)

        # self.get_logger().info(f"x: {self.x:.10f}, y: {self.y:.10f}, theta: {self.theta:.10f}")

    def set_motor_speeds(self, left_speed, right_speed, direction=1):
        # 限幅处理
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if direction:
            left_speed = -left_speed
            right_speed = -right_speed
        try:
            # 根据实际电机方向可能需要调整符号
            setspeed = f'v 0 {left_speed:.4f} 0.0\nv 1 {right_speed:.4f} 0.0\n'
            print(setspeed)
            self.motor.write(setspeed.encode())
        except Exception as e:
            self.get_logger().error(f"Motor control failed: {str(e)}")

    def set_servo_angle(self, left_angle, right_angle):
        # 限幅处理
        left_angle = max(min(left_angle, self.max_angle), self.min_angle)
        right_angle = max(min(right_angle, self.max_angle), self.min_angle)
        #
        servo0_angle = 90 + right_angle
        servo1_angle = 100 - right_angle
        servo2_angle = 90 - left_angle
        servo3_angle = 80 + left_angle
        try:
            # 根据实际电机方向可能需要调整符号
            self.motor_send_cmd(f'm 0 {servo0_angle} 1 {servo1_angle} 2 {servo2_angle} 3 {servo3_angle}')
        except Exception as e:
            self.get_logger().error(f"Servo control failed: {str(e)}")
        pass


def main(args=None):
    rclpy.init(args=args)
    motor_node = ModorNode("/dev/motor", 115200)

    # 运行 ROS2 节点
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    # 停止 ROS2 节点
    finally:
        motor_node.motor_idle()
        motor_node.motor.close()
        motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

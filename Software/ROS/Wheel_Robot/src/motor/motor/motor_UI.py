import rclpy, math, time, threading, sys
import numpy as np
import tf_transformations as tf
import matplotlib.pyplot as plt
from collections import deque
from serial import Serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Point, Quaternion, TransformStamped
from odrive.enums import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QSlider, QLabel, QFrame, QTabWidget
)
from PyQt5.QtCore import Qt


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
        self.dt = 0.005  # 采样周期
        self.roll = 0.0
        self.pitch = 0.0

    def update(self, accel, gyro):
        # 加速度计计算角度（适用于低频、长期稳定）
        accel_roll = math.atan2(accel[1], accel[2])
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))

        # 陀螺仪积分计算角度（短期稳定，易漂移）
        gyro_roll = self.roll + gyro[0] * self.dt
        gyro_pitch = self.pitch + gyro[1] * self.dt

        # 互补滤波
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch


class ModorNode(Node):
    def __init__(self, port_name, baudrate):
        super().__init__('motor_node')
        self.motor = None

        # 机器人物理参数
        self.x = 0.0
        self.y = 0.0

        # 上次回调时间
        self.last_time = time.time()

        # PID参数
        self.speed_kp, self.speed_ki, self.speed_kd = 0.0, 0.0, 0.0
        self.angle_kp, self.angle_ki, self.angle_kd = 20.0, 0.0, 0.1
        self.angular_kp, self.angular_ki, self.angular_kd = 0.8, 0.0, 0.0

        # 状态变量
        self.speed_integral = 0
        self.angle_integral = 0
        self.angular_integral = 0

        self.prev_speed_error = 0
        self.prev_angle_error = 0
        self.prev_angular_error = 0

        # 来自轮速传感器的速度估计
        self.prev_motor_pos0 = 0
        self.prev_motor_pos1 = 0
        self.prev_motor_vel0 = 0
        self.prev_motor_vel1 = 0
        self.motor_pos0 = 0
        self.motor_pos1 = 0
        self.motor_vel0 = 0
        self.motor_vel1 = 0
        self.motor_vel0_filter = 0
        self.motor_vel1_filter = 0

        self.gyro_pitch_rate_filter = 0

        # 目标角度
        self.target_roll = 0
        self.target_pitch = 0
        # 误差项
        self.prev_error = 0.0
        self.integral = 0.0
        # 运动参数
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.wheel_base = 0.174    # 车轮间距 (meters)
        self.wheel_radius = 0.026  # 轮子半径 (meters)

        self.pitch_max = 0.380
        self.pitch_min = -0.483
        self.pitch_mid = (self.pitch_max + self.pitch_min) / 2
        self.max_speed = 8.0  # 电机最大转速 (rad/s)
        self.max_angle = 90.0  #
        self.min_angle = 0.0  #

        self.max_samples = 1000  # calibate 时采样 1000 次
        self.pitch_sum = 0
        self.sample_count = 0
        self.calibration_complete = True

        self.motor_run = False
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
            self.get_logger().error(str(e))
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            if self.motor:
                self.motor.close()
            return

        # 创建发布者
        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 初始化订阅
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 数据记录
        self.plot_num = 500
        self.speed_error_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.speed_output_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.angle_error_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.angle_output_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.angular_error_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.angular_output_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.data1 = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.data2 = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.data3 = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.data4 = deque([0.0]*self.plot_num, maxlen=self.plot_num)

    def cmd_vel_callback(self, msg:Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        # self.get_logger().info(f"收到指令: 线速度={self.target_linear}m/s, 角速度={self.target_angular}rad/s")

    def imu_callback(self, msg:Imu):
        # # 提取四元数（ROS2的Imu消息中四元数顺序为 x,y,z,w）
        # q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # # 将四元数转换为欧拉角（Roll, Pitch, Yaw，单位为弧度）
        # # pitch: forward; roll, left-right
        # (roll, pitch, yaw) = tf.euler_from_quaternion(q)
        # print(f'Roll1: {roll:.10f}, Pitch1: {pitch:.10f}')

        # 互补滤波计算角度
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        roll, pitch = self.imu_processor.update(accel, gyro)
        # 卡尔曼滤波计算角度
        # roll = self.kf_roll.update(gyro[0], roll)
        # pitch = self.kf_pitch.update(gyro[1], pitch)
        # print(f'Pitch2: {pitch:.10f}')

        # # 卡尔曼滤波计算角度
        # accel = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        # gyro = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        # accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        # pitch = self.kf_pitch.update(gyro[1], accel_pitch)
        # print(f'Pitch3: {pitch:.10f}')

        # # Raw
        # accel = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        # pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        # print(f'Pitch4: {pitch:.10f}')

        if not self.calibration_complete:
            if self.sample_count < self.max_samples:
                if self.sample_count == 0:
                    print("校准中... 请保持平衡车在平衡状态，正在采样...")
                self.sample_count += 1
                self.pitch_sum += pitch
                print(pitch)
            else:
                self.target_pitch = (self.pitch_sum / self.max_samples)
                print(f"校准完成，目标平衡角度：{self.target_pitch:.10f}")
                self.calibration_complete = True
            return

        self.read_motor()
        # self.publish_msg()

        # left_speed, right_speed = self.balance_compute(pitch)
        left_speed, right_speed = self.cascaded_pid_control(pitch, gyro[1])
        # print(f'ls: {left_speed:.4f}, rs: {right_speed:.4f}')

        # 
        if abs(pitch - self.pitch_mid) > 0.30 or not self.motor_run:
            self.motor.write('v 0 0.0 0.0\nv 1 0.0 0.0\n'.encode())
        else:
            self.set_motor_speeds(left_speed, right_speed, direction=1)


    # def balance_compute(self, pitch):
    #     # 计算误差和变化率
    #     current_time = time.time()
    #     dt = current_time - self.last_time
    #     error = pitch - self.target_pitch
    #     self.integral += error * dt  # TODO: 积分限幅
    #     derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

    #     # PID 控制计算
    #     correction = self.balance_kp * error + self.balance_kd * derivative + self.balance_ki * self.integral
    #     print(f'error: {error:.10f}, derivative: {derivative:.10f}, integral: {self.integral:.10f}')
    #     print(f'error: {self.balance_kp * error:.10f}, derivative: {self.balance_kd * derivative:.10f}, integral: {self.balance_ki * self.integral:.10f}')

    #     # 更新状态
    #     self.prev_error = error
    #     self.last_time = current_time

    #     # 运动学转换
    #     linear_speed = self.target_linear / self.wheel_radius  # 转换为电机角速度
    #     angular_diff = (self.target_angular * self.wheel_base) / (2 * self.wheel_radius)
    #     # 计算左右轮速度
    #     left_speed = linear_speed - angular_diff - correction
    #     right_speed = linear_speed + angular_diff + correction

    #     # 记录日志
    #     self.angle_error_data.append(error)
    #     self.angle_output_data.append(correction)

    #     return left_speed, right_speed

    def cascaded_pid_control(self, pitch, gyro_pitch_rate):
        current_time = time.time()
        dt = current_time - self.last_time
        dt = max(dt, 1e-3)  # 防止除以0
        print(f'dr: {dt:.10f}')

        # left_speed = self.motor_vel0
        # right_speed = self.motor_vel1
        # self.motor_vel0_filter = 0.3 * self.motor_vel0 + 0.7 * self.motor_vel0_filter
        # self.motor_vel1_filter = 0.3 * self.motor_vel1 + 0.7 * self.motor_vel1_filter
        left_speed = (self.motor_pos0 - self.prev_motor_pos0) / dt
        right_speed = (self.motor_pos1 - self.prev_motor_pos1) / dt
        self.prev_motor_pos0 = self.motor_pos0 * 0.3 + self.prev_motor_pos0 * 0.7
        self.prev_motor_pos1 = self.motor_pos1 * 0.3 + self.prev_motor_pos1 * 0.7

        self.data1.append(self.motor_pos0)
        self.data2.append(self.motor_pos1)
        self.data3.append(left_speed)
        self.data4.append(right_speed)

        # 当前车速(机器人线速度)：左右轮速度平均 (未转换为车轮实际线速度)
        current_linear_speed = (self.motor_vel0 + self.motor_vel1) / 2.0
        current_angular_speed = (self.motor_vel1 - self.motor_vel0) / self.wheel_base

        self.gyro_pitch_rate_filter = 0.3 * gyro_pitch_rate + 0.7 * self.gyro_pitch_rate_filter

        # ========== 1. 速度环 PID ==========
        speed_error = current_linear_speed - self.target_linear  # 当前速度可由轮速传感器提供
        self.speed_integral += speed_error * dt
        speed_output = (
            self.speed_kp * speed_error +
            self.speed_ki * self.speed_integral +
            self.speed_kd * ((speed_error - self.prev_speed_error) / dt)
        )
        self.prev_speed_error = speed_error
        desired_pitch = self.target_pitch + speed_output  # 外环输出为期望角度

        # ========== 2. 角度环 PID ==========
        angle_error = pitch - desired_pitch
        self.angle_integral += angle_error * dt
        angle_output = (
            self.angle_kp * angle_error +
            self.angle_ki * self.angle_integral +
            self.angle_kd * ((angle_error - self.prev_angle_error) / dt)
        )
        self.prev_angle_error = angle_error
        desired_angular_velocity = angle_output  # 输出为期望角速度

        # ========== 3. 角速度环 PID ==========
        angular_error = self.gyro_pitch_rate_filter - desired_angular_velocity
        self.angular_integral += angular_error * dt
        angular_output = (
            self.angular_kp * angular_error +
            self.angular_ki * self.angular_integral +
            self.angular_kd * ((angular_error - self.prev_angular_error) / dt)
        )
        self.prev_angular_error = angular_error

        # ========== 电机速度合成 ==========
        linear_speed = self.target_linear  # 线速度: 未转换为车轮目标线速度
        angular_diff = (self.target_angular * self.wheel_base) / 2  # 角速度差分: 未转换为车轮目标线速度
        left_speed = (linear_speed - angular_diff) - angular_output
        right_speed = (linear_speed + angular_diff) + angular_output

        self.last_time = current_time

        # 记录日志
        self.speed_error_data.append(speed_error)
        self.speed_output_data.append(speed_output)
        self.angle_error_data.append(angle_error)
        self.angle_output_data.append(angle_output)
        self.angular_error_data.append(angular_error)
        self.angular_output_data.append(angular_output)

        return left_speed, right_speed

    # 实时绘图线程
    def live_plot(self):
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))  # 两个子图

        # 第一个子图：PID误差
        error_line, = ax1.plot(self.error_data, label='Error', color='tab:red')
        ax1.set_ylabel("Error")
        ax1.set_title("PID Error")
        ax1.legend()
        ax1.grid(True)

        # 第二个子图：左右轮速度
        left_line, = ax2.plot(self.speed_data, label='Speed', color='tab:blue')
        ax2.set_ylabel("Speed")
        ax2.set_ylim(-2, 2)  # 根据你电机输出的范围设定
        ax2.set_title("Motor Speeds")
        ax2.legend()
        ax2.grid(True)

        while True:
            error_line.set_ydata(self.error_data)
            left_line.set_ydata(self.speed_data)

            x_range = range(len(self.error_data))
            error_line.set_xdata(x_range)
            left_line.set_xdata(x_range)

            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()

            plt.draw()
            plt.pause(0.01)

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
        if self.motor:
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

    def read_motor(self):
        # 读取数据
        try:
            self.motor_pos0, self.motor_pos1 = map(float, self.motor_send_cmd('r p').split())
            self.motor_vel0, self.motor_vel1 = map(float, self.motor_send_cmd('r v').split())
            # print(f"pos0={-self.motor_pos0}, vel0={-self.motor_vel0}, pos1={self.motor_pos1}, vel1={self.motor_vel1}")
        except ValueError:
            self.get_logger().warn('Invalid data format')

    def publish_msg(self):
        # 计算机器人位置和姿态
        # 计算左右轮的线速度
        v_left = (-self.motor_vel0) * self.wheel_radius
        v_right = self.motor_vel1 * self.wheel_radius
        # 线速度和角速度
        linear_velocity = (v_left + v_right) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_base

        # 更新机器人的位置
        # 计算左右轮的线性行驶距离
        d_left = (-self.motor_pos0) * 2 * math.pi * self.wheel_radius
        d_right = self.motor_pos1 * 2 * math.pi * self.wheel_radius
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
        joint_msg.position = [-self.motor_pos0 * 2 * math.pi, self.motor_pos1 * 2 * math.pi]  # 轮子旋转角度
        joint_msg.velocity = [-self.motor_vel0, self.motor_vel1]  # 轮子速度
        self.joint_pub.publish(joint_msg)

        # self.get_logger().info(f"x: {self.x:.10f}, y: {self.y:.10f}, theta: {self.theta:.10f}")

    def set_motor_speeds(self, left_speed, right_speed, direction=0):
        # 限幅处理
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if direction:
            left_speed = -left_speed
            right_speed = -right_speed
        try:
            # 根据实际电机方向可能需要调整符号
            setspeed = f'v 0 {left_speed:.4f} 0.0\nv 1 {right_speed:.4f} 0.0\n'
            # print(setspeed)
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


class MplCanvas(FigureCanvas):
    def __init__(self, title="", y_min = -10, y_max=10):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(title)
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(y_min, y_max)
        self.line, = self.ax.plot([], [])
        super().__init__(self.fig)

    def update_plot(self, data, update_ylim=False):
        self.line.set_ydata(data)
        self.line.set_xdata(range(len(data)))
        if update_ylim:
            self.ax.set_ylim(min(data) - 1, max(data) + 1)
        self.ax.set_xlim(0, len(data))
        self.draw()


class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ModorNode Controller")
        self.resize(1200, 700)

        # ROS Node
        rclpy.init()
        self.node = ModorNode("/dev/motor", 115200)

        # 初始化ui
        self.init_ui()

        # 启动node线程
        if self.node.motor:
            self.motor_connect_button.setChecked(True)
            self.node_thread_running = True
            self.node_thread = threading.Thread(target=self.run_node, daemon=True)
            self.node_thread.start()
        # 启动绘图线程
        if self.node.motor:
            self.plot_thread_running = True
            self.plot_thread = threading.Thread(target=self.plot_data, daemon=True)
            self.plot_thread.start()

    def init_ui(self):
        layout = QVBoxLayout()

        # 创建 QTabWidget 用于标签页切换
        self.tab_widget = QTabWidget()
        self.add_plot_tab()
        self.tab_widget.setCurrentIndex(1)
        layout.addWidget(self.tab_widget)

        # 分割线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # 中部分：控制区
        control_layout = QHBoxLayout()

        # PID参数及滑动条设置
        pid_groups = [
            ("Speed", [
                ("Kp", self.node.speed_kp, 0.0, 10.0),
                ("Ki", self.node.speed_ki, 0.0, 2.0),
                ("Kd", self.node.speed_kd, 0.0, 2.0),
            ]),
            ("Angle", [
                ("Kp", self.node.angle_kp, 0.0, 50.0),
                ("Ki", self.node.angle_ki, 0.0, 5.0),
                ("Kd", self.node.angle_kd, 0.0, 5.0),
            ]),
            ("Angular", [
                ("Kp", self.node.angular_kp, 0.0, 10.0),
                ("Ki", self.node.angular_ki, 0.0, 5.0),
                ("Kd", self.node.angular_kd, 0.0, 2.0),
            ]),
        ]

        for group_name, params in pid_groups:
            title = QLabel(f"{group_name} PID")
            title.setStyleSheet("font-weight: bold; margin-top: 10px;")
            pid_layout = QVBoxLayout()
            pid_layout.addWidget(title)

            for suffix, default, min_val, max_val in params:
                name = f"{group_name}_{suffix}"
                slider_layout = QHBoxLayout()
                label = QLabel(f"{name}: {default:.2f}")
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(min_val * 100))
                slider.setMaximum(int(max_val * 100))
                slider.setValue(int(default * 100))

                def update_value(val, param_name=name, lbl=label):
                    float_val = val / 100.0
                    lbl.setText(f"{param_name}: {float_val:.2f}")
                    if param_name == "Speed_Kp":
                        self.node.speed_kp = float_val
                    elif param_name == "Speed_Ki":
                        self.node.speed_ki = float_val
                    elif param_name == "Speed_Kd":
                        self.node.speed_kd = float_val
                    elif param_name == "Angle_Kp":
                        self.node.angle_kp = float_val
                    elif param_name == "Angle_Ki":
                        self.node.angle_ki = float_val
                    elif param_name == "Angle_Kd":
                        self.node.angle_kd = float_val
                    elif param_name == "Angular_Kp":
                        self.node.angular_kp = float_val
                    elif param_name == "Angular_Ki":
                        self.node.angular_ki = float_val
                    elif param_name == "Angular_Kd":
                        self.node.angular_kd = float_val

                slider.valueChanged.connect(update_value)

                slider_layout.addWidget(label)
                slider_layout.addWidget(slider)
                pid_layout.addLayout(slider_layout)

            control_layout.addLayout(pid_layout)

        layout.addLayout(control_layout)

        # Node 运行控制按钮
        buttom_layout = QHBoxLayout()

        self.calibration_button = QPushButton("Calibrate")
        self.calibration_button.clicked.connect(self.calibration_pitch)

        self.motor_run_button = QPushButton("Run")
        self.motor_run_button.setCheckable(True)  # 允许按钮保持按下状态
        self.motor_run_button.clicked.connect(self.motor_run)

        self.motor_connect_button = QPushButton("Connect")
        self.motor_connect_button.setCheckable(True)  # 允许按钮保持按下状态
        # self.motor_connect_button.clicked.connect(self.motor_connect)

        buttom_layout.addWidget(self.calibration_button)
        buttom_layout.addWidget(self.motor_run_button)
        buttom_layout.addWidget(self.motor_connect_button)

        layout.addLayout(buttom_layout)
        self.setLayout(layout)

    def add_plot_tab(self):
        # 创建绘图区域
        self.speed_error_canvas = MplCanvas(title="Error")
        self.speed_output_canvas = MplCanvas(title="Output")
        self.angle_error_canvas = MplCanvas(title="Error", y_min=-0.5, y_max=0.5)
        self.angle_output_canvas = MplCanvas(title="Output")
        self.angular_error_canvas = MplCanvas(title="Error")
        self.angular_output_canvas = MplCanvas(title="Output")
        self.data1_canvas = MplCanvas(title="", )
        self.data2_canvas = MplCanvas(title="", )
        self.data3_canvas = MplCanvas(title="")
        self.data4_canvas = MplCanvas(title="")

        # 创建标签页内容
        speed_page = QWidget()
        angle_page = QWidget()
        angular_page = QWidget()
        data12_page = QWidget()
        data34_page = QWidget()

        # Speed PID 标签页布局
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(self.speed_error_canvas)  # 左侧 Error
        speed_layout.addWidget(self.speed_output_canvas)  # 右侧 Speed
        speed_page.setLayout(speed_layout)

        # Angle PID 标签页布局
        angle_layout = QHBoxLayout()
        angle_layout.addWidget(self.angle_error_canvas)  # 左侧 Error
        angle_layout.addWidget(self.angle_output_canvas)  # 右侧 Angle
        angle_page.setLayout(angle_layout)

        # Angular PID 标签页布局
        angular_layout = QHBoxLayout()
        angular_layout.addWidget(self.angular_error_canvas)  # 左侧 Error
        angular_layout.addWidget(self.angular_output_canvas)  # 右侧 Angular
        angular_page.setLayout(angular_layout)

        # Data PID 标签页布局
        data12_layout = QHBoxLayout()
        data12_layout.addWidget(self.data1_canvas)
        data12_layout.addWidget(self.data2_canvas)
        data12_page.setLayout(data12_layout)
        data34_layout = QHBoxLayout()
        data34_layout.addWidget(self.data3_canvas)
        data34_layout.addWidget(self.data4_canvas)
        data34_page.setLayout(data34_layout)

        # 将页面添加到 QTabWidget
        self.tab_widget.addTab(speed_page, "Speed")
        self.tab_widget.addTab(angle_page, "Angle")
        self.tab_widget.addTab(angular_page, "Angular")
        self.tab_widget.addTab(data12_page, "Data")
        self.tab_widget.addTab(data34_page, "Data")

    def calibration_pitch(self):
        self.node.calibration_complete = False

    def motor_run(self, checked):
        if checked:
            self.node.motor_run = True
        else:
            self.node.motor_run = False

    def run_node(self):
        # 运行 ROS2 节点
        try:
            rclpy.spin(self.node)
        except:
            pass

    def plot_data(self):
        while self.plot_thread_running:
            # self.speed_error_canvas.update_plot(list(self.node.speed_error_data))
            # self.speed_output_canvas.update_plot(list(self.node.speed_output_data))
            self.angle_error_canvas.update_plot(list(self.node.angle_error_data))
            self.angle_output_canvas.update_plot(list(self.node.angle_output_data))
            self.angular_error_canvas.update_plot(list(self.node.angular_error_data))
            self.angular_output_canvas.update_plot(list(self.node.angular_output_data))
            # self.data1_canvas.update_plot(list(self.node.data1), True)
            # self.data2_canvas.update_plot(list(self.node.data2), True)
            # self.data3_canvas.update_plot(list(self.node.data3), True)
            # self.data4_canvas.update_plot(list(self.node.data4), True)
            time.sleep(0.01)

    def closeEvent(self, event):
        # 在窗口关闭时，安全地停止线程和 ROS2 节点
        self.plot_thread_running = False
        self.node_thread_running = False

        self.node.motor_idle()
        if self.node.motor:
            self.node.motor.close()
        self.node.destroy_node()
        rclpy.shutdown()  # 关闭 ROS2 节点

        event.accept()  # 确保事件被接受，关闭窗口

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

# if __name__ == '__main__':
#     main()
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ControlGUI()
    gui.show()
    sys.exit(app.exec_())

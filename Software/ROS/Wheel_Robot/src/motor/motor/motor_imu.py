import rclpy, math, time, threading, sys, serial, struct
import numpy as np
import tf_transformations as tf
from collections import deque
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Point, Quaternion, TransformStamped
from odrive.enums import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QSlider, QLabel, QFrame, QTabWidget, QApplication
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
        self.alpha = 0.97  # 互补滤波系数
        self.roll = 0.0
        self.pitch = 0.0

    def update(self, accel, gyro, dt):
        # 加速度计计算角度（适用于低频、长期稳定）
        # accel_roll = math.atan2(accel[1], accel[2])
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        # 陀螺仪积分计算角度（短期稳定，易漂移）
        # gyro_roll = self.roll + gyro[0] * dt
        gyro_pitch = self.pitch + gyro[1] * dt
        # 互补滤波
        # self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        return self.pitch


class MotorIMUNode(Node):
    def __init__(self, motor_port_name, motor_baudrate, imu_port_name, imu_baudrate):
        super().__init__('motor_imu_node')
        self.params_init()
        self.task_init(motor_port_name, motor_baudrate, imu_port_name, imu_baudrate)

    def params_init(self):
        self.motor = None

        # 上次回调时间
        self.last_time_read_motor = time.time()
        self.last_time_pid = time.time()
        self.last_time_imu_callback = time.time()

        # PID参数
        self.speed_kp, self.speed_ki, self.speed_kd = 0.0, 0.0, 0.0
        self.angle_kp, self.angle_ki, self.angle_kd = 10, 0.0, 0.2
        self.angular_kp, self.angular_ki, self.angular_kd = 1, 0.0, 0.0

        # 状态变量
        self.speed_error = 0
        self.angle_error = 0
        self.angular_error = 0
        self.speed_integral = 0
        self.angle_integral = 0
        self.angular_integral = 0
        self.prev_speed_error = 0
        self.prev_angle_error = 0
        self.prev_angular_error = 0

        # 来自轮速传感器的速度估计
        self.motor_pos0 = 0
        self.motor_pos1 = 0
        self.motor_vel0 = 0
        self.motor_vel1 = 0
        self.motor_pos0_filter = 0
        self.motor_pos1_filter = 0
        self.motor_vel0_filter = 0
        self.motor_vel1_filter = 0

        # 目标角度
        self.target_roll = 0
        self.target_pitch = 0
        # 运动参数
        self.target_linear = 0.0
        self.target_angular = 0.0
        # 机器人物理参数
        self.x = 0.0
        self.y = 0.0
        self.wheel_base = 0.174    # 车轮间距 (meters)
        self.wheel_radius = 0.026  # 轮子半径 (meters)

        self.pitch_max = 0.4048
        self.pitch_min = -0.5031
        self.pitch_mid = (self.pitch_max + self.pitch_min) / 2
        self.max_speed = 15.0  # 电机最大转速 (rad/s)
        self.max_angle = 90.0
        self.min_angle = 0.0

        # calibration
        self.max_samples = 100  # calibate 时采样 100 次
        self.pitch_sum = 0
        self.sample_count = 0
        self.calibration_complete = True

    def task_init(self, motor_port_name, motor_baudrate, imu_port_name, imu_baudrate):
        self.motor_run = False
        self.imu_processor = IMUProcessor()
        self.kf_pitch = KalmanFilter()

        # 初始化订阅
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 创建 IMU Publisher
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        # 启动 Motor 驱动线程
        self.motor_thread_running = True
        self.serial_lock = threading.Lock()
        self.motor_thread = threading.Thread(target=self.motor_thread_loop, args=(motor_port_name, motor_baudrate, ), daemon=True)
        self.motor_thread.start()

        # 创建 Odometry Publisher
        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 启动 IMU 驱动线程
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.angle = [0.0, 0.0, 0.0]
        self.pitch = 0
        self.imu_key = 0
        self.imu_falg = 0
        self.imu_buff = {}
        self.IMU_thread_running = True
        self.IMU_thread = threading.Thread(target=self.imu_thread_loop, args=(imu_port_name, imu_baudrate, ), daemon=True)
        self.IMU_thread.start()

        # 数据记录
        self.plot_num = 200
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

    def publish_thread_loop(self):
        while self.publish_thread_running and self.motor:
            current_time = time.time()
            dt = max(current_time - self.last_time_read_motor, 1e-6)  # 防止除以0
            self.last_time_read_motor = current_time
            # print(f'dt: {dt:.5f}, pos0: {self.motor_pos0:.5f}, vel0: {self.motor_vel0:.5f}')

            self.read_motor()
            self.publish_msg()

    def imu_thread_loop(self, port_name, baudrate):
        # 打开串口
        try:
            self.imu = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.5)
            if self.imu.isOpen():
                self.get_logger().info("\033[32mIMU serial port connection establised...\033[0m")
                self.get_logger().info("\033[32mIMU Node started...\033[0m")
            else:
                self.imu.open()
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().info("\033[31mMotor serial port opening failure\033[0m")

        # 循环读取IMU数据
        while self.IMU_thread_running and self.imu:
            # 读取加速度计数据
            try:
                buff_count = self.imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
            else:
                if buff_count > 0:
                    buff_data = self.imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = self.handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_data()
                if not self.calibration_complete:
                    if self.sample_count == 0:
                        self.motor_send_cmd(f'w axis0.requested_state {AxisState.IDLE}')
                        self.motor_send_cmd(f'w axis1.requested_state {AxisState.IDLE}')
                        print('IDLE')
                    elif self.sample_count == self.max_samples:
                        self.motor_send_cmd(f'w axis0.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
                        self.motor_send_cmd(f'w axis1.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
                        print('LOOP')
                    if self.sample_count < self.max_samples:
                        if self.sample_count == 0:
                            print("校准中... 请保持平衡车在平衡状态，正在采样...")
                        self.sample_count += 1
                        self.pitch_sum += self.pitch
                        print(f'pitch: {self.pitch:.5f}')
                    else:
                        self.target_pitch = (self.pitch_sum / self.max_samples)
                        print(f"校准完成，目标平衡角度：{self.target_pitch:.5f}")
                        self.calibration_complete = True
                        self.sample_count = 0
                        self.pitch_sum = 0

    def motor_thread_loop(self, port_name, baudrate):
        # 连接 Motor
        try:
            self.motor = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.1)
            self.set_servo_angle(25, 25)
            self.get_logger().info("\033[32mMotor serial port connection established...\033[0m")
            self.motor_setup()
            self.get_logger().info("\033[32mMotor Setup Successed...\033[0m")
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().info("\033[31mIMU serial port opening failure\033[0m")
            if self.motor:
                self.motor.close()
            return

        # Publish thread
        self.publish_thread_running = True
        self.publish_thread = threading.Thread(target=self.publish_thread_loop, daemon=True)
        self.publish_thread.start()

        time.sleep(1.5)
        while self.motor_thread_running and self.motor:
            current_time = time.time()
            dt = max(current_time - self.last_time_pid, 1e-6)  # 防止除以0
            self.last_time_pid = current_time

            left_speed, right_speed = self.cascaded_pid_control(dt)
            print(f'dt: {dt:.5f}, pitch: {round(self.pitch, 3)}, error: {self.angle_error:.5f}')
            # print(f'ls: {left_speed:.4f}, rs: {right_speed:.4f}')
            if not self.motor_run or not self.calibration_complete or abs(self.angle_error) > 0.30:
                self.motor.write('v 0 0.0\nv 1 0.0\n'.encode())
            else:
                self.set_motor_speeds(left_speed, right_speed, direction=1)
            time.sleep(0.005)

    def cascaded_pid_control(self, dt):
        # self.motor_vel0_filter = 0.3 * self.motor_vel0 + 0.7 * self.motor_vel0_filter
        # self.motor_vel1_filter = 0.3 * self.motor_vel1 + 0.7 * self.motor_vel1_filter
        # left_speed = self.motor_vel0_filter
        # right_speed = self.motor_vel1_filter
        left_speed = self.motor_vel0
        right_speed = self.motor_vel1

        # self.data1.append(self.motor_pos0)
        self.data2.append(right_speed)

        # 当前车速(机器人线速度)：左右轮速度平均 (未转换为车轮实际线速度)
        current_linear_speed = (self.motor_vel0 + self.motor_vel1) / 2.0
        current_angular_speed = (self.motor_vel1 - self.motor_vel0) / self.wheel_base

        # self.gyro_y_filter = 0.3 * self.gyro[1] + 0.7 * self.gyro_y_filter  # TODO

        # ========== 1. 速度环 PID ==========
        self.speed_error = current_linear_speed - self.target_linear  # 当前速度可由轮速传感器提供
        self.speed_integral += self.speed_error * dt
        speed_derivative = ((self.speed_error - self.prev_speed_error) / dt)
        speed_output = (
            self.speed_kp * self.speed_error +
            self.speed_ki * self.speed_integral +
            self.speed_kd * speed_derivative
        )
        self.prev_speed_error = self.speed_error
        desired_pitch = self.target_pitch + speed_output  # 外环输出为期望角度

        # ========== 2. 角度环 PID ==========
        self.angle_error = round(self.pitch, 3) - desired_pitch
        self.angle_integral += self.angle_error * dt
        angle_derivative = ((self.angle_error - self.prev_angle_error) / dt)
        angle_output = (
            self.angle_kp * self.angle_error +
            self.angle_ki * self.angle_integral +
            self.angle_kd * angle_derivative
        )
        self.prev_angle_error = self.angle_error
        desired_angular_velocity = angle_output  # 输出为期望角速度

        # ========== 3. 角速度环 PID ==========
        self.angular_error = 0 - desired_angular_velocity
        self.angular_integral += self.angular_error * dt
        angular_derivative = ((self.angular_error - self.prev_angular_error) / dt)
        angular_output = (
            self.angular_kp * self.angular_error +
            self.angular_ki * self.angular_integral +
            self.angular_kd * angular_derivative
        )
        self.prev_angular_error = self.angular_error

        # ========== 电机速度合成 ==========
        linear_speed = self.target_linear  # 线速度: 未转换为车轮目标线速度
        angular_diff = (self.target_angular * self.wheel_base) / 2  # 角速度差分: 未转换为车轮目标线速度
        left_speed = (linear_speed - angular_diff) - angular_output
        right_speed = (linear_speed + angular_diff) + angular_output

        # 记录日志
        self.speed_error_data.append(self.speed_error)
        self.speed_output_data.append(speed_output)
        self.angle_error_data.append(self.angle_error)
        self.angle_output_data.append(angle_output)
        self.angular_error_data.append(self.angular_error)
        self.angular_output_data.append(angular_output)
        self.data1.append(angle_output)

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
                # self.motor_send_cmd(f'w axis0.requested_state {AxisState.IDLE}')
                # self.motor_send_cmd(f'w axis1.requested_state {AxisState.IDLE}')
                break

    def motor_send_cmd(self, cmd:str):
        # 在写操作前后上锁
        with self.serial_lock:
            # 底层写串口
            if self.motor:
                try:
                    self.motor.write((cmd + '\n').encode())
                    return self.motor.readline().decode().strip()
                except Exception as e:
                    print(f'motor cmd send failed {e}')

    def motor_loop(self):
        self.motor_send_cmd(f'w axis0.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
        self.motor_send_cmd(f'w axis1.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
        self.motor_send_cmd('v 0 0.0 0.0')
        self.motor_send_cmd('v 1 0.0 0.0')

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
            pos = self.motor_send_cmd('r p')
            vel = self.motor_send_cmd('r v')
            if pos and vel:
                self.motor_pos0, self.motor_pos1 = map(float, pos.split())
                self.motor_vel0, self.motor_vel1 = map(float, vel.split())
                # print(f"pos0={-self.motor_pos0}, vel0={-self.motor_vel0}, pos1={self.motor_pos1}, vel1={self.motor_vel1}")
        except ValueError:
            print('Invalid data format')

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
        joint_msg.velocity = [float(-self.motor_vel0), float(self.motor_vel1)]  # 轮子速度
        self.joint_pub.publish(joint_msg)

    def set_motor_speeds(self, left_speed, right_speed, direction=0):
        # 限幅处理
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if direction:
            left_speed = -left_speed
            right_speed = -right_speed
        try:
            # 根据实际电机方向可能需要调整符号
            setspeed = f'v 0 {left_speed:.4f}\nv 1 {right_speed:.4f}\n'
            self.motor.write(setspeed.encode())
        except Exception as e:
            self.get_logger().error(f"Motor control failed: {str(e)}")

    def set_servo_angle(self, left_angle, right_angle):
        # 限幅处理
        left_angle = max(min(left_angle, self.max_angle), self.min_angle)
        right_angle = max(min(right_angle, self.max_angle), self.min_angle)
        # 角度转换
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

    def hex_to_short(self, raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))

    def check_sum(self, list_data, check_data):
        return sum(list_data) & 0xff == check_data

    def handle_serial_data(self, raw_data):
        angle_flag = False
        self.imu_buff[self.imu_key] = raw_data

        self.imu_key += 1
        if self.imu_buff[0] != 0x55:
            self.imu_key = 0
            return
        # According to the judgment of the data length bit, the corresponding length data can be obtained
        if self.imu_key < 11:
            return
        else:
            data_buff = list(self.imu_buff.values())  # Get dictionary ownership value
            if self.imu_buff[1] == 0x51:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                    accel_scale = 16 / 32768.0
                    self.accel = [acceleration[0] * accel_scale, acceleration[1] * accel_scale, acceleration[2] * accel_scale]
                else:
                    print('0x51 Check failure')
            elif self.imu_buff[1] == 0x52:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    angularVelocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
                    gyro_scale = 2000 / 32768.0
                    self.gyro = [math.radians(angularVelocity[0] * gyro_scale), math.radians(angularVelocity[1] * gyro_scale), math.radians(angularVelocity[2] * gyro_scale)]
                else:
                    print('0x52 Check failure')
            elif self.imu_buff[1] == 0x53:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angle = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                    angle_flag = True
                else:
                    print('0x53 Check failure')
            elif self.imu_buff[1] == 0x54:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    magnetometer = self.hex_to_short(data_buff[2:10])
                    print('0x54')
                else:
                    print('0x54 Check failure')
            else:
                self.imu_buff = {}
                self.imu_key = 0

            self.imu_buff = {}
            self.imu_key = 0
            return angle_flag

    def imu_data(self):
        current_time = time.time()
        dt = max(current_time - self.last_time_imu_callback, 1e-6)
        self.last_time_imu_callback = current_time

        # 互补滤波计算角度
        self.pitch = self.imu_processor.update(self.accel, self.gyro, dt)
        # 卡尔曼滤波计算角度
        # self.pitch = self.kf_pitch.update(self.gyro[1], self.pitch)

        # print(f'dt: {dt:.8f}, pitch: {self.pitch:.7f}')
        self.data3.append(self.pitch)
        # self.data4.append()

        # 更新IMU消息
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = self.accel[0]
        self.imu_msg.linear_acceleration.y = self.accel[1]
        self.imu_msg.linear_acceleration.z = self.accel[2]
        self.imu_msg.angular_velocity.x = self.gyro[0]
        self.imu_msg.angular_velocity.y = self.gyro[1]
        self.imu_msg.angular_velocity.z = self.gyro[2]

        angle_radian = [self.angle[i] * math.pi / 180 for i in range(3)]
        qua = self.get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
        
        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        # 发布IMU消息
        self.imu_pub.publish(self.imu_msg)

    def compute_orientation(self, wx, wy, wz, ax, ay, az, dt):
        # 计算旋转矩阵
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(ax), -math.sin(ax)],
                       [0, math.sin(ax), math.cos(ax)]])
        Ry = np.array([[math.cos(ay), 0, math.sin(ay)],
                       [0, 1, 0],
                       [-math.sin(ay), 0, math.cos(ay)]])
        Rz = np.array([[math.cos(wz), -math.sin(wz), 0],
                       [math.sin(wz), math.cos(wz), 0],
                       [0, 0, 1]])
        R = Rz.dot(Ry).dot(Rx)

        # 计算欧拉角
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], math.sqrt(R[2][1] ** 2 + R[2][2] ** 2))
        yaw = math.atan2(R[1][0], R[0][0])

        return roll, pitch, yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]


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

        # 清除旧的文本（如果有的话）
        if hasattr(self, 'value_text'):
            self.value_text.remove()
        # 显示最新数据点的值
        if data:
            latest_x = len(data) - 1
            latest_y = data[-1]
            self.value_text = self.ax.text(
                latest_x, latest_y, f' {latest_y:.4f}',
                fontsize=10, color='red', ha='left', va='bottom'
            )

        self.draw()

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ModorNode Controller")
        self.resize(1200, 700)

        # ROS Node
        rclpy.init()
        self.node = MotorIMUNode("/dev/motor", 512000, "/dev/imu", 921600)

        # 初始化ui
        self.init_ui()

        # 启动node线程
        if self.node.motor:
            self.motor_connect_button.setChecked(True)
            self.node_thread_running = True
            self.node_thread = threading.Thread(target=self.run_node, daemon=True)
            self.node_thread.start()
        # 启动绘图线程
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
                ("Kd", self.node.angle_kd, -1.5, 1.5),
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
        self.angle_output_canvas = MplCanvas(title="Output", y_min=-15, y_max=15)
        self.angular_error_canvas = MplCanvas(title="Error")
        self.angular_output_canvas = MplCanvas(title="Output")
        self.data1_canvas = MplCanvas(title="")
        self.data2_canvas = MplCanvas(title="")
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
        self.last_time_plot = time.time()
        while self.plot_thread_running:
            # current_time = time.time()
            # dt = max(current_time - self.last_time_plot, 1e-6)
            # self.last_time_plot = current_time
            # print(f'dt: {dt:.5f}')

            # self.speed_error_canvas.update_plot(list(self.node.speed_error_data))
            # self.speed_output_canvas.update_plot(list(self.node.speed_output_data))
            self.angle_error_canvas.update_plot(list(self.node.angle_error_data))
            self.angle_output_canvas.update_plot(list(self.node.angle_output_data))
            # self.angular_error_canvas.update_plot(list(self.node.angular_error_data))
            # self.angular_output_canvas.update_plot(list(self.node.angular_output_data))
            # self.data1_canvas.update_plot(list(self.node.data1), True)
            # self.data2_canvas.update_plot(list(self.node.data2), True)
            self.data3_canvas.update_plot(list(self.node.data3), True)
            self.data4_canvas.update_plot(list(self.node.data4), True)
            time.sleep(0.005)

    def closeEvent(self, event):
        # 在窗口关闭时，安全地停止线程和 ROS2 节点
        self.node.publish_thread_running = False
        self.node.IMU_thread_running = False
        self.node.motor_thread_running = False
        self.plot_thread_running = False

        self.node.motor_idle()
        if self.node.motor:
            self.node.motor.close()
        if self.node.imu:
            self.node.imu.close()
        self.node.destroy_node()
        rclpy.shutdown()  # 关闭 ROS2 节点

        event.accept()  # 确保事件被接受，关闭窗口


def main():
    # 初始化 ROS2 节点
    rclpy.init()
    motorimu_node = MotorIMUNode("/dev/motor", 512000, "/dev/imu", 921600)

    motorimu_node.motor_run = True
    motorimu_node.target_pitch = -0.055
    motorimu_node.calibration_complete = True
    motorimu_node.motor_loop()

    # 运行 ROS2 节点
    try:
        rclpy.spin(motorimu_node)
    except KeyboardInterrupt:
        pass
    # 停止 ROS2 节点
    finally:
        motorimu_node.publish_thread_running = False
        motorimu_node.IMU_thread_running = False
        motorimu_node.motor_thread_running = False
        motorimu_node.motor_idle()
        motorimu_node.destroy_node()
        motorimu_node.motor.close()
        motorimu_node.imu.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     gui = ControlGUI()
#     gui.show()
#     sys.exit(app.exec_())

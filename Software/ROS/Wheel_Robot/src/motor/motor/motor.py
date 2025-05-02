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

class KalmanFilter:
    def __init__(self):
        self.dt = 0.01  # 采样时间
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
        self.dt = 0.01  # 采样周期
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

        # 创建发布者
        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 初始化订阅
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 机器人物理参数
        self.x = 0.0
        self.y = 0.0
        self.wheel_radius = 0.0265  # 轮子半径 (meters)
        self.wheel_base = 0.174  # 车轮间距 (meters)

        # 连接 Motor
        try:
            self.motor = Serial(port=port_name, baudrate=baudrate, timeout=0.5)
            self.get_logger().info("\033[32mMotor serial port connection established...\033[0m")
            self.motor_setup()
            self.get_logger().info("\033[32mMotor Setup Successed...\033[0m")
            # self.set_servo_angle(15, 15)
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            if self.motor:
                self.motor.close()
            exit(0)

        # 控制参数
        self.balance_kp = 30.0
        self.balance_ki = 1
        self.balance_kd = 1
        self.velocity_kp = 5
        self.velocity_ki = 0
        self.velocity_kd = 0

        # 目标角度
        self.target_roll = 0
        self.target_pitch = -0.04
        # 上次回调时间
        self.last_callback_time = time.time()
        self.last_time = time.time()
        # 误差项
        self.prev_error = 0.0
        self.integral = 0.0
        # 运动参数
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.wheel_base = 0.174    # 车轮间距 (meters)
        self.wheel_radius = 0.026  # 轮子半径 (meters)
        # 初始化模糊控制器
        self.fuzzy_enable = False
        if self.fuzzy_enable:
            self.setup_fuzzy_controller()

        self.pitch_max = 0.4832998220
        self.pitch_min = -0.6481068829
        self.pitch_mid = (self.pitch_max + self.pitch_min) / 2
        self.max_speed = 50.0  # 电机最大转速 (rad/s)
        self.max_angle = 90.0  #
        self.min_angle = 0.0  #

        self.max_samples = 1000    # 启动时采样 1000 次
        self.pitch_sum = 0
        self.sample_count = 0
        self.calibration_complete = True

        self.imu_processor = IMUProcessor()
        self.kf_roll = KalmanFilter()
        self.kf_pitch = KalmanFilter()

    def setup_fuzzy_controller(self):
        """ 设置模糊逻辑控制器 """
        # 定义模糊变量
        self.error = ctrl.Antecedent(np.arange(-0.03, 0.03, 0.001), 'error')
        self.d_error = ctrl.Antecedent(np.arange(-0.1, 0.1, 0.001), 'd_error')
        self.pid_output = ctrl.Consequent(np.arange(-1, 1, 0.001), 'pid_output')

        # 隶属函数
        self.error.automf(5)
        self.d_error.automf(5)
        self.pid_output.automf(5)

        # 规则集
        self.rules = [
            ctrl.Rule(self.error['poor'] & self.d_error['poor'], self.pid_output['good']),
            ctrl.Rule(self.error['poor'] & self.d_error['mediocre'], self.pid_output['decent']),
            ctrl.Rule(self.error['poor'] & self.d_error['average'], self.pid_output['average']),
            ctrl.Rule(self.error['poor'] & self.d_error['decent'], self.pid_output['mediocre']),
            ctrl.Rule(self.error['poor'] & self.d_error['good'], self.pid_output['poor']),
            
            ctrl.Rule(self.error['mediocre'] & self.d_error['poor'], self.pid_output['good']),
            ctrl.Rule(self.error['mediocre'] & self.d_error['mediocre'], self.pid_output['decent']),
            ctrl.Rule(self.error['mediocre'] & self.d_error['average'], self.pid_output['average']),
            ctrl.Rule(self.error['mediocre'] & self.d_error['decent'], self.pid_output['mediocre']),
            ctrl.Rule(self.error['mediocre'] & self.d_error['good'], self.pid_output['poor']),
            
            ctrl.Rule(self.error['average'] & self.d_error['poor'], self.pid_output['decent']),
            ctrl.Rule(self.error['average'] & self.d_error['mediocre'], self.pid_output['average']),
            ctrl.Rule(self.error['average'] & self.d_error['average'], self.pid_output['average']),
            ctrl.Rule(self.error['average'] & self.d_error['decent'], self.pid_output['average']),
            ctrl.Rule(self.error['average'] & self.d_error['good'], self.pid_output['mediocre']),
            
            ctrl.Rule(self.error['decent'] & self.d_error['poor'], self.pid_output['poor']),
            ctrl.Rule(self.error['decent'] & self.d_error['mediocre'], self.pid_output['mediocre']),
            ctrl.Rule(self.error['decent'] & self.d_error['average'], self.pid_output['average']),
            ctrl.Rule(self.error['decent'] & self.d_error['decent'], self.pid_output['decent']),
            ctrl.Rule(self.error['decent'] & self.d_error['good'], self.pid_output['good']),
            
            ctrl.Rule(self.error['good'] & self.d_error['poor'], self.pid_output['poor']),
            ctrl.Rule(self.error['good'] & self.d_error['mediocre'], self.pid_output['poor']),
            ctrl.Rule(self.error['good'] & self.d_error['average'], self.pid_output['mediocre']),
            ctrl.Rule(self.error['good'] & self.d_error['decent'], self.pid_output['decent']),
            ctrl.Rule(self.error['good'] & self.d_error['good'], self.pid_output['good'])
        ]

        # 控制系统
        self.fuzzy_ctrl = ctrl.ControlSystem(self.rules)
        self.fuzzy_sim = ctrl.ControlSystemSimulation(self.fuzzy_ctrl)

    def cmd_vel_callback(self, msg:Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        # self.get_logger().info(f"收到指令: 线速度={self.target_linear}m/s, 角速度={self.target_angular}rad/s")

    def imu_callback(self, msg:Imu):
        self.last_callback_time = time.time()

        # 提取四元数（ROS2的Imu消息中四元数顺序为 x,y,z,w）
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # 将四元数转换为欧拉角（Roll, Pitch, Yaw，单位为弧度）
        # pitch: forward; roll, left-right
        (roll, pitch, yaw) = tf.euler_from_quaternion(q)
        # self.get_logger().info(f'Roll1: {roll:.10f}, Pitch1: {pitch:.10f}')

        # # 互补滤波计算角度
        # accel_x, accel_y, accel_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        # gyro_x, gyro_y, gyro_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        # roll, pitch = self.imu_processor.update([accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z])
        # # 卡尔曼滤波计算角度
        # roll = self.kf_roll.update(gyro_x, roll)
        # pitch = self.kf_pitch.update(gyro_y, pitch)
        # self.get_logger().info(f'Roll2: {roll:.10f}, Pitch2: {pitch:.10f}')

        # 安全保护（角度过大时停止）
        # if (abs(pitch-self.pitch_mid) > 0.30):
        #     # self.motor_idle()
        #     self.motor.axis0.controller.input_vel = 0
        #     self.motor.axis1.controller.input_vel = 0
        #     return

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
            return  # 在校准期间不进行 PID 控制

        left_speed, right_speed = self.balance_compute(pitch)
        # print(f"Left={left_speed}, Right={right_speed}")
        # self.set_motor_speeds(left_speed, right_speed, direction=1)

        self.read_and_publish()

    def balance_compute(self, pitch):
        # 计算误差和变化率
        current_time = time.time()
        dt = current_time - self.last_time
        error = self.target_pitch - pitch
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        if self.fuzzy_enable:
            # 运行模糊控制器
            self.fuzzy_sim.input['error'] = error
            self.fuzzy_sim.input['d_error'] = derivative
            self.fuzzy_sim.compute()
            correction = self.fuzzy_sim.output['pid_output']
            # self.get_logger().info(f'error: {error:.10f}, d_error: {derivative:.10f}')
        else:
            # PID 控制计算
            correction = self.balance_kp * error + self.balance_ki * self.integral + self.balance_kd * derivative
            # print(f'error: {error:.10f}, integral: {self.integral:.10f}, derivative: {derivative:.10f}')
            # print(f'error: {self.balance_kp * error:.10f}, integral: {self.balance_ki * self.integral:.10f}, derivative: {self.balance_kd * derivative:.10f}')

        # 更新状态
        self.prev_error = error
        self.last_time = current_time

        # 运动学转换
        linear_speed = self.target_linear / self.wheel_radius  # 转换为电机角速度
        angular_diff = (self.target_angular * self.wheel_base) / (2 * self.wheel_radius)
        # 计算左右轮速度
        left_speed = linear_speed - angular_diff - correction
        right_speed = linear_speed + angular_diff + correction

        return left_speed, right_speed

    def motor_setup(self):
        while (True):
            # 切换到闭环控制
            self.motor_send_cmd(f'w axis0.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
            self.motor_send_cmd(f'w axis1.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
            time.sleep(0.1)
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

    def set_motor_speeds(self, left_speed, right_speed, direction=1):
        # 限幅处理
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if direction:
            left_speed = -left_speed
            right_speed = -right_speed
        # self.get_logger().info(f"Left1={left_speed}, Right1={right_speed}")
        try:
            # 根据实际电机方向可能需要调整符号
            self.motor.axis0.controller.input_vel = left_speed  # 左电机
            self.motor.axis1.controller.input_vel = right_speed  # 右电机
            # left_speed = self.motor.axis0.encoder.vel_estimate
            # right_speed = self.motor.axis0.encoder.vel_estimate
        except Exception as e:
            self.get_logger().error(f"电机控制失败: {str(e)}")
        # self.get_logger().info(f"Left2={left_speed}, Right2={right_speed}")

    def set_servo_angle(self, left_angle, right_angle):
        # left_angle = max(min(left_angle, self.max_angle), self.min_angle)
        # right_angle = max(min(right_angle, self.max_angle), self.min_angle)
        try:
            # 根据实际电机方向可能需要调整符号
            self.motor.servo0.angle = 90 - right_angle
            self.motor.servo1.angle = 80 + right_angle
            self.motor.servo2.angle = 90 + left_angle
            self.motor.servo3.angle = 100 - left_angle
        except Exception as e:
            self.get_logger().error(f"舵机控制失败: {str(e)}")
        pass

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
                        pass
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
            pos0 = float(self.motor_send_cmd('r axis0.encoder.pos_estimate'))
            pos1 = float(self.motor_send_cmd('r axis1.encoder.pos_estimate'))
            vel0 = float(self.motor_send_cmd('r axis0.encoder.vel_estimate'))
            vel1 = float(self.motor_send_cmd('r axis1.encoder.vel_estimate'))
            print(f"pos0={-pos0}, vel0={-vel0}, pos1={pos1}, vel1={vel1}")
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
        self.joint_pub.publish(joint_msg)

        # self.get_logger().info(f"x: {self.x:.10f}, y: {self.y:.10f}, theta: {self.theta:.10f}")


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
        motor_node.motor.close()
        motor_node.motor_idle()
        motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

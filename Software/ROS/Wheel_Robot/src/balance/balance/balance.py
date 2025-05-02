import rclpy, math, time, odrive
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from odrive.enums import *
from odrive.rich_text import RichText, Color, Style
import tf_transformations as tf
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl

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


class BalanceBot(Node):
    def __init__(self):
        super().__init__('balance_bot')

        # 初始化订阅
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 连接Odrive
        try:
            self.odrive = odrive.find_any(timeout=10)
            self.get_logger().info("\033[32mOdrive Connected...\033[0m")
            self.odrive_setup()
            self.get_logger().info("\033[32mMotor Setup Successed...\033[0m")
            self.set_servo_angle(15, 15)
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            raise

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

        # 安全定时器
        # self.create_timer(0.1, self.safety_check)
        # self.last_callback_time = time.time()

    def safety_check(self):
        if time.time() - self.last_callback_time > 0.2:
            self.emergency_stop()

    def emergency_stop(self):
        self.odrive.axis0.controller.input_vel = 0.0
        self.odrive.axis1.controller.input_vel = 0.0
        self.get_logger().warn("紧急停止激活")

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

        self.balance_compute(pitch, motor_enable=False)

    def balance_compute(self, pitch, motor_enable=True):
        # 安全保护（角度过大时停止）
        # if (abs(pitch-self.pitch_mid) > 0.30):
        #     # self.odrive_idle()
        #     self.odrive.axis0.controller.input_vel = 0
        #     self.odrive.axis1.controller.input_vel = 0
        #     return

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

        print(f"Correction={correction}, Left={left_speed}, Right={right_speed}")
        if motor_enable:
            self.set_motor_speeds(left_speed, right_speed, direction=1)

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
            self.odrive.axis0.controller.input_vel = left_speed  # 左电机
            self.odrive.axis1.controller.input_vel = right_speed  # 右电机
            # left_speed = self.odrive.axis0.encoder.vel_estimate
            # right_speed = self.odrive.axis0.encoder.vel_estimate
        except Exception as e:
            self.get_logger().error(f"电机控制失败: {str(e)}")
        # self.get_logger().info(f"Left2={left_speed}, Right2={right_speed}")

    def set_servo_angle(self, left_angle, right_angle):
        # left_angle = max(min(left_angle, self.max_angle), self.min_angle)
        # right_angle = max(min(right_angle, self.max_angle), self.min_angle)
        try:
            # 根据实际电机方向可能需要调整符号
            self.odrive.servo0.angle = 90 - right_angle
            self.odrive.servo1.angle = 80 + right_angle
            self.odrive.servo2.angle = 90 + left_angle
            self.odrive.servo3.angle = 100 - left_angle
        except Exception as e:
            self.get_logger().error(f"舵机控制失败: {str(e)}")
        pass

    def odrive_idle(self):
        self.odrive.axis0.requested_state = AxisState.IDLE
        self.odrive.axis1.requested_state = AxisState.IDLE
        self.odrive.axis0.controller.input_vel = 0
        self.odrive.axis1.controller.input_vel = 0
        # self.odrive.servo0.pulse = 0
        # self.odrive.servo1.pulse = 0
        # self.odrive.servo2.pulse = 0
        # self.odrive.servo3.pulse = 0

    def odrive_setup(self):
        # 配置输入模式和控制模式
        self.odrive.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        self.odrive.axis1.controller.config.input_mode = InputMode.PASSTHROUGH
        self.odrive.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrive.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        while (True):
            # 切换到闭环控制
            self.odrive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.odrive.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
            time.sleep(0.1)
            errors = self.check_odrive_error()
            if errors:
                self.handle_odrive_error(errors)
                # 等待错误处理完成
                while (self.odrive.axis0.current_state != AxisState.IDLE or
                    self.odrive.axis1.current_state != AxisState.IDLE):
                    time.sleep(0.1)
            else:
                break

    def check_odrive_error(self, clear=True, full=False):
        lines = []
        errors = []

        STYLE_GOOD = (Color.GREEN, Color.DEFAULT, Style.BOLD)
        STYLE_WARN = (Color.YELLOW, Color.DEFAULT, Style.BOLD)
        STYLE_BAD = (Color.RED, Color.DEFAULT, Style.BOLD)

        axes = [(name, getattr(self.odrive, name)) for name in dir(self.odrive) if name.startswith('axis')]
        axes.sort()

        def decode_flags(val, enum_type, name, errors):
            errorcodes = {v.value: f"{enum_type.__name__}.{v.name}" for v in enum_type}
            if val == 0:
                if full:
                    return [RichText("no error", *STYLE_GOOD)]
                else:
                    return []
            else:
                errors.append((name, enum_type, val))
                return [RichText("Error(s):", *STYLE_BAD)] + [
                    RichText(errorcodes.get((1 << bit), 'UNKNOWN ERROR: 0x{:08X}'.format(1 << bit)), *STYLE_BAD)
                    for bit in range(64) if val & (1 << bit) != 0]

        def dump_item(indent, name, obj, path, decoder):
            prefix = indent + name.strip('0123456789') + ": "
            for elem in path.split('.'):
                if not hasattr(obj, elem):
                    return [prefix + RichText("not found", *STYLE_WARN)]
                obj = getattr(obj, elem)

            lines = decoder(obj)
            if lines:
                lines = [indent + name + ": " + lines[0]] + [
                    indent + "  " + line
                    for line in lines[1:]
                ]
            return lines

        lines += dump_item("", "system", self.odrive, 'error', lambda x: decode_flags(x, odrive.enums.LegacyODriveError, "system", errors))

        for name, axis in axes:
            errors_str = []
            errors_str += dump_item("  ", 'axis', axis, 'error', lambda x: decode_flags(x, odrive.enums.AxisError, name, errors))
            errors_str += dump_item("  ", 'motor', axis, 'motor.error', lambda x: decode_flags(x, odrive.enums.MotorError, name, errors))
            errors_str += dump_item("  ", 'encoder', axis, 'encoder.error', lambda x: decode_flags(x, odrive.enums.EncoderError, name, errors))
            errors_str += dump_item("  ", 'controller', axis, 'controller.error', lambda x: decode_flags(x, odrive.enums.ControllerError, name, errors))
            if errors_str:
                lines.append(name)
                lines += errors_str

        if clear:
            self.odrive.clear_errors()

        if lines:
            odrive.rich_text.print_rich_text(RichText('\n').join(lines))

        return errors

    def handle_odrive_error(self, errors):
        for name, enum_type, error_var in errors:
            # 根据错误代码执行相应的错误处理函数
            if name == "system":
                time.sleep(1)
            if enum_type == odrive.enums.AxisError:
                pass
            elif enum_type == odrive.enums.MotorError:
                if error_var == MotorError.UNKNOWN_PHASE_ESTIMATE:
                    self.motor_calibration(name)
            elif enum_type == odrive.enums.EncoderError:
                self.motor_calibration(name)
            elif enum_type == odrive.enums.ControllerError:
                pass
            else:
                pass

    def motor_calibration(self, name):
        # 启动编码器偏移校准
        if name == 'axis0':
            self.odrive.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
        elif name == 'axis1':
            self.odrive.axis1.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BalanceBot()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Node crashed: {str(e)}")
    finally:
        node.odrive_idle()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

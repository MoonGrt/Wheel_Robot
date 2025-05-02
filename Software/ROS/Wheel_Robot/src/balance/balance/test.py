import time, math, serial, struct, threading, rclpy, odrive
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from odrive.enums import *
from odrive.rich_text import RichText, Color, Style

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


# 定义IMU驱动节点类
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    angle_flag = False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    # According to the judgment of the data length bit, the corresponding length data can be obtained
    if key < 11:
        return
    else:
        data_buff = list(buff.values())  # Get dictionary ownership value
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print('0x51 Check failure')

        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in
                                   range(0, 3)]

            else:
                print('0x52 Check failure')

        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True
            else:
                print('0x53 Check failure')
        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
            else:
                print('0x54 Check failure')
        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        return angle_flag
        # if angle_flag:
        #     stamp = rospy.get_rostime()
        #
        #     imu_msg.header.stamp = stamp
        #     imu_msg.header.frame_id = "base_link"
        #
        #     mag_msg.header.stamp = stamp
        #     mag_msg.header.frame_id = "base_link"
        #
        #     angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        #     qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
        #
        #     imu_msg.orientation.x = qua[0]
        #     imu_msg.orientation.y = qua[1]
        #     imu_msg.orientation.z = qua[2]
        #     imu_msg.orientation.w = qua[3]
        #
        #     imu_msg.angular_velocity.x = angularVelocity[0]
        #     imu_msg.angular_velocity.y = angularVelocity[1]
        #     imu_msg.angular_velocity.z = angularVelocity[2]
        #
        #     imu_msg.linear_acceleration.x = acceleration[0]
        #     imu_msg.linear_acceleration.y = acceleration[1]
        #     imu_msg.linear_acceleration.z = acceleration[2]
        #
        #     mag_msg.magnetic_field.x = magnetometer[0]
        #     mag_msg.magnetic_field.y = magnetometer[1]
        #     mag_msg.magnetic_field.z = magnetometer[2]
        #
        #     imu_pub.publish(imu_msg)
        #     mag_pub.publish(mag_msg)


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


class IMUDriverNode(Node):
    def __init__(self, port_name, baudrate):
        super().__init__('imu_driver_node')

        # 初始化IMU消息
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        # 创建IMU数据发布器
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        # 启动IMU驱动线程
        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port_name, baudrate, ))
        self.driver_thread.start()


        # 初始化订阅
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 机器人物理参数
        self.wheel_base = 0.174    # 车轮间距 (meters)
        self.wheel_radius = 0.026  # 轮子半径 (meters)

        # 控制参数
        self.kp = 7.0
        self.ki = 3.2
        self.kd = 1.5
        self.max_speed = 50.0  # 电机最大转速 (rad/s)
        self.max_angle = 90.0  # 
        self.min_angle = 0.0  # 

        self.imu_processor = IMUProcessor()
        self.kf_roll = KalmanFilter()
        self.kf_pitch = KalmanFilter()

        self.integral = 0.0
        self.prev_error = 0.0
        self.target_linear = 0.0    # 目标线速度 (m/s)
        self.target_angular = 0.0   # 目标角速度 (rad/s)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.target_roll = 0
        self.target_pitch = 0
        self.target_yaw = 0
        self.last_time = time.time()

    def driver_loop(self, port_name, baudrate):
        # 打开串口
        try:
            imu = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.5)
            if imu.isOpen():
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
            else:
                imu.open()
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            exit(0)

        # 连接Odrive
        try:
            self.odrive = odrive.find_any(timeout=10)
            self.get_logger().info("\033[32mOdrive Connected...\033[0m")
            self.odrive_setup()
            self.get_logger().info("\033[32mMotor Setup Successed...\033[0m")
            self.set_servo_angle(25, 25)
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            raise

        # 循环读取IMU数据
        while True:
            # 读取加速度计数据
            try:
                buff_count = imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_data()
                            self.balance()

    # def imu_data(self):
    #     accel_x, accel_y, accel_z = acceleration[0], acceleration[1], acceleration[2]  # struct.unpack('hhh', accel_raw)
    #     accel_scale = 16 / 32768.0
    #     accel_x, accel_y, accel_z = accel_x * accel_scale, accel_y * accel_scale, accel_z * accel_scale

    #     # 读取陀螺仪数据
    #     gyro_x, gyro_y, gyro_z = angularVelocity[0], angularVelocity[1], angularVelocity[
    #         2]  # struct.unpack('hhh', gyro_raw)
    #     gyro_scale = 2000 / 32768.0
    #     gyro_x, gyro_y, gyro_z = math.radians(gyro_x * gyro_scale), math.radians(gyro_y * gyro_scale), math.radians(
    #         gyro_z * gyro_scale)

    #     # 计算角速度
    #     dt = 0.01
    #     wx, wy, wz = gyro_x, gyro_y, gyro_z
    #     ax, ay, az = accel_x, accel_y, accel_z
    #     roll, pitch, yaw = self.compute_orientation(wx, wy, wz, ax, ay, az, dt)

    #     # 更新IMU消息
    #     self.imu_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.imu_msg.linear_acceleration.x = accel_x
    #     self.imu_msg.linear_acceleration.y = accel_y
    #     self.imu_msg.linear_acceleration.z = accel_z
    #     self.imu_msg.angular_velocity.x = gyro_x
    #     self.imu_msg.angular_velocity.y = gyro_y
    #     self.imu_msg.angular_velocity.z = gyro_z

    #     angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]

    #     qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

    #     self.imu_msg.orientation.x = qua[0]
    #     self.imu_msg.orientation.y = qua[1]
    #     self.imu_msg.orientation.z = qua[2]
    #     self.imu_msg.orientation.w = qua[3]

    #     # 发布IMU消息
    #     self.imu_pub.publish(self.imu_msg)

    # def compute_orientation(self, wx, wy, wz, ax, ay, az, dt):
    #     # 计算旋转矩阵
    #     Rx = np.array([[1, 0, 0],
    #                    [0, math.cos(ax), -math.sin(ax)],
    #                    [0, math.sin(ax), math.cos(ax)]])
    #     Ry = np.array([[math.cos(ay), 0, math.sin(ay)],
    #                    [0, 1, 0],
    #                    [-math.sin(ay), 0, math.cos(ay)]])
    #     Rz = np.array([[math.cos(wz), -math.sin(wz), 0],
    #                    [math.sin(wz), math.cos(wz), 0],
    #                    [0, 0, 1]])
    #     R = Rz.dot(Ry).dot(Rx)

    #     # 计算欧拉角
    #     roll = math.atan2(R[2][1], R[2][2])
    #     pitch = math.atan2(-R[2][0], math.sqrt(R[2][1] ** 2 + R[2][2] ** 2))
    #     yaw = math.atan2(R[1][0], R[0][0])

    #     return roll, pitch, yaw

    def imu_data(self):
        # 读取传感器数据
        accel_x, accel_y, accel_z = acceleration
        gyro_x, gyro_y, gyro_z = angularVelocity

        # 数据缩放
        accel_scale = 16 / 32768.0
        gyro_scale = 2000 / 32768.0
        accel_x, accel_y, accel_z = [a * accel_scale for a in (accel_x, accel_y, accel_z)]
        gyro_x, gyro_y, gyro_z = [math.radians(g * gyro_scale) for g in (gyro_x, gyro_y, gyro_z)]

        dt = 0.01  # 采样时间

        # 互补滤波计算角度
        roll, pitch = self.imu_processor.update([accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z])

        # 卡尔曼滤波计算角度
        self.roll = self.kf_roll.update(gyro_x, roll)
        self.pitch = self.kf_pitch.update(gyro_y, pitch)

        # 偏航角可以使用陀螺仪积分
        self.yaw = self.yaw + gyro_z * dt

        # 计算四元数
        qua = self.get_quaternion_from_euler(self.roll, self.pitch, self.yaw)

        # 更新IMU消息
        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        self.imu_pub.publish(self.imu_msg)

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

    def balance(self):
        # pitch: forward; roll, left-right
        (roll, pitch, yaw) = angle_degree
        # self.get_logger().info(f'Roll1: {roll:.10f}, Pitch1: {pitch:.10f}, Yaw1: {yaw:.10f}')
        self.get_logger().info(f'Roll2: {self.roll:.10f}, Pitch2: {self.pitch:.10f}, Yaw2: {self.yaw:.10f}')

        # # 安全保护（角度过大时停止）
        # if (abs(pitch) > 22):
        #     # self.odrive_idle()
        #     self.odrive.axis0.controller.input_vel = 0
        #     self.odrive.axis1.controller.input_vel = 0
        #     return

        # PID 控制计算
        current_time = time.time()
        dt = current_time - self.last_time
        error = self.target_pitch - self.pitch
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 更新状态
        self.prev_error = error
        self.last_time = current_time

        # 运动学转换
        linear_speed = self.target_linear / self.wheel_radius  # 转换为电机角速度
        angular_diff = (self.target_angular * self.wheel_base) / (2 * self.wheel_radius)

        # 计算左右轮速度
        left_speed = linear_speed - angular_diff - correction
        right_speed = linear_speed + angular_diff + correction

        # self.set_motor_speeds(left_speed, right_speed)

    def cmd_vel_callback(self, msg:Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        # self.get_logger().info(f"收到指令: 线速度={self.target_linear}m/s, 角速度={self.target_angular}rad/s")

    def set_motor_speeds(self, left_speed, right_speed, direction=1):
        # 限幅处理
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if direction:
            left_speed = -left_speed
            right_speed = -right_speed
        self.get_logger().info(f"Left={left_speed}, Right={right_speed}")
        try:
            # 根据实际电机方向可能需要调整符号
            self.odrive.axis0.controller.input_vel = left_speed  # 左电机
            self.odrive.axis1.controller.input_vel = right_speed  # 右电机
        except Exception as e:
            self.get_logger().error(f"电机控制失败: {str(e)}")

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


def main():
    # 初始化ROS 2节点
    rclpy.init()
    imu_node = IMUDriverNode("/dev/imu", 921600)

    # 运行ROS 2节点
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass

    # 停止ROS 2节点
    imu_node.odrive_idle()
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy, math, time, odrive
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from odrive.enums import *
from odrive.rich_text import RichText, Color, Style
import tf_transformations as tf

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
            self.set_servo_angle(25, 25)
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            raise

        # 机器人物理参数
        self.wheel_base = 0.174    # 车轮间距 (meters)
        self.wheel_radius = 0.026  # 轮子半径 (meters)

        # 控制参数
        self.kp = 40.0
        self.ki = 0.001
        self.kd = 1
        self.max_speed = 50.0  # 电机最大转速 (rad/s)
        self.max_angle = 90.0  # 
        self.min_angle = 0.0  # 
        self.integral = 0.0
        self.prev_error = 0.0
        self.target_linear = 0.0    # 目标线速度 (m/s)
        self.target_angular = 0.0   # 目标角速度 (rad/s)
        self.pitch = 0.0
        self.last_time = time.time()
        self.target_roll = 0
        self.target_pitch = 0

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

    def imu_callback(self, msg:Imu):
        self.last_callback_time = time.time()

        # 提取四元数（ROS2的Imu消息中四元数顺序为 x,y,z,w）
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        # 将四元数转换为欧拉角（Roll, Pitch, Yaw，单位为弧度）
        # pitch: forward; roll, left-right
        (roll, pitch, yaw) = tf.euler_from_quaternion(q)
        self.get_logger().info(f'Roll: {roll:.10f}, Pitch: {pitch:.10f}, Yaw: {yaw:.10f}')

        # PID 控制计算
        current_time = time.time()
        dt = current_time - self.last_time
        error = self.target_pitch - pitch
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

        self.set_motor_speeds(left_speed, right_speed)

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
        # self.get_logger().info(f"Left={left_speed}, Right={right_speed}")
        try:
            # 根据实际电机方向可能需要调整符号
            self.odrive.axis0.controller.input_vel = left_speed  # 左电机
            self.odrive.axis1.controller.input_vel = right_speed  # 右电机
        except Exception as e:
            self.get_logger().error(f"电机控制失败: {str(e)}")
        # pass

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

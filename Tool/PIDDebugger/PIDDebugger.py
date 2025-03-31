import sys, time, math, serial, struct
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# import tf_transformations as tf


# 外部定义的 output 函数
def output_function(control_value):
    """
    外部 output 函数，用于接收控制数据并输出。
    """
    pass

# 创建输入线程，负责从串口读取数据
class InputThread(QThread):
    input_signal = pyqtSignal(float)  # 用于传递实时输入数据

    def __init__(self, port, baudrate=9600):
        super().__init__()
        self.running = True
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        self.key = 0
        self.flag = 0
        self.buff = {}
        self.angularVelocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0, 0, 0]

    def run(self):
        """读取串口数据并通过信号传递"""
        # 打开串口
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            if self.ser.is_open:
                print(f"Serial port opened successfully")
            else:
                print(f"Serial port opening failure")
        except Exception as e:
            print(f"Serial port opening failure")
            exit(0)

        while self.running:
            try:
                buff_count = self.ser.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = self.ser.read(buff_count)
                    for i in range(0, buff_count):
                        tag = self.handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_data()
                            print(self.angle_radian)
                            self.input_signal.emit(self.angle_radian[1])

    def hex_to_short(self, raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))

    def check_sum(self, list_data, check_data):
        return sum(list_data) & 0xff == check_data

    def handle_serial_data(self, raw_data):
        angle_flag = False
        self.buff[self.key] = raw_data

        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            return
        # According to the judgment of the data length bit, the corresponding length data can be obtained
        if self.key < 11:
            return
        else:
            data_buff = list(self.buff.values())  # Get dictionary ownership value
            if self.buff[1] == 0x51:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                else:
                    print('0x51 Check failure')
            elif self.buff[1] == 0x52:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angularVelocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in
                                    range(0, 3)]
                else:
                    print('0x52 Check failure')
            elif self.buff[1] == 0x53:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angle_degree = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                    angle_flag = True
                else:
                    print('0x53 Check failure')
            elif self.buff[1] == 0x54:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.magnetometer = self.hex_to_short(data_buff[2:10])
                else:
                    print('0x54 Check failure')
            else:
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            return angle_flag

    def imu_data(self):
        accel_x, accel_y, accel_z = self.acceleration
        accel_scale = 16 / 32768.0
        accel_x, accel_y, accel_z = accel_x * accel_scale, accel_y * accel_scale, accel_z * accel_scale

        # 读取陀螺仪数据
        gyro_x, gyro_y, gyro_z = self.angularVelocity
        gyro_scale = 2000 / 32768.0
        gyro_x, gyro_y, gyro_z = math.radians(gyro_x * gyro_scale), math.radians(gyro_y * gyro_scale), math.radians(
            gyro_z * gyro_scale)

        # 计算角速度
        dt = 0.01
        wx, wy, wz = gyro_x, gyro_y, gyro_z
        ax, ay, az = accel_x, accel_y, accel_z
        roll, pitch, yaw = self.compute_orientation(wx, wy, wz, ax, ay, az, dt)
        self.angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
        self.qua = self.get_quaternion_from_euler(self.angle_radian[0], self.angle_radian[1], self.angle_radian[2])

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

    def stop(self):
        """停止线程并关闭串口"""
        self.running = False
        if self.ser:
            self.ser.close()
        self.quit()
        self.wait()


# PID控制器类
class PIDController:
    def __init__(self, kp, ki, kd, target):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.previous_error = 0
        self.integral = 0

    def compute(self, input_value):
        # 计算误差
        error = self.target - input_value
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        # PID公式
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output


# PID调试助手类
class PIDDebugger(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('PID调试助手')

        # 设置PID控制器初始参数
        self.kp = 35
        self.ki = 3
        self.kd = 1
        self.target = 0

        # 创建PID控制器
        self.controller = PIDController(self.kp, self.ki, self.kd, self.target)

        # 创建输入线程（指定串口和波特率）
        self.input_thread = InputThread(port='COM3', baudrate=921600)  # 这里修改为实际的串口号
        self.input_thread.input_signal.connect(self.handle_input_data)  # 连接信号与处理函数
        self.input_thread.start()  # 启动输入线程

        # 创建界面
        self.init_ui()

        # 存储数据用于绘图
        self.MAX_DATA_POINTS = 250  # 最大数据点数
        self.time_data = []
        self.input_data = []  # 存储输入数据（来自串口）

        # 设置定时器更新控制数据
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)  # 定时更新控制
        self.timer.start(10)  # 每10ms触发一次

        self.time = 0  # 时间变量，用于记录每次更新

    def init_ui(self):
        # 创建界面控件
        layout = QHBoxLayout()

        # 左侧参数布局
        param_layout = QVBoxLayout()
        param_layout.addWidget(QLabel('kp:'))
        self.kp_input = QLineEdit(str(self.kp))
        param_layout.addWidget(self.kp_input)

        param_layout.addWidget(QLabel('ki:'))
        self.ki_input = QLineEdit(str(self.ki))
        param_layout.addWidget(self.ki_input)

        param_layout.addWidget(QLabel('kd:'))
        self.kd_input = QLineEdit(str(self.kd))
        param_layout.addWidget(self.kd_input)

        param_layout.addWidget(QLabel('target:'))
        self.target_input = QLineEdit(str(self.target))
        param_layout.addWidget(self.target_input)

        # 创建按钮并连接到更新函数
        button = QPushButton('更新参数')
        button.clicked.connect(self.update_parameters)  # 连接信号
        param_layout.addWidget(button)

        layout.addLayout(param_layout)

        # 右侧曲线显示
        self.figure = plt.Figure(figsize=(5, 3), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

    def handle_input_data(self, input_value):
        """
        处理从输入线程发射的输入数据
        """
        self.input_value = input_value  # 更新输入数据

        # 将输入数据添加到数组中
        self.input_data.append(input_value)
        self.time_data.append(self.time)

        # 如果数据量超过最大限制，删除最旧的数据
        if len(self.input_data) > self.MAX_DATA_POINTS:
            self.input_data.pop(0)
            self.time_data.pop(0)

    def update_parameters(self):
        # 获取用户输入的参数并更新PID控制器
        self.kp = float(self.kp_input.text())
        self.ki = float(self.ki_input.text())
        self.kd = float(self.kd_input.text())
        self.target = float(self.target_input.text())
        self.controller = PIDController(self.kp, self.ki, self.kd, self.target)

    def update(self):
        # 计算PID控制输出
        if hasattr(self, 'input_value'):
            output_value = self.controller.compute(self.input_value)
            # 发送控制数据到output函数
            output_function(output_value)  # 将控制数据发送给外部的output函数
            # 更新数据
            self.time += 1

        # 更新图表
        self.plot()

    def plot(self):
        # 绘制输入数据曲线
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.plot(self.time_data, self.input_data, label='Input')
        ax.axhline(y=self.target, color='r', linestyle='--', label=f'Target: {self.target}')
        ax.set_xlabel('Time')
        ax.set_ylabel('Input')
        ax.set_ylim(-0.2, 0.2)
        ax.legend()

        self.canvas.draw()

    def closeEvent(self, event):
        """
        窗口关闭时停止输入线程
        """
        self.input_thread.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PIDDebugger()
    window.show()
    sys.exit(app.exec_())

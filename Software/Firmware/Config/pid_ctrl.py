import sys
import time
import threading
import odrive
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QSlider, QLabel, QWidget,
    QHBoxLayout, QRadioButton, QLineEdit, QPushButton
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from odrive.enums import *
from odrive.rich_text import RichText, Color, Style

class ODriveGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # 连接 ODrive
        try:
            self.odrive = odrive.find_any(timeout=10)
            print("Odrive Connected")
            self.odrive_setup()
            print("Motor Setup Successed")
            self.set_servo_angle(25, 25)
        except Exception as e:
            print(f"error: {str(e)}")
            raise

        # 初始化界面
        self.setWindowTitle("ODrive 运行控制")
        self.resize(800, 600)

        # 主窗口布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # Matplotlib 画布
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # 记录数据（分别存储位置和速度数据）
        self.x_pos_data, self.y_pos_data = [], []
        self.x_vel_data, self.y_vel_data = [], []
        self.start_time = time.time()
        self.max_data_points = 500

        # 记录当前模式
        self.current_mode = "velocity"

        # 创建模式切换
        self.create_mode_selection(layout)
        # 创建参数调节 UI
        self.create_sliders(layout)
        # 创建目标值输入框 & ODrive 运行控制按钮
        self.create_target_controls(layout)

        # 启动绘图线程
        self.running = True
        self.plot_thread = threading.Thread(target=self.update_plot, daemon=True)
        self.plot_thread.start()

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

    def create_mode_selection(self, layout):
        """ 创建模式选择按钮 """
        hbox = QHBoxLayout()

        self.mode_velocity = QRadioButton("速度模式")
        self.mode_position = QRadioButton("位置模式")
        self.mode_velocity.setChecked(True)  # 默认位置模式

        self.mode_velocity.toggled.connect(self.update_mode)
        self.mode_position.toggled.connect(self.update_mode)

        hbox.addWidget(self.mode_velocity)
        hbox.addWidget(self.mode_position)
        layout.addLayout(hbox)

    def update_mode(self):
        """ 更新控制模式，并修改采样对象 """
        if self.mode_position.isChecked():
            self.odrive.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL  # 位置模式
            self.current_mode = "position"
            print("切换到 **位置模式**")
        elif self.mode_velocity.isChecked():
            self.odrive.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL  # 速度模式
            self.current_mode = "velocity"
            print("切换到 **速度模式**")

    def create_target_controls(self, layout):
        """ 创建目标值输入框 和 ODrive 运行按钮 """
        hbox = QHBoxLayout()

        # 目标值输入
        self.target_label = QLabel("目标值:")
        self.target_input = QLineEdit()
        self.target_button = QPushButton("运行")
        self.target_button.clicked.connect(self.run_odrive)

        # ODrive 运行控制按钮
        self.odrive_control_button = QPushButton("停止")
        self.odrive_control_button.setCheckable(True)  # 允许按钮保持按下状态
        self.odrive_control_button.setChecked(True)
        self.odrive_control_button.clicked.connect(self.toggle_odrive_state)
        self.odrive_running = True  # 初始状态

        hbox.addWidget(self.target_label)
        hbox.addWidget(self.target_input)
        hbox.addWidget(self.target_button)
        hbox.addWidget(self.odrive_control_button)
        layout.addLayout(hbox)

    def run_odrive(self):
        """ 运行 ODrive 目标值 """
        try:
            target_value = float(self.target_input.text())
            if self.current_mode == "position":
                self.odrive.axis0.controller.input_pos = target_value
                print(f"设置目标 **位置**: {target_value}")
            elif self.current_mode == "velocity":
                self.odrive.axis0.controller.input_vel = target_value
                print(f"设置目标 **速度**: {target_value}")

        except ValueError:
            print("输入无效，请输入数字！")

    def toggle_odrive_state(self):
        """ 启动/停止 ODrive CLOSED_LOOP_CONTROL """
        if self.odrive_running:
            self.odrive.axis0.requested_state = AxisState.IDLE
            self.odrive_control_button.setText("启动")
            print("ODrive 退出 **CLOSED_LOOP_CONTROL**")
        else:
            self.odrive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.odrive_control_button.setText("停止")
            print("ODrive 进入 **CLOSED_LOOP_CONTROL**")

        # 切换状态
        self.odrive_running = not self.odrive_running

    def create_sliders(self, layout):
        """ 创建滑块控件 """
        self.sliders = []
        self.labels = []

        # 定义参数
        params = [
            ("pos_gain", 0.0, 100.0, self.odrive.axis0.controller.config.pos_gain),
            ("vel_gain", 0.0, 2.5, self.odrive.axis0.controller.config.vel_gain),
            ("vel_integrator_gain", 0.0, 10.0, self.odrive.axis0.controller.config.vel_integrator_gain),
        ]

        for i, (name, min_val, max_val, default) in enumerate(params):
            hbox = QHBoxLayout()

            label = QLabel(f"{name}: {default:.2f}")
            self.labels.append(label)
            hbox.addWidget(label)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(int(min_val * 100))
            slider.setMaximum(int(max_val * 100))
            slider.setValue(int(default * 100))
            slider.valueChanged.connect(lambda value, n=i: self.update_param(n, value / 100.0))
            self.sliders.append(slider)
            hbox.addWidget(slider)

            layout.addLayout(hbox)

    def update_param(self, index, value):
        """ 调整参数 """
        params = ["pos_gain", "vel_gain", "vel_integrator_gain"]
        param_name = params[index]

        if param_name == "pos_gain":
            self.odrive.axis0.controller.config.pos_gain = value
        elif param_name == "vel_gain":
            self.odrive.axis0.controller.config.vel_gain = value
        elif param_name == "vel_integrator_gain":
            self.odrive.axis0.controller.config.vel_integrator_gain = value

        self.labels[index].setText(f"{param_name}: {value:.2f}")
        print(f"更新参数: {param_name} = {value:.2f}")

    def update_plot(self):
        """ 实时更新 Matplotlib 图像 """
        while self.running:
            elapsed_time = time.time() - self.start_time

            if self.odrive_running:
                if self.current_mode == "position":
                    value = self.odrive.axis0.encoder.pos_estimate
                    # print(f"pos: {value:.2f}")
                    self.x_pos_data.append(elapsed_time)
                    self.y_pos_data.append(value)
                    if len(self.x_pos_data) > self.max_data_points:
                        self.x_pos_data.pop(0)
                        self.y_pos_data.pop(0)
                    self.ax.clear()
                    self.ax.plot(self.x_pos_data, self.y_pos_data, 'r-')
                else:
                    value = self.odrive.axis0.encoder.vel_estimate
                    # print(f"vel: {value:.2f}")
                    self.x_vel_data.append(elapsed_time)
                    self.y_vel_data.append(value)
                    if len(self.x_vel_data) > self.max_data_points:
                        self.x_vel_data.pop(0)
                        self.y_vel_data.pop(0)
                    self.ax.clear()
                    self.ax.plot(self.x_vel_data, self.y_vel_data, 'b-')
                    self.ax.set_ylim(-5, 5)  # 速度模式固定 Y 轴范围

                self.canvas.draw()

            time.sleep(0.001)

    def closeEvent(self, event):
        """窗口关闭时，确保 ODrive 停止"""
        self.odrive.axis0.controller.input_vel = 0
        self.odrive.axis0.requested_state = AxisState.IDLE
        event.accept()  # 允许窗口关闭


# 运行 PyQt 应用
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ODriveGUI()
    window.show()
    sys.exit(app.exec())

import sys, time, threading
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QSlider, QLabel, QWidget,
    QHBoxLayout, QRadioButton, QLineEdit, QPushButton, QButtonGroup
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from odrive.enums import *
from PyQt5.QtGui import QIcon
from serial import Serial
from collections import deque

class PIDConfig(QMainWindow):
    def __init__(self, port_name="COM5", baudrate=512000):
        super().__init__()
        self.motor = None

        # 连接 motor
        self.motor_init(port_name=port_name, baudrate=baudrate)

        # 初始化界面
        self.setWindowTitle("PID Config")
        self.setWindowIcon(QIcon('icons/config.svg'))
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
        self.plot_num = 1000
        self.pos_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        self.vel_data = deque([0.0]*self.plot_num, maxlen=self.plot_num)
        # self.start_time = time.time()

        # 记录当前模式
        self.current_mode = "velocity"
        # 记录当前轴
        self.current_axis = "axis0"

        # 创建motor切换
        self.create_motor_selection(layout)
        # 创建模式切换
        self.create_mode_selection(layout)
        # 创建参数调节 UI
        self.create_sliders(layout)
        # 创建目标值输入框 & motor 运行控制按钮
        self.create_target_controls(layout)

        # 启动绘图线程
        self.line, = self.ax.plot([], [], 'r-' if self.current_mode == 'position' else 'b-')
        self.ax.set_xlim(0, self.plot_num)
        # self.ax.set_ylim(-10, 10)
        self.running = True
        self.plot_thread = threading.Thread(target=self.update_plot, daemon=True)
        self.plot_thread.start()

    def motor_init(self, port_name, baudrate):
        # 连接 Motor
        try:
            self.motor = Serial(port=port_name, baudrate=baudrate, timeout=0.1)
            print("\033[32mMotor serial port connection established...\033[0m")
            self.motor_setup()
            print("\033[32mMotor Setup Successed...\033[0m")
        except Exception as e:
            print("\033[31mSerial port opening failure\033[0m")
            if self.motor:
                self.motor.close()

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

    def create_motor_selection(self, layout):
        """ 创建motor选择按钮 """
        hbox = QHBoxLayout()

        self.mode_axis0 = QRadioButton("axis0")
        self.mode_axis1 = QRadioButton("axis1")
        self.mode_axis0.setChecked(True)  # 默认位置模式

        self.motor_button_group = QButtonGroup(self)
        self.motor_button_group.addButton(self.mode_axis0)
        self.motor_button_group.addButton(self.mode_axis1)

        self.mode_axis0.toggled.connect(self.update_axis)
        self.mode_axis1.toggled.connect(self.update_axis)

        hbox.addWidget(self.mode_axis0)
        hbox.addWidget(self.mode_axis1)
        layout.addLayout(hbox)

    def create_mode_selection(self, layout):
        """ 创建模式选择按钮 """
        hbox = QHBoxLayout()

        self.mode_velocity = QRadioButton("Speed")
        self.mode_position = QRadioButton("Position")
        self.mode_velocity.setChecked(True)  # 默认位置模式

        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.addButton(self.mode_velocity)
        self.mode_button_group.addButton(self.mode_position)

        self.mode_velocity.toggled.connect(self.update_mode)
        self.mode_position.toggled.connect(self.update_mode)

        hbox.addWidget(self.mode_velocity)
        hbox.addWidget(self.mode_position)
        layout.addLayout(hbox)

    def update_axis(self):
        """ 更新轴 """
        if self.motor is None:
            return

        self.motor_send_cmd(f'w {self.current_axis}.controller.input_vel 0.0')
        self.motor_send_cmd(f'w {self.current_axis}.requested_state {AxisState.IDLE}')

        if self.mode_axis0.isChecked():
            self.current_axis = "axis0"
            print("切换到 **axis0**")
        elif self.mode_axis1.isChecked():
            self.current_axis = "axis1"
            print("切换到 **axis1**")
        self.motor_control_button.setText("Start")
        self.motor_control_button.setChecked(False)
        self.motor_running = False

    def update_mode(self):
        """ 更新控制模式，并修改采样对象 """
        if self.motor is None:
            return

        if self.mode_position.isChecked():
            self.current_mode = "position"
            # self.ax.set_ylim(min(self.pos_data) - 1, max(self.pos_data) + 1)
            print("切换到 **位置模式**")
        elif self.mode_velocity.isChecked():
            self.current_mode = "velocity"
            # self.ax.set_ylim(-10, 10)
            print("切换到 **速度模式**")

    def create_target_controls(self, layout):
        """ 创建目标值输入框 和 motor 运行按钮 """
        hbox = QHBoxLayout()

        # 目标值输入
        self.target_label = QLabel("Target:")
        self.target_input = QLineEdit()
        self.target_button = QPushButton("Run")
        self.target_button.clicked.connect(self.run_motor)

        # motor 运行控制按钮
        self.motor_control_button = QPushButton("Start")
        self.motor_control_button.setCheckable(True)  # 允许按钮保持按下状态
        self.motor_control_button.clicked.connect(self.toggle_motor_state)
        self.motor_running = False  # 初始状态

        hbox.addWidget(self.target_label)
        hbox.addWidget(self.target_input)
        hbox.addWidget(self.target_button)
        hbox.addWidget(self.motor_control_button)
        layout.addLayout(hbox)

    def run_motor(self):
        """ 运行 motor 目标值 """
        if self.motor is None:
            return

        try:
            target_value = float(self.target_input.text())
            if self.current_mode == "position":
                self.motor_send_cmd(f'w {self.current_axis}.controller.input_pos {target_value}')
                print(f"设置目标 {self.current_axis} **位置**: {target_value}")
            elif self.current_mode == "velocity":
                self.motor_send_cmd(f'w {self.current_axis}.controller.input_vel {target_value}')
                print(f"设置目标 {self.current_axis} **速度**: {target_value}")
        except ValueError:
            print("输入无效，请输入数字！")

    def toggle_motor_state(self):
        """ 启动/停止 motor CLOSED_LOOP_CONTROL """
        if self.motor is None:
            return

        if self.motor_running:
            self.motor_send_cmd(f'w {self.current_axis}.requested_state {AxisState.IDLE}')
            self.motor_control_button.setText("启动")
            print("motor 退出 **CLOSED_LOOP_CONTROL**")
        else:
            self.motor_send_cmd(f'w {self.current_axis}.requested_state {AxisState.CLOSED_LOOP_CONTROL}')
            self.motor_control_button.setText("停止")
            print("motor 进入 **CLOSED_LOOP_CONTROL**")

        # 切换状态
        self.motor_running = not self.motor_running

    def create_sliders(self, layout):
        """ 创建滑块控件 """
        if self.motor is None:
            return

        self.sliders = []
        self.labels = []

        pos_gain = float(self.motor_send_cmd(f'r {self.current_axis}.controller.config.pos_gain'))
        vel_gain = float(self.motor_send_cmd(f'r {self.current_axis}.controller.config.vel_gain'))
        vel_integrator_gain = float(self.motor_send_cmd(f'r {self.current_axis}.controller.config.vel_integrator_gain'))

        params = [
            ("pos_gain", 0.0, 100.0, pos_gain),
            ("vel_gain", 0.0, 0.5, vel_gain),
            ("vel_integrator_gain", 0.0, 10.0, vel_integrator_gain),
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
        if self.motor is None:
            return

        params = ["pos_gain", "vel_gain", "vel_integrator_gain"]
        param_name = params[index]

        # 更新参数值
        if param_name == "pos_gain":
            self.motor_send_cmd(f'w {self.current_axis}.controller.config.pos_gain {value:.2f}')
        elif param_name == "vel_gain":
            self.motor_send_cmd(f'w {self.current_axis}.controller.config.vel_gain {value:.2f}')
        elif param_name == "vel_integrator_gain":
            self.motor_send_cmd(f'w {self.current_axis}.controller.config.vel_integrator_gain {value:.2f}')

        self.labels[index].setText(f"{param_name}: {value:.2f}")
        print(f"更新参数: {param_name} = {value:.2f}")

    def update_plot(self):
        """ 实时更新 Matplotlib 图像（高效方式） """
        while self.running and self.motor:
            try:
                if self.motor_running:
                    value = float(self.motor_send_cmd(
                        f'r {self.current_axis}.encoder.pos_estimate' if self.current_mode == "position"
                        else f'r {self.current_axis}.encoder.vel_estimate'
                    ))
                    if self.current_mode == "position":
                        self.pos_data.append(value)
                        self.line.set_xdata(range(len(self.pos_data)))
                        self.line.set_ydata(self.pos_data)
                    else:
                        self.vel_data.append(value)
                        self.line.set_xdata(range(len(self.vel_data)))
                        self.line.set_ydata(self.vel_data)

                    self.ax.relim()           # 重新计算坐标轴范围（可选）
                    self.ax.autoscale_view()  # 自动缩放视图（可选）
                    self.canvas.draw()
            except Exception as e:
                print(f"{e}")
            time.sleep(0.01)

    def motor_idle(self):
        self.motor_send_cmd(f'w axis0.requested_state {AxisState.IDLE}')
        self.motor_send_cmd(f'w axis1.requested_state {AxisState.IDLE}')
        self.motor_send_cmd('v 0 0.0')
        self.motor_send_cmd('v 1 0.0')

    def closeEvent(self, event):
        """窗口关闭时，确保 motor 停止"""
        if self.motor:
            self.motor_idle()
            self.motor.close()
        event.accept()  # 允许窗口关闭


# 运行 PyQt 应用
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDConfig()
    window.show()
    sys.exit(app.exec())

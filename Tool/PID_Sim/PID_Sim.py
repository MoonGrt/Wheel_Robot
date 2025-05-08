import sys
from collections import deque
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QSizePolicy, QCheckBox
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0
        self.clear()

    def clear(self):
        self.integral = 0
        self.prev_error = 0

    def compute(self, feedback, dt):
        error = self.setpoint - feedback
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class MplCanvas(FigureCanvas):
    def __init__(self, plot_num=200, y_min=-10, y_max=120, parent=None):
        self.plot_num = plot_num
        self.target = deque([0.0] * plot_num, maxlen=plot_num)
        self.current = deque([0.0] * plot_num, maxlen=plot_num)
        self.output = deque([0.0] * plot_num, maxlen=plot_num)
        self.error = deque([0.0] * plot_num, maxlen=plot_num)

        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)

        self.y_min = y_min
        self.y_max = y_max
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.grid(True)
        self.line_target, = self.ax.plot([], [], 'r--', label='Target')
        self.line_current, = self.ax.plot([], [], 'g-', label='Current')
        self.line_output, = self.ax.plot([], [], 'y-', label='Output')
        self.line_error, = self.ax.plot([], [], 'b-', label='Error')
        self.ax.legend()
        self.x = list(range(plot_num))

    def update_plot(self, target=True, current=True, output=True, error=True):
        # 清空所有曲线的数据
        self.line_target.set_data([], [])
        self.line_current.set_data([], [])
        self.line_output.set_data([], [])
        self.line_error.set_data([], [])
        if target:
            self.line_target.set_data(self.x, list(self.target))
        if current:
            self.line_current.set_data(self.x, list(self.current))
        if output:
            self.line_output.set_data(self.x, list(self.output))
        if error:
            self.line_error.set_data(self.x, list(self.error))
        self.ax.set_ylim(min(min(self.current) - 1, self.y_min), max(max(self.current) + 1, self.y_max))
        self.ax.set_xlim(0, self.plot_num)
        self.draw()


class PIDSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID")
        self.resize(1000, 700)

        self.pid = PID()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        ctrl_layout1 = QHBoxLayout()
        ctrl_layout2 = QHBoxLayout()
        ctrl_layout3 = QHBoxLayout()

        self.sp_slider, self.sp_label = self.create_slider(0, 100, 80, "Target", self.update_params)
        self.init_slider, self.init_label = self.create_slider(0, 100, 0, "Init", self.update_params)

        self.sensor_delay_steps_slider, self.sensor_delay_steps_label = self.create_slider(1, 100, 10, "Sensor Delay", self.update_params)
        self.actuator_delay_steps_slider, self.actuator_delay_steps_label = self.create_slider(1, 100, 10, "Actuator Delay", self.update_params)
        self.response_rate_slider, self.response_rate_label = self.create_slider(0, 100, 50, "Response Rate", self.update_params, scale=0.0001)

        self.kp_slider, self.kp_label = self.create_slider(0, 200, 10, "KP", self.update_params, scale=0.1)
        self.ki_slider, self.ki_label = self.create_slider(0, 1000, 10, "KI", self.update_params, scale=1)
        self.kd_slider, self.kd_label = self.create_slider(-200, 200, 1, "KD", self.update_params, scale=1)

        for label, slider in zip(
            [self.sp_label, self.init_label],
            [self.sp_slider, self.init_slider]
        ):
            col = QVBoxLayout()
            col.addWidget(label)
            col.addWidget(slider)
            ctrl_layout1.addLayout(col)
        layout.addLayout(ctrl_layout1)

        for label, slider in zip(
            [self.sensor_delay_steps_label, self.actuator_delay_steps_label, self.response_rate_label],
            [self.sensor_delay_steps_slider, self.actuator_delay_steps_slider, self.response_rate_slider]
        ):
            col = QVBoxLayout()
            col.addWidget(label)
            col.addWidget(slider)
            ctrl_layout2.addLayout(col)
        layout.addLayout(ctrl_layout2)

        for label, slider in zip(
            [self.kp_label, self.ki_label, self.kd_label],
            [self.kp_slider, self.ki_slider, self.kd_slider]
        ):
            col = QVBoxLayout()
            col.addWidget(label)
            col.addWidget(slider)
            ctrl_layout3.addLayout(col)
        layout.addLayout(ctrl_layout3)

        # 添加四个复选框让用户选择绘制的曲线
        self.target_checkbox = QCheckBox("Target")
        self.current_checkbox = QCheckBox("Current")
        self.output_checkbox = QCheckBox("Output")
        self.error_checkbox = QCheckBox("Error")

        # 默认情况下勾选所有复选框
        self.target_checkbox.setChecked(True)
        self.current_checkbox.setChecked(True)
        self.output_checkbox.setChecked(False)
        self.error_checkbox.setChecked(False)

        # 将复选框添加到布局中
        checkbox_layout = QHBoxLayout()
        checkbox_layout.addWidget(self.target_checkbox)
        checkbox_layout.addWidget(self.current_checkbox)
        checkbox_layout.addWidget(self.output_checkbox)
        checkbox_layout.addWidget(self.error_checkbox)

        # 添加重置按钮
        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(self.reset)
        checkbox_layout.addWidget(reset_btn)
        layout.addLayout(checkbox_layout)

        # 创建canvas并添加到布局中
        self.canvas = MplCanvas(plot_num=2000)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

        # 连接复选框的状态变化事件
        self.target_checkbox.stateChanged.connect(self.update_plot)
        self.current_checkbox.stateChanged.connect(self.update_plot)
        self.output_checkbox.stateChanged.connect(self.update_plot)
        self.error_checkbox.stateChanged.connect(self.update_plot)

        self.update_params()
        self.run_simulation()

    def create_slider(self, minval, maxval, initval, name, callback, scale=1.0):
        if name == "Response Rate":
            label = QLabel(f"{name}: {initval * scale:.4f}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(minval, maxval)
            slider.setValue(initval)
            slider.valueChanged.connect(lambda: callback())
            slider.valueChanged.connect(lambda val: label.setText(f"{name}: {val * scale:.4f}"))
        else:
            label = QLabel(f"{name}: {initval * scale:.2f}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(minval, maxval)
            slider.setValue(initval)
            slider.valueChanged.connect(lambda: callback())
            slider.valueChanged.connect(lambda val: label.setText(f"{name}: {val * scale:.2f}"))
        return slider, label

    def update_plot(self):
        """根据复选框的状态更新绘图"""
        self.canvas.update_plot(self.target_checkbox.isChecked(), self.current_checkbox.isChecked(),
                                self.output_checkbox.isChecked(), self.error_checkbox.isChecked())

    def update_params(self):
        self.pid.setpoint = self.sp_slider.value()
        self.pid.kp = self.kp_slider.value() * 0.1
        self.pid.ki = self.ki_slider.value() * 0.01
        self.pid.kd = self.kd_slider.value() * 0.01
        self.run_simulation()

    def reset(self):
        self.pid.clear()
        self.canvas.target = deque([0.0] * self.canvas.plot_num, maxlen=self.canvas.plot_num)
        self.canvas.output = deque([0.0] * self.canvas.plot_num, maxlen=self.canvas.plot_num)
        self.canvas.error = deque([0.0] * self.canvas.plot_num, maxlen=self.canvas.plot_num)
        self.canvas.update_plot()

    def run_simulation(self):
        dt = 0.01
        steps = self.canvas.plot_num
        init_value = self.init_slider.value()
        value = init_value
        self.pid.clear()

        self.canvas.target.clear()
        self.canvas.output.clear()
        self.canvas.error.clear()

        sensor_delay_steps = self.sensor_delay_steps_slider.value()
        actuator_delay_steps = self.actuator_delay_steps_slider.value()
        feedback_buffer = deque([value] * sensor_delay_steps, maxlen=sensor_delay_steps)
        actuator_buffer = deque([0.0] * actuator_delay_steps, maxlen=actuator_delay_steps)

        response_rate = self.response_rate_slider.value() * 0.0001

        for _ in range(steps):
            # 1. 获取延迟后的反馈值
            delayed_feedback = feedback_buffer[0]
            # 2. PID 计算控制输出
            output = self.pid.compute(delayed_feedback, dt)
            # 3. 存入执行器延迟缓冲
            actuator_buffer.append(output)
            delayed_output = actuator_buffer[0]

            # 4. 系统响应（使用延迟后的输出）
            value += delayed_output * response_rate

            # 5. 更新反馈值缓存
            feedback_buffer.append(value)

            # 6. 更新图表数据
            self.canvas.target.append(self.pid.setpoint)
            self.canvas.current.append(value)
            self.canvas.output.append(delayed_output)  # 绘图显示延迟后的输出
            self.canvas.error.append(self.pid.setpoint - value)

        self.canvas.update_plot(
            self.target_checkbox.isChecked(),
            self.current_checkbox.isChecked(),
            self.output_checkbox.isChecked(),
            self.error_checkbox.isChecked()
        )




if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = PIDSimulator()
    win.show()
    sys.exit(app.exec_())

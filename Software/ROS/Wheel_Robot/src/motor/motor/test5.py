import sys
import time
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from serial import Serial


class MotorDataThread(QThread):
    data_updated = pyqtSignal(float, float, float, float)

    def __init__(self):
        super().__init__()
        self.running = True
        self.motor = Serial(port='/dev/motor', baudrate=512000, timeout=0.1)

    def motor_send_cmd(self, cmd):
        try:
            self.motor.write((cmd + '\n').encode())
            return self.motor.readline().decode().strip()
        except Exception as e:
            print(f"Serial error: {e}")
            return ""

    def run(self):
        rclpy.init()
        node = Node("motor_control_gui_node")
        self.last_time = time.time()

        while self.running:
            try:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                print(f"dt: {dt:.4f}")

                pos_data = self.motor_send_cmd("r p")
                vel_data = self.motor_send_cmd("r v")
                motor_pos0, motor_pos1 = map(float, pos_data.split())
                motor_vel0, motor_vel1 = map(float, vel_data.split())

                # 发送停止指令
                self.motor.write('v 0 0.0\nv 1 0.0\n'.encode())

                # 发射信号更新 GUI
                self.data_updated.emit(motor_pos0, motor_pos1, motor_vel0, motor_vel1)

            except Exception as e:
                print(f"Error: {e}")

        node.destroy_node()
        rclpy.shutdown()


class MotorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Realtime Display")

        self.label_pos0 = QLabel("Motor Pos0: --")
        self.label_pos1 = QLabel("Motor Pos1: --")
        self.label_vel0 = QLabel("Motor Vel0: --")
        self.label_vel1 = QLabel("Motor Vel1: --")

        layout = QVBoxLayout()
        layout.addWidget(self.label_pos0)
        layout.addWidget(self.label_pos1)
        layout.addWidget(self.label_vel0)
        layout.addWidget(self.label_vel1)
        self.setLayout(layout)

        # 启动后台线程
        self.thread = MotorDataThread()
        self.thread.data_updated.connect(self.update_display)
        self.thread.start()

    def update_display(self, pos0, pos1, vel0, vel1):
        self.label_pos0.setText(f"Motor Pos0: {pos0:.2f}")
        self.label_pos1.setText(f"Motor Pos1: {pos1:.2f}")
        self.label_vel0.setText(f"Motor Vel0: {vel0:.2f}")
        self.label_vel1.setText(f"Motor Vel1: {vel1:.2f}")

    def closeEvent(self, event):
        self.thread.running = False
        self.thread.quit()
        self.thread.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = MotorGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

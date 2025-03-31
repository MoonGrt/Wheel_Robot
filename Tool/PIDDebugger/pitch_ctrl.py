import numpy as np
from PyQt5.QtWidgets import QApplication
from PIDDebugger import PIDDebugger

def input_function(time):
    """
    用户自定义的输入函数。这里我们模拟一个正弦波的输入。
    根据需要，用户可以自行修改此函数来获取不同的输入数据。
    """
    return 10 * np.sin(time * 0.1)

def output_function(control_value):
    """
    用户自定义的输出函数。这里我们将控制数据打印到控制台。
    用户可以根据需要修改此函数，例如发送数据到硬件。
    """
    print(f"控制数据输出: {control_value}")

# main.py

def main():
    # 在这里创建 PIDDebugger 并传入自定义的 input 和 output 函数
    app = QApplication([])  # 初始化 Qt 应用
    pid_debugger = PIDDebugger(input_function, output_function)  # 传入自定义函数
    pid_debugger.show()  # 显示窗口
    app.exec_()  # 进入 Qt 主循环

if __name__ == '__main__':
    main()


import sys
import odrive
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, 
    QFormLayout, QLineEdit, QLabel, QScrollArea, QGroupBox
)
from PyQt5.QtCore import Qt

class ODriveReader(QWidget):
    def __init__(self):
        super().__init__()
        self.odrv0 = None
        self.param_inputs = {}  # 存储参数路径和对应的 QLineEdit

        self.initUI()

    def initUI(self):
        self.setWindowTitle('ODrive 参数配置器')

        main_layout = QVBoxLayout()

        # 读取参数按钮
        self.button_read = QPushButton('读取ODrive参数', self)
        self.button_read.clicked.connect(self.read_odrive_params)
        main_layout.addWidget(self.button_read)

        # 应用修改按钮
        self.button_apply = QPushButton('应用修改', self)
        self.button_apply.clicked.connect(self.apply_changes)
        main_layout.addWidget(self.button_apply)

        # 滚动区域容纳大量参数项
        self.scroll = QScrollArea(self)
        self.scroll.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.form_layout = QFormLayout(self.scroll_content)
        self.scroll.setWidget(self.scroll_content)
        main_layout.addWidget(self.scroll)

        self.setLayout(main_layout)
        self.setGeometry(300, 300, 700, 500)

    def read_odrive_params(self):
        self.odrv0 = odrive.find_any()
        self.param_inputs.clear()
        # 清空旧的布局项
        while self.form_layout.count():
            item = self.form_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.generate_param_fields(self.odrv0)

    def generate_param_fields(self, obj, path=''):
        for attr in dir(obj):
            if attr.startswith('_') or attr in ['parent', 'logger', 'event_loop']:
                continue
            try:
                value = getattr(obj, attr)

                if callable(value):
                    continue

                full_path = f"{path}.{attr}" if path else attr

                if hasattr(value, '__dict__'):
                    self.generate_param_fields(value, full_path)
                else:
                    # 创建 QLineEdit 显示并允许编辑参数
                    line_edit = QLineEdit(str(value))
                    self.param_inputs[full_path] = line_edit
                    self.form_layout.addRow(QLabel(full_path), line_edit)

            except Exception:
                continue

    def apply_changes(self):
        for path, input_widget in self.param_inputs.items():
            try:
                new_value = input_widget.text()
                self.set_odrive_param(self.odrv0, path, new_value)
            except Exception as e:
                print(f"设置失败: {path} = {new_value} - 错误: {e}")

    def set_odrive_param(self, root, path, value_str):
        # 解析路径，逐层找到最终属性的父对象和属性名
        parts = path.split('.')
        target = root
        for part in parts[:-1]:
            target = getattr(target, part)
        attr = parts[-1]

        # 将字符串转为正确的类型（尝试 int, float）
        old_value = getattr(target, attr)
        try:
            if isinstance(old_value, int):
                new_value = int(value_str)
            elif isinstance(old_value, float):
                new_value = float(value_str)
            else:
                new_value = value_str
            setattr(target, attr, new_value)
        except Exception as e:
            raise ValueError(f"类型转换失败: {e}")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ODriveReader()
    ex.show()
    sys.exit(app.exec_())

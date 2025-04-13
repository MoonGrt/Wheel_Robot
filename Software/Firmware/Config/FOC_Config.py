import sys, odrive
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QFormLayout, QLineEdit, QLabel, QScrollArea,
    QTabWidget, QPushButton, QHBoxLayout, QHBoxLayout,
    QRadioButton, QButtonGroup
)
from PyQt5.QtGui import QIcon
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt


class ODriveReader(QWidget):
    def __init__(self):
        super().__init__()
        self.odrv0 = None
        self.param_inputs = {}

        self.initUI()

    def initUI(self):
        self.setWindowTitle('FOC Config')
        self.setWindowIcon(QIcon('icons/config.svg'))
        self.resize(900, 600)

        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()
        main_layout = QHBoxLayout()

        # 左侧布局（参数读取）
        top_controls = QHBoxLayout()
        bottom_controls = QHBoxLayout()

        # 添加读写按钮
        self.button_read = QPushButton('Read', self)
        self.button_read.clicked.connect(self.read_odrive_params)
        top_controls.addWidget(self.button_read)
        self.button_apply = QPushButton('Write', self)
        self.button_apply.clicked.connect(self.apply_changes)
        top_controls.addWidget(self.button_apply)
        # 添加模式选择按钮
        self.button_calibrate = QPushButton('Calibrate', self)
        self.button_calibrate.clicked.connect(self.calibrate)
        bottom_controls.addWidget(self.button_calibrate)
        self.button_reboot = QPushButton('Reboot', self)
        self.button_reboot.clicked.connect(self.reboot)
        bottom_controls.addWidget(self.button_reboot)

        # 模式选择按钮组
        self.mode_group = QButtonGroup(self)
        self.radio_full = QRadioButton("Full")
        self.radio_half = QRadioButton("Half")
        self.radio_less = QRadioButton("Less")
        self.radio_half.setChecked(True)

        self.mode_group.addButton(self.radio_full)
        self.mode_group.addButton(self.radio_half)
        self.mode_group.addButton(self.radio_less)

        top_controls.addWidget(self.radio_full, 3)
        top_controls.addWidget(self.radio_half, 3)
        top_controls.addWidget(self.radio_less, 2)

        # 顶层标签页（第一级）
        self.top_tabs = QTabWidget(self)

        left_layout.addLayout(top_controls)
        left_layout.addWidget(self.top_tabs)
        left_layout.addLayout(bottom_controls)

        # 右侧布局（图形）
        self.current_figure, self.current_ax = plt.subplots()
        self.current_canvas = FigureCanvas(self.current_figure)
        right_layout.addWidget(self.current_canvas)

        self.voltage_figure, self.voltage_ax = plt.subplots()
        self.voltage_canvas = FigureCanvas(self.voltage_figure)
        right_layout.addWidget(self.voltage_canvas)


        # 整体布局
        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 1)
        self.setLayout(main_layout)

    def get_allowed_prefixes(self):
        if self.radio_full.isChecked():
            return None  # 全部
        elif self.radio_half.isChecked():
            return ['axis0', 'axis1', 'can', 'config', 'servo0', 'servo1', 'servo2', 'servo3']
        elif self.radio_less.isChecked():
            return ['axis0', 'axis1']
        return None

    def calibrate(self):
        self.odrv0.axis0.requested_state = 3
        self.odrv0.axis1.requested_state = 3

    def reboot(self):
        self.odrv0.reboot()

    def read_odrive_params(self):
        self.odrv0 = odrive.find_any()
        self.param_inputs.clear()
        self.top_tabs.clear()

        allowed_prefixes = self.get_allowed_prefixes()

        all_params = {}
        self.collect_params(self.odrv0, all_params, allowed_prefixes=allowed_prefixes)

        # 根据前两级路径分组
        grouped = {}
        for path, value in all_params.items():
            parts = path.split('.')
            if len(parts) >= 2:
                group1 = parts[0]
                group2 = parts[1]
            elif len(parts) == 1:
                group1 = parts[0]
                group2 = "_"
            else:
                continue

            grouped.setdefault(group1, {}).setdefault(group2, []).append((path, value))

        # 确定当前模式
        if self.radio_full.isChecked():
            allowed_top = None  # 显示全部
        elif self.radio_half.isChecked():
            allowed_top = ['axis0', 'axis1', 'can', 'config', 'servo0', 'servo1', 'servo2', 'servo3']
        elif self.radio_less.isChecked():
            allowed_top = ['axis0', 'axis1']
        else:
            allowed_top = None

        # 构建页面
        for group1, subgroups in grouped.items():
            if allowed_top is not None and group1 not in allowed_top:
                continue  # 跳过未在允许列表中的 group1

            # 一级 tab
            group1_tab = QWidget()
            group1_layout = QVBoxLayout(group1_tab)

            # 二级 tab
            group2_tabs = QTabWidget()
            for group2, items in subgroups.items():
                scroll = QScrollArea()
                scroll.setWidgetResizable(True)
                inner_widget = QWidget()
                form_layout = QFormLayout(inner_widget)

                for path, value in items:
                    line_edit = QLineEdit(str(value))
                    self.param_inputs[path] = line_edit
                    short_name = '.'.join(path.split('.')[2:])  # 去掉前两层
                    form_layout.addRow(QLabel(short_name), line_edit)

                scroll.setWidget(inner_widget)
                group2_tabs.addTab(scroll, group2)

            group1_layout.addWidget(group2_tabs)
            self.top_tabs.addTab(group1_tab, group1)

    def collect_params(self, obj, result_dict, path='', allowed_prefixes=None):
        for attr in dir(obj):
            if attr.startswith('_') or attr in ['parent', 'logger', 'event_loop']:
                continue
            full_path = f"{path}.{attr}" if path else attr

            # 只收集允许的顶级前缀
            top_prefix = full_path.split('.')[0]
            if allowed_prefixes is not None and top_prefix not in allowed_prefixes:
                continue

            try:
                value = getattr(obj, attr)
                if callable(value):
                    continue
                if hasattr(value, '__dict__'):
                    self.collect_params(value, result_dict, full_path, allowed_prefixes)
                else:
                    result_dict[full_path] = value
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
        parts = path.split('.')
        target = root
        for part in parts[:-1]:
            target = getattr(target, part)
        attr = parts[-1]

        old_value = getattr(target, attr)
        try:
            if isinstance(old_value, int):
                new_value = int(value_str)
            elif isinstance(old_value, float):
                new_value = float(value_str)
            elif isinstance(old_value, bool):
                new_value = value_str.lower() in ['true', '1', 'yes']
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

import serial
import time
from odrive.enums import AxisError, MotorError, EncoderError, ControllerError, LegacyODriveError

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

class ODriveUART:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        self.last_errors = []

    def close(self):
        self.ser.close()

    def send_cmd(self, cmd):
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.05)
        return self.ser.readline().decode().strip()

    def check_path_error(self, indent, axis_label, label, path, enum_type):
        val = self.send_cmd(f"r {path}")
        decoded = decode_flags(val, enum_type)
        self.last_errors.append({
            "axis": axis_label,
            "label": label,
            "path": path,
            "raw_value": val,
            "decoded": decoded
        })

        color = RED if decoded else RESET
        print(f"{indent}{color}{label} = {val}{RESET}")
        for msg in decoded:
            print(f"{indent}  {YELLOW}⮡ {msg}{RESET}")

    def check_odrive_error(self, clear=True):
        self.last_errors = []

        self.check_path_error("", "system", "system.error", "error", LegacyODriveError)

        for axis in [0, 1]:
            axis_label = f"axis{axis}"
            print(f"{axis_label}:")
            self.check_path_error("  ", axis_label, "axis.error", f"{axis_label}.error", AxisError)
            self.check_path_error("  ", axis_label, "motor.error", f"{axis_label}.motor.error", MotorError)
            self.check_path_error("  ", axis_label, "encoder.error", f"{axis_label}.encoder.error", EncoderError)
            self.check_path_error("  ", axis_label, "controller.error", f"{axis_label}.controller.error", ControllerError)

        if clear:
            self.send_cmd("sc")


# 示例使用
odrv = ODriveUART('/dev/motor')  # 请替换为你的设备
odrv.check_odrive_error(clear=True)
odrv.close()

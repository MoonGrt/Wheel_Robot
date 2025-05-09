import time
import math
import serial
import struct
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ================== 全局变量 ==================
key = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]

angle_degree_list = []
angles_list = []
time_list = []

start_time = time.time()

# ================== 四元数相关 ==================
class Quaternion:
    def __init__(self, q0=1, q1=0, q2=0, q3=0):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def normalize(self):
        norm = np.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

RAD2DEG = 180 / np.pi
q = Quaternion()
gyro_error_int = np.zeros(3)
Kp = 0.8
Ki = 0.0003

def update_imu(accel, gyro, dt):
    global q, gyro_error_int
    acc_norm = np.linalg.norm(accel)
    if acc_norm == 0:
        return 0, 0, 0
    acc = accel / acc_norm

    gravity = np.array([
        2 * (q.q1 * q.q3 - q.q0 * q.q2),
        2 * (q.q0 * q.q1 + q.q2 * q.q3),
        1 - 2 * (q.q1**2 + q.q2**2)
    ])

    error = np.cross(acc, gravity)
    gyro_error_int += error * Ki * dt
    corrected_gyro = gyro + Kp * error + gyro_error_int

    half_dt = 0.5 * dt
    q_dot = Quaternion(
        -q.q1 * corrected_gyro[0] - q.q2 * corrected_gyro[1] - q.q3 * corrected_gyro[2],
         q.q0 * corrected_gyro[0] + q.q2 * corrected_gyro[2] - q.q3 * corrected_gyro[1],
         q.q0 * corrected_gyro[1] - q.q1 * corrected_gyro[2] + q.q3 * corrected_gyro[0],
         q.q0 * corrected_gyro[2] + q.q1 * corrected_gyro[1] - q.q2 * corrected_gyro[0]
    )

    q.q0 += q_dot.q0 * half_dt
    q.q1 += q_dot.q1 * half_dt
    q.q2 += q_dot.q2 * half_dt
    q.q3 += q_dot.q3 * half_dt
    q.normalize()

    roll = np.arctan2(2 * (q.q0 * q.q1 + q.q2 * q.q3), 1 - 2 * (q.q1**2 + q.q2**2)) * RAD2DEG
    pitch = np.arcsin(2 * (q.q0 * q.q2 - q.q3 * q.q1)) * RAD2DEG
    yaw = np.arctan2(2 * (q.q0 * q.q3 + q.q1 * q.q2), 1 - 2 * (q.q2**2 + q.q3**2)) * RAD2DEG

    return roll, pitch, yaw

# ================== 串口数据解析 ==================
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity
    angle_flag = False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:
        return
    else:
        data_buff = list(buff.values())
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                raw = hex_to_short(data_buff[2:10])
                acceleration = [raw[i] / 32768.0 * 16 * 9.8 for i in range(3)]
            else:
                print('0x51 Check failure')
        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                raw = hex_to_short(data_buff[2:10])
                angularVelocity = [raw[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
            else:
                print('0x52 Check failure')
        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                raw = hex_to_short(data_buff[2:10])
                angle_degree = [raw[i] / 32768.0 * 180 for i in range(3)]
                angle_flag = True
            else:
                print('0x53 Check failure')
        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
            else:
                print('0x54 Check failure')
        buff = {}
        key = 0
        return angle_flag

# ================== 串口读取线程 ==================
def driver_loop(port_name, baudrate):
    try:
        imu = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.5)
        print(f"Serial port {port_name} opened at {baudrate} bps")
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    while True:
        try:
            buff_count = imu.inWaiting()
            if buff_count > 0:
                buff_data = imu.read(buff_count)
                for i in range(buff_count):
                    tag = handle_serial_data(buff_data[i])
                    if tag:
                        current_time = time.time() - start_time
                        time_list.append(current_time)
                        angle_degree_list.append(angle_degree.copy())
                        fused = update_imu(acceleration, angularVelocity, 0.005)
                        angles_list.append(fused)
        except Exception as e:
            print("IMU error:", e)
            break

# ================== 实时绘图 ==================
fig, axes = plt.subplots(3, 1, figsize=(10, 8))
titles = ['Roll (°)', 'Pitch (°)', 'Yaw (°)']
lines = []

for ax, title in zip(axes, titles):
    line_raw, = ax.plot([], [], 'r--', label='Raw IMU')
    line_fused, = ax.plot([], [], 'b-', label='Fused')
    ax.set_title(title)
    ax.set_xlim(0, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True)
    ax.legend()
    lines.append((line_raw, line_fused))

def update_plot(frame):
    if len(time_list) < 2:
        return lines

    t = time_list
    raw = np.array(angle_degree_list)
    fused = np.array(angles_list)

    for i in range(3):
        lines[i][0].set_data(t, raw[:, i])
        lines[i][1].set_data(t, fused[:, i])
        axes[i].set_xlim(max(0, t[-1] - 10), t[-1])

    return lines

# ================== 主程序入口 ==================
if __name__ == '__main__':
    threading.Thread(target=driver_loop, args=('COM3', 921600), daemon=True).start()  # 改为你的串口名
    ani = FuncAnimation(fig, update_plot, interval=100)
    plt.tight_layout()
    plt.show()

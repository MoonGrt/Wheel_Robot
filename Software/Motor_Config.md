## Motor Configuration
[电机配置实例](https://blog.csdn.net/abf1234444/article/details/104967023)

> 进行配置前建议首先执行一遍擦除配置:
> odrv0.erase_configuration()
> odrv0.reboot()

### Odrive 参数
| 参数 | Odrive          |
|------|-----------------|
| 版本 | ODrive 3.6-24V  |

### 锂电池参数
| 参数                        | 锂电池                          |
|-----------------------------|---------------------------------|
| 电池容量                    | 3.0Ah                           |
| 输出电流                    | 5 A                             |
| 输出电压                    | 24 V                            |

### 编码器参数
| 参数                       | AS5147P                         |
|----------------------------|---------------------------------|
| CPR                        | 4096                            |
| PPR                        | 1024                            |

### PM3510 参数
| 参数                        | PM3510                          |
|-----------------------------|---------------------------------|
| 额定电压 (Nominal voltage)  | 12 V                            |
| 额定电流 (Nominal current)  | 0.53 A                          |
| 额定扭矩 (Nominal torque)   | 0.11 N.M                        |
| 额定转速 (Nominal Speed)    | 945 rpm                         |
| 最大转速 (Max Speed)        | 1250 rpm                        |
| 堵转扭矩 (Stall torque)     | 0.16 N.M                        |
| 堵转电流 (Stall current)    | 0.80 A                          |
| 相间电阻 (Phase to phase resistance) | 7.8                    |
| 相间电感 (Phase to phase inductance) | 3.95 Mh                |
| 转速常数 (Speed constant)   | 45 rpm/V                        |
| 扭矩常数 (Torque constant)  | 0.20 N.M/A                      |
| 转子惯量 (Rotor inertia)    | 81 gm²                          |
| 极对数 (Number of poles)    | 11                              |
| 电机重量 (Motor weight)     | 73 g                            |
| 工作温度范围 (Working temperature) | -20 / 80 ℃              |
| 最高退磁温度 (Max demagnetize temperature) | 120 ℃           |
| 定子尺寸 (Stator dimension (D×L)) | 35×10                     |
| 绕线圈数 (Turns)            | 36                              |
| 滑环尺寸 (Slipring)         | 12.5 mm                         |

### 1. 配置参数
#### 1.1 基本配置
- `odrv0.config.brake_resistance = 0`
  - PM3510 电机的功率较小, 且使用锂电池供电, 不需要外部刹车电阻。
- `odrv0.config.dc_bus_undervoltage_trip_level = 20`
  - 锂电池输出电压为 24V, 考虑放电特性, 设置欠压保护在 20V 可避免过放。
- `odrv0.config.dc_bus_overvoltage_trip_level = 26`
  - 保护 ODrive 电路, 避免电压瞬间超过 ODrive 的额定电压范围24V。
- `odrv0.config.dc_max_positive_current = 8`
  - 限制最大正向电流为锂电池输出电流 8A, 避免过流损坏电池和 ODrive。
- `odrv0.config.dc_max_negative_current = -1`
  - 允许轻微的再生电流回灌, 避免过度充电电池。
- `odrv0.config.max_regen_current = 1`
  - 限制再生电流, 保护电池不被回灌过多电流。开关电源进行供电所以不具备电能回收功能。
#### 1.2 电机配置
- `odrv0.axis0.motor.config.pole_pairs = 11`
  - PM3510 电机的极对数为 11。
- `odrv0.axis0.motor.config.calibration_current = 0.5`
  - 根据电机额定电流 0.53A, 设置校准电流为 0.5A, 避免过热或损坏电机。
- `odrv0.axis0.motor.config.resistance_calib_max_voltage = 2`
  - 电机的相间电阻较高（7.8Ω）, 使用较低的校准电压, 减少热量和误差。
- `odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT`
  - PM3510 是无刷直流电机, 适合使用该模式。
- `odrv0.axis0.motor.config.current_lim = 0.8`
  - 限制最大电流在 0.8A, 匹配电机堵转电流, 避免电机过载。
- `odrv0.axis0.motor.config.requested_current_range = 1`
  - 设定电流范围为 1A, 满足电机需求并保持电流采样稳定。
#### 1.3 编码器配置
- `odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL`
  - AS5147P 为增量式编码器, 使用增量模式。
- `odrv0.axis0.encoder.config.use_index = True`
  - 使用索引信号进行初始化。
- `odrv0.axis0.encoder.config.cpr = 4096`
  - AS5147P 编码器 CPR 为 4096。
- `odrv0.axis0.encoder.config.bandwidth = 2000`
  - 设置编码器带宽为 2000Hz, 可滤除噪声且保持响应速度。
- `odrv0.axis0.config.calibration_lockin.current = 0.5`
  - 使用 0.5A 校准电流, 避免电机过热。
- `odrv0.axis0.config.calibration_lockin.ramp_time = 0.5`
  - 0.5 秒的斜坡时间让电流缓慢上升, 减少电机冲击。
- `odrv0.axis0.config.calibration_lockin.ramp_distance = 3.14159`
  - 让电机旋转半圈（π弧度）确保充分校准。
- `odrv0.axis0.config.calibration_lockin.accel = 10`
  - 设定加速度, 确保电机稳定旋转。
- `odrv0.axis0.config.calibration_lockin.vel = 20`
  - 校准时设定速度为 20 rad/s。
#### 1.4 控制器配置
- `odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL`
  - 选择速度控制模式。
- `odrv0.axis0.controller.config.vel_limit = 100`
  - 将最大转速 1250 rpm 转换为弧度/秒, 确保速度受限。1250 * 2 * 3.1416 / 60 = 130.9
- `odrv0.axis0.controller.config.pos_gain = 20.0`
  - 设置合适的比例增益。
- `odrv0.axis0.controller.config.vel_gain = 0.05`
  - 根据电机特性设定速度环增益。
- `odrv0.axis0.controller.config.vel_integrator_gain = 0.1`
  - 设定速度积分增益, 确保稳定性。
#### 1.5 梯形加减速配置
- `odrv0.axis0.trap_traj.config.vel_limit = 100`
  - 限制速度在最大转速范围内。1250 * 2 * 3.1416 / 60 = 130.9
- `odrv0.axis0.trap_traj.config.accel_limit = 5`
  - 限制加速度, 避免电机启动时电流过冲。
- `odrv0.axis0.trap_traj.config.decel_limit = 5`
  - 限制减速度, 保护电机和驱动器。
- `odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ`
  - 使用梯形轨迹作为输入模式, 确保平滑加减速。
#### 1.6 保存配置
- odrv0.save_configuration()
- odrv0.reboot()

### 2. 校准工作
注意：进行参数校准前请确保电机转子能够自由旋转而且不能有偏载，即负载均匀和较弱的摩擦负载才行，重载或类似弹簧载荷不行，否则将影响参数自动校准
#### 2.1 校准电机
- `odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION`
> 注意暂停，等待电机校准完成
- `dump_errors(odrv0)`
  - 列出状态信息，如果返回状态没有错误则编码器索引校准OK，可以继续进行下面步骤，如果出现错误则要根据错误信息分析原因，然后输入 dump_errors(odrv0, True) 对错误进行清除后重新尝试
- `odrv0.axis0.motor.config.pre_calibrated = True`
#### 2.2 编码器校准
- `odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH`
  - 索引校准时电机的转动方向(以下三个参数正负号应保持一致)
  - `odrv0.axis0.config.calibration_lockin.ramp_distance = -3.14159`
  - `odrv0.axis0.config.calibration_lockin.accel = -10`
  - `odrv0.axis0.config.calibration_lockin.vel = -20`
> 注意暂停，等待编码器索引完成
- `dump_errors(odrv0)`
- `odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION`
> 注意暂停，等待编码器校准完成
- `dump_errors(odrv0)`
- `odrv0.axis0.encoder.config.pre_calibrated = True`
#### 2.3 保存校准结果
- `odrv0.save_configuration()`
- `odrv0.reboot()`

### 3. 控制电机运行
- `odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH`
- `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`
- `odrv0.axis0.controller.input_pos = 50`
- `odrv0.axis0.requested_state = AXIS_STATE_IDLE`
- `odrv0.axis0.config.startup_encoder_index_search = True`
- `odrv0.axis0.config.startup_closed_loop_control = True`
- `odrv0.save_configuration()`

import odrive
odrv0 = odrive.find_any()  # 自动连接 ODrive

# 电机类型（无刷、直流、步进等）
print("Motor Type:", odrv0.axis0.motor.config.motor_type)
# 极对数（Pole Pairs）
print("Pole Pairs:", odrv0.axis0.motor.config.pole_pairs)
# 电流限制（Current Limit）
print("Current Limit:", odrv0.axis0.motor.config.current_lim)
# 转矩常数（Torque Constant）
print("Torque Constant:", odrv0.axis0.motor.config.torque_constant)
# 电阻（Phase Resistance）
print("Phase Resistance:", odrv0.axis0.motor.config.phase_resistance)
# 电感（Phase Inductance）
print("Phase Inductance:", odrv0.axis0.motor.config.phase_inductance)
# 电流控制带宽（Current Control Bandwidth）
print("Current Control Bandwidth:", odrv0.axis0.motor.config.current_control_bandwidth)
# 最大电机速度（Vel Limit）
print("Velocity Limit:", odrv0.axis0.controller.config.vel_limit)
# 编码器 CPR（Counts Per Revolution）
print("Encoder CPR:", odrv0.axis0.encoder.config.cpr)

print()

# 查看所有电机配置项
# print(odrv0.axis0.motor.config)
# 查看所有编码器配置项
print(odrv0.axis0.encoder.config)
# 查看控制器配置项
# print(odrv0.axis0.controller.config)


# # 设置编码器模式为增量编码器（ABI）
# odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
# # 设置 CPR（1024 PPR -> 1024 counts/rev）
# odrv0.axis0.encoder.config.cpr = 1024
# # 设置电机极对数（根据你的电机来调整）
# odrv0.axis0.motor.config.pole_pairs = 7
# # 校准编码器
# odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
# # 保存配置
# odrv0.save_configuration()
# odrv0.reboot()


## MT3510-AS5147-ASCII

### Calibration

```
sc
w axis0.requested_state 7
w axis1.requested_state 7
```

### IDLE Mode
```
w axis0.requested_state 1
w axis1.requested_state 1
```

### Loop Control Mode: Velocity Control 

`v motor velocity current_ff`
`v` 表示速度控制模式
`motor` 表示电机编号, 0 或 1
`velocity` 目标转速，单位为 counts/s
`current_ff` 电流前馈, 单位为 A（可选的）

> 请注意，如果您不了解前馈和它的作用，只需将其忽略即可。以上命令发送后会触发对应电机的内部看门狗喂狗动作。

```
w axis0.requested_state 8
w axis1.requested_state 8
v 0 2.0 0.0
v 1 2.0 0.0
```

### Loop Control Mode: Position Control 

`q motor position velocity_lim current_lim`
`q` 表示位置控制模式
`motor` 表示电机编号, 0 或 1
`position` 目标位置，值表示编码器的计数
`velocity_lim` 转速限制，单位为counts/s （可选的）
`current_lim` 电流限制，单位为A（可选的）

> 请注意，如果您不了解前馈和它的作用，只需将其忽略即可。以上命令发送后会触发对应电机的内部看门狗喂狗动作。

```
w axis0.requested_state 8
w axis1.requested_state 8
q 0 50 100 10
q 0 50 100 10
```


### Servo

`m servo angle`

`m 0 110 1 80 2 70 3 100`

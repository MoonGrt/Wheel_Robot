
## Wheel Robot

### imu
#### 绑定 imu USB设备
> 连接上 imu 后，运行以下命令绑定 imu USB设备。
```
sudo chmod 777 imu/bind_usb.sh
sudo sh imu/bind_usb.sh
```
> 重新插拔连接 IMU 模块的USB数据线, 输入以下指令检测绑定端口是否成功:
```
ll /dev/imu
```
#### 运行 imu 节点
```
source install/setup.bash
ros2 run imu imu
```

### ydlidar
#### 绑定 ydlidar USB设备
> 连接上 ydlidar 后，运行以下命令绑定 ydlidar USB设备。
```
sudo chmod 777 ydlidar/bind_usb.sh
sudo sh ydlidar/bind_usb.sh
```
> 重新插拔连接 YDlidar 模块的USB数据线, 输入以下指令检测绑定端口是否成功:
```
ll /dev/ydlidar
```
#### 运行 ydlidar 节点
```
source install/setup.bash
ros2 launch ydlidar x3_ydlidar_launch.py
```

### odrive
#### 绑定 odrive USB设备
> 连接上 odrive 后，运行以下命令绑定 odrive USB设备。
```
sudo chmod 777 odrive/bind_usb.sh
sudo sh odrive/bind_usb.sh
```
#### 运行 odrive 节点
```
source install/setup.bash
ros2 run odrive odrive
```

### balance
#### 运行 balance 节点
> 到目录中直接运行py文件
```
python3 src/balance/balance/balance.py
```

### carto
#### 运行 carto 节点
```
source install/setup.bash
ros2 launch carto cartographer.launch.py
```

### wheel_robot
#### 运行 wheel_robot 节点
```
source install/setup.bash
ros2 launch wheel_robot robot.launch.py
```

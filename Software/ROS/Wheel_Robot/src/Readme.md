
## Wheel Robot

### imu
#### 绑定 imu USB设备
> 连接上 imu 后，运行以下命令绑定 imu USB设备。
```
sudo chmod 777 imu/bind_usb.sh
sudo sh imu/bind_usb.sh
```
> 重新插拔连接IMU模块的USB数据线, 输入以下指令检测绑定端口是否成功:
```
ll /dev/imu
```

### ydlidar
#### 绑定 ydlidar USB设备
> 连接上 ydlidar 后，运行以下命令绑定 ydlidar USB设备。
```
sudo chmod 777 ydlidar/bind_usb.sh
sudo sh ydlidar/bind_usb.sh
```

### odrive
#### 绑定 odrive USB设备
> 连接上 odrive 后，运行以下命令绑定 odrive USB设备。
```
sudo chmod 777 odrive/bind_usb.sh
sudo sh odrive/bind_usb.sh
```

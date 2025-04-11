
## Wheel Robot


### wheel_robot
#### 运行 wheel_robot 节点
> 该 launch 文件同时运行robot, imu, ydlidar, odom节点
```
source install/setup.bash
ros2 launch wheel_robot robot.launch.py
```


### balance
#### 运行 balance 节点
> 到目录中直接运行py文件(使用ROS2运行时ODrive库报错)
```
python3 src/balance/balance/balance.py
```


### carto
#### 运行 carto 节点
```
source install/setup.bash
ros2 launch carto cartographer.launch.py
```


### imu
#### 绑定 imu USB设备
> 连接上 imu 后，运行以下命令绑定 imu USB设备。
```
sudo chmod 777 src/imu/bind_usb.sh
sudo sh src/imu/bind_usb.sh
```
> 输入以下指令, 强制 udev 重新加载, 并测绑定端口是否成功:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
ll /dev/imu
```
#### 运行 imu 节点
```
source install/setup.bash
ros2 launch imu imu.launch.py
```


### ydlidar
#### 安装SDK
解压 document文件夹下的 YDLidar-SDK-master.tar.xz, 在该文件夹下打开终端, 输入:
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

#### 绑定 ydlidar USB设备
> 注意: imu 和 ydlidar 的USB-ID完全相同，因此使用CP210xSetIDs.exe工具将 ydlidar 的 serial 项 从默认的"0001"改为"0002"

> 连接上 ydlidar 后，运行以下命令绑定 ydlidar USB设备。
```
sudo chmod 777 src/ydlidar/bind_usb.sh
sudo sh src/ydlidar/bind_usb.sh
```
> 输入以下指令, 强制 udev 重新加载, 并测绑定端口是否成功:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
ll /dev/ydlidar
```
#### 运行 ydlidar 节点
```
source install/setup.bash
ros2 launch ydlidar ydlidar.launch.py
```


### odom
#### 绑定 odrive USB设备
> 如果是用虚拟机usb连接Odribe则可以绑定usb; Raspberry Pi 用引脚UART连接则不需要
> 连接上 odrive 后，运行以下命令绑定 odrive USB设备。
```
sudo chmod 777 src/odom/bind_usb.sh
sudo sh src/odom/bind_usb.sh
```
> 输入以下指令, 强制 udev 重新加载, 并测绑定端口是否成功:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
ll /dev/odrive_uart
```
#### 运行 odom 节点
```
source install/setup.bash
ros2 run odom odom
```

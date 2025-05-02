
## Wheel Robot


### wheel_robot
#### 运行 wheel_robot 节点
> 该 launch 文件同时运行robot, imu, ydlidar, odom, web节点
```
source install/setup.bash
ros2 launch wheel_robot robot.launch.py
```


### balance
#### 运行 balance 节点
> 到目录中直接运行py文件(使用ROS2运行时motor库报错)
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
#### 安装 YDLidar-SDK
解压YDLidar-SDK-master.tar.xz, 在解压后的文件夹中, 执行:
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
> 然后就可以删除该文件夹

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


### motor
#### 绑定 motor USB设备
> 如果是用虚拟机usb连接Odribe则可以绑定usb; Raspberry Pi 用引脚UART连接则不需要
> 连接上 motor 后，运行以下命令绑定 motor USB设备。
```
sudo chmod 777 src/motor/bind_usb.sh
sudo sh src/motor/bind_usb.sh
```
> 输入以下指令, 强制 udev 重新加载, 并测绑定端口是否成功:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
ll /dev/motor
```
#### 运行 motor 节点
```
source install/setup.bash
ros2 run motor motor
```


### Issue

#### colcon build 卡死

1. **内存不足**

树莓派（特别是 2GB 或 4GB RAM 版本）在编译大型 ROS2 包时可能会内存溢出。

**解决办法：**

* **安装 dphys-swapfile 工具**
  ```bash
  sudo apt-get update
  sudo apt-get install dphys-swapfile
  ```

* **启用交换分区（Swap）**
  ```bash
  sudo dphys-swapfile swapoff
  sudo nano /etc/dphys-swapfile
  ```
  修改：
  ```ini
  CONF_SWAPSIZE=2048  # 或更大，比如 4096
  ```
  然后：
  ```bash
  sudo dphys-swapfile setup
  sudo dphys-swapfile swapon
  ```

2. **并行编译线程太多**

默认 `colcon build` 会使用多线程，但树莓派资源有限，容易卡死。

**解决办法：**

* 限制并行编译线程数，例如用 2 个线程：
  ```bash
  colcon build --executor sequential --parallel-workers 2
  ```

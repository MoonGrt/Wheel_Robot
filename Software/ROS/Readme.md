
## Raspberry Pi 4 ROS2 Set

### 寻找树莓派IP地址
`ping raspberrypi.local`

### 安装 ubuntu 20.04

### WIFI 配置
- Step1: 编辑文件: `sudo nano /etc/netplan/50-cloud-init.yaml`

    > 注意格式，不要有多余的空格(2个空格的缩进)

    ```
    network:
      version: 2
      wifis:
        wlan0:
          dhcp4: true
          optional: true
          access-points:
            "Moon":
              password: "asdfghjkl"
            "TP-LINK_90D4":
              password: "15908133965"
    ```

- Step2: 修改内容:
    ```
    sudo netplan -debug try
    sudo netplan -debug generate
    sudo netplan -debug apply
    ```

### 安装 ROS2 Foxy
`wget http://fishros.com/install -O fishros && bash fishros`

### 安装 YDLidar-SDK
解压YDLidar-SDK-master.tar.xz, 在解压后的文件夹中, 执行:
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
> 然后就可以删除该文件夹

### Issues
#### 1. Odrive 和 Raspberry 连接，导致 Raspberry 无法进入系统
- Step1. 禁用 UART 作为系统控制台
  - 编辑 cmdline.txt: `sudo nano /boot/firmware/cmdline.txt`
  - 移除 console=serial0,115200
- Step2. 取消 u-boot 开机等待
  - 串口输入任意内容，进入 u-boot
  - 输入: `setenv bootdelay -2`
  - 保存: `saveenv`
- Step3. 重新启动 Raspberry Pi

#### 2. Odrive GPIO8 无法输出PWM信号
> 原因: Odrive GPIO8 与 Raspberry-Pi SPI CE0 共享，被拉低电平
- Step1: 关闭 Raspberry-Pi 的 SPI 功能
  - 编辑 config.txt: `sudo nano /boot/firmware/config.txt`
  - 取消 SPI 功能: `dtparam=spi=off`
- Step2: (上述方法无用) 强制禁用 SPI
  - 编辑 usercfg.txt: `sudo nano /boot/firmware/usercfg.txt`
  - 添加 `dtparam=spi=off`
- Step3. 重新启动 Raspberry Pi

#### 3. 树莓派串口无法使用
> 原因: 树莓派的串口默认连接到 ttyAMA0，而 ttyAMA0 被 终端和蓝牙
- Step1: 关闭终端和蓝牙
  - 关闭终端: 查看issue1
  - 关闭蓝牙: 
    - 编辑 config.txt: `sudo nano /boot/firmware/config.txt`
    - 添加: `dtoverlay=disable-bt`
  - bash运行: `sudo systemctl disable bluetooth`
    <!-- - 进入root用户，bash运行: `sudo su` -->


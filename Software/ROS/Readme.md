
## Raspberry Pi 4 ROS2 Set

### 安装 ubuntu 20.04

### 安装 ROS2 Foxy
`wget http://fishros.com/install -O fishros && bash fishros`

### Issues
#### 1. Odrive 和 Raspberry 连接，导致 Raspberry 无法进入系统
- Step1. 禁用 UART 作为系统控制台
  - 编辑 cmdline.txt: `sudo nano /boot/firmware/cmdline.txt`
  - 移除 console=serial0,115200
- Step2. 取消 u-boot 开机等待
  - 串口输入任意内容，进入 u-boot
  - 输入: `setenv bootwait no` or `setenv bootdelay -2`
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
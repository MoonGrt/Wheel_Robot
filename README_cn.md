**简体中文 | [English](README.md)**
<div id="top"></div>

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]


<!-- PROJECT LOGO -->
<br />
<div align="center">
    <a href="https://github.com/MoonGrt/Wheel_Robot">
    <img src="Document/images/logo.png" alt="Logo" width="80" height="80">
    </a>
<h3 align="center">Wheel_Robot</h3>
    <p align="center">
    项目简介
    <br />
    <a href="https://github.com/MoonGrt/Wheel_Robot"><strong>浏览文档 »</strong></a>
    <br />
    <a href="https://github.com/MoonGrt/Wheel_Robot">查看 Demo</a>
    ·
    <a href="https://github.com/MoonGrt/Wheel_Robot/issues">反馈 Bug</a>
    ·
    <a href="https://github.com/MoonGrt/Wheel_Robot/issues">请求新功能</a>
    </p>
</div>




<!-- CONTENTS -->
<details open>
  <summary>目录</summary>
  <ol>
    <li><a href="#文件树">文件树</a></li>
    <li>
      <a href="#关于本项目">关于本项目</a>
      <ul>
      </ul>
    </li>
    <li><a href="#贡献">贡献</a></li>
    <li><a href="#许可证">许可证</a></li>
    <li><a href="#联系我们">联系我们</a></li>
    <li><a href="#致谢">致谢</a></li>
  </ol>
</details>





<!-- 文件树 -->
## 文件树

```
└─ Project
  ├─ Software
  ├─ Tool
  ├─ .gitignore
  ├─ LICENSE
  ├─ README.md
  ├─ /Document/
  └─ /Hardware/

```



<!-- 关于本项目 -->
## 关于本项目

- [📦 硬件系统设计](#-硬件系统设计)
  - [🔋 电源管理模块](#-电源管理模块)
  - [🧩 PCB 设计](#-pcb-设计)
  - [🎯 传感器选型](#-传感器选型)
- [🔧 结构设计与执行单元](#-结构设计与执行单元)
  - [⚙️ 动力系统](#-动力系统)
  - [🧱 外壳与机构](#-外壳与机构)
- [💻 软件系统设计](#-软件系统设计)
  - [⚡ FOC电机控制子系统](#-FOC电机控制子系统)
  - [🤖 ROS系统集成](#-ROS系统集成)

### 硬件选型
| 模块       | 组件/芯片型号        | 功能描述                             | 器件参数                             |
|------------|----------------------|--------------------------------------|--------------------------------------|
| 主控       | 树莓派               | 搭载ROS，负责SLAM算法                | -                                    |
|            | STM32F405            | 专用于电机控制（FOC、PID）          | 主频180MHz，Cortex-M4内核           |
| 传感器     | AS5147P              | 超高速磁性旋转位置传感器            | 16位分辨率，SSI接口                  |
|            | CMP10A              | IMU传感器                            | 三轴陀螺仪、三轴加速度计，三轴磁力计和气压计 |
|            | YDLIDAR X3           | 激光雷达                             | 360°扫描，测距范围0.12-8m            |
| 电源管理   | 锂电池               | -                                    | 24V，3000mAh，XT60插口               |
|            | TPS54160 + RT9193/LP5907 | 24V→12V/5V→3.3V两级降压稳压         | TPS54160-Buck, RT9193-VCC, LP5907-AVCC |
|            | FSMD012              | 自恢复保险丝-过流保护               | 50A峰值电流                          |
|            | NCP18XH103F0         | 实时监测电路温度，防止过热损害      | B值3435K，10kΩ @ 25℃                |
|            | LM5109B + RTP50W     | 高速半桥驱动器控制耗散电阻          | 5V-18V驱动器 + 50W功率电阻，2ohm      |
|            | SS34                 | 肖特基二极管                        | 3A，40V，低压降                      |
|            | SMAJ30CA             | TVS二极管，浪涌抑制保护             | 30V，双向，400W                      |
|            | 1812电容×10         | 去耦滤波                             | 50V，47µF，陶瓷电容                  |
| 电机驱动   | DRV8301              | 无刷直流BLDC电机驱动芯片            | 4.5V-60V，峰值电流4A                 |
|            | KNY3406C             | 场效应管（MOSFET）                  | Vds=40V，Id=60A，Rds(on)=6mΩ        |
|            | SG995                | 舵机，位置控制                      | 180°转角，扭矩`10kg.cm @ 6V`          |
|            | PM3510               | 平台无刷电机                        | 1250rpm 0.11N.m                      |

<font size=5>**Note**</font>
- **PM3510 电机**
  - 更换电机径向磁铁：原有电机径向磁铁磁性较差，导致编码器噪声大。

### 📦 硬件系统设计

#### 🔋 电源管理模块
- **电池管理系统（BMS）**
  - 使用 **24V 3000mAh 锂电池组**（XT60 接口）作为主电源。
  - 充电系统支持快速插拔和过压保护
  - 
- **电压转换与稳压**
  - **两级降压架构**：
    - 第一级：TPS54160（24V→12V/5V，最大电流1.5A）
    - 第二级：RT9193（5V→3.3V，LDO线性稳压，VCC）、LP5907（3.3V→5V，稳压器，AVCC）
  - **去耦滤波网络**：10×1812陶瓷电容（50V/47µF）
- **电路保护设计**
  - **过流保护**：FSMD012自恢复保险丝（50A峰值）
  - **温度监测**：NCP18XH103F0热敏电阻（B值3435K）
  - **瞬态抑制**：SMAJ30CA（30V电源总线保护）
  - **反接保护**：SS34肖特基二极管（40V/3A）
- **驱动器保护**
  - **LM5109B** 高速半桥驱动器 + **RTP50W** 2Ω/50W 耗散电阻，提升电机系统稳定性。

#### 🧩 PCB 设计
- **电机驱动板**
  - **STM32F405**：用于电机底层控制（FOC/PID），主频 180MHz，Cortex-M4 内核。
  - **DRV8301**：三相无刷电机驱动芯片（4.5V~60V，峰值电流 4A）；
  - **KNY3406C MOSFET** ×3：Vds=40V，Id=60A，Rds(on)=6mΩ，构建高效功率桥。
  - **接口**
    - **调试/下载接口**: 提供 USB、SWD、UART 接口，兼容 ST-Link/J-Link 调试工具。
    - **传感器连接**: 支持 SSI 接口编码器、I2C/SPI 传感器连接。
  - **LM5109B+RTP50W**: 高速半桥驱动器 + 2Ω/50W 耗散电阻，提升电机系统稳定性。

## 🎯 传感器选型
| 传感器类型   | 型号            | 性能指标                         | 接口方式        |
|--------------|-----------------|---------------------------------|----------------|
| 位置传感器   | AS5147P        | 16位分辨率，0-28kRPM              | SSI数字接口    |
| 惯性测量单元 | CMP10A         | ±16g加速度计，±2000dps陀螺仪      | I2C+SPI双接口  |
| 激光雷达     | YDLIDAR X3     | 8m测距，360°×0.33°角分辨率        | USB2.0         |

---

### 🔧 结构设计与执行单元

#### ⚙️ 动力系统
- **无刷直流电机**
  - 型号：**PM3510**
  - 参数：1250rpm，0.11N·m，驱动芯片为 DRV8301。
- **舵机**
  - 型号：**SG995**
  - 参数：180°旋转角，10kg·cm 扭矩（@6V），用于舵向或机构调整。

#### 🧱 外壳与机构
- **3D 打印外壳**
  - 材质：PLA 或 X树脂
  - 结构：模块化拼装，带快拆接口，便于维护和功能拓展。
  - 内部预留电池仓、电路板固定孔位、散热设计。
- **连接**
  - 轴承 轧带 醋酸胶布 电线 导线 螺栓 螺母 

---

### 💻 软件系统设计

#### ⚡ FOC电机控制子系统

##### RT-Thread 实时操作系统
- 实现任务隔离：控制线程、电流采样线程、通信线程独立运行；
- 支持动态设备管理，便于电机扩展。

##### 驱动程序开发
- **PWM** 输出控制；
- **ADC** 电流采样；
- **SPI/SSI** 编码器读取（兼容 AS5147P）；
- **UART/USB/CAN** 多通信方式。

##### 安全监控
- 基于NCP18XH103F0的温度保护策略
- 电源异常状态检测（TVS触发记录）
- 电压、电流检测
- 异常中断记录

##### 通信协议
- **USB CDC/HID**：与上位机交互；
- **CAN 总线**：多电机间同步通信；
- **UART 串口**：与外部传感器或树莓派通信。

##### 电机配置工具 UI
- 可视化调参：电机极对数、PID 参数设置；
- 一键校准：包括转向识别与零点校准；
- 实时监控：转速、电流、温度反馈。

---

#### 🤖 ROS系统集成

##### 🧪 仿真平台构建
- **URDF/xacro**：机器人模型结构建模；
- **Gazebo**：实现完整仿真环境；
- **Cartographer + Navigation2**：实现地图构建与导航路径规划。

##### 🚗 实物部署与控制
- **传感器接入与驱动**
  - **AS5147P**：16位旋转编码器（SSI 接口），用于电机位置反馈；
  - **CMP10A IMU**：三轴陀螺仪 + 加速度计 + 磁力计 + 气压计；
  - **YDLIDAR X3**：激光雷达，360°扫描，0.12–8m 测距范围；
  - **编码器驱动**：支持同步采样，配合电机实现闭环控制。

- **自主运动控制**
  - 支持平衡控制算法（如 PID + 卡尔曼滤波）；
  - 差速与全向底盘兼容；
  - 支持本地导航 + 避障（融合雷达与IMU数据）。

- **Web 控制台**
  - 地图浏览：实时地图更新、路径显示；
  - 控制功能：方向按钮、虚拟摇杆控制；
  <!-- - 数据可视化：传感器状态、电机反馈、电池状态等。 -->

### 后续工作
1. 构建24V高功率供电体系，支持更大扭矩电机
2. 采用工业级SSI接口编码器提升控制精度
3. 优化电源保护设计，TVS覆盖3.3V-30V全电压域
4. 引入热管理监控系统，确保长期运行稳定性
5. 选用低内阻MOSFET降低驱动损耗
6. 散热设计，降低风扇噪声影响

<p align="right">(<a href="#top">top</a>)</p>



<!-- 贡献 -->
## 贡献

贡献让开源社区成为了一个非常适合学习、互相激励和创新的地方。你所做出的任何贡献都是**受人尊敬**的。

如果你有好的建议，请复刻（fork）本仓库并且创建一个拉取请求（pull request）。你也可以简单地创建一个议题（issue），并且添加标签「enhancement」。不要忘记给项目点一个 star！再次感谢！

1. 复刻（Fork）本项目
2. 创建你的 Feature 分支 (`git checkout -b feature/AmazingFeature`)
3. 提交你的变更 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到该分支 (`git push origin feature/AmazingFeature`)
5. 创建一个拉取请求（Pull Request）
<p align="right">(<a href="#top">top</a>)</p>



<!-- 许可证 -->
## 许可证

根据 MIT 许可证分发。打开 [LICENSE](LICENSE) 查看更多内容。
<p align="right">(<a href="#top">top</a>)</p>



<!-- 联系我们 -->
## 联系我们

MoonGrt - 1561145394@qq.com
Project Link: [MoonGrt/Wheel_Robot](https://github.com/MoonGrt/Wheel_Robot)

<p align="right">(<a href="#top">top</a>)</p>



<!-- 致谢 -->
## 致谢

* [Choose an Open Source License](https://choosealicense.com)
* [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
* [Malven's Flexbox Cheatsheet](https://flexbox.malven.co/)
* [Malven's Grid Cheatsheet](https://grid.malven.co/)
* [Img Shields](https://shields.io)
* [GitHub Pages](https://pages.github.com)
* [Font Awesome](https://fontawesome.com)
* [React Icons](https://react-icons.github.io/react-icons/search)
<p align="right">(<a href="#top">top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/MoonGrt/Wheel_Robot.svg?style=for-the-badge
[contributors-url]: https://github.com/MoonGrt/Wheel_Robot/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/MoonGrt/Wheel_Robot.svg?style=for-the-badge
[forks-url]: https://github.com/MoonGrt/Wheel_Robot/network/members
[stars-shield]: https://img.shields.io/github/stars/MoonGrt/Wheel_Robot.svg?style=for-the-badge
[stars-url]: https://github.com/MoonGrt/Wheel_Robot/stargazers
[issues-shield]: https://img.shields.io/github/issues/MoonGrt/Wheel_Robot.svg?style=for-the-badge
[issues-url]: https://github.com/MoonGrt/Wheel_Robot/issues
[license-shield]: https://img.shields.io/github/license/MoonGrt/Wheel_Robot.svg?style=for-the-badge
[license-url]: https://github.com/MoonGrt/Wheel_Robot/blob/master/LICENSE

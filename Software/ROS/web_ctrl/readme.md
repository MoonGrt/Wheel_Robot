## [ROS2 Web遥控器](https://blog.csdn.net/qq_27865227/article/details/139332468)

### 准备工作
安装并启动rosbridge_server的rosbridge_websocket插件：
```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-suite 
ros2 run rosbridge_server rosbridge_websocket
```
> 启动后, 将在本地 WebSocket 服务器的 9090 端口上启动 rosbridge_websocket

### 启用服务
在 Web1 目录使用 python 来新建一个服务器：
```python3 -m http.server 8000```

### 访问服务
本机访问: [localhost:8000](http://localhost:8000/)
远程访问: 将 app,js 中 localhost 改成ROS主机的确切IP地址

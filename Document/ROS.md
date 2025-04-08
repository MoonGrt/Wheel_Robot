
## ROS2 基础

### 安装ROS2
```wget http://fishros.com/install -O fishros && bash fishros```

### 创建工作空间和功能包
##### 创建工作空间
```mkdir -p xxx_ws/src && cd xxx_ws/src```
##### 创建Python功能包
```ros2 pkg create xxx --build-type ament_python --dependencies rclpy```
##### 创建C++功能包
```ros2 pkg create xxx --build-type ament_cmake --dependencies rclcpp```

### 编写节点
#### Python节点
1. xxx(功能包名)/xxx(功能包名) 目录下创建 python 文件
2. 修改 setup.py，添加内容:
```
        'console_scripts': [
            "node_name = xxx.xxxx:main" # xxx为功能包名, xxxx为节点文件名
        ],
```
#### C++节点
1. xxx(功能包名)/src 目录下创建 cpp 文件
2. 修改 CmakeLists.txt, 添加内容:
```
add_executable(node_name src/xxxx.cpp) # xxxx为节点文件名
ament_target_dependencies(node_name rclcpp)

install(TARGETS
  node_name
  DESTINATION lib/${PROJECT_NAME}
)
```

### 编译
```
colcon build
--symlink-install # 编译并创建软链接
--packages-select xxx # 只编译xxx包
--packages-up-to xxx # 编译xxx包及其依赖项
--packages-skip xxx # 跳过xxx包及其依赖项
```

### 运行节点
#### source环境
```source install/setup.bash```
#### 运行节点
```ros2 run xxx node_name # xxx为功能包名, node_name为节点文件名```


### ROS2 基础指令
| **操作**                   | **指令**                                                   |
|----------------------------|------------------------------------------------------------|
| **查看 ROS2 可用节点、参数、服务** | `ros2 node/param/service list`                       |
| **查看 ROS2 消息类型、主题、接口** | `ros2 msg/topic/interface list`                    |
| **查看 ROS2 可用节点信息**  | `ros2 node info /node_name`                                 |
| **查看 ROS2 可用消息类型信息** | `ros2 interface show /msg_type_name`                      |
| **查看 ROS2 可用参数信息**  | `ros2 param describe /node_name /param_name`                |
| **查看 ROS2 可用服务信息**  | `ros2 service info /service_name`                          |
| **查看 ROS2 可用主题信息**  | `ros2 topic info /topic_name`                              |
| **查看 ROS2 日志**          | `ros2 log`                                                 |
| **查看 ROS2 日志详细信息**  | `ros2 log config`                                          |
| **查看 ROS2 日志级别**      | `ros2 log level`                                           |
| **查看 ROS2 配置**          | `ros2 config`                                              |
| **查看 ROS2 配置详细信息**  | `ros2 config list`                                         |
| **查看 ROS2 配置项**        | `ros2 config get /node_name /param_name`                   |
| **设置 ROS2 配置项**        | `ros2 config set /node_name /param_name value`             |

查看 TF 树
```ros2 run rqt_tf_tree rqt_tf_tree```


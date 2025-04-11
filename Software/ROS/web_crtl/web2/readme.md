
### 使用步骤

#### ROS部分
- Step1. 启动机器人
  - 启动机器人仿真和建图程序
    ```bash
    source install/setup.bash
    ros2 launch fishbot_description gazebo.launch.py
    ```
    ```bash
    source install/setup.bash
    ros2 launch fishbot_cartographer cartographer.launch.py
    ```
- Step2. 启动rosbridge websocket
  - `ros2 run rosbridge_server rosbridge_websocket`


#### 网页部分
- Step1. 查ROS主机IP地址
  - 获取主机IP地址
- Step2. 启动ROSbridge
  - 打开index.html以及main.js文件，并修改url，改成ROS主机IP地址
- Step3. 启动网页
  - 使用浏览器打开index.html文件: 在文件夹中，双击文件，便可自动在浏览器打开
- Step4. 连接ROS节点
  - 点击Connect按钮，连接rosbridge_server


### Issues
#### 1. 'OccupancyGrid' object has no attribute '_slot_types'
```
Traceback (most recent call last):
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/internal/subscription_modifiers.py", line 161, in run
    MessageHandler.handle_message(self, msg)
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/internal/subscription_modifiers.py", line 73, in handle_message
    self.publish(msg)
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/capabilities/subscribe.py", line 159, in _publish
    self.publish(message, self.fragment_size, self.compression)
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/capabilities/subscribe.py", line 318, in publish
    outgoing_msg = message.get_cbor(outgoing_msg)
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/internal/outgoing_message.py", line 38, in get_cbor
    outgoing_msg["msg"] = self.get_cbor_values()
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/internal/outgoing_message.py", line 33, in get_cbor_values
    self._cbor_values = extract_cbor_values(self._message)
  File "/opt/ros/foxy/lib/python3.8/site-packages/rosbridge_library/internal/cbor_conversion.py", line 53, in extract_cbor_values
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
AttributeError: 'OccupancyGrid' object has no attribute '_slot_types'
```

**Solution**
这个问题是由于ROS2（Foxy）的消息结构与ROS1不同，导致rosbridge_library在处理OccupancyGrid消息时无法找到预期的_slot_types属性。以下是解决方案：

修改以下cbor_conversion.py文件中的extract_cbor_values函数:
```python
def extract_cbor_values(msg):
    """Extract a dictionary of CBOR-friendly values from a ROS message.

    Primitive values will be casted to specific Python primitives.

    Typed arrays will be tagged and packed into byte arrays.
    """
    out = {}
    for slot, slot_type in msg.get_fields_and_field_types().items():
        val = getattr(msg, slot)

        # string
        if slot_type in STRING_TYPES:
            out[slot] = str(val)

        # bool
        elif slot_type in BOOL_TYPES:
            out[slot] = bool(val)

        # integers
        elif slot_type in INT_TYPES:
            out[slot] = int(val)

        # floats
        elif slot_type in FLOAT_TYPES:
            out[slot] = float(val)

        # time/duration
        elif slot_type in TIME_TYPES:
            out[slot] = {
                "sec": int(val.sec),
                "nanosec": int(val.nanosec),
            }

        # byte array
        elif slot_type in BYTESTREAM_TYPES:
            out[slot] = bytes(val)

        # bool array
        elif slot_type in BOOL_ARRAY_TYPES:
            out[slot] = [bool(i) for i in val]

        elif slot_type in STRING_ARRAY_TYPES:
            out[slot] = [str(i) for i in val]

        # numeric arrays
        elif slot_type in TAGGED_ARRAY_FORMATS:
            tag, fmt = TAGGED_ARRAY_FORMATS[slot_type]
            fmt_to_length = fmt.format(len(val))
            packed = struct.pack(fmt_to_length, *val)
            out[slot] = Tag(tag=tag, value=packed)

        # array of messages
        elif type(val) in LIST_TYPES:
            out[slot] = [extract_cbor_values(i) for i in val]

        # message
        else:
            out[slot] = extract_cbor_values(val)

    return out
```

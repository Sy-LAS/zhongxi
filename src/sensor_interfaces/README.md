# 传感器接口包

这个包包含了用于连接Jetson Orin Nano与各种传感器的驱动程序和接口，包括摄像头、激光雷达和控制板。

## 组件

### 1. 摄像头驱动 (`camera_driver`)
- 处理USB摄像头输入(C100摄像头)
- 发布图像到 `/camera/image_raw` 话题
- 发布摄像头信息到 `/camera/camera_info` 话题
- 可配置参数: 分辨率、帧率、摄像头ID

### 2. 激光雷达驱动 (`lidar_driver`)
- 使用官方YDLidar SDK处理YDLidar X3 Pro传感器
- 发布激光扫描到 `/scan` 话题
- 使用真实的YDLidar硬件接口而不是模拟
- 可配置参数: 设备路径、扫描频率、距离限制

### 3. 控制板接口 (`control_board_interface`)
- 通过串口与STM32F407VET6控制板通信
- 订阅 `/cmd_vel` 以接收运动命令
- 发布里程计数据到 `/odom` 话题
- 发布IMU数据到 `/imu/data` 话题

## ROS话题

- `/camera/image_raw` (sensor_msgs/Image) - 原始摄像头图像
- `/scan` (sensor_msgs/LaserScan) - 来自YDLidar的激光扫描数据
- `/cmd_vel` (geometry_msgs/Twist) - 机器人运动的速度命令
- `/odom` (nav_msgs/Odometry) - 来自控制板的里程计数据
- `/imu/data` (sensor_msgs/Imu) - 来自控制板的IMU数据

## ROS参数

### 摄像头参数
- `camera_id`: 摄像头设备ID (默认: 0)
- `frame_rate`: 图像捕获帧率 (默认: 30.0)
- `width`: 图像宽度 (默认: 640)
- `height`: 图像高度 (默认: 480)

### 激光雷达参数 (YDLidar X3 Pro)
- `port`: 激光雷达设备路径 (默认: '/dev/ydlidar')
- `baudrate`: 串口通信波特率 (默认: 230400 for X3 Pro)
- `frame_id`: 激光扫描的坐标系ID (默认: 'laser_link')
- `range_min`: 最小距离 (默认: 0.01m for X3 Pro)
- `range_max`: 最大距离 (默认: 25.0m for X3 Pro)
- `angle_min`: 最小扫描角度 (默认: -180.0度)
- `angle_max`: 最大扫描角度 (默认: 180.0度)
- `frequency`: 扫描频率 (默认: 10.0Hz)
- `isSingleChannel`: 激光雷达是否为单通道 (默认: false for X3 Pro)
- `intensity`: 是否发布强度数据 (默认: false)
- `lidar_type`: 激光雷达类型 (默认: 1 for 三角测距类型)
- `sample_rate`: 激光雷达采样率 (默认: 3)

### 控制板参数
- `serial_port`: 串口通信端口 (默认: '/dev/ttyUSB0')
- `baud_rate`: 串口通信波特率 (默认: 115200)
- `wheel_separation`: 轮子间距 (默认: 0.3)
- `wheel_radius`: 轮子半径 (默认: 0.05)
- `publish_rate`: 里程计发布频率 (默认: 50.0)

## 启动文件

### `sensors.launch.py`
启动所有传感器节点(摄像头、激光雷达、控制板)。

使用方法:
```bash
ros2 launch sensor_interfaces sensors.launch.py
```

### `full_system.launch.py`
启动完整的系统，包括传感器、机器人状态发布器和SLAM。

使用方法:
```bash
ros2 launch sensor_interfaces full_system.launch.py
```

## 硬件连接

- **Jetson Orin Nano** 连接到:
  - USB摄像头(C100)通过USB
  - YDLidar X3 Pro通过USB/UART
  - STM32F407VET6控制板通过UART(GPIO引脚)

- **STM32F407VET6** 控制:
  - 4个麦克纳姆轮通过TB6612驱动器
  - 从Jetson接收运动命令
  - 将里程计和IMU数据发送回Jetson

## 依赖项

- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `std_srvs`
- `serial`
- `OpenCV`
- `YDLidar SDK`(包含在包中)

## YDLidar X3 Pro 集成

激光雷达驱动现在使用官方YDLidar SDK与X3 Pro硬件直接通信。主要特点包括:

- 与YDLidar X3 Pro硬件的直接集成
- 正确处理X3 Pro特定参数
- 从传感器实时获取健康状况和设备信息
- 正确处理传感器数据

## 构建
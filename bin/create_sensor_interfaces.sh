#!/bin/bash
# 创建传感器接口包的一次性脚本

echo "=== 创建传感器接口包 ==="

# 备份现有文件
cp -r /home/chuil/Desktop/zhongxi/src /home/chuil/zhongxibackup/src_backup_$(date +%Y%m%d_%H%M%S)

# 如果包不存在则创建
if [ ! -d "/home/chuil/Desktop/zhongxi/src/sensor_interfaces" ]; then
    cd /home/chuil/Desktop/zhongxi
    ros2 pkg create --build-type ament_cmake sensor_interfaces --dependencies \
        rclcpp sensor_msgs cv_bridge image_transport std_msgs geometry_msgs \
        nav_msgs tf2 tf2_ros std_srvs serial opencv
    echo "传感器接口包创建完成"
else
    echo "传感器接口包已存在，跳过创建"
fi

echo "传感器接口包已准备就绪！"
echo "接下来需要编译项目："
echo "cd /home/chuil/Desktop/zhongxi && colcon build --packages-select sensor_interfaces"
#!/bin/bash
# 构建传感器接口包的脚本

echo "=== 构建传感器接口包 ==="

cd /home/chuil/Desktop/zhongxi

# 检查是否已有sensor_interfaces目录
if [ ! -d "src/sensor_interfaces" ]; then
    echo "错误: sensor_interfaces包不存在"
    echo "请先运行: ./bin/create_sensor_interfaces.sh"
    exit 1
fi

# 构建项目
echo "开始构建项目..."
colcon build --packages-select sensor_interfaces

if [ $? -eq 0 ]; then
    echo "构建成功！"
    echo "传感器接口包已编译完成"
    source install/setup.bash
    echo "环境已更新"
else
    echo "构建失败，请检查错误信息"
    exit 1
fi

echo "构建完成！"
echo "要启动传感器系统，请运行:"
echo "ros2 launch sensor_interfaces sensors.launch.py"
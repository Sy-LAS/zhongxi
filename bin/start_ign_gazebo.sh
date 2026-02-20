#!/bin/bash

# 启动 Ignition Gazebo 仿真脚本
# 适用于 Ignition Gazebo 6.16.0 版本

echo "=== Zhongxi 机器人 Ignition Gazebo 仿真启动脚本 ==="
echo "检测到的 Gazebo 版本: Ignition Gazebo 6.16.0"

# 设置工作目录
cd /home/chuil/Desktop/zhongxi

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "环境设置完成"

# 导出模型路径 - 重要：需要同时设置 Gazebo 和 ROS2 的资源路径
export GZ_SIM_RESOURCE_PATH=/home/chuil/Desktop/zhongxi/src/zhongxi_description:$GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-6/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH

# 添加 Gazebo 模型路径
export GZ_SIM_MODEL_PATH=/home/chuil/Desktop/zhongxi/src/zhongxi_description/models:$GZ_SIM_MODEL_PATH

echo "模型路径设置: $GZ_SIM_RESOURCE_PATH"

# 启动 Ignition Gazebo
echo "正在启动 Ignition Gazebo..."
ign gazebo -v 4 -r &

# 等待 Gazebo 启动
sleep 8

# 检查 Gazebo 是否成功启动
if pgrep -f "ign gazebo" > /dev/null
then
    echo "Gazebo 成功启动，正在插入机器人模型..."
    
    # 自动插入机器人模型
    ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 3000 --req 'sdf_filename: "/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description.urdf" name: "zhongxi_robot" pose: {position: {x: 0, y: 0, z: 0.1}}'
else
    echo "警告: Gazebo 启动失败，请检查错误信息"
fi

# 启动机器人状态发布器
echo "启动机器人状态发布器..."
ros2 run robot_state_publisher robot_state_publisher \
  /home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description.urdf \
  --ros-args -p use_sim_time:=true &

echo "=== 启动完成 ==="
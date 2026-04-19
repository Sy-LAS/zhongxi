#!/bin/bash

# 综合Gazebo仿真启动脚本
# 集成Gazebo、ROS2节点和topic通信

echo "=== Zhongxi机器人综合仿真启动 ==="

# 设置环境
cd /home/chuil/Desktop/zhongxi
source /opt/ros/humble/setup.bash
source install/setup.bash

# 设置Gazebo环境变量
export GZ_SIM_RESOURCE_PATH=/home/chuil/Desktop/zhongxi/src/zhongxi_description:$GZ_SIM_RESOURCE_PATH

echo "环境设置完成"

# 启动Gazebo后台运行
echo "启动Gazebo仿真环境..."
ign gazebo -v 4 -r &
GAZEBO_PID=$!

# 等待Gazebo启动
sleep 5

# 启动robot_state_publisher
echo "启动机器人状态发布器..."
ros2 run robot_state_publisher robot_state_publisher \
  src/zhongxi_description/urdf/zhongxi_description_enhanced.urdf \
  --ros-args -p use_sim_time:=true &
RSP_PID=$!

# 等待节点启动
sleep 3

# 插入机器人模型
echo "插入增强版机器人模型..."
ign service -s /world/default/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description_enhanced.sdf" name: "zhongxi_robot_enhanced" pose: {position: {x: 0, y: 0, z: 0.1}}'

# 启动传感器数据桥接（如果需要）
echo "启动传感器数据桥接..."
# 这里可以添加具体的topic桥接命令

echo "=== 系统启动完成 ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Robot State Publisher PID: $RSP_PID"
echo ""
echo "可用的topic:"
echo "------------------------"
ros2 topic list
echo "------------------------"
echo ""
echo "要关闭系统，请运行:"
echo "kill $GAZEBO_PID $RSP_PID"

# 保持脚本运行
wait
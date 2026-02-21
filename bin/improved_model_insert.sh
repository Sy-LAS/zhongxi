#!/bin/bash

# 改进的Gazebo模型插入脚本
# 解决超时和服务连接问题

echo "=== 改进版Gazebo模型插入 ==="

# 检查Gazebo是否运行
if ! pgrep -f "ign gazebo" > /dev/null; then
    echo "错误: Gazebo未运行"
    echo "请先运行: ign gazebo -v 4 -r"
    exit 1
fi

echo "检测到运行中的Gazebo进程"

# 等待更长时间确保完全初始化
echo "等待Gazebo完全初始化..."
sleep 10

# 方法1: 使用文件路径插入
echo "尝试方法1: 文件路径插入"
ign service -s /world/default/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 10000 \
  --req 'sdf_filename: "/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description_enhanced.sdf" name: "zhongxi_robot_enhanced" pose: {position: {x: 0, y: 0, z: 0.1}}'

if [ $? -eq 0 ]; then
    echo "✓ 模型插入成功"
else
    echo "✗ 方法1失败，尝试方法2"
    
    # 方法2: 直接使用SDF内容
    echo "尝试方法2: 直接SDF内容插入"
    SDF_CONTENT=$(cat /home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description_enhanced.sdf | tr '\n' ' ')
    ign service -s /world/default/create \
      --reqtype ignition.msgs.EntityFactory \
      --reptype ignition.msgs.Boolean \
      --timeout 10000 \
      --req "sdf: \"$SDF_CONTENT\" name: \"zhongxi_robot_enhanced\" pose: {position: {x: 1, y: 0, z: 0.1}}"
      
    if [ $? -eq 0 ]; then
        echo "✓ 模型插入成功"
    else
        echo "✗ 两种方法都失败"
        echo "建议: 在Gazebo GUI中手动拖拽模型文件"
    fi
fi

echo "=== 操作完成 ==="
echo "检查Gazebo GUI中是否出现模型"
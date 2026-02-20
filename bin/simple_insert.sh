#!/bin/bash

# 最简化的Gazebo模型插入方法
echo "=== Gazebo模型插入 ==="
sleep 3

# 插入测试方块
ign service -s /world/default/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf: "<model name=\"test\"><link name=\"link\"><visual name=\"visual\"><geometry><box><size>0.2 0.2 0.2</size></box></geometry><material><ambient>1 0 0 1</ambient></material></visual></link></model>" name: "red_box"'

echo "测试模型插入完成，请检查Gazebo界面"

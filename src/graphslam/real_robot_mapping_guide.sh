#!/bin/bash
# 真机SLAM建图操作指南

echo "=== 真机SLAM建图操作指南 ==="

echo "1. 系统启动准备："
echo "   ├── 确保Jetson电源连接稳定"
echo "   ├── 检查激光雷达USB连接"
echo "   └── 验证ROS2环境 sourced"

echo -e "\n2. 启动SLAM系统："
echo "   # 终端1：启动核心SLAM节点"
echo "   source ~/Desktop/zhongxi/install/setup.bash"
echo "   ros2 run graphslam graphslam_node"
echo ""
echo "   # 终端2：启动可视化界面（可选）"
echo "   ros2 run graphslam graphslam_g2o"

echo -e "\n3. 手持建图操作："
echo "   ├── 握住小车保持稳定"
echo "   ├── 以0.5-1m/s速度缓慢移动"
echo "   ├── 覆盖整个建图区域"
echo "   ├── 注意避免急转弯和快速移动"
echo "   └── 确保激光雷达视野内无障碍物遮挡"

echo -e "\n4. 实时监控："
echo "   ├── 观察GUI界面的地图构建进度"
echo "   ├── 确认轨迹显示连续无跳跃"
echo "   └── 检查是否有异常的位姿估计"

echo -e "\n5. 建图完成："
echo "   ├── 回到起始位置形成闭环"
echo "   ├── 保存最终地图"
echo "   └── 关闭系统"

echo -e "\n注意事项："
echo "⚠️  移动过程中保持匀速，避免突然加速/减速"
echo "⚠️  转弯时半径不要过小（建议>1米）"
echo "⚠️  环境光照变化可能影响激光雷达性能"
echo "⚠️  狭窄通道可能造成特征匹配困难"
echo "⚠️  建议先在小范围内测试系统稳定性"

echo -e "\n预期效果："
echo "✅ 实时显示2D环境地图"
echo "✅ 绿色轨迹表示机器人的历史路径"  
echo "✅ 红色点表示当前位姿估计"
echo "✅ 地图随你的移动实时更新扩展"
echo "✅ 最终获得完整的环境occupancy grid地图"
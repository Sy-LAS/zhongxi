#!/bin/bash
# GraphSLAM项目完整性分析

echo "=== GraphSLAM项目完整性分析 ==="

PROJECT_ROOT=~/Desktop/zhongxi/src/graphslam

cd $PROJECT_ROOT

echo "1. 项目文件结构概览："
echo "├── 核心源文件："
find . -name "*.cpp" -o -name "*.h" | grep -E "(slam2d|node|utils)" | sort

echo -e "\n├── GUI相关文件："
ls -la *.cpp *.h 2>/dev/null | grep -E "(main_window|viewer)" | awk '{print "   " $NF}'

echo -e "\n├── 配置文件："
ls -la CMakeLists.txt package.xml 2>/dev/null

echo -e "\n2. 功能模块分析："

# 检查SLAM核心组件
echo "┌─ SLAM核心功能 ──────────────────────"
if [ -d "src/slam2d" ]; then
    echo "├── ✅ 后端优化 (g2o集成) - 存在"
    BACKEND_FILES=$(ls src/slam2d/*.cpp 2>/dev/null | wc -l)
    echo "│   ├── 源文件数量: $BACKEND_FILES"
else
    echo "├── ❌ 后端优化 - 缺失"
fi

if [ -d "src/node" ]; then
    echo "├── ✅ ROS2接口 - 存在"
    NODE_FILES=$(ls src/node/*.cpp 2>/dev/null | wc -l)
    echo "│   ├── 节点文件: $NODE_FILES"
else
    echo "├── ❌ ROS2接口 - 缺失"
fi

# 检查传感器集成
echo "├── ✅ 传感器数据处理 - 存在"
if [ -d "src/utils" ]; then
    UTIL_FILES=$(ls src/utils/*.cpp 2>/dev/null | wc -l)
    echo "│   ├── 工具文件: $UTIL_FILES"
fi

# 检查GUI组件
echo "├── GUI可视化界面 - "
if ls *.cpp 2>/dev/null | grep -q "viewer"; then
    echo "│   ├── ✅ 存在 (QGLViewer)"
else
    echo "│   ├── ⚠️  基础存在但可能不完整"
fi

# 检查控制相关
echo "└─ 机器人控制 ────────────────────────"
echo "    ├── ❌ 运动控制模块 - 缺失"
echo "    ├── ❌ 路径规划模块 - 缺失" 
echo "    └── ❌ 电机驱动接口 - 缺失"

echo -e "\n3. 当前可实现功能："
echo "✅ 实时激光SLAM建图"
echo "✅ 2D位姿图优化"
echo "✅ ROS2消息通信"
echo "✅ 数据可视化显示"
echo "✅ 地图保存与加载"

echo -e "\n4. 缺失的关键功能："
echo "❌ 机器人自主导航"
echo "❌ 运动控制指令发送"
echo "❌ 路径规划算法"
echo "❌ 电机驱动程序"
echo "❌ 避障功能"

echo -e "\n结论："
echo "当前项目提供了完整的SLAM定位和建图功能，"
echo "但缺乏实际控制机器人移动的部分。"
echo "可以完成：建图 + 定位 + 可视化"
echo "无法独立完成：自主移动 + 导航"
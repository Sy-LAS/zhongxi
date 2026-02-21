#!/bin/bash
# Jetson Orin Nano 依赖检查脚本

echo "=== GraphSLAM 依赖检查 ==="

# 检查基本工具
echo "检查基本编译工具..."
which gcc > /dev/null && echo "✓ GCC: $(gcc --version | head -n1)" || echo "✗ GCC 未安装"
which g++ > /dev/null && echo "✓ G++: $(g++ --version | head -n1)" || echo "✗ G++ 未安装"
which cmake > /dev/null && echo "✓ CMake: $(cmake --version | head -n1)" || echo "✗ CMake 未安装"
which make > /dev/null && echo "✓ Make: $(make --version | head -n1)" || echo "✗ Make 未安装"

echo ""
echo "检查ROS2环境..."
if [ -n "$ROS_DISTRO" ]; then
    echo "✓ ROS2 发行版: $ROS_DISTRO"
    echo "✓ ROS2 环境已设置"
else
    echo "✗ ROS2 环境未设置"
    echo "请运行: source /opt/ros/$ROS_DISTRO/setup.bash"
fi

echo ""
echo "检查Qt5..."
pkg-config --exists Qt5Core && echo "✓ Qt5 已安装" || echo "✗ Qt5 未安装"

echo ""
echo "检查QGLViewer..."
dpkg -l | grep -i qglviewer > /dev/null && echo "✓ QGLViewer 已安装" || echo "✗ QGLViewer 未安装"
echo "建议安装: sudo apt install libqglviewer-dev-qt5"

echo ""
echo "检查g2o..."
pkg-config --exists g2o-core && echo "✓ g2o 已安装" || echo "✗ g2o 未安装"
echo "建议安装: sudo apt install libg2o-dev"

echo ""
echo "检查Eigen3..."
pkg-config --exists eigen3 && echo "✓ Eigen3 已安装" || echo "✗ Eigen3 未安装"
echo "建议安装: sudo apt install libeigen3-dev"

echo ""
echo "=== 建议的安装命令 ==="
echo "sudo apt update"
echo "sudo apt install build-essential cmake"
echo "sudo apt install qtbase5-dev libqt5opengl5-dev"
echo "sudo apt install libqglviewer-dev-qt5"
echo "sudo apt install libg2o-dev"
echo "sudo apt install libeigen3-dev"

echo ""
echo "如果使用ROS2，请确保已安装:"
echo "sudo apt install ros-$ROS_DISTRO-desktop"
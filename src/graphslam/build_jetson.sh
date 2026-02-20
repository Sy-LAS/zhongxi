#!/bin/bash
# Jetson Orin Nano 编译脚本

echo "开始编译 GraphSLAM 项目..."

# 创建构建目录
mkdir -p build
cd build

# 清理之前的构建
rm -rf *

# 配置项目
echo "配置 CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17

# 编译项目
echo "开始编译..."
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "编译成功完成！"
    echo "可执行文件位置："
    find . -name "graphslam*" -type f -executable
else
    echo "编译失败，请检查错误信息"
    exit 1
fi
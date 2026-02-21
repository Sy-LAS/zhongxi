#!/bin/bash
# 在Jetson上执行此脚本以诊断GraphSLAM编译问题

echo "=== GraphSLAM 编译问题诊断 ==="

cd ~/Desktop/zhongxi/src/graphslam

echo "1. 检查源文件结构..."
echo "src/node/ 目录内容："
ls -la src/node/

echo -e "\n2. 检查主函数定义..."
# 查找包含main函数的文件
grep -rn "int main" src/node/ || echo "未找到main函数定义"

echo -e "\n3. 检查CMakeLists.txt中的源文件配置..."
grep -A 10 -B 5 "NODE_SOURCES" CMakeLists.txt

echo -e "\n4. 检查package.xml..."
cat package.xml

echo -e "\n5. 检查编译错误详情..."
mkdir -p build_debug
cd build_debug
cmake .. -DCMAKE_BUILD_TYPE=Debug
make VERBOSE=1 2>&1 | grep -A 20 -B 5 "undefined reference to.*main"
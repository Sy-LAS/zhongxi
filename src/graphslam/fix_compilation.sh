#!/bin/bash
# GraphSLAM 编译问题修复脚本

echo "开始修复 GraphSLAM 编译问题..."

cd ~/Desktop/zhongxi/src/graphslam

# 1. 修复包名问题（临时方案）
echo "1. 检查并修复包名问题..."
CURRENT_NAME=$(grep "<name>" package.xml | sed 's/.*<name>\(.*\)<\/name>.*/\1/')
if [ "$CURRENT_NAME" = "graphslam" ]; then
    echo "检测到包名问题，创建软链接作为临时解决方案..."
    # 在ROS2工作空间中创建小写名称的软链接
    cd ~/Desktop/zhongxi/src/
    ln -sf graphslam graph_slam 2>/dev/null || echo "软链接已存在或创建失败"
fi

# 2. 检查node目录中的主函数
echo "2. 检查node源文件..."
cd ~/Desktop/zhongxi/src/graphslam

if [ ! -d "src/node" ]; then
    echo "错误：src/node 目录不存在"
    exit 1
fi

MAIN_FILE=""
for file in src/node/*.cpp; do
    if grep -q "int main" "$file" 2>/dev/null; then
        MAIN_FILE="$file"
        echo "找到主函数文件: $file"
        break
    fi
done

if [ -z "$MAIN_FILE" ]; then
    echo "错误：未找到包含main函数的源文件"
    echo "请检查 src/node/ 目录中的文件"
    exit 1
fi

# 3. 更新CMakeLists.txt确保包含主文件
echo "3. 验证CMake配置..."
if ! grep -q "$(basename $MAIN_FILE)" CMakeLists.txt; then
    echo "警告：CMakeLists.txt可能未包含主文件"
    echo "请检查 NODE_SOURCES 的文件匹配模式"
fi

# 4. 清理并重新编译
echo "4. 清理构建目录..."
cd ~/Desktop/zhongxi
rm -rf build/graphslam install/graphslam log/graphslam

echo "5. 重新编译..."
colcon build --packages-select graphslam --cmake-clean-cache

if [ $? -eq 0 ]; then
    echo "✅ 编译成功完成！"
else
    echo "❌ 编译仍然失败，请查看详细错误信息"
fi
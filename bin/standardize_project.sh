#!/bin/bash

# ROS项目脚本标准化整理脚本

PROJECT_DIR="/home/chuil/Desktop/zhongxi"
BIN_DIR="$PROJECT_DIR/bin"
TOOLS_DIR="$PROJECT_DIR/tools"

echo "=== ROS项目脚本标准化整理 ==="

# 创建标准目录结构
echo "1. 创建标准目录..."
mkdir -p "$BIN_DIR"
mkdir -p "$TOOLS_DIR"

# 移动启动脚本到bin目录
echo "2. 整理启动脚本..."

# GraphSLAM相关脚本
if [ -f "$PROJECT_DIR/run_graphslam.sh" ]; then
    mv "$PROJECT_DIR/run_graphslam.sh" "$BIN_DIR/graphslam"
    echo "   ✓ run_graphslam.sh → $BIN_DIR/graphslam"
fi

if [ -f "$PROJECT_DIR/run_graphslam_complete.sh" ]; then
    mv "$PROJECT_DIR/run_graphslam_complete.sh" "$BIN_DIR/graphslam-complete"
    echo "   ✓ run_graphslam_complete.sh → $BIN_DIR/graphslam-complete"
fi

# 仿真相关脚本
if [ -f "$PROJECT_DIR/simulations/rviz/run_rviz.sh" ]; then
    mv "$PROJECT_DIR/simulations/rviz/run_rviz.sh" "$BIN_DIR/rviz"
    echo "   ✓ run_rviz.sh → $BIN_DIR/rviz"
fi

if [ -f "$PROJECT_DIR/start_simulation.sh" ]; then
    mv "$PROJECT_DIR/start_simulation.sh" "$BIN_DIR/simulation"
    echo "   ✓ start_simulation.sh → $BIN_DIR/simulation"
fi

# 工具脚本
if [ -f "$PROJECT_DIR/nomachine_recovery.sh" ]; then
    mv "$PROJECT_DIR/nomachine_recovery.sh" "$TOOLS_DIR/recovery"
    echo "   ✓ nomachine_recovery.sh → $TOOLS_DIR/recovery"
fi

# 添加执行权限
chmod +x "$BIN_DIR"/* 2>/dev/null
chmod +x "$TOOLS_DIR"/* 2>/dev/null

# 创建使用说明
echo "3. 创建使用说明..."
cat > "$PROJECT_DIR/USAGE.md" << EOF
# 项目使用说明

## 可执行脚本 (bin/)
- \`./bin/graphslam\` - 启动GraphSLAM程序
- \`./bin/graphslam-complete\` - 完整环境的GraphSLAM启动
- \`./bin/rviz\` - 启动Rviz可视化
- \`./bin/simulation\` - 仿真环境启动器

## 工具脚本 (tools/)
- \`./tools/recovery\` - NoMachine故障恢复

## ROS 2包 (src/)
标准ROS 2包结构，使用colcon构建

## 使用示例:
\`\`\`bash
cd /home/chuil/Desktop/zhongxi
./bin/rviz                    # 启动可视化
./bin/graphslam              # 启动SLAM
./tools/recovery             # 系统恢复
\`\`\`
EOF

# 显示新的目录结构
echo "4. 新的项目结构:"
echo "   项目根目录: $PROJECT_DIR"
echo "   可执行脚本: $BIN_DIR"
ls -la "$BIN_DIR" 2>/dev/null
echo "   工具脚本: $TOOLS_DIR"  
ls -la "$TOOLS_DIR" 2>/dev/null

echo ""
echo "=== 整理完成 ==="
echo "查看使用说明: cat $PROJECT_DIR/USAGE.md"
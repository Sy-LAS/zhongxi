#!/bin/bash

# Jetson与STM32通信检查脚本
# 用于检查串口连接、STM32通信状态以及相关硬件配置

echo "==========================================="
echo "    Jetson与STM32通信状态检查脚本"
echo "==========================================="

# 创建日志目录和文件
LOG_DIR="/tmp/jetson_stm32_check"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/check_$(date +%Y%m%d_%H%M%S).log"
echo "检查日志将保存到: $LOG_FILE"

# 函数：记录日志
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_message "开始执行Jetson与STM32通信检查"

# 检查1：串口设备
echo ""
echo "1. 检查串口设备..."
log_message "检查串口设备"
AVAILABLE_PORTS=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "无可用串口设备")
echo "可用的串口设备:"
echo "$AVAILABLE_PORTS"
log_message "可用串口设备: $AVAILABLE_PORTS"

# 检查默认串口设备
DEFAULT_PORT="/dev/ttyUSB0"
if [ -e "$DEFAULT_PORT" ]; then
    echo "✓ $DEFAULT_PORT 存在"
    log_message "$DEFAULT_PORT 存在"
    PORT_EXISTS=1
else
    echo "✗ $DEFAULT_PORT 不存在"
    log_message "$DEFAULT_PORT 不存在"
    PORT_EXISTS=0
fi

# 检查2：串口库安装
echo ""
echo "2. 检查串口库安装..."
log_message "检查串口库安装"
if dpkg -l | grep -q libserial-dev; then
    echo "✓ libserial-dev 已安装"
    log_message "libserial-dev 已安装"
    SERIAL_LIB_INSTALLED=1
else
    echo "✗ libserial-dev 未安装"
    log_message "libserial-dev 未安装"
    SERIAL_LIB_INSTALLED=0
fi

if python3 -c "import serial" &>/dev/null; then
    echo "✓ pyserial 已安装"
    log_message "pyserial 已安装"
    PY_SERIAL_INSTALLED=1
else
    echo "✗ pyserial 未安装"
    log_message "pyserial 未安装"
    PY_SERIAL_INSTALLED=0
fi

# 检查3：ROS2环境
echo ""
echo "3. 检查ROS2环境..."
log_message "检查ROS2环境"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble 已安装"
    log_message "ROS2 Humble 已安装"
    ROS2_INSTALLED=1
else
    echo "✗ ROS2 Humble 未安装"
    log_message "ROS2 Humble 未安装"
    ROS2_INSTALLED=0
fi

# 检查工作空间
if [ -f "/home/chuil/Desktop/zhongxi/install/setup.bash" ]; then
    echo "✓ zhongxi工作空间已构建"
    log_message "zhongxi工作空间已构建"
    WORKSPACE_BUILT=1
else
    echo "✗ zhongxi工作空间未构建"
    log_message "zhongxi工作空间未构建"
    WORKSPACE_BUILT=0
fi

# 检查4：传感器接口包
echo ""
echo "4. 检查传感器接口包..."
log_message "检查传感器接口包"
if [ -f "/home/chuil/Desktop/zhongxi/install/sensor_interfaces/lib/sensor_interfaces/control_board_interface" ] || [ -f "/home/chuil/Desktop/zhongxi/build/sensor_interfaces/control_board_interface" ]; then
    echo "✓ control_board_interface 可执行文件存在"
    log_message "control_board_interface 可执行文件存在"
    CB_INTERFACE_EXISTS=1
else
    echo "✗ control_board_interface 可执行文件不存在"
    log_message "control_board_interface 可执行文件不存在"
    CB_INTERFACE_EXISTS=0
fi

# 检查5：运行中的ROS2节点
echo ""
echo "5. 检查ROS2节点状态..."
log_message "检查ROS2节点状态"
if command -v ros2 &>/dev/null; then
    RUNNING_NODES=$(timeout 5s ros2 node list 2>/dev/null || echo "无法获取节点列表")
    echo "当前运行的ROS2节点:"
    echo "$RUNNING_NODES"
    log_message "当前运行的ROS2节点: $RUNNING_NODES"
else
    echo "✗ ROS2未正确安装或未添加到PATH"
    log_message "ROS2未正确安装或未添加到PATH"
fi

# 检查6：串口连接测试（如果端口存在）
echo ""
echo "6. 测试串口连接..."
log_message "测试串口连接"
if [ $PORT_EXISTS -eq 1 ]; then
    # 尝试打开串口并设置参数
    if timeout 3s stty -F "$DEFAULT_PORT" 115200 cs8 -cstopb -parenb 2>/dev/null; then
        echo "✓ 成功设置 $DEFAULT_PORT 参数为115200波特率"
        log_message "成功设置 $DEFAULT_PORT 参数为115200波特率"
        
        # 尝试读取数据
        echo "  正在尝试从串口读取数据 (3秒)..."
        READ_DATA=$(timeout 3s cat "$DEFAULT_PORT" 2>/dev/null | head -c 100)
        if [ -n "$READ_DATA" ]; then
            echo "  从串口读取到数据: $(echo "$READ_DATA" | tr '\r\n' ' ')"
            log_message "从串口读取到数据: $(echo "$READ_DATA" | tr '\r\n' ' ')"
        else
            echo "  未从串口读取到数据（可能是正常的，取决于STM32是否主动发送数据）"
            log_message "未从串口读取到数据"
        fi
    else
        echo "✗ 无法设置 $DEFAULT_PORT 参数，可能已被占用或设备不支持"
        log_message "无法设置 $DEFAULT_PORT 参数"
    fi
else
    echo "  跳过串口测试（端口不存在）"
    log_message "跳过串口测试"
fi

# 检查7：STM32通信协议测试
echo ""
echo "7. 检查STM32通信协议配置..."
log_message "检查STM32通信协议配置"
if [ -f "/home/chuil/Desktop/zhongxi/src/sensor_interfaces/include/sensor_interfaces/stm32_protocol.h" ]; then
    echo "✓ STM32通信协议头文件存在"
    log_message "STM32通信协议头文件存在"
    
    # 显示协议定义
    PROTOCOL_START=$(grep -o '#define.*START_BYTE.*' "/home/chuil/Desktop/zhongxi/src/sensor_interfaces/include/sensor_interfaces/stm32_protocol.h" | head -1)
    PROTOCOL_END=$(grep -o '#define.*END_BYTE.*' "/home/chuil/Desktop/zhongxi/src/sensor_interfaces/include/sensor_interfaces/stm32_protocol.h" | head -1)
    echo "  协议起始字符: $PROTOCOL_START"
    echo "  协议结束字符: $PROTOCOL_END"
    log_message "协议定义 - 起始: $PROTOCOL_START, 结束: $PROTOCOL_END"
else
    echo "✗ STM32通信协议头文件不存在"
    log_message "STM32通信协议头文件不存在"
fi

# 检查8：系统资源
echo ""
echo "8. 检查系统资源..."
log_message "检查系统资源"
CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')
MEM_INFO=$(free -h | grep "^Mem:")
echo "  CPU使用率: $CPU_USAGE%"
echo "  内存信息: $MEM_INFO"
log_message "CPU使用率: $CPU_USAGE%, 内存信息: $MEM_INFO"

# 汇总报告
echo ""
echo "==========================================="
echo "              检查汇总报告"
echo "==========================================="

TOTAL_CHECKS=8
PASSED_CHECKS=0

if [ $PORT_EXISTS -eq 1 ]; then
    printf "%-30s [✓ PASS]\n" "串口设备检查"
    ((PASSED_CHECKS++))
else
    printf "%-30s [✗ FAIL]\n" "串口设备检查"
fi

if [ $SERIAL_LIB_INSTALLED -eq 1 ]; then
    printf "%-30s [✓ PASS]\n" "串口库安装检查"
    ((PASSED_CHECKS++))
else
    printf "%-30s [✗ FAIL]\n" "串口库安装检查"
fi

if [ $PY_SERIAL_INSTALLED -eq 1 ]; then
    printf "%-30s [✓ PASS]\n" "Python串口库检查"
    ((PASSED_CHECKS++))
else
    printf "%-30s [✗ FAIL]\n" "Python串口库检查"
fi

if [ $ROS2_INSTALLED -eq 1 ]; then
    printf "%-30s [✓ PASS]\n" "ROS2环境检查"
    ((PASSED_CHECKS++))
else
    printf "%-30s [✗ FAIL]\n" "ROS2环境检查"
fi

if [ $WORKSPACE_BUILT -eq 1 ]; then
    printf "%-30s [✓ PASS]\n" "工作空间构建检查"
    ((PASSED_CHECKS++))
else
    printf "%-30s [✗ FAIL]\n" "工作空间构建检查"
fi

if [ $CB_INTERFACE_EXISTS -eq 1 ]; then
    printf "%-30s [✓ PASS]\n" "控制板接口检查"
    ((PASSED_CHECKS++))
else
    printf "%-30s [✗ FAIL]\n" "控制板接口检查"
fi

printf "%-30s [? INFO]\n" "节点运行状态检查"

if [ $PORT_EXISTS -eq 1 ]; then
    printf "%-30s [? INFO]\n" "串口连接测试"
else
    printf "%-30s [? SKIP]\n" "串口连接测试"
fi

printf "%-30s [? INFO]\n" "协议配置检查"
printf "%-30s [? INFO]\n" "系统资源检查"

echo ""
echo "通过检查: $PASSED_CHECKS/$TOTAL_CHECKS"
PERCENTAGE=$((PASSED_CHECKS * 100 / TOTAL_CHECKS))
echo "成功率: $PERCENTAGE%"

log_message "检查完成，通过$PASSED_CHECKS/$TOTAL_CHECKS项检查，成功率$PERCENTAGE%"

# 提供建议
echo ""
echo "==========================================="
echo "              改进建议"
echo "==========================================="

if [ $SERIAL_LIB_INSTALLED -eq 0 ]; then
    echo "- 运行以下命令安装串口库:"
    echo "  sudo apt update && sudo apt install libserial-dev"
    echo ""
fi

if [ $PY_SERIAL_INSTALLED -eq 0 ]; then
    echo "- 安装Python串口库:"
    echo "  pip3 install pyserial"
    echo ""
fi

if [ $WORKSPACE_BUILT -eq 0 ]; then
    echo "- 构建工作空间:"
    echo "  cd /home/chuil/Desktop/zhongxi && colcon build --packages-select sensor_interfaces"
    echo ""
fi

if [ $PORT_EXISTS -eq 0 ]; then
    echo "- 检查STM32是否正确连接到Jetson，确认USB线缆连接正常"
    echo "- 或者检查是否需要安装STM32的USB转串口驱动（如CH340/CH341芯片驱动）"
    echo ""
fi

echo "- 如需启动机器人系统，运行:"
echo "  /home/chuil/Desktop/zhongxi/bin/start_robot_system.sh"
echo ""

echo "检查完成！日志文件: $LOG_FILE"
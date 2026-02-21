#!/bin/bash

# STM32通信诊断脚本
# 提供详细的诊断信息帮助解决通信问题

echo "==========================================="
echo "        STM32通信诊断脚本"
echo "==========================================="

# 创建日志目录和文件
LOG_DIR="/tmp/stm32_diagnosis"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/diagnosis_$(date +%Y%m%d_%H%M%S).log"
echo "诊断日志将保存到: $LOG_FILE"

# 函数：记录日志
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_message "开始执行STM32通信诊断"

# 1. 检查系统串口设备
echo ""
echo "1. 检查系统串口设备..."
log_message "检查系统串口设备"
echo "列出所有tty设备:"
ls -la /dev/tty* | grep -E "(USB|ACM|S|AMA)" | tee -a "$LOG_FILE"

# 检查USB设备
echo ""
echo "2. 检查USB设备..."
log_message "检查USB设备"
USB_DEVICES=$(lsusb 2>/dev/null || echo "lsusb command not found")
echo "$USB_DEVICES" | tee -a "$LOG_FILE"
echo "查找可能的串口转换芯片:"
echo "$USB_DEVICES" | grep -i -E "(ch34|ftdi|cp210|pl230)" | tee -a "$LOG_FILE"

# 3. 检查内核消息
echo ""
echo "3. 检查内核消息..."
log_message "检查内核消息"
echo "最近的串口相关内核消息:"
dmesg | grep -i -E "(tty|serial|usb|ch341|ftdi|cp210)" | tail -10 | tee -a "$LOG_FILE"

# 4. 检查串口驱动
echo ""
echo "4. 检查串口驱动..."
log_message "检查串口驱动"
echo "已加载的串口相关模块:"
lsmod | grep -i -E "(usbserial|ch341|ftdi_sio|cp210x|pl2303)" | tee -a "$LOG_FILE"

# 5. 检查Python串口库
echo ""
echo "5. 检查Python串口库..."
log_message "检查Python串口库"
PYTHON_SERIAL_TEST=$(python3 -c "import serial; print('PySerial version:', serial.__version__)" 2>&1)
if [[ $? -eq 0 ]]; then
    echo "✓ PySerial已安装: $PYTHON_SERIAL_TEST" | tee -a "$LOG_FILE"
else
    echo "✗ PySerial未安装或存在问题: $PYTHON_SERIAL_TEST" | tee -a "$LOG_FILE"
fi

# 6. 检查串口权限
echo ""
echo "6. 检查串口权限..."
log_message "检查串口权限"
CURRENT_USER=$(whoami)
echo "当前用户: $CURRENT_USER"
GROUPS=$(groups "$CURRENT_USER")
echo "用户组: $GROUPS" | tee -a "$LOG_FILE"

# 检查用户是否在dialout组中
if echo "$GROUPS" | grep -q "dialout"; then
    echo "✓ 用户在dialout组中" | tee -a "$LOG_FILE"
else
    echo "✗ 用户不在dialout组中，可能需要添加到该组：" | tee -a "$LOG_FILE"
    echo "  sudo usermod -a -G dialout $CURRENT_USER" | tee -a "$LOG_FILE"
fi

# 7. 检查STM32固件兼容性
echo ""
echo "7. 检查STM32固件兼容性..."
log_message "检查STM32固件兼容性"
echo "检查协议定义:"
if [ -f "/home/chuil/Desktop/zhongxi/src/sensor_interfaces/include/sensor_interfaces/stm32_protocol.h" ]; then
    echo "  协议文件存在，检查定义..."
    grep -A 5 -B 5 "START_BYTE\|END_BYTE\|SEPARATOR" "/home/chuil/Desktop/zhongxi/src/sensor_interfaces/include/sensor_interfaces/stm32_protocol.h" | tee -a "$LOG_FILE"
else
    echo "  协议文件不存在" | tee -a "$LOG_FILE"
fi

# 8. 检查ROS2节点状态
echo ""
echo "8. 检查ROS2节点状态..."
log_message "检查ROS2节点状态"
if command -v ros2 &>/dev/null; then
    NODES=$(timeout 5s ros2 node list 2>/dev/null || echo "无法获取节点列表")
    echo "当前ROS2节点:" | tee -a "$LOG_FILE"
    echo "$NODES" | tee -a "$LOG_FILE"
    
    # 检查话题
    TOPICS=$(timeout 5s ros2 topic list 2>/dev/null || echo "无法获取话题列表")
    echo "当前ROS2话题:" | tee -a "$LOG_FILE"
    echo "$TOPICS" | tee -a "$LOG_FILE"
else
    echo "ROS2未正确安装或未添加到PATH" | tee -a "$LOG_FILE"
fi

# 9. 检查控制板接口配置
echo ""
echo "9. 检查控制板接口配置..."
log_message "检查控制板接口配置"
CONFIG_FILE="/home/chuil/Desktop/zhongxi/src/sensor_interfaces/config/control_board_params.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "配置文件存在，内容如下:" | tee -a "$LOG_FILE"
    cat "$CONFIG_FILE" | tee -a "$LOG_FILE"
else
    echo "配置文件不存在，使用默认参数" | tee -a "$LOG_FILE"
fi

# 10. 检查网络和系统资源
echo ""
echo "10. 检查系统资源..."
log_message "检查系统资源"
echo "内存使用情况:" | tee -a "$LOG_FILE"
free -h | tee -a "$LOG_FILE"

echo "磁盘使用情况:" | tee -a "$LOG_FILE"
df -h | grep -E "(/$|/home)" | tee -a "$LOG_FILE"

echo "CPU使用情况:" | tee -a "$LOG_FILE"
top -bn1 | head -5 | tee -a "$LOG_FILE"

# 汇总报告
echo ""
echo "==========================================="
echo "              诊断汇总"
echo "==========================================="

echo "检查完成，以下是主要发现："
echo ""

# 检查是否安装了串口驱动
DRIVER_PRESENT=0
if lsmod | grep -q -E "(usbserial|ch341|ftdi_sio|cp210x|pl2303)"; then
    DRIVER_PRESENT=1
    echo "✓ 串口驱动已加载"
else
    echo "✗ 串口驱动未加载，可能需要安装"
fi

# 检查是否有可用串口
AVAILABLE_PORTS=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l)
if [ $AVAILABLE_PORTS -gt 0 ]; then
    echo "✓ 检测到 $AVAILABLE_PORTS 个可用串口设备"
else
    echo "✗ 未检测到串口设备，检查硬件连接"
fi

# 检查USB设备中是否有串口转换芯片
USB_SERIAL_CHIP=$(echo "$USB_DEVICES" | grep -i -E "(ch34|ftdi|cp210|pl230)" | wc -l)
if [ $USB_SERIAL_CHIP -gt 0 ]; then
    echo "✓ 检测到 $USB_SERIAL_CHIP 个USB串口转换设备"
else
    echo "✗ 未检测到USB串口转换设备"
fi

# 检查用户权限
if echo "$GROUPS" | grep -q "dialout"; then
    echo "✓ 用户权限正常"
else
    echo "✗ 用户可能需要添加到dialout组"
fi

# 检查PySerial
if [[ $? -eq 0 ]]; then
    echo "✓ PySerial已安装"
else
    echo "✗ PySerial未安装"
fi

echo ""
echo "==========================================="
echo "              改进建议"
echo "==========================================="

if [ $DRIVER_PRESENT -eq 0 ]; then
    echo "- 安装串口驱动:"
    echo "  sudo apt update && sudo apt install usbutils"
    echo "  对于CH340/CH341芯片: 可能需要手动安装驱动"
    echo ""
fi

if [ $AVAILABLE_PORTS -eq 0 ]; then
    echo "- 检查STM32硬件连接:"
    echo "  1. 确认USB线缆连接正常"
    echo "  2. 检查STM32是否已供电"
    echo "  3. 确认STM32上的USB转串口芯片型号"
    echo ""
fi

if [ $USB_SERIAL_CHIP -eq 0 ]; then
    echo "- 如果使用USB转TTL模块，请确认型号并安装相应驱动:"
    echo "  CH340/CH341: sudo apt install python3-serial"
    echo "  CP210x: 可能需要添加udev规则"
    echo "  FTDI: 需要安装ftdi-sio模块"
    echo ""
fi

if ! echo "$GROUPS" | grep -q "dialout"; then
    echo "- 将用户添加到dialout组:"
    echo "  sudo usermod -a -G dialout $CURRENT_USER"
    echo "  注意：需要重新登录或重启后生效"
    echo ""
fi

if ! python3 -c "import serial" &>/dev/null; then
    echo "- 安装Python串口库:"
    echo "  pip3 install pyserial"
    echo ""
fi

echo "- 启动机器人系统前，请确保STM32已连接并运行正确的固件"
echo "- 可以使用以下命令测试连接:"
echo "  python3 /home/chuil/Desktop/zhongxi/bin/test_stm32_connection.py --scan"
echo ""
echo "诊断完成！日志文件: $LOG_FILE"
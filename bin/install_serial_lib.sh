#!/bin/bash

# 安装串口通信库的脚本
echo "正在安装串口通信库..."

# 更新包列表
sudo apt update

# 安装串口库及其开发包
sudo apt install -y libserial-dev libserial-doc

# 安装Python串口库（以防需要Python串口通信）
pip3 install pyserial

echo "串口通信库安装完成！"
echo "现在可以使用串口与STM32F407VET6通信了"
#!/usr/bin/env python3

"""
STM32连接测试脚本
用于测试Jetson与STM32之间的串口通信
"""

import serial
import time
import sys
import argparse
from pathlib import Path


def test_stm32_connection(port='/dev/ttyUSB0', baudrate=115200):
    """
    测试与STM32的连接
    
    Args:
        port: 串口设备路径
        baudrate: 波特率
    """
    print(f"正在测试STM32连接: {port} @ {baudrate} baud")
    
    try:
        # 尝试打开串口
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"✓ 成功打开串口 {port}")
        
        # 等待一点时间让连接稳定
        time.sleep(1)
        
        # 发送测试命令 - 获取状态
        status_cmd = "#6!"  # GET_STATUS命令
        print(f"发送状态查询命令: {status_cmd}")
        ser.write(status_cmd.encode())
        
        # 等待响应
        time.sleep(0.5)
        
        # 检查是否有数据可读
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode().strip()
            if response:
                print(f"✓ 收到STM32响应: {response}")
            else:
                print("? 未收到STM32响应，可能设备未就绪")
        else:
            print("? 未收到STM32响应，可能设备未就绪或命令不支持")
        
        # 发送重置里程计命令
        reset_cmd = "#3!"  # RESET_ODOM命令
        print(f"发送重置里程计命令: {reset_cmd}")
        ser.write(reset_cmd.encode())
        
        # 关闭串口
        ser.close()
        print(f"✓ 已关闭串口 {port}")
        
        return True
        
    except serial.SerialException as e:
        print(f"✗ 串口错误: {e}")
        return False
    except Exception as e:
        print(f"✗ 连接测试失败: {e}")
        return False


def find_serial_ports():
    """查找系统上的所有串口设备"""
    import glob
    
    ports = []
    # 查找USB串口
    ports.extend(glob.glob('/dev/ttyUSB*'))
    # 查找ACM串口（如Arduino）
    ports.extend(glob.glob('/dev/ttyACM*'))
    # 查找其他串口
    ports.extend(glob.glob('/dev/ttyS*'))
    
    return sorted(ports)


def main():
    parser = argparse.ArgumentParser(description='STM32连接测试脚本')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='串口设备路径 (默认: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('--scan', action='store_true',
                        help='扫描所有可用串口')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("           STM32连接测试")
    print("=" * 50)
    
    if args.scan:
        print("正在扫描串口设备...")
        ports = find_serial_ports()
        
        if ports:
            print(f"找到 {len(ports)} 个串口设备:")
            for port in ports:
                print(f"  - {port}")
                
            # 尝试测试每个端口
            for port in ports:
                print(f"\n测试 {port}...")
                if test_stm32_connection(port, args.baudrate):
                    print(f"✓ {port} 测试成功!")
                    break
            else:
                print("✗ 所有端口测试失败")
        else:
            print("未找到任何串口设备")
    else:
        success = test_stm32_connection(args.port, args.baudrate)
        if success:
            print("\n✓ STM32连接测试成功!")
        else:
            print("\n✗ STM32连接测试失败!")
            print("请检查:")
            print("  - USB线缆是否正确连接")
            print("  - STM32是否已通电")
            print("  - 串口设备路径是否正确")
            print("  - 波特率设置是否匹配")
    
    print("=" * 50)


if __name__ == '__main__':
    main()
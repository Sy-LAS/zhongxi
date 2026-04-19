#ifndef STM32_COMMS_MANAGER_H
#define STM32_COMMS_MANAGER_H

#ifdef USE_SERIAL_LIB
#include <serial/serial.h>
#endif

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

namespace sensor_interfaces {

/**
 * @brief 与STM32F407VET6控制板通信的管理器类
 */
class Stm32CommsManager {
public:
    /**
     * @brief 构造函数
     * @param port 串口设备路径
     * @param baud_rate 波特率
     */
    Stm32CommsManager(const std::string& port = "/dev/ttyUSB0", int baud_rate = 115200);
    
    /**
     * @brief 析构函数
     */
    ~Stm32CommsManager();
    
    /**
     * @brief 连接到STM32控制板
     * @return 连接是否成功
     */
    bool connect();
    
    /**
     * @brief 断开与STM32控制板的连接
     */
    void disconnect();
    
    /**
     * @brief 检查连接状态
     * @return 是否已连接
     */
    bool isConnected() const;
    
    /**
     * @brief 发送命令到STM32
     * @param command 命令字符串
     * @return 发送是否成功
     */
    bool sendCommand(const std::string& command);
    
    /**
     * @brief 读取来自STM32的数据
     * @param timeout_ms 超时时间（毫秒）
     * @return 读取到的数据
     */
    std::string readData(int timeout_ms = 100);
    
    /**
     * @brief 清空缓冲区
     */
    void flush();
    
    /**
     * @brief 设置串口参数
     * @param port 串口设备路径
     * @param baud_rate 波特率
     */
    void setParams(const std::string& port, int baud_rate);
    
    /**
     * @brief 获取可用数据字节数
     * @return 字节数
     */
    size_t available() const;

private:
#ifdef USE_SERIAL_LIB
    serial::Serial ser_;
#endif
    std::string port_;
    int baud_rate_;
    mutable std::mutex comms_mutex_;  // 保护串口通信的互斥锁
    bool connected_;
    
    // 禁止拷贝构造和赋值
    Stm32CommsManager(const Stm32CommsManager&) = delete;
    Stm32CommsManager& operator=(const Stm32CommsManager&) = delete;
};

using Stm32CommsManagerPtr = std::shared_ptr<Stm32CommsManager>;

} // namespace sensor_interfaces

#endif // STM32_COMMS_MANAGER_H
#include "sensor_interfaces/stm32_comms_manager.h"
#include "sensor_interfaces/stm32_protocol.h"
#include <chrono>
#include <thread>

namespace sensor_interfaces {

Stm32CommsManager::Stm32CommsManager(const std::string& port, int baud_rate)
    : port_(port), baud_rate_(baud_rate), connected_(false) {
}

Stm32CommsManager::~Stm32CommsManager() {
    disconnect();
}

bool Stm32CommsManager::connect() {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    try {
        if (ser_.isOpen()) {
            ser_.close();
        }
        
        ser_.setPort(port_);
        ser_.setBaudrate(baud_rate_);
        // 设置合理的时间参数：读取超时
        ser_.setTimeout(serial::Timeout::max(), 1, 0, 1, 0); // 读取一个字节最多等待10ms
        
        ser_.open();
        connected_ = ser_.isOpen();
        
        if (connected_) {
            RCLCPP_INFO(rclcpp::get_logger("stm32_comms"), "成功连接到STM32: %s (波特率: %d)", 
                        port_.c_str(), baud_rate_);
            
            // 发送重置命令确保STM32处于已知状态
            std::string reset_cmd = Stm32Protocol::buildResetOdomCommand();
            sendCommand(reset_cmd);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("stm32_comms"), "无法打开串口: %s", port_.c_str());
        }
        
        return connected_;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("stm32_comms"), "串口连接异常: %s", e.what());
        connected_ = false;
        return false;
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("stm32_comms"), "串口库未找到，无法连接STM32");
    return false;
#endif
}

void Stm32CommsManager::disconnect() {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    if (ser_.isOpen()) {
        // 发送停止命令
        std::string stop_cmd = Stm32Protocol::buildMoveCommand(0.0, 0.0);
        sendCommand(stop_cmd);
        
        ser_.close();
        connected_ = false;
        RCLCPP_INFO(rclcpp::get_logger("stm32_comms"), "已断开与STM32的连接");
    }
#endif
}

bool Stm32CommsManager::isConnected() const {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    bool currently_open = ser_.isOpen();
    connected_ = currently_open;  // 同步内部状态
    return currently_open;
#else
    return false;
#endif
}

bool Stm32CommsManager::sendCommand(const std::string& command) {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    if (!connected_ || !ser_.isOpen()) {
        RCLCPP_WARN(rclcpp::get_logger("stm32_comms"), "未连接到STM32，无法发送命令: %s", command.c_str());
        return false;
    }
    
    try {
        size_t written = ser_.write(command);
        RCLCPP_DEBUG(rclcpp::get_logger("stm32_comms"), "发送命令: %s (长度: %zu)", 
                     command.c_str(), written);
        return written == command.length();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("stm32_comms"), "发送命令失败: %s", e.what());
        return false;
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("stm32_comms"), "串口库未找到，无法发送命令");
    return false;
#endif
}

std::string Stm32CommsManager::readData(int timeout_ms) {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    if (!connected_ || !ser_.isOpen()) {
        return "";
    }
    
    try {
        // 检查是否有可用数据
        size_t bytes_available = ser_.available();
        if (bytes_available > 0) {
            // 读取直到遇到结束字符或超时
            std::string response = "";
            auto start_time = std::chrono::steady_clock::now();
            
            while (std::chrono::duration_cast<std::chrono::milliseconds>
                   (std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {
                if (ser_.available() > 0) {
                    char c;
                    size_t read_count = ser_.read(&c, 1);
                    if (read_count > 0) {
                        response += c;
                        // 如果收到结束字符则停止读取
                        if (c == Stm32Protocol::END_BYTE) {
                            break;
                        }
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 避免忙等待
            }
            
            if (!response.empty()) {
                RCLCPP_DEBUG(rclcpp::get_logger("stm32_comms"), "收到数据: %s", response.c_str());
                return response;
            }
        }
        return "";
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("stm32_comms"), "读取数据失败: %s", e.what());
        return "";
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("stm32_comms"), "串口库未找到，无法读取数据");
    return "";
#endif
}

void Stm32CommsManager::flush() {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    if (ser_.isOpen()) {
        ser_.flush();
    }
#endif
}

void Stm32CommsManager::setParams(const std::string& port, int baud_rate) {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
    if (connected_) {
        disconnect();
    }
    
    port_ = port;
    baud_rate_ = baud_rate;
}

size_t Stm32CommsManager::available() const {
    std::lock_guard<std::mutex> lock(comms_mutex_);
    
#ifdef USE_SERIAL_LIB
    if (ser_.isOpen()) {
        return ser_.available();
    }
#endif
    return 0;
}

} // namespace sensor_interfaces
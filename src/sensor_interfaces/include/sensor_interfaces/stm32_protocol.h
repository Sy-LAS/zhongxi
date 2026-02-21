#ifndef STM32_PROTOCOL_H
#define STM32_PROTOCOL_H

#include <string>
#include <vector>
#include <cstdint>
#include <sstream>
#include <stdexcept>

namespace sensor_interfaces {

/**
 * @brief 与STM32F407VET6控制板通信的协议定义
 */
struct Stm32Protocol {
    // 协议分隔符
    static constexpr char START_BYTE = '#';
    static constexpr char END_BYTE = '!';
    static constexpr char SEPARATOR = ',';

    // 命令类型枚举
    enum class CommandType {
        MOVE_CMD = 0,     // 移动命令
        GET_ODOM = 1,     // 请求里程计数据
        GET_ENCODER = 2,  // 请求编码器数据
        RESET_ODOM = 3,   // 重置里程计
        GET_IMU = 4,      // 请求IMU数据
        SET_PID = 5,      // 设置PID参数
        GET_STATUS = 6    // 获取系统状态
    };

    /**
     * @brief 构建移动命令字符串
     * @param linear_x 线速度x方向
     * @param angular_z 角速度z方向
     * @return 格式化的命令字符串
     */
    static std::string buildMoveCommand(double linear_x, double angular_z) {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "%c%.3f%c%.3f%c", 
                 START_BYTE, linear_x, SEPARATOR, angular_z, END_BYTE);
        return std::string(buffer);
    }

    /**
     * @brief 构建获取里程计命令
     * @return 格式化的命令字符串
     */
    static std::string buildGetOdomCommand() {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%c%d%c", 
                 START_BYTE, static_cast<int>(CommandType::GET_ODOM), END_BYTE);
        return std::string(buffer);
    }

    /**
     * @brief 构建获取编码器命令
     * @return 格式化的命令字符串
     */
    static std::string buildGetEncoderCommand() {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%c%d%c", 
                 START_BYTE, static_cast<int>(CommandType::GET_ENCODER), END_BYTE);
        return std::string(buffer);
    }

    /**
     * @brief 构建重置里程计命令
     * @return 格式化的命令字符串
     */
    static std::string buildResetOdomCommand() {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%c%d%c", 
                 START_BYTE, static_cast<int>(CommandType::RESET_ODOM), END_BYTE);
        return std::string(buffer);
    }

    /**
     * @brief 构建获取IMU数据命令
     * @return 格式化的命令字符串
     */
    static std::string buildGetImuCommand() {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%c%d%c", 
                 START_BYTE, static_cast<int>(CommandType::GET_IMU), END_BYTE);
        return std::string(buffer);
    }

    /**
     * @brief 构建设置PID参数命令
     * @param kp P参数
     * @param ki I参数
     * @param kd D参数
     * @return 格式化的命令字符串
     */
    static std::string buildSetPidCommand(double kp, double ki, double kd) {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "%c%d%c%.3f%c%.3f%c%.3f%c", 
                 START_BYTE, static_cast<int>(CommandType::SET_PID), 
                 SEPARATOR, kp, SEPARATOR, ki, SEPARATOR, kd, END_BYTE);
        return std::string(buffer);
    }

    /**
     * @brief 解析从STM32返回的数据
     * @param data 从串口读取的原始数据
     * @return 解析后的double数值向量
     */
    static std::vector<double> parseResponse(const std::string& data) {
        std::vector<double> result;
        
        if (data.empty() || data[0] != START_BYTE || data[data.length()-1] != END_BYTE) {
            // 尝试解析没有协议头的数据（向后兼容）
            return parseSimpleData(data);
        }

        // 去掉首尾字符
        std::string content = data.substr(1, data.length()-2);
        
        std::stringstream ss(content);
        std::string item;
        
        while (std::getline(ss, item, SEPARATOR)) {
            try {
                result.push_back(std::stod(item));
            } catch (const std::exception& e) {
                // 解析错误，返回空向量
                return std::vector<double>();
            }
        }
        
        return result;
    }

private:
    /**
     * @brief 解析简单数据格式（向后兼容）
     * @param data 从串口读取的原始数据
     * @return 解析后的double数值向量
     */
    static std::vector<double> parseSimpleData(const std::string& data) {
        std::vector<double> result;
        
        std::stringstream ss(data);
        std::string item;
        
        while (std::getline(ss, item, SEPARATOR)) {
            // 尝试移除可能的空白字符
            item.erase(0, item.find_first_not_of(" \t\r\n"));
            item.erase(item.find_last_not_of(" \t\r\n") + 1);
            
            if (!item.empty()) {
                try {
                    result.push_back(std::stod(item));
                } catch (const std::exception& e) {
                    // 解析错误，返回空向量
                    return std::vector<double>();
                }
            }
        }
        
        return result;
    }
};

} // namespace sensor_interfaces

#endif // STM32_PROTOCOL_H
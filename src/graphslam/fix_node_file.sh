#!/bin/bash
# 修复GraphSLAM节点文件缺失问题

echo "=== 修复GraphSLAM节点文件 ==="

cd ~/Desktop/zhongxi/src/graphslam

# 1. 检查当前node文件状态
echo "1. 检查当前node文件..."
ls -la src/node/
FILE_SIZE=$(stat -c%s "src/node/slam2d_node.cpp")
echo "slam2d_node.cpp 文件大小: $FILE_SIZE 字节"

# 2. 如果文件为空，则替换为模板
if [ $FILE_SIZE -eq 0 ]; then
    echo "2. 检测到空文件，正在替换为模板..."
    
    # 备份原文件（如果存在内容）
    if [ $FILE_SIZE -gt 0 ]; then
        cp src/node/slam2d_node.cpp src/node/slam2d_node.cpp.backup
        echo "已备份原文件为 slam2d_node.cpp.backup"
    fi
    
    # 写入基础模板
    cat > src/node/slam2d_node.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class GraphSLAMNode : public rclcpp::Node
{
public:
    GraphSLAMNode() : Node("graphslam_node")
    {
        RCLCPP_INFO(this->get_logger(), "GraphSLAM node initialized");
        
        // 订阅激光雷达数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GraphSLAMNode::laserCallback, this, std::placeholders::_1));
            
        // TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "Subscriptions created");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // TODO: 实现SLAM算法
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan data");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GraphSLAMNode>();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting GraphSLAM node...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
EOF
    
    echo "✅ 已创建基础SLAM节点文件"
else
    echo "2. 节点文件非空，跳过替换"
fi

# 3. 验证文件内容
echo "3. 验证文件内容..."
if grep -q "int main" src/node/slam2d_node.cpp; then
    echo "✅ 找到main函数"
else
    echo "❌ 未找到main函数，请检查文件"
    exit 1
fi

# 4. 清理构建缓存
echo "4. 清理构建缓存..."
cd ~/Desktop/zhongxi
rm -rf build/graphslam install/graphslam log/graphslam

# 5. 重新编译
echo "5. 重新编译..."
colcon build --packages-select graphslam --cmake-clean-cache

BUILD_RESULT=$?
if [ $BUILD_RESULT -eq 0 ]; then
    echo "✅ 编译成功完成！"
    
    # 6. 验证可执行文件
    echo "6. 验证生成的可执行文件..."
    EXEC_FILE=$(find install -name "graphslam_node" -type f 2>/dev/null)
    if [ -n "$EXEC_FILE" ]; then
        echo "✅ 找到可执行文件: $EXEC_FILE"
        echo "文件大小: $(stat -c%s "$EXEC_FILE") 字节"
    else
        echo "⚠️ 未找到graphslam_node可执行文件"
    fi
else
    echo "❌ 编译失败"
    echo "请查看上面的错误信息"
fi
cmake_minimum_required(VERSION 3.14) # 与 g2o 保持一致

project(zhongxi)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
# G2O - 添加子目录
add_subdirectory(src/g2o)

# 添加 GraphSLAM 子目录
add_subdirectory(src/GraphSLAM)

# 定义ament包
ament_package()
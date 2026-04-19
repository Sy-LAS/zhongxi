cmake_minimum_required(VERSION 3.8)
project(zhongxi_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)

# 安装文件
install(DIRECTORY 
  config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
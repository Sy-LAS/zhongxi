cmake_minimum_required(VERSION 3.14)

set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(GraphSLAM)

# 设置自动处理Qt文件
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# qt components used (also by qglviewer): Core Gui Xml OpenGL Widgets
find_package(Qt5 REQUIRED COMPONENTS Core Gui Xml OpenGL Widgets)
find_package(QGLViewer REQUIRED)

# Include directories
include_directories(SYSTEM
    ${QGLVIEWER_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
)

# Include directories for this project
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 递归查找源文件
file(GLOB_RECURSE SLAM2D_CORE_SOURCES "src/slam2d/*.cpp")
file(GLOB_RECURSE NODE_SOURCES "src/node/*.cpp")
file(GLOB_RECURSE UTIL_SOURCES "src/utils/*.cpp")

# 检查源文件是否存在
if(NOT SLAM2D_CORE_SOURCES)
    message(FATAL_ERROR "No SLAM2D core source files found in src/slam2d/")
endif()

if(NOT NODE_SOURCES)
    message(WARNING "No node source files found in src/node/")
endif()

# 创建主要的GraphSLAM可执行文件（GUI版本）
add_executable(graphslam_g2o
  main_window.cpp
  slam2d_viewer.cpp
  slam2d_viewer.h
  slam2d_g2o.cpp
  base_main_window.ui  # CMake会自动处理UI文件
  main_window.h        # CMake会自动处理moc文件
)

# 创建ROS2节点可执行文件（只有当节点源文件存在时）
if(NODE_SOURCES)
    add_executable(graphslam_node
      ${NODE_SOURCES}
      ${SLAM2D_CORE_SOURCES}
      ${UTIL_SOURCES}
    )
    
    # 链接ROS2相关库
    ament_target_dependencies(graphslam_node
        rclcpp
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
    )
    
    # 设置输出名称
    set_target_properties(graphslam_node PROPERTIES OUTPUT_NAME graphslam_node)
endif()

# 设置GUI可执行文件属性
set_target_properties(graphslam_g2o PROPERTIES OUTPUT_NAME graphslam_g2o)

if(Qt5_POSITION_INDEPENDENT_CODE)
    set_property(TARGET graphslam_g2o PROPERTY COMPILE_FLAGS -fPIC)
    message(STATUS "Generating position independent code for GraphSLAM because Qt5 was built with -reduce-relocations")
endif()

# 链接GUI相关库
target_link_libraries(graphslam_g2o 
    core solver_eigen types_slam2d opengl_helper
    ${QGLVIEWER_LIBRARY}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
)

# 安装规则
install(TARGETS graphslam_g2o
  DESTINATION lib/${PROJECT_NAME})

if(TARGET graphslam_node)
    install(TARGETS graphslam_node
      DESTINATION lib/${PROJECT_NAME})
endif()

# 安装launch文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
    install(DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}/)
endif()

# 安装配置文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
    install(DIRECTORY config
      DESTINATION share/${PROJECT_NAME}/)
endif()

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
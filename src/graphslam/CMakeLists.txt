cmake_minimum_required(VERSION 3.14)
project(graphslam)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt自动处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# -----------------------------
# 查找ROS2依赖
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # 新增nav_msgs依赖
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 新增
find_package(std_msgs REQUIRED)  # 新增

# -----------------------------
# 查找Qt5
# -----------------------------
find_package(Qt5 REQUIRED COMPONENTS Core Gui Xml OpenGL Widgets)

# -----------------------------
# 查找QGLViewer（自动查找，适配不同平台）
# -----------------------------
find_package(libQGLViewer-qt5 QUIET)
if(libQGLViewer-qt5_FOUND)
    set(QGLViewer_INCLUDE_DIRS ${libQGLViewer-qt5_INCLUDE_DIRS})
    set(QGLViewer_LIBRARIES ${libQGLViewer-qt5_LIBRARIES})
else()
    # 备用方案：尝试常见路径
    find_path(QGLVIEWER_INCLUDE_DIR qglviewer.h
        PATHS /usr/include/QGLViewer /usr/include/aarch64-linux-gnu/QGLViewer
        NO_DEFAULT_PATH
    )
    find_library(QGLVIEWER_LIBRARY QGLViewer-qt5
        PATHS /usr/lib /usr/lib/aarch64-linux-gnu
        NO_DEFAULT_PATH
    )
    
    if(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
        set(QGLViewer_INCLUDE_DIRS ${QGLVIEWER_INCLUDE_DIR})
        set(QGLViewer_LIBRARIES ${QGLVIEWER_LIBRARY})
        message(STATUS "Found QGLViewer: ${QGLVIEWER_INCLUDE_DIR}")
    else()
        message(WARNING "QGLViewer not found, GUI target will be disabled")
        set(DISABLE_GUI TRUE)
    endif()
endif()

# -----------------------------
# 查找g2o和Eigen3
# -----------------------------
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

# g2o兼容处理
if(NOT TARGET g2o::core)
    message(STATUS "Using legacy g2o variables")
    set(G2O_LIBRARIES 
        g2o_core 
        g2o_solver_eigen 
        g2o_types_slam2d 
        g2o_opengl_helper
    )
else()
    set(G2O_LIBRARIES g2o::core g2o::solver_eigen g2o::types_slam2d g2o::opengl_helper)
endif()

# -----------------------------
# 头文件路径
# -----------------------------
include_directories(SYSTEM
    ${QGLViewer_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
)

include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# -----------------------------
# 目标1：graphslam_g2o（GUI版本）- 条件编译
# -----------------------------
if(NOT DISABLE_GUI AND QGLViewer_LIBRARIES)
    add_executable(graphslam_g2o
        main_window.cpp
        slam2d_viewer.cpp
        slam2d_viewer.cpp
        slam2d_viewer.h
        slam2d_g2o.cpp
        main_window.ui
        main_window.h
    )

    target_include_directories(graphslam_g2o PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    ament_target_dependencies(graphslam_g2o
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        std_msgs
    )

    target_link_libraries(graphslam_g2o
        ${G2O_LIBRARIES}
        ${QGLViewer_LIBRARIES}
        Qt5::Core
        Qt5::Gui
        Qt5::Xml
        Qt5::Widgets
        Qt5::OpenGL
    )

    set_target_properties(graphslam_g2o PROPERTIES OUTPUT_NAME graphslam_g2o)
    
    # 安装GUI目标
    install(TARGETS graphslam_g2o DESTINATION lib/${PROJECT_NAME})
else()
    message(STATUS "Skipping GUI target due to missing dependencies")
endif()

# -----------------------------
# 目标2：graphslam_node（ROS2节点）
# -----------------------------
file(GLOB_RECURSE SLAM2D_CORE_SOURCES "src/slam2d/*.cpp")
file(GLOB_RECURSE NODE_SOURCES "src/node/*.cpp")
file(GLOB_RECURSE UTIL_SOURCES "src/utils/*.cpp")

if(NODE_SOURCES)
    add_executable(graphslam_node
        ${NODE_SOURCES}
        ${SLAM2D_CORE_SOURCES}
        ${UTIL_SOURCES}
    )
    
    target_include_directories(graphslam_node PRIVATE
        include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
    )
    
    ament_target_dependencies(graphslam_node
        rclcpp
        sensor_msgs
        nav_msgs          # 添加nav_msgs
        geometry_msgs
        tf2
        tf2_ros
    )
    
    target_link_libraries(graphslam_node
        ${G2O_LIBRARIES}
    )
    
    set_target_properties(graphslam_node PROPERTIES OUTPUT_NAME graphslam_node)
endif()

# -----------------------------
# 目标3：map_manager_node（地图管理节点）
# -----------------------------
add_executable(map_manager_node
    map_manager_node.cpp
)

target_include_directories(map_manager_node PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(map_manager_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
)

target_link_libraries(map_manager_node
    ${G2O_LIBRARIES}
)

set_target_properties(map_manager_node PROPERTIES OUTPUT_NAME map_manager_node)

# -----------------------------
# 目标4：map_saver_node（地图保存节点）
# -----------------------------
add_executable(map_saver_node
    map_saver_node.cpp
)

target_include_directories(map_saver_node PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(map_saver_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
)

set_target_properties(map_saver_node PROPERTIES OUTPUT_NAME map_saver_node)

# -----------------------------
# 安装规则
# -----------------------------
if(TARGET graphslam_node)
    install(TARGETS graphslam_node map_manager_node map_saver_node DESTINATION lib/${PROJECT_NAME})
endif()

# 只有当GUI目标存在时才安装
if(TARGET graphslam_g2o)
    install(TARGETS graphslam_g2o DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
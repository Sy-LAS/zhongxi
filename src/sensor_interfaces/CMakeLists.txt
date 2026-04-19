cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 添加此行

find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

# Find serial library
find_package(PkgConfig REQUIRED)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(SERIAL serial QUIET)
  if(SERIAL_FOUND)
    set(HAVE_SERIAL_LIB TRUE)
  endif()
endif()

# Alternative method: Try finding serial using CMake's find_library
if(NOT SERIAL_FOUND)
  find_path(SERIAL_INCLUDE_DIR serial/serial.h
    PATHS /usr/include /usr/local/include
    PATH_SUFFIXES serial)
  find_library(SERIAL_LIBRARY
    NAMES serial
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)
  
  if(SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
    set(SERIAL_FOUND TRUE)
    set(SERIAL_INCLUDE_DIRS ${SERIAL_INCLUDE_DIR})
    set(SERIAL_LIBRARIES ${SERIAL_LIBRARY})
    set(HAVE_SERIAL_LIB TRUE)
  endif()
endif()

# Define a compile definition if serial library is found
if(HAVE_SERIAL_LIB)
  add_compile_definitions(USE_SERIAL_LIB)
endif()

# Include directories
include_directories(include)

# Include directories for YDLidar SDK
include_directories(
  sdk/src
  sdk
  sdk/core
  sdk/core/serial
  sdk/core/base
  sdk/core/common
  sdk/core/math
  sdk/core/network
)

# Build YDLidar SDK as static library
file(GLOB_RECURSE YDLIDAR_SDK_SOURCES 
  "sdk/src/*.cpp"
  "sdk/core/serial/*.cpp"
  "sdk/core/base/*.cpp"
  "sdk/core/common/*.cpp"
  "sdk/core/math/*.cpp"
  "sdk/core/network/*.cpp"
)

add_library(ydlidar_sdk STATIC ${YDLIDAR_SDK_SOURCES})

# Create a library for sensor interfaces base classes
add_library(sensor_interfaces_base src/sensor_interfaces.cpp)
target_include_directories(sensor_interfaces_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sensor_interfaces_base
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
)

# Camera driver node
add_executable(camera_driver src/camera_driver.cpp)
ament_target_dependencies(camera_driver 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport 
  std_msgs
  OpenCV
)

# Link yaml-cpp to camera driver
target_link_libraries(camera_driver 
  yaml-cpp
  sensor_interfaces_base
)

# Lidar driver node - specifically for X3 Pro
add_executable(lidar_driver src/lidar_driver.cpp)
ament_target_dependencies(lidar_driver 
  rclcpp 
  sensor_msgs 
  std_msgs
)

# Add include directories for lidar driver
target_include_directories(lidar_driver PRIVATE
  sdk/src
  sdk
  sdk/core
  sdk/core/serial
  sdk/core/base
  sdk/core/common
  sdk/core/math
  sdk/core/network
)

# Link YDLidar SDK and base library to lidar driver
target_link_libraries(lidar_driver 
  ydlidar_sdk
  sensor_interfaces_base
)

# Control board interface node
add_executable(control_board_interface 
  src/control_board_interface.cpp 
  src/stm32_comms_manager.cpp
  include/sensor_interfaces/stm32_protocol.h
  include/sensor_interfaces/stm32_comms_manager.h)
ament_target_dependencies(control_board_interface
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)

# Conditionally link serial library
if(HAVE_SERIAL_LIB)
  target_link_libraries(control_board_interface
    sensor_interfaces_base
    ${SERIAL_LIBRARIES}
  )
  target_include_directories(control_board_interface PRIVATE ${SERIAL_INCLUDE_DIRS})
else()
  message(WARNING "serial library not found, control_board_interface may not work properly")
  # Provide fallback: attempt to find and link serial library directly
  find_library(SERIAL_LIB serial HINTS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)
  if(SERIAL_LIB)
    target_link_libraries(control_board_interface
      sensor_interfaces_base
      serial
    )
  endif()
endif()

# Install executables
install(TARGETS
  camera_driver
  lidar_driver  # Now including lidar_driver for X3 Pro
  control_board_interface
  ydlidar_sdk
  sensor_interfaces_base
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
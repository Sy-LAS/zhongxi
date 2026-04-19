import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # 定义enable参数
    enable_lidar_cmd = LaunchConfiguration('enable_lidar')
    enable_camera_cmd = LaunchConfiguration('enable_camera')
    enable_control_board_cmd = LaunchConfiguration('enable_control_board')
    
    # 声明launch参数
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='True',
        description='Enable lidar driver'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='True',
        description='Enable camera driver'
    )
    
    enable_control_board_arg = DeclareLaunchArgument(
        'enable_control_board',
        default_value='True',
        description='Enable control board interface'
    )

    # 获取参数文件路径
    param_config = os.path.join(
        get_package_share_directory('sensor_interfaces'),
        'config',
        'lidar_params.yaml'
    )

    # 定义节点
    lidar_node = Node(
        package='sensor_interfaces',
        executable='lidar_driver',
        name='lidar_driver',
        parameters=[param_config],
        remappings=[
            ('/scan', '/scan')
        ]
    )

    camera_node = Node(
        package='sensor_interfaces',
        executable='camera_driver',
        name='camera_driver',
        parameters=[
            {'image_width': 640},
            {'image_height': 480},
            {'fps': 30},
            {'camera_name': 'camera'},
            {'camera_info_url': 'package://sensor_interfaces/config/camera_info.yaml'}
        ]
    )
    
    control_board_node = Node(
        package='sensor_interfaces',
        executable='control_board_interface',
        name='control_board_interface',
<<<<<<< HEAD
        parameters=[{'serial_port': '/dev/ttyTHS1', 'baud_rate': 115200}]  # STM32F407 - UART1硬件串口 (Pin 8/10)
=======
        parameters=[{'serial_port': '/dev/ttyUSB1'}]  # STM32F407VET6 控制板
>>>>>>> dd8d0fe6d3f1432a37d2566daf8d0127a1310c90
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_camera_arg,
        enable_control_board_arg,
        lidar_node,
        camera_node,
        control_board_node
    ])
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_sensor_interfaces = get_package_share_directory('sensor_interfaces')
    pkg_graphslam = get_package_share_directory('graphslam')
    pkg_zhongxi_description = get_package_share_directory('zhongxi_description')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_robot_state_publisher = LaunchConfiguration('enable_robot_state_publisher', default='true')
    
    # 传感器启动文件
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sensor_interfaces, 'launch', 'sensors.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_camera': enable_camera,
            'enable_lidar': enable_lidar
        }.items()
    )
    
    # Robot State Publisher节点
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_zhongxi_description, 'launch', 'simple_view.launch.py'])
        ),
        condition=IfCondition(enable_robot_state_publisher),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # GraphSLAM节点
    graphslam_node = Node(
        package='graphslam',
        executable='graphslam_node',
        name='graphslam_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_slam),
        output='screen'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera driver'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable lidar driver'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'enable_robot_state_publisher',
        default_value='true',
        description='Enable robot state publisher'
    ))
    
    # 添加启动项
    ld.add_action(sensors_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(graphslam_node)
    
    return ld
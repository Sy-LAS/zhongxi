import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_graphslam = get_package_share_directory('graphslam')
    pkg_sensor_interfaces = get_package_share_directory('sensor_interfaces')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    run_rviz = LaunchConfiguration('run_rviz', default='true')
    
    # 启动传感器接口
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensor_interfaces, 'launch', 'sensors.launch.py')
        )
    )
    
    # GraphSLAM节点
    graphslam_node = Node(
        package='graphslam',
        executable='graphslam_node',
        name='graphslam_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 智能探索节点
    smart_explore_node = Node(
        package='graphslam',
        executable='smart_explore_node',
        name='smart_explore_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 地图管理节点
    map_manager_node = Node(
        package='graphslam',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 地图保存节点
    map_saver_node = Node(
        package='graphslam',
        executable='map_saver_node',
        name='map_saver_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 启动RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(run_rviz)
    )
    
    # 创建launch描述
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Run RViz if true'
        ),
        sensors_launch,
        graphslam_node,
        smart_explore_node,
        map_manager_node,
        map_saver_node,
        rviz_node
    ])
    
    return ld
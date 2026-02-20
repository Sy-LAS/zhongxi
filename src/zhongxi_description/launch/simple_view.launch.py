import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_zhongxi_description = get_package_share_directory('zhongxi_description')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # URDF 文件路径
    urdf_file = os.path.join(pkg_zhongxi_description, 'urdf', 'zhongxi_description.urdf')
    
    # 检查文件存在性
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # 读取 URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
        arguments=[urdf_file]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        robot_state_publisher,
        rviz_node
    ])
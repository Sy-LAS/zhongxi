import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_zhongxi_description = get_package_share_directory('zhongxi_description')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    verbose = LaunchConfiguration('verbose', default='true')
    
    # URDF 文件路径
    urdf_file = os.path.join(pkg_zhongxi_description, 'urdf', 'zhongxi_description.urdf')
    
    # 检查 URDF 文件是否存在
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # 启动 Ignition Gazebo
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r'],
        output='screen'
    )
    
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
    
    # Spawn 实体到 Ignition Gazebo
    spawn_entity = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/create@ignition.msgs.EntityFactory@ignition.transport.TopicInfo'
        ],
        output='screen'
    )
    
    # 使用 ros2 service 调用来spawn模型
    spawn_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/world/default/create',
             'ignition.msgs.EntityFactory', '"sdf: \'' + robot_desc + '\'"'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Set to true to enable verbose output'
        ),
        ign_gazebo,
        robot_state_publisher,
        # spawn_entity,  # 暂时注释掉，需要先安装ros_ign_bridge
        # spawn_service   # 暂时注释掉，需要调试服务调用格式
    ])
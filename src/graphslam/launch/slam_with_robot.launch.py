import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_zhongxi_description = get_package_share_directory('zhongxi_description')
    pkg_graphslam = get_package_share_directory('graphslam')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    run_rviz = LaunchConfiguration('run_rviz', default='true')
    
    # URDF 文件路径
    urdf_file = os.path.join(pkg_zhongxi_description, 'urdf', 'zhongxi_description.urdf')
    
    # 检查 URDF 文件是否存在
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Gazebo 启动描述
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'paused': 'false'
        }.items()
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
    
    # 关节状态发布器（设置默认值避免移动关节的问题）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['joint_states']
        }]
    )
    
    # 关节状态发布器GUI（可选）
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(run_rviz)
    )
    
    # Spawn 实体到 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'zhongxi_robot',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # 启动RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zhongxi_description, 'launch', 'simple_view.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(run_rviz)
    )
    
    # GraphSLAM节点
    graphslam_node = Node(
        package='graphslam',
        executable='graphslam_node',  # 这里假设你有这个可执行文件
        name='graphslam_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 创建launch描述
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Run RViz if true'
        ),
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        spawn_entity,
        rviz,
        graphslam_node
    ])
    
    return ld
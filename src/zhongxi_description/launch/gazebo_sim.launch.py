import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_zhongxi_description = get_package_share_directory('zhongxi_description')
    
    # 尝试使用最新的ros_gz包（Gazebo Garden）
    try:
        pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
        
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-r -v 3 empty.sdf'
            }.items()
        )
        spawn_entity_package = 'ros_gz_sim'
        spawn_entity_executable = 'create_sensor'
    except:
        # 如果ros_gz_sim不可用，尝试ros_ign_gazebo（旧一些但仍可用）
        try:
            pkg_gazebo_ros = get_package_share_directory('ros_ign_gazebo')
            
            gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args': '-r -v 3 empty.sdf'
                }.items()
            )
            spawn_entity_package = 'ros_ign_gazebo'
            spawn_entity_executable = 'create_sensor'
        except:
            # 如果都不行，尝试安装指导
            print("错误：未找到Gazebo相关包。请运行以下命令安装：")
            print("sudo apt update")
            print("sudo apt install ros-humble-ros-gz")
            return LaunchDescription([])

    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # URDF 文件路径
    urdf_file = os.path.join(pkg_zhongxi_description, 'urdf', 'zhongxi_description.urdf')
    
    # 检查 URDF 文件是否存在
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # 读取 URDF 内容
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
    
    # 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 启动Gazebo仿真
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items()
    )
    
    # Spawn 实体到 Gazebo - 使用ros_gz_sim的spawn_entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='spawn_entity.py',
        arguments=[
            '-string', robot_desc,
            '-entity', 'zhongxi_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.TFMessage',
                   '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.TFMessage',
                   '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gz_sim,
        bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])
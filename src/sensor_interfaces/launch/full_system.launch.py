import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_sensor_interfaces = get_package_share_directory('sensor_interfaces')
    pkg_zhongxi_description = get_package_share_directory('zhongxi_description')

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
<<<<<<< HEAD
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyTHS1')
=======
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
>>>>>>> dd8d0fe6d3f1432a37d2566daf8d0127a1310c90
    baud_rate = LaunchConfiguration('baud_rate', default='115200')

    # 摄像头节点
    camera_node = Node(
        package='sensor_interfaces',
        executable='camera_driver',
        name='camera_driver',
        parameters=[
            {'camera_id': 0},
            {'frame_rate': 30.0},
            {'width': 640},
            {'height': 480},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 激光雷达节点
    lidar_node = Node(
        package='sensor_interfaces',
        executable='lidar_driver',
        name='lidar_driver',
        parameters=[
            {'port': '/dev/ydlidar'},
            {'baudrate': 230400},
            {'frame_id': 'laser_link'},
            {'range_min': 0.01},
            {'range_max': 25.0},
            {'angle_min': -180.0},
            {'angle_max': 180.0},
            {'frequency': 10.0},
            {'isSingleChannel': False},
            {'intensity': False},
            {'lidar_type': 1},
            {'sample_rate': 3},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 控制板接口节点
    control_board_node = Node(
        package='sensor_interfaces',
        executable='control_board_interface',
        name='control_board_interface',
        parameters=[
            {'serial_port': serial_port},
            {'baud_rate': baud_rate},
            {'wheel_separation': 0.3},
<<<<<<< HEAD
            {'wheel_radius': 0.03},
=======
            {'wheel_radius': 0.05},
>>>>>>> dd8d0fe6d3f1432a37d2566daf8d0127a1310c90
            {'encoder_resolution': 400.0},
            {'publish_rate': 50.0},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # GraphSLAM节点
    graphslam_node = Node(
        package='graphslam',
        executable='graphslam_node',
        name='graphslam_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(pkg_zhongxi_description, 'config', 'robot.urdf')).read()
        }],
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
        'serial_port',
<<<<<<< HEAD
        default_value='/dev/ttyTHS1',
        description='串口设备路径 (UART1硬件串口)'
=======
        default_value='/dev/ttyUSB0',
        description='串口设备路径'
>>>>>>> dd8d0fe6d3f1432a37d2566daf8d0127a1310c90
    ))

    ld.add_action(DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    ))

    # 添加所有节点
    ld.add_action(camera_node)
    ld.add_action(lidar_node)
    ld.add_action(control_board_node)
    ld.add_action(graphslam_node)
    ld.add_action(robot_state_publisher)

    return ld
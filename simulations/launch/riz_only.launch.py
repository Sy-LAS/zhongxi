from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    default_cfg = str(Path(__file__).resolve().parents[1] / "rviz" / "default.rviz")
    cfg_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_cfg,
        description="Path to rviz2 config (.rviz)"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([cfg_arg, rviz_node])

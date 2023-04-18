import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

RVIZ_PKG = "rr1_rviz"
RVIZ_CONFIG_FILE = "urdf_viz_topic.rviz"

def generate_launch_description():
    rviz_pkg = get_package_share_directory(RVIZ_PKG)

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    rviz_config = os.path.join(rviz_pkg, "rviz", RVIZ_CONFIG_FILE)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rviz
    ])

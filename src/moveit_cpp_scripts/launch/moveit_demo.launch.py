# launch/moveit_demo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = get_package_share_directory('tiago_pro_moveit_config')
    moveit_launch = os.path.join(moveit_config, 'launch', 'move_group.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_launch)),
        Node(
            package='moveit_cpp_scripts',
            executable='moveit_plan',
            name='moveit_incremental_plan',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

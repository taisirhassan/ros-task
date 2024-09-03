from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('limo_simulation'), 'launch'),
            '/limo.launch.py'
        ])
    )

    # Launch the controller node
    controller_node = Node(
        package='limo_control',
        executable='limo_controller',
        name='limo_controller',
        output='screen'
    )

    return LaunchDescription([
        simulation_launch,
        controller_node
    ])
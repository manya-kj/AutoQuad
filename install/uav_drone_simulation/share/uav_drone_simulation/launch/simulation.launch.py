import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='simple_drone')

    uav_drone_bringup = get_package_share_directory('uav_drone_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'namespace',
            default_value='simple_drone',
            description='Namespace for the drone'),

        # Launch only Gazebo and spawn the drone model
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(uav_drone_bringup, 'launch', 'uav_drone_bringup.launch.py')
            )
        ),
    ])

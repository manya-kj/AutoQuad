import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    uav_drone_simulation = get_package_share_directory('uav_drone_simulation')
    
    return LaunchDescription([
        # Launch the simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(uav_drone_simulation, 'launch', 'simulation.launch.py')
            )
        ),

        # Launch the autonomous explorer node
        Node(
            package='uav_drone_control',
            executable='autonomous_explorer',
            name='autonomous_explorer',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/cmd_vel', '/simple_drone/cmd_vel'),
                ('/scan', '/simple_drone/scan'),
                ('/odom', '/simple_drone/odom'),
            ],
        )
    ])

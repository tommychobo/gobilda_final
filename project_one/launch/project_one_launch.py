import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#gobilda nodes, lidar nodes, bump_and_go_node

def generate_launch_description():

    bump_n_go_node = Node(
        package='project_one',
        executable='bump_and_go',
        name='bump_and_go'
    )
    
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765}]
    )

    lidar_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
            '/rplidar_a1_launch.py'])
    )

    robot_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gobilda_robot'), 'launch'),
            '/gobilda.launch.py'])
    )
        
    return LaunchDescription([
        foxglove_node,
        lidar_interface,
        robot_interface,
        bump_n_go_node
    ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Find the official F1TENTH simulator launch file
    sim_launch_file = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py'
    )

    # 2. Tell ROS to include the simulator in our launch
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file)
    )

    # 3. Define your custom Wall Follow node
    wall_follow_node = Node(
        package='wall_follow',
        executable='wall_follow_node.py',
        name='wall_follow_node',
        output='screen'
    )

    # 4. Launch them both together!
    return LaunchDescription([
        simulator,
        wall_follow_node
    ])
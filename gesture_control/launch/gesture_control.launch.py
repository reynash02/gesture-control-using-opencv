from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    gesture_control_node = Node(
        package='gesture_control',
        executable='volume_control_node.py',
        name='volume_control_node',
        output='screen'
    )

    return LaunchDescription([
        set_turtlebot3_model,
        turtlebot3_world_launch,
        gesture_control_node
    ])

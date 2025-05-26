from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('dobot_bringup'),
                    '',
                    'dobot_magician_control_system.launch.py'
                )
            )
        ),

        Node(
            package='dobot_pkg',
            executable='dobot_homing_service_node',
            output='screen'
        ),


        Node(
            package='dobot_pkg',
            executable='dobot_target_follower_node',
            output='screen'
        ),

        Node(
            package='vision_pkg',
            executable='perspective_transformer_node',
            output='screen'
        ),

        # Node(
        #     package='yolo_pkg',
        #     executable='yolo_node',
        #     output='screen'
        # ),

        Node(
            package='server_pkg',
            executable='server_node',
            output='screen'
        )
    ])

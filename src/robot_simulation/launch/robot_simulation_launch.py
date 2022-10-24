from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package="robot_simulation",
            executable="cup_node",
            name="custom_cup_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0, "sim_link_name": "sim_link", "bot_link_name":"bot_link", "cup_link_name": "cup_link"}
            ]
        )
    ])

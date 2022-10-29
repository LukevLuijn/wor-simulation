import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_pos_x = 0.0  # define x position of robot.
    robot_pos_y = 0.0  # define y position of robot.

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/lynxmotion_arm.urdf'

    urdf = os.path.join(
        get_package_share_directory('robot_simulation'),
        urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package="robot_simulation",
            executable="cup_node",
            name="custom_cup_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pos_x": -0.32, # + 0.1,# 0.0,#-0.226912,
                 "pos_y": -0.32,
                 "pos_z": 0.0,
                 "sim_link_name": "sim_link",
                 "bot_link_name": "base_link",
                 "cup_link_name": "cup_link"}
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),
        Node(
            package="robot_simulation",
            executable="arm_node",
            name="custom_arm_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pos_x": robot_pos_x,
                 "pos_y": robot_pos_y,
                 "pos_z": 0.0,
                 "sim_link_name": "sim_link",
                 "bot_link_name": "base_link",
                 "cup_link_name": "cup_link"}
            ]
        ),

    ])

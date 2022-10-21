from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_simulation",
            executable="cup_node",
            name="custom_cup_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"posX": 10.0, "posY": 11.0, "posZ": 12.0}
            ]
        )
    ])
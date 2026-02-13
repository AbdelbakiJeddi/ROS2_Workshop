from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_example',
            executable='talker_params',
            name='talker_node',
            parameters=[{
                'message': 'Hello ARE Members !'
                }
            ]
        ),
        Node(
            package='py_example',
            executable='listener',
            name='listener_node'
        )
    ])

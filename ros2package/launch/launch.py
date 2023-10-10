from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            #namespace='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='ros2package',
            #namespace='turtlesim',
            executable='colChange',
            name='colChange'
        ),
        Node(
            package='turtlesim',
            #namespace='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_teleop_key'
        ),

        #This does not work since it crashes
        #Node(
        #    package='turtlesim',
        #    executable='turtle_teleop_key',
        #    name='turtle_teleop_key',
        #)
        ])
    
    
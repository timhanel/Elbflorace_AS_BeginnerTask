from launch import LaunchDescription
from launch_ros.actions import RosTimer
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            #namespace='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            output='screen',
            prefix = 'xterm -e',#very important to open a new terminal window
            name='teleop'),
        Node(
            package='ros2package',
            #namespace='turtlesim',
            executable='colChange',
            name='colChange'
        )      ])

    
    
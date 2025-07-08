from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    Astart_node =Node(
            package='py_pub_sub',
            executable='Astar',
            name='Astar'
        )
    Start_node = Node(
            package='py_pub_sub',
            executable='Start',
            name='Start'
        )
    
    return LaunchDescription([
        Start_node,
        Astart_node
    ])
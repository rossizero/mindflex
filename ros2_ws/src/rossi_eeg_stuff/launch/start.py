from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='8888',
            description='Port for the micro-ROS agent UDP transport'),

        #Node(
        #    package='micro_ros_agent',
        #    executable='micro_ros_agent',
        #    name='micro_ros_agent',
        #    output='screen',
        #    arguments=[
        #        'udp4',
        #        '--port', LaunchConfiguration('port'),
        #    ]
        #),
        Node(
            package='rossi_eeg_stuff',
            executable='node',
            name='better_publisher',
            output='screen',
            arguments=[]
        ),
    ])

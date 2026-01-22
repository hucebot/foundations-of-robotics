from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    topic_name = LaunchConfiguration('topic_name')
    timer_period = LaunchConfiguration('timer_period')

    return LaunchDescription([

        DeclareLaunchArgument(
            'topic_name',
            default_value='chatter',
            description='Topic name [type: str]'
        ),

        DeclareLaunchArgument(
            'timer_period',
            default_value='1.0',
            description='Timer period in sec [type: float]'
        ),

        Node(
            package='exercise_pub_sub',
            executable='pub_node',
            name='pub_node',
            parameters=[{
                'topic_name': topic_name,
                'timer_period': timer_period
            }]
        ),
        # TODO: add the 'sub_node' passing the 'topic_name' parameter
        Node(
            package='exercise_pub_sub',
            executable='sub_node',
            name='sub_node',
            parameters=[{
                'topic_name': topic_name
            }]
        ),
    ])

# Contents of tf_filter.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='a200_1057',
        description='Namespace for the robot and its topics/frames'
    )
    robot_namespace = LaunchConfiguration('robot_namespace')

    return LaunchDescription([
        robot_namespace_arg,
        Node(
            package='tf_filter_pkg', # This is the name of YOUR package
            executable='tf_filter_node', # THIS MUST MATCH THE NAME IN setup.py's entry_points
            name='tf_filter_node_launched', # Name of the node when launched
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                #'namespace': robot_namespace,
                'source_tf_topic': 'tf',
                'source_tf_static_topic': 'tf_static',
                'filtered_tf_topic': 'tf_filtered',
                'filtered_tf_static_topic': 'tf_static_filtered',
                'parent_frame_to_block': 'odom',
                'child_frame_to_block': 'base_link'
            }]
        )
    ])
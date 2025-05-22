# ~/brad/ros2_ws/src/tf_filter_pkg/launch/tf_filter.launch.py
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
            package='tf_filter_pkg',
            executable='tf_filter_executable', # Or whatever you named it in setup.py
            name='tf_filter_node_launched',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'source_tf_topic': 'tf',
                # 'source_tf_static_topic': 'tf_static', # REMOVE
                'filtered_tf_topic': 'tf_filtered',
                # 'filtered_tf_static_topic': 'tf_static_filtered', # REMOVE
                'parent_frame_to_block': 'odom',
                'child_frame_to_block': 'base_link'
            }]
        )
    ])
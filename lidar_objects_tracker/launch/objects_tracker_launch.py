from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown, OpaqueFunction, LogInfo
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Path to the rosbag2 directory to play back. Leave empty to skip bag playback.',
    )

    reset_rviz_arg = DeclareLaunchArgument(
        'reset_rviz',
        default_value='false',
        description='Whether to reset RViz time on startup.',
    )

    config = PathJoinSubstitution(
        [FindPackageShare('lidar_objects_tracker'), 'config', 'objects_tracker.yaml']
    )

    objects_tracker_node = Node(
        package='lidar_objects_tracker',
        executable='objects_tracker_node',
        name='objects_tracker_node',
        output='screen',
        remappings=[
            ('scan', 'lidar/base/front/scan'),
        ],
        parameters=[config],
    )

    def launch_conditionals(context):
        actions = []

        bag_path = context.launch_configurations.get('bag_path', '')
        if bag_path:
            actions.append(LogInfo(msg=f'Playing bag: {bag_path}'))
            actions.append(ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    bag_path,
                    '--disable-keyboard-controls',
                ],
                output='screen',
                on_exit=Shutdown()
            ))
        else:
            actions.append(LogInfo(msg='No bag_path provided, skipping bag playback.'))

        reset_rviz = context.launch_configurations.get('reset_rviz', 'false')
        if reset_rviz.lower() == 'true':
            actions.append(LogInfo(msg='Resetting RViz time.'))
            actions.append(ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/rviz/reset_time', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            ))
        else:
            actions.append(LogInfo(msg='Skipping RViz reset (reset_rviz:=false).'))

        return actions

    return LaunchDescription([
        bag_path_arg,
        reset_rviz_arg,
        objects_tracker_node,
        OpaqueFunction(function=launch_conditionals),
    ])


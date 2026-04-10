from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the rosbag2 directory to play back',
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

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
                LaunchConfiguration('bag_path'),
                '--disable-keyboard-controls',
        ],
        output='screen',
        on_exit=Shutdown()
    )

    # Reset RViz time on startup in simulation
    reset_rviz = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/rviz/reset_time', 'std_srvs/srv/Empty', '{}'],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        objects_tracker_node,
        bag_play,
        reset_rviz
    ])

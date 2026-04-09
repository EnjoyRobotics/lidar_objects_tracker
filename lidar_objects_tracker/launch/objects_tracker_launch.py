from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

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
            'ros2', 'bag', 'play', '/home/ubuntu/ros2-service-robot/workspace/src/lidar_objects_tracker/rosbag2_filtered',
                '--disable-keyboard-controls',
        ],
        output='screen'
    )

    # Reset RViz time on startup in simulation
    reset_rviz = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/rviz/reset_time', 'std_srvs/srv/Empty', '{}'],
        output='screen'
    )

    return LaunchDescription([
        objects_tracker_node,
        bag_play,
        reset_rviz
    ])

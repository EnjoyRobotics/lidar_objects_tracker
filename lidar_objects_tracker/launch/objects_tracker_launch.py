from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    objects_tracker_node = Node(
        package='lidar_objects_tracker',
        executable='objects_tracker_node',
        name='objects_tracker_node',
        output='screen',
        remappings=[
            ('scan', 'lidar/base/front/scan'),
        ],
        parameters=[
            {'target_frame': 'odom'},
            {'cluster_neighbor_radius': 0.4},
            {'cluster_min_points': 10},
            {'cluster_max_points': 1000},
            {'birth_existence_prob': 0.01},
            {'survival_prob': 0.999},
            {'detection_prob': 0.2},
            {'kf_acc_uncertainty': 0.6},
        ],
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '/home/ubuntu/ros2-service-robot/workspace/src/lidar_objects_tracker/rosbag2_filtered',
                '--disable-keyboard-controls',
        ],
        output='screen'
    )

    return LaunchDescription([
        objects_tracker_node,
        bag_play,
    ])

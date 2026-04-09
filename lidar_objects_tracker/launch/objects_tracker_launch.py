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
            {'cluster_min_points': 15},
            {'cluster_max_points': 200},
            {'lmb_tracker.birth_existence_prob': 0.01},
            {'lmb_tracker.survival_prob': 0.999},
            {'lmb_tracker.detection_prob': 0.05},
            {'lmb_tracker.kf_pos_uncertainty': 0.2},
            {'lmb_tracker.kf_vel_uncertainty': 0.4},
            {'lmb_tracker.kf_acc_uncertainty': 1.0},
        ],
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

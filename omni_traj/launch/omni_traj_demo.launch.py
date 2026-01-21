from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Two empty 360 lidars
    lidar1 = Node(
        package='omni_traj',
        executable='empty_scan_pub',
        name='lidar1_empty',
        output='screen',
        parameters=[{
            'topic': '/lidar1/scan',
            'frame_id': 'lidar1_link',
            'rate_hz': 10.0,
            'num_readings': 360,
            'range_max': 10.0
        }]
    )

    lidar2 = Node(
        package='omni_traj',
        executable='empty_scan_pub',
        name='lidar2_empty',
        output='screen',
        parameters=[{
            'topic': '/lidar2/scan',
            'frame_id': 'lidar2_link',
            'rate_hz': 10.0,
            'num_readings': 360,
            'range_max': 10.0
        }]
    )

    # Static TFs (later you’ll replace offsets with your real mounts)
    # odom -> base_link identity (for now)
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    base_to_lidar1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar1',
        arguments=['0.0', '0.0', '0.10', '0', '0', '0', 'base_link', 'lidar1_link']
    )

    base_to_lidar2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar2',
        arguments=['0.0', '0.0', '0.10', '0', '0', '0', 'base_link', 'lidar2_link']
    )

    traj = Node(
        package='omni_traj',
        executable='waypoint_traj',
        name='waypoint_traj',
        output='screen',
        parameters=[{
            'dt': 0.01,
            'v_max': 0.3,
            'turn_radius_ref': 0.4,
            'v_at_radius_ref': 0.3,
            'omega_dir_max': -1.0,     # use computed omega = v/R
            'ds_geom': 0.03,

            'wheel_radius': 0.09,
            'wheel_base': 0.2,
            'max_wheel_speed': 12.0,
            'max_wheel_accel': 2.0,
            'map_width_m': 3.0,
            'map_height_m': 3.0,
            'map_frame': 'odom',
            'base_frame': 'base_link',

            'soft_inflate_radius': 0.2,
            'hard_inflate_radius': 0.2,

            'start_pose': [0.0, 0.0, 0.0],
            'wp_n': 0,
            'waypoints': [float('nan'), float('nan')],
            'add_wp': [float('nan'), float('nan')],
            'lidar1_topic': '/lidar1/scan',
            'lidar2_topic': '/lidar2/scan',
        }]
    )

    return LaunchDescription([
        lidar1, lidar2,
        odom_to_base, base_to_lidar1, base_to_lidar2,
        traj
    ])

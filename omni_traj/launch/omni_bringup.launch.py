from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Use stable by-id paths (recommended)
    lidar1_port = '/dev/serial/by-id/USB_ID_FOR_LIDAR1'
    lidar2_port = '/dev/serial/by-id/USB_ID_FOR_LIDAR2'

    # IMPORTANT: copy the exact params (baud/scan_mode/etc) from:
    # ~/ros2_ws/src/sllidar_ros2/launch/view_sllidar_c1_launch.py
    lidar1 = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='lidar1',
        output='screen',
        parameters=[{
            'serial_port': lidar1_port,
            'frame_id': 'lidar1_link',
            # 'serial_baudrate': <COPY FROM view_sllidar_c1_launch.py>,
            # other params: <COPY FROM view_sllidar_c1_launch.py>
        }],
        remappings=[
            ('scan', '/lidar1/scan'),
        ]
    )

    lidar2 = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='lidar2',
        output='screen',
        parameters=[{
            'serial_port': lidar2_port,
            'frame_id': 'lidar2_link',
            # 'serial_baudrate': <COPY FROM view_sllidar_c1_launch.py>,
            # other params: <COPY FROM view_sllidar_c1_launch.py>
        }],
        remappings=[
            ('scan', '/lidar2/scan'),
        ]
    )

    # 500ms stagger
    lidar2_staggered = TimerAction(period=0.5, actions=[lidar2])

    # Your static TFs (keep yours)
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
            'lidar1_topic': '/lidar1/scan',
            'lidar2_topic': '/lidar2/scan',
            # keep the rest of your params...
        }]
    )

    return LaunchDescription([
        lidar1,
        lidar2_staggered,
        base_to_lidar1,
        base_to_lidar2,
        traj,
    ])

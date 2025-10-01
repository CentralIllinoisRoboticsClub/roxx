from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    pkg_share = FindPackageShare('roxx_jetson_bringup')
    cfg_dir = PathJoinSubstitution([pkg_share, 'config'])

    # Config files
    ekf_odom_yaml = PathJoinSubstitution([cfg_dir, 'ekf_odom.yaml'])
    ekf_map_yaml  = PathJoinSubstitution([cfg_dir, 'ekf_map.yaml'])
    navsat_yaml   = PathJoinSubstitution([cfg_dir, 'navsat_transform.yaml'])
    vslam_yaml    = PathJoinSubstitution([cfg_dir, 'visual_slam.yaml'])
    nvblox_yaml   = PathJoinSubstitution([cfg_dir, 'nvblox.yaml'])
    nav2_yaml     = PathJoinSubstitution([cfg_dir, 'nav2_params.yaml'])

    # 1) OAK-D Pro via depthai-ros (stereo + depth + IMU)
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('depthai_ros_driver'), '/launch/camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'OAK-D-Pro',
            'enable_imu': 'true',
            'enable_depth': 'true',
            'enable_rectified': 'true',
            'publish_tf': 'true',
            'use_rviz': 'false',
            'parent_frame': 'base_link',
            'frame_prefix': 'oak'
        }.items()
    )

    # 2) Isaac ROS Visual SLAM - wait for OAK to be ready
    vslam_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='isaac_ros_visual_slam',
                executable='visual_slam_node',
                name='visual_slam',
                parameters=[vslam_yaml, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('stereo_camera/left/image',  '/oak/left/image_rect'),
                    ('stereo_camera/right/image', '/oak/right/image_rect'),
                    ('stereo_camera/left/camera_info',  '/oak/left/camera_info'),
                    ('stereo_camera/right/camera_info', '/oak/right/camera_info'),
                    ('imu', '/oak/imu/data')
                ],
                output='screen'
            )
        ]
    )

    # 3) robot_localization: EKF for odom
    ekf_odom = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_odom',
                output='screen',
                parameters=[ekf_odom_yaml, {'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 4) navsat_transform
    navsat = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                output='screen',
                parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('imu', '/oak/imu/data'),
                    ('gps/fix', '/gnss/fix'),
                    ('gps/filtered', '/gps/filtered'),
                    ('odometry/filtered', '/odometry/filtered_odom'),
                    ('odometry/gps', '/odometry/gps')
                ]
            )
        ]
    )

    # 5) robot_localization: EKF for map
    ekf_map = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_map',
                output='screen',
                parameters=[ekf_map_yaml, {'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 6) NVBlox
    nvblox_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='isaac_ros_nvblox',
                executable='nvblox_node',
                name='nvblox_node',
                output='screen',
                parameters=[nvblox_yaml, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('depth/image', '/oak/stereo/image_depth'),
                    ('depth/camera_info', '/oak/stereo/camera_info')
                ]
            )
        ]
    )

    # 7) Nav2 bringup - start last
    nav2_bringup = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('nav2_bringup'), '/launch/bringup_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_yaml,
                    'autostart': autostart
                }.items()
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        depthai_launch,  # Start immediately
        vslam_node,      # Wait 3s
        ekf_odom,        # Wait 4s
        navsat,          # Wait 5s
        ekf_map,         # Wait 6s
        nvblox_node,     # Wait 4s
        nav2_bringup     # Wait 7s
    ])

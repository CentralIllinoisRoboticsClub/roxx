from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
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
            # Common, adjust to your camera/needs:
            # (If camera.launch.py exposes different names, adapt here)
            'camera_model': 'OAK-D-Pro',
            'enable_imu': 'true',
            'enable_depth': 'true',
            'enable_rectified': 'true',
            'publish_tf': 'true',
            'use_rviz': 'false',
            'parent_frame': 'base_link',          # OAK mounted to robot base
            'frame_prefix': 'oak'                 # yields oak_* frames
        }.items()
    )

    # 2) Isaac ROS Visual SLAM (stereo + IMU -> odometry in base_link/odom)
    vslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        parameters=[vslam_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            # DepthAI topic names: adapt if yours differ
            ('stereo_camera/left/image',  '/oak/left/image_rect'),
            ('stereo_camera/right/image', '/oak/right/image_rect'),
            ('stereo_camera/left/camera_info',  '/oak/left/camera_info'),
            ('stereo_camera/right/camera_info', '/oak/right/camera_info'),
            ('imu', '/oak/imu/data')
        ],
        output='screen'
    )

    # 3) robot_localization: EKF for odom (VO+IMU)
    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[ekf_odom_yaml, {'use_sim_time': use_sim_time}]
    )

    # 4) navsat_transform to convert GPS to local frame using IMU + odom
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', '/oak/imu/data'),
            ('gps/fix', '/gnss/fix'),
            ('gps/filtered', '/gps/filtered'),        # optional output
            ('odometry/filtered', '/odometry/filtered_odom'),  # from ekf_odom
            ('odometry/gps', '/odometry/gps')         # navsat output
        ]
    )

    # 5) robot_localization: EKF for map (fuse VO/IMU with navsat odom)
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[ekf_map_yaml, {'use_sim_time': use_sim_time}]
    )

    # 6) NVBlox (depth -> 3D + 2D costmap for Nav2)
    nvblox_node = Node(
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

    # 7) Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'), '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_yaml,
            'autostart': autostart
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        GroupAction([
            depthai_launch,
            vslam_node,
            ekf_odom,
            navsat,
            ekf_map,
            nvblox_node,
            nav2_bringup
        ])
    ])

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    points_topic = LaunchConfiguration('points_topic', default='/livox/lidar')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='lidar')
    globalmap_pcd = LaunchConfiguration(
        'globalmap_pcd',
        default='/home/lenovo/onboard/slam-location-ros2/output/mid360_map_filtered.pcd',
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')
    use_global_localization = LaunchConfiguration('use_global_localization', default='false')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction', default='false')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id', default='odom')

    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'odom', odom_child_frame_id],
        parameters=[{'use_sim_time': use_sim_time}],
    )


    # 全局地图加载独立容器，避免地图加载超时影响定位节点
    globalmap_container = ComposableNodeContainer(
        name='globalmap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServerNodelet',
                name='GlobalmapServerNodelet',
                parameters=[
                    {'globalmap_pcd': globalmap_pcd},
                    {'convert_utm_to_local': False},
                    {'downsample_resolution': 0.5},
                ],
            ),
        ],
        output='screen',
    )

    localization_container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalizationNodelet',
                name='HdlLocalizationNodelet',
                remappings=[('/velodyne_points', points_topic), ('/gpsimu_driver/imu_data', imu_topic)],
                parameters=[
                    {'odom_child_frame_id': odom_child_frame_id},
                    {'use_imu': use_imu},
                    {'invert_acc': False},
                    {'invert_gyro': False},
                    {'cool_time_duration': 2.0},
                    {'enable_robot_odometry_prediction': enable_robot_odometry_prediction},
                    {'robot_odom_frame_id': robot_odom_frame_id},
                    {'reg_method': 'NDT_OMP'},
                    {'ndt_neighbor_search_method': 'DIRECT7'},
                    {'ndt_neighbor_search_radius': 1.0},
                    {'ndt_resolution': 1.0},
                    {'downsample_resolution': 0.5},
                    {'specify_init_pose': True},
                    {'init_pos_x': 0.0},
                    {'init_pos_y': 0.0},
                    {'init_pos_z': 0.0},
                    {'init_ori_w': 1.0},
                    {'init_ori_x': 0.0},
                    {'init_ori_y': 0.0},
                    {'init_ori_z': 0.0},
                    {'use_global_localization': use_global_localization},
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('points_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('odom_child_frame_id', default_value='lidar'),
        DeclareLaunchArgument('globalmap_pcd', default_value='/home/lenovo/onboard/slam-location-ros2/output/mid360_map_filtered.pcd'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_global_localization', default_value='false'),
        DeclareLaunchArgument('enable_robot_odometry_prediction', default_value='false'),
        DeclareLaunchArgument('robot_odom_frame_id', default_value='odom'),
        launch_ros.actions.SetParameter(name='use_sim_time', value=use_sim_time),
        lidar_tf,
        globalmap_container,
        localization_container,
    ])

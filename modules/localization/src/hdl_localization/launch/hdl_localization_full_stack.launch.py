from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    points_topic = LaunchConfiguration('points_topic', default='/livox/lidar')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu')
    chassis_topic = LaunchConfiguration('chassis_topic', default='/chassis')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='livox_frame')
    globalmap_pcd = LaunchConfiguration(
        'globalmap_pcd',
        default='/home/lenovo/edu/summary_perception/slam-location-ros2/output/mid360_map_test.pcd',
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')
    use_global_localization = LaunchConfiguration('use_global_localization', default='false')
    enable_wheel_odometry = LaunchConfiguration('enable_wheel_odometry', default='true')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction', default='false')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id', default='odom')
    origin_lat = LaunchConfiguration('origin_lat', default='32.0')
    origin_lon = LaunchConfiguration('origin_lon', default='118.0')
    use_rtk_flag = LaunchConfiguration('use_rtk_flag', default='false')

    static_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'odom', odom_child_frame_id],
        condition=UnlessCondition(enable_wheel_odometry),
    )

    wheel_odometry_node = Node(
        package='hdl_localization',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        parameters=[{
            'sub_chassis_topic': chassis_topic,
            'sub_imu_topic': imu_topic,
            'pub_odom_topic': '/odom_wheel',
            'frame_id': 'odom',
            'child_frame_id': odom_child_frame_id,
            'publish_tf': True,
            'publish_odom': True,
            'use_gps_angle': False,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(enable_wheel_odometry),
    )

    container = ComposableNodeContainer(
        name='container',
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
                    {'downsample_resolution': 0.1},
                ],
            ),
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
                    {'downsample_resolution': 0.2},
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

    tf2location_bridge = Node(
        package='hdl_localization',
        executable='tf2location_bridge_node',
        name='tf2location_bridge',
        parameters=[{
            'map_frame': 'map',
            'target_frame': odom_child_frame_id,
            'odom_frame': 'odom',
            'status_topic': '/status',
            'wheel_odom_topic': '/odom_wheel',
            'nav_odom_topic': '/odom',
            'chassis_topic': chassis_topic,
            'location_topic': '/localization/slam/Location',
            'odometry_topic': '/localization/slam/Odometry',
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'use_rtk_flag': use_rtk_flag,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('points_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('chassis_topic', default_value='/chassis'),
        DeclareLaunchArgument('odom_child_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('globalmap_pcd', default_value='/home/lenovo/project/yh/slam-location-ros2/output/mid360_map_test.pcd'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_global_localization', default_value='false'),
        DeclareLaunchArgument('enable_wheel_odometry', default_value='true'),
        DeclareLaunchArgument('enable_robot_odometry_prediction', default_value='false'),
        DeclareLaunchArgument('robot_odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('origin_lat', default_value='32.0'),
        DeclareLaunchArgument('origin_lon', default_value='118.0'),
        DeclareLaunchArgument('use_rtk_flag', default_value='false'),
        launch_ros.actions.SetParameter(name='use_sim_time', value=use_sim_time),
        static_tf,
        wheel_odometry_node,
        container,
        tf2location_bridge,
    ])

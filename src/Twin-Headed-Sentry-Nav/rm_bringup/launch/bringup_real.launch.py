import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('rm_bringup')
    pb_rm_simulation_launch_dir = os.path.join(get_package_share_directory('pb_rm_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    # Get the default map/config directory
    slam_toolbox_map_dir = PathJoinSubstitution([bringup_dir, 'map', world])
    slam_toolbox_localization_file_dir = os.path.join(bringup_dir, 'config', 'mapper_params_localization.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')

    nav2_map_dir = PathJoinSubstitution([bringup_dir, 'map', world]), ".yaml"
    nav2_params_file_dir = os.path.join(bringup_dir, 'config', 'nav2_params_real.yaml')

    icp_pcd_dir = PathJoinSubstitution([bringup_dir, 'PCD', world]), ".pcd"
    icp_node_params_dir = os.path.join(bringup_dir, 'config', 'icp_node_params.yaml')
    icp_config_dir = os.path.join(bringup_dir, 'config', 'icp_config.yaml')
    icp_input_filters_config_path = os.path.join(bringup_dir, 'config', 'icp_input_filters_mid360.yaml')

    pointcloud_downsampling_config_dir = os.path.join(bringup_dir, 'config', 'pointcloud_downsampling.yaml')

    ####################### Livox_ros_driver2 parameters start #######################
    xfer_format   = 4    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format, 4-both 0 and 1
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    # cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    # cur_config_path = cur_path + '../config'
    # user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
    user_config_path = os.path.join(bringup_dir, 'config', 'MID360_config.json')

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    ####################### Livox_ros_driver2 parameters end #########################

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Visualize robot tf and FAST_LIO cloud_map if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='328',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl, icp')

    # Specify the actions
    start_livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # robot_description publisher
    # TODO : change my robot description
    start_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pb_rm_simulation_launch_dir, 'rm_real.launch.py')),
        launch_arguments={'rviz': use_rviz}.items()
    )

    pointcloud_downsampling_node = Node(
        # Only run pointcloud_downsampling_node when using icp_localization
        condition = LaunchConfigurationEquals('mode', 'nav') and LaunchConfigurationEquals('localization', 'icp'),
        package='pointcloud_downsampling',
        executable='pointcloud_downsampling_node',
        output='screen',
        parameters=[
            pointcloud_downsampling_config_dir,
            {'use_sim_time': use_sim_time }
        ]
    )

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition = LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file_dir,
                    'map': nav2_map_dir}.items()
            ),

            Node(
                condition = LaunchConfigurationEquals('localization', 'icp'),
                package='icp_localization_ros2',
                executable='icp_localization',
                output='screen',
                parameters=[
                    icp_node_params_dir,
                    {'use_sim_time': use_sim_time,
                    'pcd_file_path': icp_pcd_dir,
                    'icp_config_path': icp_config_dir,
                    'input_filters_config_path': icp_input_filters_config_path}
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    'params_file': nav2_params_file_dir,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir}.items()
    )

    start_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'common.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': use_rviz}.items()
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(start_livox_ros_driver2_node)
    ld.add_action(start_robot_description)
    ld.add_action(start_common)
    ld.add_action(pointcloud_downsampling_node)
    ld.add_action(start_localization_group)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)

    return ld

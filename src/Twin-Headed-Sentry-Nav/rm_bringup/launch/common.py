import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('rm_bringup')

    ##################### linefit_ground_segementation parameters start #####################
    segmentation_params = os.path.join(bringup_dir, 'config', 'segmentation_params.yaml')
    ##################### linefit_ground_segementation parameters end #####################
    

    ################################### FAST_LIO parameters start ###################################
    fastlio_mid360_params = os.path.join(bringup_dir, 'config', 'fastlio_mid360.yaml')
    fastlio_rviz_cfg = os.path.join(bringup_dir, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false') # bool
    feature_extract_enable_param = LaunchConfiguration('feature_extract_enable', default='false') # bool
    point_filter_num_param = LaunchConfiguration('point_filter_num', default='3')                 # int
    max_iteration_param = LaunchConfiguration('max_iteration', default='3')                       # int
    filter_size_surf_param = LaunchConfiguration('filter_size_surf', default='0.5')               # double
    filter_size_map_param = LaunchConfiguration('filter_size_map', default='0.5')                 # double
    cube_side_length_param = LaunchConfiguration('cube_side_length', default='1000.0')            # double
    runtime_pos_log_enable_param = LaunchConfiguration('runtime_pos_log_enable', default='false') # bool
    rviz_cfg = LaunchConfiguration('rviz_cfg', default=fastlio_rviz_cfg) # string
    use_rviz = LaunchConfiguration('rviz', default='true') # bool
    ################################### FAST_LIO parameters end ###################################
    
    # imu互补滤波，如果很吃cpu，可以考虑关闭
    bringup_imu_complementary_filter_group = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    # FAST_LIO里程计
    bringup_FAST_LIO_group = GroupAction([
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[
                fastlio_mid360_params,
                {'use_sim_time': use_sim_time},
                {'feature_extract_enable': feature_extract_enable_param},
                {'point_filter_num': point_filter_num_param},
                {'max_iteration': max_iteration_param},
                {'filter_size_surf': filter_size_surf_param},
                {'filter_size_map': filter_size_map_param},
                {'cube_side_length': cube_side_length_param},
                {'runtime_pos_log_enable': runtime_pos_log_enable_param}
            ],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz)
        )
    ])
    
    # 地面分割
    bringup_linefit_ground_segmentation_group = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params]
    )

    # 点云转激光雷达
    bringup_pointcloud_to_laserscan_group = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.55,
            'range_max': 12.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    ld = LaunchDescription()

    ld.add_action(bringup_FAST_LIO_group)
    ld.add_action(bringup_imu_complementary_filter_group)
    ld.add_action(bringup_linefit_ground_segmentation_group)
    ld.add_action(bringup_pointcloud_to_laserscan_group)

    return ld
    

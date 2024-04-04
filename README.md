# SCNU-Twin-Headed-Sentry

华南师范大学VANGUARD战队双头哨兵代码，包含导航和双头视觉


```
src
├── Twin-Headed-Sentry-Nav
│   ├── rm_bringup
│   │   ├── config
│   │   ├── launch
│   │   ├── map
│   │   ├── PCD
│   │   └── rviz
│   ├── rm_driver
│   │   └── livox_ros_driver2
│   │       └── src
│   │           ├── 3rdparty
│   │           │   └── rapidjson
│   │           │       ├── error
│   │           │       ├── internal
│   │           │       └── msinttypes
│   │           ├── cmake
│   │           ├── config
│   │           ├── launch
│   │           ├── lib
│   │           ├── msg
│   │           └── src
│   │               ├── call_back
│   │               ├── comm
│   │               ├── include
│   │               └── parse_cfg_file
│   ├── rm_localization
│   │   ├── FAST_LIO
│   │   │   ├── config
│   │   │   ├── doc
│   │   │   │   └── results
│   │   │   ├── include
│   │   │   │   ├── ikd-Tree
│   │   │   │   └── IKFoM_toolkit
│   │   │   │       ├── esekfom
│   │   │   │       └── mtk
│   │   │   │           ├── src
│   │   │   │           └── types
│   │   │   ├── launch
│   │   │   ├── Log
│   │   │   ├── msg
│   │   │   ├── rviz_cfg
│   │   │   └── src
│   │   └── icp_localization_ros2
│   │       ├── config
│   │       ├── doc
│   │       ├── include
│   │       │   └── icp_localization_ros2
│   │       │       ├── common
│   │       │       └── transform
│   │       ├── launch
│   │       └── src
│   │           ├── common
│   │           └── transform
│   ├── rm_navigation
│   │   ├── launch
│   │   └── params
│   ├── rm_perception
│   │   ├── imu_complementary_filter
│   │   │   ├── include
│   │   │   │   └── imu_complementary_filter
│   │   │   ├── launch
│   │   │   └── src
│   │   ├── linefit_ground_segementation_ros2
│   │   │   ├── doc
│   │   │   ├── linefit_ground_segmentation
│   │   │   │   ├── include
│   │   │   │   │   └── ground_segmentation
│   │   │   │   └── src
│   │   │   └── linefit_ground_segmentation_ros
│   │   │       ├── launch
│   │   │       └── src
│   │   ├── pointcloud_downsampling
│   │   │   ├── config
│   │   │   ├── include
│   │   │   ├── launch
│   │   │   └── src
│   │   └── pointcloud_to_laserscan
│   │       ├── include
│   │       │   └── pointcloud_to_laserscan
│   │       ├── launch
│   │       └── src
│   └── rm_simulation
│       ├── livox_laser_simulation_RO2
│       │   ├── include
│       │   │   └── ros2_livox
│       │   ├── scan_mode
│       │   ├── src
│       │   └── urdf
│       └── pb_rm_simulation
│           ├── launch
│           ├── meshes
│           │   ├── obstacles
│           │   │   ├── obstacle1
│           │   │   ├── obstacle10
│           │   │   ├── obstacle11
│           │   │   ├── obstacle12
│           │   │   ├── obstacle2
│           │   │   ├── obstacle3
│           │   │   ├── obstacle4
│           │   │   ├── obstacle5
│           │   │   ├── obstacle6
│           │   │   ├── obstacle7
│           │   │   ├── obstacle8
│           │   │   ├── obstacle9
│           │   │   └── obstacle_plugin
│           │   │       └── lib
│           │   └── RMUL2024_world
│           │       └── meshes
│           ├── rviz
│           ├── urdf
│           └── world
│               ├── RMUC2024_world
│               └── RMUL2024_world
└── Twin-Headed-Sentry-RV
    ├── choose
    │   ├── include
    │   │   └── choose
    │   └── src
    ├── info_separate
    │   ├── include
    │   │   └── info_separate
    │   └── src
    ├── rm_auto_aim-main
    │   ├── armor_detector_first
    │   │   ├── docs
    │   │   ├── include
    │   │   │   └── armor_detector
    │   │   ├── model
    │   │   ├── src
    │   │   └── test
    │   ├── armor_detector_second
    │   │   ├── docs
    │   │   ├── include
    │   │   │   └── armor_detector
    │   │   ├── model
    │   │   ├── src
    │   │   └── test
    │   ├── armor_tracker
    │   │   ├── docs
    │   │   ├── include
    │   │   │   └── armor_tracker
    │   │   └── src
    │   ├── auto_aim_interfaces
    │   │   └── msg
    │   ├── docs
    │   └── rm_auto_aim
    ├── rm_gimbal_description
    │   ├── docs
    │   └── urdf
    ├── rm_serial_driver
    │   ├── config
    │   ├── docs
    │   ├── include
    │   │   └── rm_serial_driver
    │   ├── launch
    │   └── src
    ├── rm_vision
    │   ├── docs
    │   └── rm_vision_bringup
    │       ├── config
    │       └── launch
    ├── ros2-hik-camera
    │   ├── config
    │   ├── hikSDK
    │   │   ├── include
    │   │   └── lib
    │   │       ├── amd64
    │   │       └── arm64
    │   ├── launch
    │   └── src
    ├── ros2-hik-camera_second
    │   ├── config
    │   ├── hikSDK
    │   │   ├── include
    │   │   └── lib
    │   │       ├── amd64
    │   │       └── arm64
    │   ├── launch
    │   └── src
    └── test
        ├── include
        │   └── test
        └── src
```

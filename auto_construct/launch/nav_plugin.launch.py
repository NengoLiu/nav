import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ──────────────────────────────────────────────────────────
    # 1. 路径与参数定义
    # ──────────────────────────────────────────────────────────
    
    # 获取包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    localizer_share = FindPackageShare("localizer")
    auto_construct_share = FindPackageShare("auto_construct")

    # 声明地图文件夹路径参数 (由外部传入)
    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value='/home/nic/ROS/ROS/map/maps/map_20260324_234437',
        description='Full path to the directory containing map.yaml and map.pcd'
    )

    # 动态拼接地图和配置文件路径
    map_yaml_file = PathJoinSubstitution([LaunchConfiguration('map_dir'), 'pcd2map_map.yaml'])
    
    nav2_params_file = PathJoinSubstitution(
        [auto_construct_share, 'config', 'nav2_params.yaml']
    )
    
    localizer_config_path = PathJoinSubstitution(
        [localizer_share, "config", "localizer.yaml"]
    )

    # ──────────────────────────────────────────────────────────
    # 2. 节点定义
    # ──────────────────────────────────────────────────────────

    # A. 3D 全局定位节点 (假设它独立运行，不归 Nav2 生命周期管)
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{"config_path": localizer_config_path}]
    )





    # B. Nav2 综合启动 (包含 Map Server, Lifecycle Manager 和所有导航服务器)
    # 使用 bringup_launch.py 是最稳妥的，因为它内部逻辑处理了所有节点的顺序启动
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'use_composition': 'False',
        }.items()
    )

    # 单独启动 map_server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': False,
        }]
    )

    # map_server 的生命周期管理
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],   # 只管 map_server，没有 amcl
        }]
    )










    # C. 点云转激光雷达 (用于避障)
    pc_to_ls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_ls',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
        }],
        remappings=[
            ('/cloud_in', '/fastlio2/body_cloud'),
            ('/scan', '/scan')
        ]
    )

    # ──────────────────────────────────────────────────────────
    # 3. 返回启动描述
    # ──────────────────────────────────────────────────────────
    return LaunchDescription([
        map_dir_arg,
        localizer_node,
        nav2_navigation_launch,
        map_server_node,
        lifecycle_manager_map,
        pc_to_ls_node
    ])

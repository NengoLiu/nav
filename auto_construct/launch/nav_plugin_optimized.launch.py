import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ──────────────────────────────────────────────────────────
    # 参数声明
    # ──────────────────────────────────────────────────────────

    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value='',
        description='Full path to directory containing map.yaml and map.pcd'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='body',
        description='Robot base frame'
    )

    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar',
        description='Lidar frame'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ──────────────────────────────────────────────────────────
    # 覆盖路径相关参数
    # ──────────────────────────────────────────────────────────

    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value='',
        description='覆盖路径文件（YAML格式）'
    )

    coverage_frame_id_arg = DeclareLaunchArgument(
        'coverage_frame_id',
        default_value='map',
        description='覆盖路径坐标系ID'
    )

    skip_on_failure_arg = DeclareLaunchArgument(
        'skip_on_failure',
        default_value='true',
        description='失败时是否跳过当前路径点'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='是否自动开始执行覆盖路径'
    )

    # ──────────────────────────────────────────────────────────
    # 路径与参数定义
    # ──────────────────────────────────────────────────────────

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    localizer_share = FindPackageShare("localizer")
    auto_construct_share = FindPackageShare("auto_construct")

    # 动态拼接地图和配置文件路径
    map_yaml_file = PathJoinSubstitution([LaunchConfiguration('map_dir'), 'pcd2map_map.yaml'])

    nav2_params_file = PathJoinSubstitution(
        [auto_construct_share, 'config', 'nav2_params.yaml']
    )

    localizer_config_path = PathJoinSubstitution(
        [localizer_share, "config", "localizer.yaml"]
    )

    # ──────────────────────────────────────────────────────────
    # 导航模式专用节点
    # ──────────────────────────────────────────────────────────

    # 1. 3D 全局定位节点
    localizer_node = Node(
        package="localizer",
        namespace="localization",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{
            "config_path": localizer_config_path,
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }]
    )

    # 2. Nav2 map_server —— navigation_launch.py 的 lifecycle_manager_navigation
    #    的 node_names 硬编码为 [controller_server, planner_server, bt_navigator …]，
    #    不包含 map_server，所以必须：
    #    a) 显式启动 map_server 进程
    #    b) 另起一个专属 lifecycle_manager_map 来 configure/activate 它，
    #       否则 /map_server/load_map 服务永远不会上线。
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 4.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # 3. Nav2 导航系统 (planner / controller / bt_navigator / lifecycle_manager_navigation 等)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file,
            'use_composition': 'False',
        }.items()
    )

    # 4. 覆盖路径执行节点
    coverage_path_node = Node(
        package='auto_construct',
        namespace='coverage',
        executable='coverage_path_node',
        name='coverage_path',
        output="screen",
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'frame_id': LaunchConfiguration('coverage_frame_id'),
            'skip_on_failure': LaunchConfiguration('skip_on_failure'),
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 返回启动描述
    # ──────────────────────────────────────────────────────────
    return LaunchDescription([
        # 参数
        map_dir_arg,
        base_frame_arg,
        lidar_frame_arg,
        map_frame_arg,
        use_sim_time_arg,
        path_file_arg,
        coverage_frame_id_arg,
        skip_on_failure_arg,
        autostart_arg,

        # 导航专用节点
        localizer_node,
        map_server_node,
        lifecycle_manager_map,
        nav2_navigation_launch,

        # 覆盖路径执行节点
        coverage_path_node,
    ])
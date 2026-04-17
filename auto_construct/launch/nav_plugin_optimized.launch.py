from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    localizer_share = FindPackageShare("localizer")
    auto_construct_share = FindPackageShare("auto_construct")

    nav2_params_file = PathJoinSubstitution(
        [auto_construct_share, 'config', 'nav2_params.yaml']
    )

    localizer_config_path = PathJoinSubstitution(
        [localizer_share, "config", "localizer.yaml"]
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    common_params = [nav2_params_file, {'use_sim_time': use_sim_time}]

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
            "use_sim_time": use_sim_time,
        }],
    )

    # 2. Nav2 map_server —— apt 的 navigation_launch.py 不启动 map_server，
    #    且 lifecycle_manager_navigation 也不管 map_server，所以单独拉起并由
    #    专属 lifecycle_manager_map 负责 configure/activate。
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=common_params,
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
            'use_sim_time': use_sim_time,
        }],
    )

    # 3. Nav2 核心节点
    #    不再 include nav2_bringup/navigation_launch.py，因为它会启动
    #    /opt/ros/humble/lib/nav2_bt_navigator/bt_navigator —— 该可执行文件
    #    在本机会与 coverage_ws/install/backported_bt_navigator/lib/
    #    libbt_navigator_core.so 发生 ABI 冲突（同名但构造签名不同）而秒崩。
    #    这里显式声明每个 Nav2 节点，bt_navigator 改用 backported 包自带的
    #    可执行文件，与其 .so 配套，ABI 一致。

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_params,
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common_params,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_params,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_params,
    )

    # bt_navigator：必须用 backported_bt_navigator 包的可执行文件，
    # 跟它的 libbt_navigator_core.so ABI 匹配。
    bt_navigator_node = Node(
        package='backported_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_params,
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=common_params,
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_params,
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,
            'use_sim_time': use_sim_time,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
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
            'use_sim_time': use_sim_time,
        }],
    )

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

        # 定位
        localizer_node,

        # 地图服务器
        map_server_node,
        lifecycle_manager_map,

        # Nav2 核心
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_navigation,

        # 覆盖路径执行
        coverage_path_node,
    ])

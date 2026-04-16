"""
all_in_one.launch.py

一键启动所有子系统：建图 + 路径规划 + 导航
各功能互不依赖，同时常驻运行，通过 service 独立控制：

  建图控制：
    /sys/start_mapping   (仅标记，FastLIO2/PGO/Octomap 已就绪)
    /sys/finish_mapping  (触发 PGO 存图 + Nav2 地图保存)

  覆盖路径规划：
    /sys/set_region
    /sys/confirm_region
    /sys/update_params
    /sys/get_map_list

  导航执行：
    /sys/start_navigation (仅标记，Nav2/Localizer 已就绪)
    /sys/set_path_and_start
    /sys/stop_all
"""

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
    # 公共参数
    # ──────────────────────────────────────────────────────────

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='body',
        description='Robot base frame')

    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame', default_value='lidar',
        description='Lidar frame')

    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Map frame')

    # ──────────────────────────────────────────────────────────
    # 导航/定位参数
    # ──────────────────────────────────────────────────────────

    map_dir_arg = DeclareLaunchArgument(
        'map_dir', default_value='',
        description='已保存地图目录（含 map.yaml / map.pcd），导航定位时提供')

    # ──────────────────────────────────────────────────────────
    # 覆盖路径规划参数
    # ──────────────────────────────────────────────────────────

    robot_width_arg = DeclareLaunchArgument(
        'robot_width', default_value='0.5',
        description='机器人宽度（m）')

    operation_width_arg = DeclareLaunchArgument(
        'operation_width', default_value='0.5',
        description='作业宽度/条带间距（m）')

    min_turning_radius_arg = DeclareLaunchArgument(
        'min_turning_radius', default_value='0.5',
        description='最小转弯半径（m）')

    headland_width_arg = DeclareLaunchArgument(
        'headland_width', default_value='0.5',
        description='地头宽度（m）')

    route_type_arg = DeclareLaunchArgument(
        'route_type', default_value='BOUSTROPHEDON',
        description='路线类型：BOUSTROPHEDON | SNAKE | SPIRAL | CUSTOM')

    curve_type_arg = DeclareLaunchArgument(
        'curve_type', default_value='DUBIN',
        description='转弯曲线：DUBIN | REEDS_SHEPP')

    swath_angle_arg = DeclareLaunchArgument(
        'swath_angle', default_value='-1.0',
        description='条带角度（-1 = 自动）')

    gml_output_dir_arg = DeclareLaunchArgument(
        'gml_output_dir', default_value='/tmp/coverage_gml',
        description='GML 临时文件输出目录')

    map_base_dir_arg = DeclareLaunchArgument(
        'map_base_dir', default_value='/home/nic/ROS/ROS/map/maps',
        description='地图根目录（包含所有地图文件夹）')

    # ──────────────────────────────────────────────────────────
    # 覆盖路径执行参数
    # ──────────────────────────────────────────────────────────

    path_file_arg = DeclareLaunchArgument(
        'path_file', default_value='',
        description='覆盖路径文件（YAML），可在运行时通过 /sys/set_path_and_start 动态设置')

    coverage_frame_id_arg = DeclareLaunchArgument(
        'coverage_frame_id', default_value='map',
        description='覆盖路径坐标系')

    skip_on_failure_arg = DeclareLaunchArgument(
        'skip_on_failure', default_value='true',
        description='失败时是否跳过当前路径点')

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='是否自动开始执行覆盖路径')

    # ══════════════════════════════════════════════════════════
    # 节点定义
    # ══════════════════════════════════════════════════════════

    # ──────────────────────────────────────────────────────────
    # 1. 传感器驱动：Livox MID360
    # ──────────────────────────────────────────────────────────
    livox_ros_driver2_pkg = get_package_share_directory('livox_ros_driver2')
    include_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_ros_driver2_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # ──────────────────────────────────────────────────────────
    # 2. FastLIO2 — 激光惯性里程计（常驻，建图/导航共用）
    # ──────────────────────────────────────────────────────────
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'config', 'lio.yaml'])

    fastlio2_node = Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{
            'config_path': lio_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 3. PGO — 位姿图优化（常驻，finish_mapping 时触发存图）
    # ──────────────────────────────────────────────────────────
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare('pgo'), 'config', 'pgo.yaml'])

    pgo_node = Node(
        package='pgo',
        namespace='mapping',
        executable='pgo_node',
        name='pgo_node',
        output='screen',
        parameters=[{
            'config_path': pgo_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 4. Octomap Server — 3D 占用地图（常驻，建图实时可视化）
    # ──────────────────────────────────────────────────────────
    octomap_node = Node(
        package='octomap_server',
        namespace='mapping',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.05,
            'frame_id': LaunchConfiguration('map_frame'),
            'base_frame_id': LaunchConfiguration('base_frame'),
            'sensor_model/max_range': 15.0,
            'latch': True,
            'filter_ground': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('/cloud_in', '/fastlio2/body_cloud')]
    )

    # ──────────────────────────────────────────────────────────
    # 5. SaveMap — 点云→OccupancyGrid 转换与存储（常驻）
    # ──────────────────────────────────────────────────────────
    save_map_config = PathJoinSubstitution(
        [FindPackageShare('auto_construct'), 'config', 'save_map.yaml'])

    save_map_node = Node(
        package='auto_construct',
        namespace='mapping',
        executable='save_map_node',
        name='save_map_node',
        output='screen',
        parameters=[{
            'config_path': save_map_config,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 6. Nav2 Map Saver Server（常驻，finish_mapping 时保存 2D 地图）
    # ──────────────────────────────────────────────────────────
    map_saver_server = Node(
        package='nav2_map_server',
        namespace='nav2',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{
            'save_map_timeout': 15.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    lifecycle_manager_map_saver = Node(
        package='nav2_lifecycle_manager',
        namespace='nav2',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_saver_server'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 7. 点云→激光雷达转换（常驻，Nav2 避障感知）
    # ──────────────────────────────────────────────────────────
    pc_to_ls_node = Node(
        package='pointcloud_to_laserscan',
        namespace='perception',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_ls',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('base_frame'),
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/cloud_in', '/fastlio2/body_cloud'),
            ('/scan', '/scan'),
        ]
    )

    # ──────────────────────────────────────────────────────────
    # 8. Localizer — 3D 全局定位（常驻，导航时自动生效）
    # ──────────────────────────────────────────────────────────
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare('localizer'), 'config', 'localizer.yaml'])

    localizer_node = Node(
        package='localizer',
        namespace='localization',
        executable='localizer_node',
        name='localizer_node',
        output='screen',
        parameters=[{
            'config_path': localizer_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 9. Nav2 导航栈（常驻，BT Navigator + 控制器 + 规划器）
    # ──────────────────────────────────────────────────────────
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('auto_construct'), 'config', 'nav2_params.yaml'])

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

    # ──────────────────────────────────────────────────────────
    # 10. opennav_coverage — 覆盖路径算法服务器（常驻）
    # ──────────────────────────────────────────────────────────
    coverage_server_node = Node(
        package='opennav_coverage',
        namespace='coverage',
        executable='opennav_coverage',
        name='opennav_coverage',
        output='screen',
        parameters=[{
            'coordinates_in_cartesian_frame': True,
            'robot_width': LaunchConfiguration('robot_width'),
            'operation_width': LaunchConfiguration('operation_width'),
            'min_turning_radius': LaunchConfiguration('min_turning_radius'),
            'default_headland_width': LaunchConfiguration('headland_width'),
            'default_route_type': LaunchConfiguration('route_type'),
            'default_path_type': LaunchConfiguration('curve_type'),
            'default_swath_angle': LaunchConfiguration('swath_angle'),
            'linear_curv_change': 2.0,
            'default_headland_type': 'CONSTANT',
            'default_swath_angle_type': 'BRUTE_FORCE',
            'default_step_angle': 0.1,
            'default_swath_type': 'LENGTH',
            'default_allow_overlap': False,
            'default_path_continuity_type': 'CONTINUOUS',
            'default_turn_point_distance': 0.1,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    lifecycle_manager_coverage = Node(
        package='nav2_lifecycle_manager',
        namespace='coverage',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['opennav_coverage'],
            'bond_timeout': 4.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 11. CoverageManager — 覆盖路径规划管理（常驻）
    # ──────────────────────────────────────────────────────────
    coverage_manager_node = Node(
        package='auto_construct',
        namespace='coverage',
        executable='convert_map_node',
        name='coverage_manager',
        output='screen',
        parameters=[{
            'robot_width': LaunchConfiguration('robot_width'),
            'headland_width': LaunchConfiguration('headland_width'),
            'route_type': LaunchConfiguration('route_type'),
            'curve_type': LaunchConfiguration('curve_type'),
            'swath_angle': LaunchConfiguration('swath_angle'),
            'gml_output_dir': LaunchConfiguration('gml_output_dir'),
            'map_base_dir': LaunchConfiguration('map_base_dir'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 12. CoveragePath — 覆盖路径执行引擎（常驻）
    # ──────────────────────────────────────────────────────────
    coverage_path_node = Node(
        package='auto_construct',
        namespace='coverage',
        executable='coverage_path_node',
        name='coverage_path',
        output='screen',
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'frame_id': LaunchConfiguration('coverage_frame_id'),
            'skip_on_failure': LaunchConfiguration('skip_on_failure'),
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 13. MappingManager — 统一 service 入口（常驻）
    # ──────────────────────────────────────────────────────────
    mapping_manager_node = Node(
        package='auto_construct',
        executable='mapping_manager_node',
        name='mapping_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ══════════════════════════════════════════════════════════
    # LaunchDescription
    # ══════════════════════════════════════════════════════════
    return LaunchDescription([
        # 参数
        use_sim_time_arg,
        base_frame_arg,
        lidar_frame_arg,
        map_frame_arg,
        map_dir_arg,
        robot_width_arg,
        operation_width_arg,
        min_turning_radius_arg,
        headland_width_arg,
        route_type_arg,
        curve_type_arg,
        swath_angle_arg,
        gml_output_dir_arg,
        map_base_dir_arg,
        path_file_arg,
        coverage_frame_id_arg,
        skip_on_failure_arg,
        autostart_arg,

        # ── 传感器 ──
        include_livox,

        # ── 建图子系统 ──
        fastlio2_node,
        pgo_node,
        octomap_node,
        save_map_node,
        map_saver_server,
        lifecycle_manager_map_saver,

        # ── 感知 ──
        pc_to_ls_node,

        # ── 导航子系统 ──
        localizer_node,
        nav2_navigation_launch,

        # ── 覆盖路径规划子系统 ──
        coverage_server_node,
        lifecycle_manager_coverage,
        coverage_manager_node,
        coverage_path_node,

        # ── 统一 service 入口 ──
        mapping_manager_node,
    ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ──────────────────────────────────────────────────────────
    # 参数声明
    # ──────────────────────────────────────────────────────────

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
    # 导航模式相关参数
    # ──────────────────────────────────────────────────────────

    robot_width_arg = DeclareLaunchArgument(
        'robot_width',
        default_value='0.5',
        description='机器人宽度（m）'
    )

    operation_width_arg = DeclareLaunchArgument(
        'operation_width',
        default_value='0.5',
        description='作业宽度/条带间距（m）'
    )

    min_turning_radius_arg = DeclareLaunchArgument(
        'min_turning_radius',
        default_value='0.5',
        description='最小转弯半径（m）'
    )

    headland_width_arg = DeclareLaunchArgument(
        'headland_width',
        default_value='0.5',
        description='地头宽度（m）'
    )

    route_type_arg = DeclareLaunchArgument(
        'route_type',
        default_value='BOUSTROPHEDON',
        description='路线类型：BOUSTROPHEDON | SNAKE | SPIRAL | CUSTOM'
    )

    curve_type_arg = DeclareLaunchArgument(
        'curve_type',
        default_value='DUBIN',
        description='转弯曲线：DUBIN | REEDS_SHEPP'
    )

    swath_angle_arg = DeclareLaunchArgument(
        'swath_angle',
        default_value='-1.0',
        description='条带角度'
    )

    gml_output_dir_arg = DeclareLaunchArgument(
        'gml_output_dir',
        default_value='/tmp/coverage_gml',
        description='GML 输出目录'
    )

    map_base_dir_arg = DeclareLaunchArgument(
        'map_base_dir',
        default_value='/home/nic/ROS/ROS/map/maps',
        description='地图基础目录'
    )

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
    # 常驻节点配置
    # ──────────────────────────────────────────────────────────

    # 1. Livox 雷达驱动
    livox_ros_driver2_pkg = get_package_share_directory('livox_ros_driver2')

    include_livox_ros_driver2_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_ros_driver2_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 2. FastLIO2 SLAM (常驻)
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    fastlio2_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{
            "config_path": lio_config_path,
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }]
    )

    # 3. Mapping Manager (系统模式管理，常驻)
    mapping_manager_node = Node(
        package="auto_construct",
        executable="mapping_manager_node",
        name="mapping_manager",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }]
    )

    # 4. Save Map Node (建图核心，常驻)
    save_map_config = PathJoinSubstitution(
        [FindPackageShare("auto_construct"), "config", "save_map.yaml"]
    )

    save_map_node = Node(
        package="auto_construct",
        namespace="mapping",
        executable="save_map_node",
        name="save_map_node",
        output="screen",
        parameters=[{
            "config_path": save_map_config,
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }]
    )

    # 5. 点云转激光雷达 (常驻，用于避障)
    pc_to_ls_node = Node(
        package='pointcloud_to_laserscan',
        namespace='perception',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_ls',
        output="screen",
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
            ('/scan', '/scan')
        ]
    )

    # 6. Nav2 Map Saver Server (常驻，用于保存地图)
    map_saver_server = Node(
        package='nav2_map_server',
        namespace='nav2',
        executable='map_saver_server',
        name='map_saver_server',
        output="screen",
        parameters=[{
            'save_map_timeout': 15.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 7. Nav2 Lifecycle Manager (管理 map_saver_server)
    lifecycle_manager_map_saver = Node(
        package='nav2_lifecycle_manager',
        namespace='nav2',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output="screen",
        parameters=[{
            'autostart': True,
            'node_names': ['map_saver_server'],
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ──────────────────────────────────────────────────────────
    # 导航和覆盖路径规划节点（按需启动）
    # ──────────────────────────────────────────────────────────

    # ──────────────────────────────────────────────────────────
    # 注释：导航和覆盖路径规划节点
    #        这些节点现在通过服务动态启动，不再在此 launch 文件中启动
    #        使用以下服务启动：
    #        - /sys/start_navigation: 启动导航模式
    #        - /sys/set_region: 设置覆盖区域
    #        - /sys/confirm_region: 确认区域并规划路径
    #        - /sys/set_path_and_start: 设置路径并开始执行
    # ──────────────────────────────────────────────────────────

    # ──────────────────────────────────────────────────────────
    # 返回启动描述
    # ──────────────────────────────────────────────────────────
    return LaunchDescription([
        # 参数
        base_frame_arg,
        lidar_frame_arg,
        map_frame_arg,
        use_sim_time_arg,

        # 驱动和核心算法
        include_livox_ros_driver2_system,
        fastlio2_node,

        # 业务控制
        mapping_manager_node,
        save_map_node,

        # 感知
        pc_to_ls_node,

        # 地图保存
        map_saver_server,
        lifecycle_manager_map_saver,
    ])

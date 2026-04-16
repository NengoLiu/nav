from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ──────────────────────────────────────────────────────────
    # 参数声明
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ──────────────────────────────────────────────────────────
    # opennav_coverage 节点
    # ──────────────────────────────────────────────────────────
    coverage_server_node = Node(
        package='opennav_coverage',
        namespace='coverage',
        executable='opennav_coverage',
        name='opennav_coverage',
        output="screen",
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

    # ──────────────────────────────────────────────────────────
    # lifecycle_manager 激活 opennav_coverage
    # ──────────────────────────────────────────────────────────
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        namespace='coverage',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output="screen",
        parameters=[{
            'autostart': True,
            'node_names': ['opennav_coverage'],
            'bond_timeout': 4.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ──────────────────────────────────────────────────────────
    # coverage_manager（覆盖路径管理节点）
    # ──────────────────────────────────────────────────────────
    coverage_manager_node = Node(
        package='auto_construct',
        namespace='coverage',
        executable='convert_map_node',
        name='coverage_manager',
        output="screen",
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
    # 返回启动描述
    # ──────────────────────────────────────────────────────────
    return LaunchDescription([
        # 参数
        robot_width_arg,
        operation_width_arg,
        min_turning_radius_arg,
        headland_width_arg,
        route_type_arg,
        curve_type_arg,
        swath_angle_arg,
        gml_output_dir_arg,
        map_base_dir_arg,
        use_sim_time_arg,

        # 覆盖路径规划节点
        coverage_server_node,
        lifecycle_manager_node,
        coverage_manager_node,
    ])

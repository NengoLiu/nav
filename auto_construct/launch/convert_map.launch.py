from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 只保留会按项目修改的参数 ─────────────────────────────
    args = [
        DeclareLaunchArgument(
            'robot_width', default_value='0.5',
            description='机器人宽度（m）'),

        DeclareLaunchArgument(
            'operation_width', default_value='0.5',
            description='作业宽度/条带间距（m）'),

        DeclareLaunchArgument(
            'min_turning_radius', default_value='0.5',
            description='最小转弯半径（m）'),

        DeclareLaunchArgument(
            'headland_width', default_value='0.5',
            description='地头宽度（m）'),

        DeclareLaunchArgument(
            'route_type', default_value='BOUSTROPHEDON',
            description='路线类型：BOUSTROPHEDON | SNAKE | SPIRAL | CUSTOM'),

        DeclareLaunchArgument(
            'curve_type', default_value='DUBIN',
            description='转弯曲线：DUBIN | REEDS_SHEPP'),
    ]

    # ── opennav_coverage 节点 ─────────────────────────────────
    coverage_server_node = Node(
        package='opennav_coverage',
        executable='opennav_coverage',
        name='opennav_coverage',
        output='screen',
        parameters=[{
            'coordinates_in_cartesian_frame': True,

            # 会变的参数从 launch 读
            'robot_width':            LaunchConfiguration('robot_width'),
            'operation_width':        LaunchConfiguration('operation_width'),
            'min_turning_radius':     LaunchConfiguration('min_turning_radius'),
            'default_headland_width': LaunchConfiguration('headland_width'),
            'default_route_type':     LaunchConfiguration('route_type'),
            'default_path_type':      LaunchConfiguration('curve_type'),

            # 固定值：一般不需要改
            'linear_curv_change':           2.0,
            'default_headland_type':        'CONSTANT',
            'default_swath_angle_type':     'BRUTE_FORCE',
            'default_swath_angle':          0.0,
            'default_step_angle':           0.1,
            'default_swath_type':           'LENGTH',
            'default_allow_overlap':        False,
            'default_path_continuity_type': 'CONTINUOUS',
            'default_turn_point_distance':  0.1,
        }]
    )

    # ── lifecycle_manager 激活 opennav_coverage ────────────────
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[{
            'autostart':    True,
            'node_names':   ['opennav_coverage'],
            'bond_timeout': 4.0,
        }]
    )

    # ── coverage_manager（我们自己的节点）─────────────────────
    coverage_manager_node = Node(
        package='auto_construct',
        executable='convert_map_node',
        name='convert_map_node',
        output='screen',
        parameters=[{
            'robot_width':    LaunchConfiguration('robot_width'),
            'headland_width': LaunchConfiguration('headland_width'),
            'route_type':     LaunchConfiguration('route_type'),
            'curve_type':     LaunchConfiguration('curve_type'),
            'swath_angle':    -1.0,
            'gml_output_dir': '/tmp/coverage_gml',
            'pgm_path':       '',
        }]
    )

    return LaunchDescription(args + [
        coverage_server_node,
        lifecycle_manager_node,
        coverage_manager_node,
    ])
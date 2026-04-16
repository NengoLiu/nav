from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ──────────────────────────────────────────────────────────
    # 参数声明
    # ──────────────────────────────────────────────────────────

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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

    # ──────────────────────────────────────────────────────────
    # 建图模式专用节点
    # ──────────────────────────────────────────────────────────

    # 1. PGO (位姿图优化)
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    pgo_node = Node(
        package="pgo",
        namespace="mapping",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{
            "config_path": pgo_config_path,
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }]
    )

    # 2. Octomap Server (3D 地图)
    octomap_server_node = Node(
        package='octomap_server',
        namespace='mapping',
        executable='octomap_server_node',
        name='octomap_server',
        output="screen",
        parameters=[{
            'resolution': 0.05,
            'frame_id': LaunchConfiguration('map_frame'),
            'base_frame_id': LaunchConfiguration('base_frame'),
            'sensor_model/max_range': 15.0,
            'latch': True,
            'filter_ground': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/cloud_in', '/fastlio2/body_cloud')
        ]
    )

    # ──────────────────────────────────────────────────────────
    # 返回启动描述
    # ──────────────────────────────────────────────────────────
    return LaunchDescription([
        # 参数
        use_sim_time_arg,
        base_frame_arg,
        lidar_frame_arg,
        map_frame_arg,

        # 建图专用节点
        pgo_node,
        octomap_server_node,
    ])

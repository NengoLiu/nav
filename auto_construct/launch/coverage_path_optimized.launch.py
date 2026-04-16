from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ──────────────────────────────────────────────────────────
    # 参数声明
    # ──────────────────────────────────────────────────────────

    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value='',
        description='路径文件（YAML格式）'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='坐标系ID'
    )

    skip_on_failure_arg = DeclareLaunchArgument(
        'skip_on_failure',
        default_value='true',
        description='失败时是否跳过当前路径点'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='是否自动开始执行路径'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ──────────────────────────────────────────────────────────
    # coverage_path_node（覆盖路径执行节点）
    # ──────────────────────────────────────────────────────────
    coverage_path_node = Node(
        package='auto_construct',
        namespace='coverage',
        executable='coverage_path_node',
        name='coverage_path',
        output="screen",
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'frame_id': LaunchConfiguration('frame_id'),
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
        path_file_arg,
        frame_id_arg,
        skip_on_failure_arg,
        autostart_arg,
        use_sim_time_arg,

        # 覆盖路径执行节点
        coverage_path_node,
    ])

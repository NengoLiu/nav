import os
import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    livox_ros_driver2_pkg = get_package_share_directory('livox_ros_driver2')

    include_livox_ros_driver2_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_ros_driver2_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )


    # 1. 常驻计步器：FastLIO2
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    fastlio2_node = launch_ros.actions.Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config_path.perform(launch.LaunchContext())}]
    )
    
    # 2. 多线程业务大管家 (负责系统模式切换)
    mapping_manager_node = launch_ros.actions.Node(
        package="auto_construct",
        executable="mapping_manager_node", 
        name="mapping_manager",
        output="screen"
    )

    # 3. 自定义建图核心节点 (你的 save_map_node)
    save_map_config = os.path.join(
        get_package_share_directory('auto_construct'),
        'config',
        'save_map.yaml'
    )

    save_map_node = launch_ros.actions.Node(
        package="auto_construct",
        executable="save_map_node", 
        name="save_map_node",
        output="screen",
        parameters=[save_map_config]  # 🌟 把参数文件喂给它
    )
    
    # 4. Nav2 存图服务器 (常驻后台接单)
    map_saver_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{'save_map_timeout': 15.0}]  # 给足耐心，防止大地图切片保存超时
    )

    # 5. Nav2 生命周期管理器 (专门负责把 map_saver_server 唤醒)
    lifecycle_manager_map_saver = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_saver_server']}  # 必须和上面的 name 严格一致
        ]
    )

    
    return launch.LaunchDescription([
        
        include_livox_ros_driver2_system,

        # 核心算法
        fastlio2_node,
        
        # 业务控制逻辑
        mapping_manager_node,
        save_map_node,
        map_saver_server,
        lifecycle_manager_map_saver

    ])
    
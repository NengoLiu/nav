import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution          # ← 补上
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare          # ← 补上
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    pgo_node = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_config_path}]         # ← 直接传，不要 .perform()
    )

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

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.05,
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'sensor_model/max_range': 15.0,
            'latch': True,
            'filter_ground': True,
        }],
        remappings=[
            ('/cloud_in', '/fastlio2/body_cloud')
        ]
    )

    map_saver_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{'save_map_timeout': 5.0}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_saver_server']
        }]
    )

    return LaunchDescription([
        pgo_node,
        pc_to_ls_node,
        octomap_server_node,
        #map_saver_server_node,
        #lifecycle_manager_node,
    ])
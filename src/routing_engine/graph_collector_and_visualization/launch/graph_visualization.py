import os
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 手動指定你的 Waypoint JSON 路徑
    default_waypoint_path = "/home/csl/isaac_routing2_v2/src/routing_engine/graph_collector_and_visualization/waypoints.json"
    default_rviz_config = "/home/csl/isaac_routing2_v2/src/spot_config/rviz/waypoints.rviz"

    return LaunchDescription([
        DeclareLaunchArgument(
            name='waypoint',
            default_value=default_waypoint_path,
            description='Path to the waypoint JSON file'
        ),

        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz config file'
        ),

        # 啟動 RViz2 並自動開啟 map frame
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),

        # 啟動你自己寫的 waypoint visualizer
        Node(
            package='spot_config',
            executable='waypoint_visualizer.py',
            name='waypoint_visualizer',
            output='screen',
            parameters=[{
                'waypoint': LaunchConfiguration('waypoint')
            }]
        ),
    ])

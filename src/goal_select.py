#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import json
import subprocess
import time

WAYPOINT_FILE = '/home/wenjia/isaac_routing2/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_1f.json'
TASK_JSON_FILE = '/home/wenjia/isaac_routing2/isaac_routing/src/routing_engine/graph_collector_and_visualization/task_data.json'

class ClickedPointReceiver(Node):
    def __init__(self):
        super().__init__('clicked_point_receiver')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10)

        self.x = None
        self.y = None
        self.z = 0.0

    def point_callback(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.z = msg.point.z

        self.get_logger().info(f'接收到點:x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}')
        nearest_wp, dist = self.nearest_waypoint()
        if nearest_wp:
            self.get_logger().info(f'最近的 waypoint 是：{nearest_wp}，距離為 {dist:.2f}')
            self.generate_task_json([nearest_wp])  # 這裡直接把它當任務
        else:
            self.get_logger().warn('無法找到最近的 waypoint。')

    def load_waypoints(self):
        with open(WAYPOINT_FILE, 'r') as f:
            return json.load(f)

    def nearest_waypoint(self):
        waypoints = self.load_waypoints()
        nearest_wp = None
        min_distance = float('inf')

        for wp_id, wp_info in waypoints.items():
            wp_x, wp_y = wp_info["local_location"][:2]
            distance = ((self.x - wp_x) ** 2 + (self.y - wp_y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_wp = wp_id

        return nearest_wp, min_distance

    def generate_task_json(self, task_sequence):
        demand = [1] * len(task_sequence)
        data = {
            "task_sequence": task_sequence,
            "demand": demand
        }

        with open(TASK_JSON_FILE, 'w') as f:
            json.dump(data, f, indent=4)

        self.get_logger().info(f"已生成任務 JSON:{TASK_JSON_FILE}")

def launch_ros2():
    try:
        subprocess.run(
            "source /opt/ros/humble/setup.bash && ros2 launch routing_agent RunServer.launch.py",
            shell=True,
            executable="/bin/bash",
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"執行 ROS launch 失敗：{e}")

def main(args=None):
    # 可先生成空任務檔案
    generate_empty_task_json()

    # 啟動 ROS launch（非同步）
    subprocess.Popen(
        "source /opt/ros/humble/setup.bash && ros2 launch routing_agent RunServer.launch.py",
        shell=True,
        executable="/bin/bash"
    )

    # 等待 ROS launch 起來
    time.sleep(3)

    rclpy.init(args=args)
    node = ClickedPointReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def generate_empty_task_json():
    data = {
        "task_sequence": [],
        "demand": []
    }
    with open(TASK_JSON_FILE, 'w') as f:
        json.dump(data, f, indent=4)
    print(f"已建立空任務 JSON:{TASK_JSON_FILE}")

if __name__ == '__main__':
    main()

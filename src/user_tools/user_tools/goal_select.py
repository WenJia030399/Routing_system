#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import json
import subprocess

# WAYPOINT_FILE = '/home/csl/isaac_routing2_v2/src/routing_engine/graph_collector_and_visualization/tsmc_1f.json'
TASK_JSON_FILE = 'src/routing_engine/graph_collector_and_visualization/task_data.json'
ENV_FILE = 'src/env_config.json'

class ClickedPointReceiver(Node):
    def __init__(self):
        super().__init__('clicked_point_receiver')

        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10)

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10
        )

        self.waypoints = self.load_waypoints()
        self.launched = False
        self.x = None
        self.y = None
        self.z = 0.0

        self.waypoint_selected = None

        # 定時發送 RViz Marker
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('節點啟動成功')

    def point_callback(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.z = msg.point.z

        # if self.waypoint_selected == None:
        self.get_logger().info(f'接收點:x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}')
        nearest_wp, dist = self.nearest_waypoint()
        if nearest_wp:
            self.get_logger().info(f'最鄰近 waypoint:{nearest_wp}')
            self.generate_task_json([nearest_wp])
            self.waypoint_selected = nearest_wp

        # if not self.launched:
        self.get_logger().info('啟動 ROS Launch...')
        self.launch_ros2()
        self.launched = True
        # else:
            # self.get_logger().info(f'已選擇 waypoint:{self.waypoint_selected}')

    def load_waypoints(self):
        try:    
            with open(ENV_FILE,'r',encoding='utf-8') as f:
                config = json.load(f)
                WAYPOINT_FILE = config['waypoint_graph']
            with open(WAYPOINT_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f'讀取 waypoint 失敗：{e}')
            return {}

    def nearest_waypoint(self):
        nearest_wp = None
        min_distance = float('inf')

        for wp_id, wp_info in self.waypoints.items():
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

    def launch_ros2(self):
        try:
            subprocess.Popen(
                "source /opt/ros/humble/setup.bash && ros2 launch routing_agent RunServer.launch.py",
                shell=True,
                executable="/bin/bash"
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"執行 ROS launch 失敗：{e}")

    def create_waypoint_marker(self, waypoint_id, position):
        # Sphere marker
        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = "waypoints"
        sphere.id = hash(waypoint_id) % 2147483647
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = position[0]
        sphere.pose.position.y = position[1]
        sphere.pose.position.z = 0.0
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.4
        sphere.scale.y = 0.4
        sphere.scale.z = 0.4
        if self.waypoint_selected == waypoint_id:
            sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            sphere.color = ColorRGBA(r=0.6, g=0.6, b=1.0, a=1.0)

        # Text marker
        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "waypoint_labels"
        text.id = hash(waypoint_id + "_text") % 2147483647
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = position[0]
        text.pose.position.y = position[1]
        text.pose.position.z = 0.3
        text.pose.orientation.w = 1.0
        text.text = waypoint_id
        text.scale.z = 0.3
        text.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

        return [sphere, text]

    def create_edge_marker(self, edge_id, start_pos, end_pos):
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "edges"
        line.id = edge_id
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.lifetime.sec = 1
        line.scale.x = 0.1
        line.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        p1 = Point(x=start_pos[0], y=start_pos[1], z=0.0)
        p2 = Point(x=end_pos[0], y=end_pos[1], z=0.0)
        line.points = [p1, p2]

        return line

    def publish_markers(self):
        marker_array = MarkerArray()
        edge_id = 0

        for wp_id, wp_info in self.waypoints.items():
            markers = self.create_waypoint_marker(wp_id, wp_info['local_location'])
            marker_array.markers.extend(markers)

            for edge_id_str in wp_info.get('edges', []):
                if edge_id_str in self.waypoints and edge_id_str > wp_id:
                    edge_marker = self.create_edge_marker(
                        edge_id,
                        wp_info['local_location'],
                        self.waypoints[edge_id_str]['local_location']
                    )
                    marker_array.markers.append(edge_marker)
                    edge_id += 1

        self.marker_publisher.publish(marker_array)

def generate_empty_task_json():
    data = {
        "task_sequence": [],
        "demand": []
    }
    with open(TASK_JSON_FILE, 'w') as f:
        json.dump(data, f, indent=4)
    print(f"已建立空任務 JSON:{TASK_JSON_FILE}")

def SaveCurrevtntState(currentnodeid):
    with open('src/routing_engine/test_run/sample_data/vehicle_data.json', 'r') as f:
        config = json.load(f)
    with open('src/routing_engine/test_run/sample_data/vehicle_data.json', 'w') as f:
        config['vehicle_locations'][0] = currentnodeid
        json.dump(config, f, indent=4)

def main(args=None):
    SaveCurrevtntState("000_000")

    generate_empty_task_json()
    rclpy.init(args=args)
    node = ClickedPointReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import json
import os
from datetime import datetime
import argparse

from pynput import keyboard

class PoseCollector(Node):
    def __init__(self, map_name, start_count):
        super().__init__('pose_collector')
        
        # Store map name for ID generation
        self.map_name = map_name.zfill(3)
        
        # Subscribe to initial pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.pose_callback,
            10
        )
        
        # Create a service for saving the file
        self.srv = self.create_service(
            Trigger,
            'save_waypoints',
            self.save_service_callback
        )

        # Create publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10
        )
        
        # Store collected poses
        self.poses = {}
        self.pose_count = start_count
        self.edge_count = 0
        self.pose_pre = ""
        self.ToUpdate = False



        # Initialize marker array
        self.marker_array = MarkerArray()
        
        self.get_logger().info(f'Pose collector node started for map {self.map_name}. Ready to collect poses from Rviz2...')
        self.get_logger().info('Use "ros2 service call /save_waypoints std_srvs/srv/Trigger" to save the waypoints.')

    def create_waypoint_marker(self, waypoint_id, position):
        # Create sphere marker
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "waypoints"
        sphere_marker.id = hash(waypoint_id) % 2147483647
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        
        # Set position
        sphere_marker.pose.position.x = position[0]
        sphere_marker.pose.position.y = position[1]
        sphere_marker.pose.position.z = 0.0
        sphere_marker.pose.orientation.w = 1.0
        
        # Set scale
        sphere_marker.scale.x = 0.3
        sphere_marker.scale.y = 0.3
        sphere_marker.scale.z = 0.3
        
        # Set color (blue)
        sphere_marker.color = ColorRGBA()
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 1.0

        # Create text marker for ID
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "waypoint_labels"
        text_marker.id = hash(waypoint_id + "_text") % 2147483647
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = 0.3
        text_marker.pose.orientation.w = 1.0
        
        text_marker.text = waypoint_id
        text_marker.scale.z = 0.3
        
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        
        return [sphere_marker, text_marker]

    def create_edge_marker(self, edge_id, start_pos, end_pos):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "edges"
        marker.id = edge_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Add start and end points
        point_start = Point()
        point_start.x = start_pos[0]
        point_start.y = start_pos[1]
        point_start.z = 0.0
        
        point_end = Point()
        point_end.x = end_pos[0]
        point_end.y = end_pos[1]
        point_end.z = 0.0
        
        marker.points = [point_start, point_end]
        
        # Set scale (line width)
        marker.scale.x = 0.05
        
        # Set color (white with transparency)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        return marker
        
    def pose_callback(self, msg):
        # Generate unique ID for new pose
        pose_id = f"{self.map_name}_{self.pose_count:03d}"
        
        # Extract position
        position = msg.pose.pose.position
        
        same_point_check, select_point = self.same_point_check(position)

        if self.ToUpdate:
            self.get_logger().info(f'Updating existing point {self.pose_pre} location to [{position.x:.3f}, {position.y:.3f}]')
            self.poses[self.pose_pre]["local_location"] = [
                float(position.x),
                float(position.y),
                0.0
            ]
            self.update_waypoint_marker(self.pose_pre, self.poses[self.pose_pre]["local_location"])
            self.update_edges_for_point(self.pose_pre)
            self.ToUpdate = False
            self.marker_publisher.publish(self.marker_array)

        elif same_point_check:
            self.get_logger().info(f'Connecting from point {select_point}')
            
            if self.pose_pre != select_point and self.pose_pre != "":
                self.connect_edge_continue(self.pose_pre, select_point)
                self.get_logger().info(f'Added edge between existing points {self.pose_pre} and {select_point}')
                self.pose_pre = select_point
                self.marker_publisher.publish(self.marker_array)
            
            elif self.pose_pre == select_point:
                # self.poses[select_point]["local_location"] = [
                #     float(position.x),
                #     float(position.y),
                #     0.0
                # ]
                self.ToUpdate = True
                self.get_logger().info(f'Updated existing point {select_point}')

            else:
                self.pose_pre = select_point

        else:
            # Create new pose entry
            self.poses[pose_id] = {
                "edges": [],
                "local_location": [
                    float(position.x),
                    float(position.y),
                    0.0
                ]
            }
        
            # Create visualization markers for the new pose
            markers = self.create_waypoint_marker(pose_id, self.poses[pose_id]["local_location"])
            self.marker_array.markers.extend(markers)
            
            # self.update_edges_for_new_point(pose_id)
            if self.pose_pre != "":
                self.connect_edge_continue(self.pose_pre,pose_id)

            # Publish updated markers
            self.marker_publisher.publish(self.marker_array)
            
            self.pose_count += 1
            self.get_logger().info(f'Received new pose: {pose_id} at position: [{position.x:.3f}, {position.y:.3f}]')
            self.pose_pre = pose_id
    
    def update_waypoint_marker(self, waypoint_id, new_position):
        new_markers = self.create_waypoint_marker(waypoint_id, new_position)

        new_ids = [m.id for m in new_markers]
        self.marker_array.markers = [
            m for m in self.marker_array.markers if m.id not in new_ids
        ] + new_markers

    def update_edges_for_point(self, point_id):
        # 先把 marker_array 中屬於 edge 的 marker 都刪掉
        self.marker_array.markers = [m for m in self.marker_array.markers if m.ns != "edges"]

        self.waypoint_plot()

    def get_edge_ids_by_marker(self, marker):
        if len(marker.points) != 2:
            return []
        start = marker.points[0]
        end = marker.points[1]
        def match(p1, p2):
            return abs(p1.x - p2[0]) < 1e-3 and abs(p1.y - p2[1]) < 1e-3
        for id1, pose1 in self.poses.items():
            for id2, pose2 in self.poses.items():
                if id1 == id2:
                    continue
                if match(start, pose1["local_location"]) and match(end, pose2["local_location"]):
                    return [id1, id2]
        return []

    def same_point_check(self, position):
        for existing_id, existing_data in self.poses.items():
            if self.check_same([position.x, position.y], existing_data['local_location']):
                return 1, existing_id
        
        return 0, "None"

    def check_same(self, p1, p2, threshold = 0.25):
        distance = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
        return distance < threshold

    def connect_edge_continue(self, pre_point_id, new_point_id):
        new_pos = self.poses[new_point_id]["local_location"]
        
        self.poses[new_point_id]["edges"].append(pre_point_id)
        self.poses[pre_point_id]["edges"].append(new_point_id)
        
        # Create edge marker
        edge_marker = self.create_edge_marker(
            self.edge_count,
            new_pos,
            self.poses[pre_point_id]["local_location"]
        )
        self.marker_array.markers.append(edge_marker)
        self.edge_count += 1
        
        self.get_logger().info(f'Added edge between points {new_point_id} and {pre_point_id}')
        
    def save_service_callback(self, request, response):
        try:
            if self.pose_count > 0:
                filename = self.save_to_file()
                response.success = True
                response.message = f'Successfully saved {self.pose_count} poses to {filename}'
            else:
                response.success = False
                response.message = 'No poses collected yet'
        except Exception as e:
            response.success = False
            response.message = f'Failed to save file: {str(e)}'
        return response
        
    def save_to_file(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'waypointgraph_{self.map_name}_{timestamp}.json'
        
        with open(filename, 'w') as f:
            json.dump(self.poses, f, indent=2)
        
        self.get_logger().info(f'Saved {self.pose_count} poses to {filename}')
        return filename

    def stop_edge_connect(self,key):
        try:
            if key.char == 'q':
                self.pose_pre=""
                print("Stop continue connection")
            elif key.char == 'd':
                self.delete_selected_point()    
        except AttributeError:
            pass

    def start_keyboard_listener(self):
        listener = keyboard.Listener(on_press = lambda key: self.stop_edge_connect(key))
        listener.daemon = True
        listener.start()

    def load_file(self, filename):
        with open(filename, 'r') as f:
            self.poses = json.load(f)
        self.pose_count = len(self.poses)
        self.get_logger().info(f'Loaded {self.pose_count} poses from {filename}')
        self.marker_array.markers.clear()

        for pose_id, pose_data in self.poses.items():
            markers = self.create_waypoint_marker(pose_id, pose_data["local_location"])
            self.marker_array.markers.extend(markers)

        # Create edges
        self.waypoint_plot()

    def waypoint_plot(self):
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()

        delete_array = MarkerArray()
        delete_array.markers.append(delete_marker)
        self.marker_publisher.publish(delete_array)

        self.marker_array.markers.clear()
        self.edge_count = 0
        for id1, pose1 in self.poses.items():
            for conn_id in pose1["edges"]:
                if id1 < conn_id: 
                    marker = self.create_edge_marker(
                        self.edge_count,
                        self.poses[id1]["local_location"],
                        self.poses[conn_id]["local_location"]
                    )
                    self.marker_array.markers.append(marker)
                    self.edge_count += 1
            markers = self.create_waypoint_marker(id1, self.poses[id1]["local_location"])
            self.marker_array.markers.extend(markers)

        self.marker_publisher.publish(self.marker_array)

    def delete_selected_point(self):
        if self.pose_pre == "":
            self.get_logger().info("No selected point to delete.")
            return

        point_id = self.pose_pre
        if point_id not in self.poses:
            self.get_logger().info(f"Point {point_id} not found.")
            return

        for other_id, data in self.poses.items():
            if point_id in data["edges"]:
                data["edges"].remove(point_id)

        del self.poses[point_id]

        self.get_logger().info(f"Deleted point {point_id} and related edges.")

        self.marker_array.markers.clear()

        for pid, pdata in self.poses.items():
            markers = self.create_waypoint_marker(pid, pdata["local_location"])
            self.marker_array.markers.extend(markers)

        self.waypoint_plot()
        self.pose_pre = ""

def main():
    parser = argparse.ArgumentParser(description='Collect waypoints from Rviz2')
    parser.add_argument('--map_name', type=str, default='0', help='Map identifier (will be used as XXX in point IDs XXX_YYY)')
    parser.add_argument('--start_count', type=int, default=0, help='Map identifier (will be used as YYY in point IDs XXX_YYY)')
    parser.add_argument('--file', type=str, default='empty.json', help='File to load existing waypoints from')
    args = parser.parse_args()
    
    rclpy.init()
    node = PoseCollector(args.map_name, args.start_count)
    
    node.start_keyboard_listener()
    node.load_file(args.file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.pose_count > 0:
            node.save_to_file()
            print("\nSaved waypoints file before shutting down.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

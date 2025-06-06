import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
import carb
import numpy as np
import sys
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.quadruped.robots import SpotFlatTerrainPolicy
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dataclasses import dataclass

@dataclass
class Pose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation



class SpotNav(Node):
    def __init__(self, env_usd_path, spot_poses: list):
        super().__init__("Spot_Nav")

        # 設定環境
        self.loadEnv(env_usd_path)

        self.first_step = True
        self.reset_needed = False
        self.cmd_scale = 1
        self.cmds = [np.zeros(3) for _ in range(len(spot_poses))]
        self.spots = []

        self.initSpots(spot_poses)

        self.ros_subs = []         

        self.odom_publishers = []
        for i in range(len(spot_poses)):
            topic_name = f"cmd_vel_{i}"
            sub = self.create_subscription(Twist, topic_name, lambda msg, i=i: self.spotCmdCallback(msg, i), 1)
            self.ros_subs.append(sub)
            
            odom_topic = f"/odom_{i}"
            odom_pub = self.create_publisher(Odometry, odom_topic, 10)
            self.odom_publishers.append(odom_pub)

        self.world.reset()

    def spotCmdCallback(self, msg: Twist, spot_index: int):
        """接收不同機器人的 cmd_vel 指令"""
        if self.world.is_playing():
            self.cmds[spot_index][0] = msg.linear.x * self.cmd_scale
            self.cmds[spot_index][2] = msg.angular.z * self.cmd_scale

    def initSpots(self, spot_poses: list):
        """初始化多台 Spot"""
        for i, pose in enumerate(spot_poses):
            spot = SpotFlatTerrainPolicy(
                prim_path=f"/World/Spot_{i}",  # 每台 Spot 使用不同的路徑
                name=f"Spot_{i}",
                usd_path="omniverse://localhost/Library/spot.usd",
                position=pose.position,
                orientation=pose.orientation
            )
            self.spots.append(spot)

        # 註冊 physics callback
        self.world.add_physics_callback("physics_step", callback_fn=self.onPhysicStep)
        self.world.reset()

    def onPhysicStep(self, step_size):
        """更新每台 Spot 的移動並發送 Odometry 訊息"""
        if self.first_step:
            for spot in self.spots:
                spot.initialize()
            self.first_step = False
        elif self.reset_needed:
            self.world.reset(True)
            self.reset_needed = False
            self.first_step = True
        else:
            for i, spot in enumerate(self.spots):
                spot.advance(step_size, self.cmds[i])

                # 從 `robot` 獲取位姿
                position, orientation = spot.robot.get_world_pose()
                # print(position[0],type(position[0]))
                linear_velocity = spot.robot.get_linear_velocity()
                angular_velocity = spot.robot.get_angular_velocity()

                # 發佈 Odometry 訊息
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = f"base_link_{i}"

                odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = map(float, position)
                odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z = map(float, linear_velocity)
                odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z = map(float, angular_velocity)


                # 發佈到對應的 /odom_{i} topic
                self.odom_publishers[i].publish(odom_msg)


    def loadEnv(self, env_usd_path):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()
        self.world = World(stage_units_in_meters=1.0, physics_dt=1 / 500, rendering_dt=1 / 50)

        prim = define_prim("/World/Ground", "Xform")
        asset_path = env_usd_path
        prim.GetReferences().AddReference(asset_path)

    def run_simulation(self):
        self.reset_needed = False
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_stopped():
                self.reset_needed = True

        # Cleanup
        self.destroy_node()
        simulation_app.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--env",
        type=str,
        choices=["lobby", "b1", "lab"],
        default="b1",
        help="Choice of navigation environment.",
    )
    parser.add_argument(
        "--num_spots",
        type=int,
        default=2,  # 預設 2 台 Spot
        help="Number of Spot robots in the simulation.",
    )
    args, _ = parser.parse_known_args()

    B1_USD_PATH = "omniverse://localhost/Library/tsmc_b1_map.usd"
    LOBBY_USD_PATH = "omniverse://localhost/Library/tsmc_1f_map.usd"
    LAB_USD_PATH = "/home/csl/isaac_routing/src/MIT_LAB/MIT_LAB.usd"

    spot_poses = []
    # if args.env == "b1":
    #     ENV_USD_PATH = B1_USD_PATH
    #     for i in range(args.num_spots):
    #         init_spot_pose.position=np.array([12.58 + i, 15.89, 0.87])
    #         init_spot_pose.orientation=np.array([0, 0, 0, np.pi])
    #         spot_poses.append(init_spot_pose)
    # elif args.env == "lobby":
    #     ENV_USD_PATH = LOBBY_USD_PATH
    #     for i in range(args.num_spots):
    #         init_spot_pose.position=np.array([12.58 + i, 15.89, 0.87])
    #         init_spot_pose.orientation=np.array([0, 0, 0, np.pi])
    #         spot_poses.append(init_spot_pose)
    if args.env == "lab":
        ENV_USD_PATH = LAB_USD_PATH
        for i in range(args.num_spots):
            spot_poses.append(Pose(position=np.array([12.58- 2*i, 15.89, 0.87]), orientation=np.array([0, 0, 0, np.pi])))

    rclpy.init()
    subscriber = SpotNav(ENV_USD_PATH, spot_poses)
    subscriber.run_simulation()


import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
import carb
import numpy as np
import sys
import time

from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.quadruped.robots import SpotFlatTerrainPolicy
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.rotations import euler_angles_to_quat

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dataclasses import dataclass
from pxr import UsdGeom, Usd, Gf

# 啟用 ROS2 bridge
enable_extension("omni.isaac.ros2_bridge")
simulation_app.update()

@dataclass
class Pose:
    position = np.zeros(3)
    orientation = np.array([1.0, 0.0, 0.0, 0.0])

def get_world_transform_xform(prim) -> tuple:
    """
    Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
    Args:
        prim: The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    world_transform = omni.usd.get_world_transform_matrix(prim)
    translation = world_transform.ExtractTranslation()
    rotation = world_transform.ExtractRotation()
    scale = np.array([v.GetLength() for v in world_transform.ExtractRotationMatrix()])
    return translation, rotation, scale
    
class SpotNav(Node):
    def __init__(self, env_usd_path, spot_init_pose: Pose, camera_mode: str):
        super().__init__("Spot_Nav")

        self.loadEnv(env_usd_path)
        self.first_step = True
        self.reset_needed = False
        self.cmd_scale = 1
        self.cmd = np.zeros(3)
        self.initSpot(spot_init_pose)
        self.init_camera()
        self.camera_position = None
        self.camera_mode = camera_mode

        self.ros_sub = self.create_subscription(Twist, "cmd_vel", self.spotCmdCallback, 1)
        self.world.reset()

    def loadEnv(self, env_usd_path):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()
        self.world = World(stage_units_in_meters=1.0, physics_dt=1 / 500, rendering_dt=1 / 50)
        prim = define_prim("/World/Ground", "Xform")
        prim.GetReferences().AddReference(env_usd_path)

    def initSpot(self, spot_init_pose: Pose):
        self.spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            usd_path="omniverse://localhost/Library/spot.usd",
            position=spot_init_pose.position,
            orientation=spot_init_pose.orientation
        )
        self.world.add_physics_callback("physics_step", callback_fn=self.onPhysicStep)
        self.world.reset()

    def init_camera(self):
        self.camera_offset = np.array([6, 0.0, 2.8])
        self.camera = Camera(
            prim_path="/World/camera",
            position=np.array([0.0, 0.0, 10.0]),
            frequency=10,
            resolution=(640, 480),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
        )


    def update_camera_pose(self):
        spot_prim = get_prim_at_path("/World/Spot/spot/body")
        if spot_prim is None or not spot_prim.IsValid():
            return

        spot_position, _, _ = get_world_transform_xform(spot_prim)
        target_position = spot_position + self.camera_offset

        if self.camera_position is None:
            self.camera_position = target_position
        else:
            alpha = 0.1
            self.camera_position = (1 - alpha) * self.camera_position + alpha * target_position

        smoothed_position = np.round(self.camera_position, 2)
        self.camera.set_world_pose(smoothed_position)


    def spotCmdCallback(self, msg: Twist):
        if self.world.is_playing():
            self.cmd[0] = msg.linear.x * self.cmd_scale
            self.cmd[2] = msg.angular.z * self.cmd_scale

    def onPhysicStep(self, step_size):
        if self.first_step:
            self.spot.initialize()
            self.first_step = False
        elif self.reset_needed:
            self.world.reset(True)
            self.reset_needed = False
            self.first_step = True
        else:
            self.spot.advance(step_size, self.cmd)

            if self.camera_mode == "follow":
                self.update_camera_pose()

    def run_simulation(self):
        self.reset_needed = False
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_stopped():
                self.reset_needed = True
        self.destroy_node()
        simulation_app.close()
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=str, choices=["lobby", "b1", "lab", "b1p1c", "1fr", "1f", "b1p1m", "b1p2m", "b1p2"],
                        default="b1", help="Choice of navigation environment.")
    parser.add_argument("--camera_mode", type=str, choices=["follow", "none"], default="follow",help="Camera mode.") 
    args, _ = parser.parse_known_args()

    B1_USD_PATH = "omniverse://localhost/Library/tsmc_b1_map.usd"
    LOBBY_USD_PATH = "omniverse://localhost/Library/tsmc_1f_map.usd"
    LAB_USD_PATH = "/home/csl/isaac_routing/src/MIT_LAB/MIT_LAB.usd"
    TSMC_B1P1_USDC_PATH = "/home/csl/isaac_routing/usd/TSMC B1 P1.usdc"
    TSMC_1F_RAW_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_1f_raw.usd"
    TSMC_1F_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_1f.usd"
    TSMC_B1P1_MAP_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_b1p1_map.usd"
    TSMC_B1P2_MAP_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_b1p2_map.usd"
    TSMC_B1P2_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_b1p2.usd"

    spot_init_pose = Pose()
    if args.env == "b1":
        ENV_USD_PATH = B1_USD_PATH
        spot_init_pose.position = np.array([6.65, -66.1, 0.8])
    elif args.env == "lobby":
        ENV_USD_PATH = LOBBY_USD_PATH
        spot_init_pose.position = np.array([214, -257.46, 0.8])
        spot_init_pose.orientation = np.array([0.707, 0, 0, 0.707])
    elif args.env == "lab":
        ENV_USD_PATH = LAB_USD_PATH
        spot_init_pose.position = np.array([12.58, 15.89, 0.87]) 
        spot_init_pose.orientation = np.array([0, 0, 0, np.pi])
    elif args.env == "1f":
        ENV_USD_PATH = TSMC_1F_USD_PATH 
        spot_init_pose.position = np.array([40, 30, 0.87]) 
    elif args.env == "b1p1m":
        ENV_USD_PATH = TSMC_B1P1_MAP_USD_PATH
        spot_init_pose.position = np.array([20, 20, 0.87]) 
    elif args.env == "b1p2m":
        ENV_USD_PATH = TSMC_B1P2_MAP_USD_PATH
        spot_init_pose.position = np.array([47, -30, 0.87]) 

    rclpy.init()
    subscriber = SpotNav(ENV_USD_PATH, spot_init_pose, args.camera_mode)
    subscriber.run_simulation()

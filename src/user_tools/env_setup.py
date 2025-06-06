import yaml
import json
import os
import argparse
import numpy as np

def modify_yaml_initial_pose(yaml_path, new_pose, output_path=None):
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    data["amcl"]["ros__parameters"]["initial_pose"] = new_pose

    save_path = output_path if output_path else yaml_path
    with open(save_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False)
    print(f"[YAML] initial_pose updated and saved to {save_path}")

if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--env",
            type=str,
            choices=["lobby", "b1", "lab", "b1p1c", "1fr", "1f", "b1p1", "b1p2", "b1p2", "4fp1", "4fp2", "5f", "cup1f"],
            default="b1",
            help="Choice of navigation environment.",
        )
        args, _ = parser.parse_known_args()

        B1_USD_PATH = "omniverse://localhost/Library/tsmc_b1_map.usd"
        LOBBY_USD_PATH = "omniverse://localhost/Library/tsmc_1f_map.usd"
        LAB_USD_PATH = "/home/csl/isaac_routing/src/MIT_LAB/MIT_LAB.usd"
    
        TSMC_1F_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_1f.usd"
        TSMC_B1P1_MAP_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_b1p1_map.usd"
        TSMC_B1P2_MAP_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_b1p2_map.usd"

        TSMC_4FP1_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_4f_p1.usd"
        TSMC_4FP2_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_4f_p2.usd"
        TSMC_5F_USD_PATH = "/home/csl/isaac_routing/usd/tsmc_5f.usd"
        TSMC_CUP_1F = "/home/csl/isaac_routing/usd/tsmc_cup_1f.usd"

        if args.env == "b1":
            ENV_USD_PATH = B1_USD_PATH
            spot_init_pose  = np.array([6.65, -66.1, 0.8])
            spot_init_orientation = np.array([0.707, 0, 0, 0.707])
            rviz_initial_pose = [154.115, 80.52, 0.0, 0.036]
            
        elif args.env == "lobby":
            ENV_USD_PATH = LOBBY_USD_PATH
            spot_init_pose  = np.array([214, -257.46, 0.8])
            spot_init_orientation = np.array([0.707, 0, 0, 0.707])
            
        elif args.env == "lab":
            ENV_USD_PATH = LAB_USD_PATH
            spot_init_pose  = np.array([12.58, 15.89, 0.87]) 
            spot_init_orientation = np.array([0, 0, 0, np.pi])
            rviz_initial_pose = [34.04, 38.02, -0.05, 0.0] # MIT LAB
        
        elif args.env == "1f":
            ENV_USD_PATH = TSMC_1F_USD_PATH 
            spot_init_pose  = np.array([40, 30, 0.87]) 
            spot_init_orientation = np.array([1, 0, 0, 0])
            rviz_initial_pose= [ 40.0,  30.0,  0.087,  0.0]     # TSMC 1F
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_1f.yaml"
            
            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_1f.json'


        elif args.env == "b1p1":
            ENV_USD_PATH = TSMC_B1P1_MAP_USD_PATH
            spot_init_pose  = np.array([20, 20, 0.87]) 
            spot_init_orientation = np.array([1, 0, 0, 0])
            rviz_initial_pose= [ 125.295, 38.428, 0.05, 1.0]     # TSMC B1P1 M
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_b1p1.yaml"
            
            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_b1p1.json'

        elif args.env == "b1p2":
            ENV_USD_PATH = TSMC_B1P2_MAP_USD_PATH
            spot_init_pose  = np.array([47, -30, 0.87]) 
            spot_init_orientation = np.array([1, 0, 0, 0])
            rviz_initial_pose= [ -45.954,  -13.651,  0.05,  0.9567]    # TSMC B1P2 M
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_b1p2.yaml"

            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_b1p2.json'


        elif args.env == "4fp1":
            ENV_USD_PATH = TSMC_4FP1_USD_PATH
            spot_init_pose  = np.array([-18.5, 65, 0.85]) 
            spot_init_orientation = np.array([0.707, 0, 0, -0.707])
            rviz_initial_pose= [ 144.2,  190.65,  0.05,  -2.102]     # TSMC F4P1
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_4fp1.yaml"

            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_4fp1.json'


        elif args.env == "4fp2":
            ENV_USD_PATH = TSMC_4FP2_USD_PATH
            spot_init_pose  = np.array([57.0, -29.0, 0.85]) 
            spot_init_orientation = np.array([0, 0, 0, 1])
            rviz_initial_pose= [ 164.25,  96.0,  0.05,  2.549]     # TSMC F4P2
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_4fp2.yaml"

            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_4fp2.json'


        elif args.env == "5f":
            ENV_USD_PATH = TSMC_5F_USD_PATH
            spot_init_pose  = np.array([45, 60, 0.85]) 
            spot_init_orientation = np.array([1, 0, 0, 0])
            rviz_initial_pose= [ 67.285,  161.576,  0.05,  1.021]     # TSMC 5F
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_5f.yaml"

            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_5f.json'


        elif args.env == "cup1f":
            ENV_USD_PATH = TSMC_CUP_1F
            spot_init_pose  = np.array([20, 13, 0.85]) 
            spot_init_orientation = np.array([-0.707, 0, 0, 0.707])
            rviz_initial_pose= [ 112.1,  20.1,  0.05,  1.042]     # TSMC F1_cup
            rviz_map = "/home/csl/isaac_routing2_v2/yaml/tsmc_cup_1f.yaml"

            waypoint_graph = '/home/csl/isaac_routing/src/routing_engine/graph_collector_and_visualization/tsmc_cup_1f_v2.json'

        config_data = {
            "ENV_USD_PATH": ENV_USD_PATH,
            "spot_init_pose": spot_init_pose.tolist(),
            "spot_init_orientation": spot_init_orientation.tolist(),
            "rviz_initial_pose": rviz_initial_pose,
            "rviz_map": rviz_map,
            "waypoint_graph": waypoint_graph
        }

        
        with open("/home/csl/isaac_routing2_v2/src/env_config.json", "w") as f:
            json.dump(config_data, f, indent=4)

        yaml_path = '/home/csl/isaac_routing2_v2/src/champ/spot_config/config/autonomy/navigation.yaml'
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        pose_dict = {
            "x": rviz_initial_pose[0],
            "y": rviz_initial_pose[1],
            "z": rviz_initial_pose[2],
            "yaw": rviz_initial_pose[3]
        }

        yaml_data['amcl']['ros__parameters']['initial_pose'] = pose_dict
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, sort_keys=False)

        print("Successfully updated.")

    except Exception as e:
        print(f"Error: {e}")
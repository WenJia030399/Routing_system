import argparse
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni
import carb
import numpy as np
import sys
import json

from omni.isaac.core.utils.extensions import enable_extension
# enable ROS2 bridge extension, cuopt
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.cuopt.service")
from omni.cuopt.service.waypoint_graph_model import (
    WaypointGraphModel,
    load_waypoint_graph_from_file,
)
from omni.cuopt.service.transport_orders import TransportOrders
from omni.cuopt.service.transport_vehicles import TransportVehicles
from omni.cuopt.service.cuopt_data_proc import preprocess_cuopt_data
from omni.cuopt.service.cuopt_microservice_manager import cuOptRunner
from omni.cuopt.service.common import (
    show_vehicle_routes,
    test_connection_microservice,
    test_connection_managed_service
)



class CuOptServer:
    def __init__(self, cuopt_url: str):
        self.cuopt_url = cuopt_url
        self.cuopt_runner = cuOptRunner(cuopt_url)

        self.graph = WaypointGraphModel()
        self.orders = TransportOrders()
        self.vehicles = TransportVehicles()
        self.time_limit = 0.01  # Default time limit for cuOpt solver

        self.graph_path = "/home/csl/isaac_routing2_v2/src/routing_engine/routing_agent_cuopt/waypoint_graph_1.json"

    def get_solution(self, environment_data):
        return self.cuopt_runner.get_routes(environment_data)
    
    def load_graph_from_file(self, file_path):
        """
        Load a waypoint graph from a JSON file.
        """
        self.graph_format_converter(file_path)
        self.graph = load_waypoint_graph_from_file(None, self.graph_path)
        
    def load_orders_from_file(self, file_path):
        """
        Load transport orders from a JSON file.
        """
        self.orders.load_sample(file_path)

    def load_vehicles_from_file(self, file_path):
        """
        Load transport vehicles from a JSON file.
        """
        self.vehicles.load_sample(file_path)

    def weight_redistribution(self):
        for i in range(len(self.graph.edges)):
            self.graph.weights.append(1.0)

    def get_environment_data(self):
        """
        Preprocess the data for cuOpt and return it.
        """
        waypoint_graph_data, fleet_data, task_data = preprocess_cuopt_data(
            self.graph, self.orders, self.vehicles
        )

        solver_config = {"time_limit": self.time_limit}

        environment_data = {
            "cost_waypoint_graph_data": waypoint_graph_data,
            "fleet_data": fleet_data,
            "task_data": task_data,
            "solver_config": solver_config
        }
        return environment_data
    
    def graph_format_converter(self, file_path=None):
        with open(file_path, 'r') as f:
            original_graph = json.load(f)
        if original_graph.keys() != ["node_locations", "graph"]:
            node_id_to_index = {node_id: idx for idx, node_id in enumerate(original_graph.keys())}
            
            # 建立 node_locations 陣列
            node_locations = [original_graph[node_id]["local_location"] for node_id in original_graph.keys()]
            
            # 建立 graph 結構
            graph = {}
            for node_id, idx in node_id_to_index.items():
                edges = original_graph[node_id]["edges"]
                edge_indices = [node_id_to_index[edge_id] for edge_id in edges]
                graph[str(idx)] = {"edges": edge_indices}

            new_graph = {
                "node_locations": node_locations,
                "graph": graph
            }
            with open(self.graph_path, 'w') as f:
                json.dump(new_graph, f, indent=4)

    def find_order_postion(self, order_id):
        """
        Find the position of an order by its ID.
        """
        for idx, order in enumerate(self.orders.orders):
            if order.order_id == order_id:
                return idx
        return None     

    def cmd_converter(self, cmd):
        """
        Convert a command string into a list of arguments.
        """
        return [f"000_{id:03d}" for id in cmd]

def main():
    cuopt_url = "http://10.100.1.147:6000/cuopt/"
    cuopt_server = CuOptServer(cuopt_url)
    cuopt_server.load_graph_from_file("/home/csl/isaac_routing2_v2/src/routing_engine/graph_collector_and_visualization/waypointgraph_000_20250120_142806.json")
    cuopt_server.load_orders_from_file("/home/csl/isaac_routing2_v2/src/routing_engine/routing_agent_cuopt/order_data.json")
    cuopt_server.load_vehicles_from_file("/home/csl/isaac_routing2_v2/src/routing_engine/routing_agent_cuopt/vehicle_data.json")
    cuopt_server.weight_redistribution()
    cuopt_server.orders.graph_locations = [12, 13, 22, 33, 32, 39, 45]
    environment_data = cuopt_server.get_environment_data()
    cuopt_solution = cuopt_server.get_solution(environment_data)
    print("CuOpt Solution:", cuopt_solution['vehicle_data']["0"]['route'])
    routing_cmd = cuopt_server.cmd_converter(cuopt_solution['vehicle_data']["0"]['route'])
    print("Routing Command:", routing_cmd)
    


if __name__ == "__main__":
    main()

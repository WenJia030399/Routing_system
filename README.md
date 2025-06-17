# Routing system
A system used to fulfill multi-robot routing in Omniverse based on ROS2 communication.

Based on Isaac sim 4.2

More detailed document https://hackmd.io/@jay030399/SJgeJCvjJx
## Start Routing system
### 1. Setup the environment
   Make sure the ```usd_path, args, rviz_map...``` inside ```env_setup.py``` are correct.
   Open cmd and enter
   ```
   cd /src/user_tools/
   python3 env_setup.py --env {the args you setup}
   ```
   it will return ```Successfully updated.```

### 2. Open Isaac Sim and load the model
   ```
   conda activate isaaclab # if you use conda environment
   cd /src/standalone
   python3 spot_nav_standalone.py
   ```

### 3. Open Rviz2 as user interface
   In another terminal
   ```
   colcon build
   source install/setup.bash
   ros2 launch spot_config navigate.launch.py 
   ```
   ignore the error ```2 packages had stderr output: ndt_omp_ros2 pcl_localization_ros2```


### 4. Destination assign
   There are two algorithm provided to solve rouing problem.

1. Our own algorithm
   
   Open new terminal
   ```
   source install/setup.bash
   ros2 run user_tools goal_select
   ```
   And back to Rviz window, select the tool  "Publish Point" at upside of window, and click the desired position on map

2. CuOpt
   
   Check the CuOpt service is available and enter the correct http inside program.
   Open new terminal
   ```
   python3 src/routing_agent_cuopt/routing_agent_cuopt/routing_cuopt_3.py 
   ```
   And back to Rviz window, select the tool  "Publish Point" at upside of window, and click the desired position on map
   
### 5. Start routing
   
   1 Our own algorithm
   
   Open new terminal
   ```
   source install/setup.bash
   ros2 run spot_routing_nav waypoint_nav
   ```
   
   2 CuOpt
   
   Open new terminal
   ```
   source install/setup.bash
   ros2 run spot_routing_nav waypoint_nav_cuopt 
   ```
### Hints
  1. If robot not moving after all program are activated, try to change the z-coordinate of robots to reset robots pose.
  2. Program of step 5 may take a while or you can just restart it.
  3. Make sure the light of Isaac sim is setup.
  4. Lidar's imformation may be incorrect in rviz at begining, use the tool "Nav2 Goal" control robot to move for a little distance.
  5. If the waypoints in rviz is invisible, change the Displays->MarkerArray->Topic.

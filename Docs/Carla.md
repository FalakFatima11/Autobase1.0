# CARLA Simulator Setup with ROS and LEGO-LOAM
This guide provides instructions for setting up CARLA Simulator version 0.9.11 with ROS and LEGO-LOAM. Follow the steps below for a seamless setup.
## Prerequisites
- Python 3.6
- ROS Noetic
- LEGO-LOAM
- LEGO-LOAM-BOR
## Installation Steps
1. **Download CARLA 0.9.11:**
Download the desired version of CARLA (0.9.11) from the official
repository.
2. **Extract and Copy Files:**
Extract the downloaded files and copy the contents of the **carla** and
**egg-info** folders into `~/.local/lib/python3.6/site-packages/`.
3. **Update PYTHONPATH:**
Add the following line to your **bashrc** file:
4. ```ExportPYTHONPATH=${PYTHONPATH}:/home/falak/packages/CARLA_0.9.11/PythonAPI/ carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg:/home/falak/packages/CARLA_0.9 .11/PythonAPI/carla```
5. **Run the carla simulator**
 `./CarlaUE4.sh -quality-level=low`
6. Open pythonAPI and source ros environment in other terminal
    `source /opt/ros/noetic/setup.bash`
      `roscore`
7. //Execute the following command to spawn a vehicle with LiDAR and spectator set on automatic control: 
 `python3/examples carla_main.py`
## Build SLAM with LEGO-LOAM:
8. In another terminal, source the LEGO-LOAM workspace and launch LEGO-LOAM
`source ~/catkin_ws_lego_loam/devel/setup.bash`
`roslaunch lego_loam run.launch` 
## Relocalize Vehicle Using LEGO-LOAM-BOR:
 After generating the map, in a new terminal, source the LEGO-LOAM-BOR workspace, create the map, and relocalize the vehicle in the mapped environment
`source ~/catkin_ws_loam_bor/devel/setup.bash`
'roslaunch lego_loam_bor createMap.launch lidar_topic:=/velodyne_point'
### The Map will be saved in temp folder when we shut down the algorithm and carla_main.py
## To Relocalise do:
 `Roslaunch lego_loam_bor localization.launch lidar_topic:=/velodyne_points`
 `carla_main.py`
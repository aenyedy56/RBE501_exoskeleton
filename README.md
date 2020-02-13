This package contains the code for three ROS Nodes (https://www.ros.org/). These nodes are used to process lidar sensor data in an atempt to find and locate stair geometry, then plan a path to climb the stairs given the dynamics of a lower limb exoskeleton. 

The nodes are as follows:
1) lidar_listener - C++ - Takes the raw sensor data and rotates, filters then segments each point cloud message into planar line segments. 
2) stair_param_extractor - C++  - takes the messages from the lidar_listener node and looks for lines matching the gemoetry of staris 
3) stair_path_planner - Python - takes messages from the stair_param_extractor node and plans first the individual footsteps for walking then climbing the identified stairs, then the trajectories between the footsteps using a DMP trained at startup. 

The nodes make some assumptions:
1) There is a lidar publishing on the 'cloud' or 'mock_cloud' ros topic 
2) The lidar produces a data array with 810 items (270 degrees * 3 measurements per degree) 
3) both C++ and RosPy are enabled 
4) The data to train the DPMs exists in the src/sim_walking dir. 
5) The path planner only plans for the first message it recieves and ignores subsequent updates. 
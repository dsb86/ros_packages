# Daniel Brotman
# daniel.brotman@case.edu
# 2/9/2016
# Mobile Robotics
# Project3

## my_path_server

my_path_server is a ROS package based off of exmple_server written originally by wyatt.newman@case.edu. This package is composed of 2 nodes my_path_client and my_path_server. my_path_client provides a collection of poses to my_path_server. my_path_server interprets the poses as a polyline interpolating orientation and drive dstances.

### Usage
Start Simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`

Start Client and Service
`roslaunch my_path_server my_path_server.launch`

OR

Start Service:
`rosrun my_path_server my_path_service`
Start Robot:
`rosrun my_path_server my_path_client`

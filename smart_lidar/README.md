# Daniel Brotman
# daniel.brotman@case.edu
# 1/26/2016
# Mobile Robotics
# Project2

## smart_lidar

smart_lidar is a ROS package based off of lidar_alarm written originally by wyatt.newman@case.edu. It scans a limited corridor of space that spans the width of the robot. It sends out warnings if the distance is less than .5 meters within the navigation corridor. The robot then navigates by driving around and rotating until it is out of a lidar warning area. 

### Usage
Start Simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start Lidar:
`rosrun smart_lidar smart_lidar`
Start Robot:
`rosrun stdr_control reactive_commander`

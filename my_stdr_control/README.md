# Daniel Brotman
# daniel.brotman@case.edu
# 1/26/2016
# Mobile Robotics
# Project1

## my_stdr_control

my_stdr_control is a ROS package based off of stdr_open_loop_commander written originally by wyatt.newman@case.edu. It is a minimal navigation control for an STDR in a fixed maze. Commands are based explicitly on timing, ignoring sensor inputs. 

## usage
Start Simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start Navigation:
`rosrun my_stdr_control my_stdr_control`

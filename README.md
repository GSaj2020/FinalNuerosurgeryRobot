# FinalNuerosurgeryRobot

Launching UR


 

Start ROS-IGTL-Bridge from a terminal on ROS computer 

10.211.55.5


 

$ cd ~/catkin_ws

$ source devel/setup.bash

$ roslaunch ros_igtl_bridge bridge.launch


 

Load Robot on Rviz

$ cd ~/catkin_ws

$ source devel/setup.bash

$ roslaunch NeuroRobot_moveit demo.launch


 

Load Robot controller

$ cd ~/catkin_ws

$ source devel/setup.bash

$ rosrun NeuroRobot_control igtl_importer.py

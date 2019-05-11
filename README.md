# ELFF
An assistive, gesture-controlled robot for item gathering.

Physically, ELFF is implemented using a [SunFounder Smart Video Car Kit](https://www.sunfounder.com/rpi-car.html) and a few 3D printed parts. 

# Not code related things to know about this repository 
- This repository was created for the purpose of keeping together code and documents for a class. So not all files are necessary for running ELFF
- The weekly status updates are not likely to be useful in understanding the code, but could give insight into the process of a Comptuter Science capstone at Pacific University

# Code related things to know about this repository 
- Read the report to get an understanding of how the system works
- The code requires that you have set up your .bashrc on both machines appropriately [here](https://razbotics.wordpress.com/2018/01/23/ros-on-multiple-computers-connecting-raspberry-pi-with-pc-over-lan/)
- [gesture_recognition.py](./gesture_recognition.py) runs on the computer with webcam
- [robot_control.py](/ros_catkin_workspace/src/elff_navigation/src/robot_control.py) runs on the raspberry pi on the robot
- roscore will need to be running on whatever machine you set up to be the master node in the .bashrc files



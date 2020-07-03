# ManArm Controller
## Description
This is a controller for a robot that had to be built during an Adaptive Robotics minor
at Fontys University of Applied Sciences.

Goals: 
* Provide a control interface similar to other robot arms on the market
* Support Linux including ARM, optionally windows  
* Save and load robot programs between platforms
* Control of each individual joint and IO pins


This project uses Tkinter as it supports different architectures. This toolkit is beginner friendly 
but also offers advanced features. 

## How to start
In order to start the UI you need to start the main_window.py with python 2. 

`` python src/gui/main_window.py ``

To run the guy you need to have `pillow` and `rospy` installed. You can install pillow by running.
`` pip install pillow ``

To install rospy please refer to installation of ROS. 

To run the unity simulation refer to the Simulation folder in this repository.


## Work in progress
This project is still in the early stage of development.

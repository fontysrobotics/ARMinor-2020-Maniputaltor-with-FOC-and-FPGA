Installation
======

## Pre-notes
The unity simulation and ROS system do not strictly have to be running on the same machine.
The Graphical user-interface does require a ROS environment (but is quite easy to setup even without ROS knowledge)

## ROS components
Note: This guide was written for ROS Melodic as the ROS version / distribution. Other version might also work, but you may need to check all packages or substitutes are available for these

### ROS repository packages
the following packages from the ROS repositories are required:
(ROS packages are normally named `ros-${rosversion}-${packagename}` e.g. `ros-melodic-rosbridge-server`)
- rosbridge-server: `rosbridge-server`
  non ROS dependencies: `python-tornado python-bson` (these are not automatically installed so install them manually)
- ros-control: `ros-control ros-controllers`
- moveit: `moveit`

### Other packages
- file_server: [RosSharp github](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server)
  This package allows for full urdf transfer between ROS and Unity
- unity_sim: From this github Repo (Simulation/ROS/unity_sim)
- universal_robot: [universal_robot github](https://github.com/ros-industrial/universal_robot)
  (optional / for the ur5 demo's)

## Unity project setup
All unity simulation setup instructions including setting up a robot and all communications between ROS and Unity are documented in a separate document as this is quite extensive.
See: [Unity Project Setup](UnitySetup.md)

## Graphical User-interface

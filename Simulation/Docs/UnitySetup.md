Unity Project Setup
======
This document describes how to setup the communication to ROS, how to import robot URDF models and how to connect these to ROS so they can be read out and controlled.

# Project Pre-Requirements (needed packages and scripts)
1. Download and import the `ROS#` package from the asset store (this is the most compatible and best working version). (version 1.6 was used at the time of writing)
2. Download our overrides to the RosSharp asset package ([Simulation/Unity/Assets/RosShap](https://github.com/fontysrobotics/ARMinor-2020-Maniputaltor-with-FOC-and-FPGA/tree/master/Simulation/Unity/Assets/RosSharp)) and copy then over the RosSharp asset package (`<UnityProject>/Assets/RosSharp`)
3. Download and add our additional scripts folder to your assets directory (`<UnityProject>/Assets/`) ([Simulation/Unity/Assets/RosShapFontys](https://github.com/fontysrobotics/ARMinor-2020-Maniputaltor-with-FOC-and-FPGA/tree/master/Simulation/Unity/Assets/RosSharpFontys))


# Project setup
This section covers all setup for importing robot models and setting up communication between ROS and Unity Robots.

## Importing URDF model to Unity
1. Copy the included publish launchfile and change the urdf file to the one you want to use.
2. Start the file_server by running `roslaunch file_server <launchfilename>` (remember the .launch extension needs to be included in the command).
3. In unity open the `RosBridgeClient` menu and click `Transfer URDF from ROS`.
4. When copying form a different project, some settings may retain, like the directory it will try to copy the urdf data to, so reset to defaults when copying rosbridge from other projects.
5. The address is probably `ws://127.0.0.1:9090` unless you changed the port or are connecting over the network.
6. Check the `Asset Path` in the `Settings` is within your project (otherwise things will not work).
7. Click `Read Robot Description` and after downloading let it create the game-objects and your Robot should show up in your simulation.

## Exposing the Robot to ROS and Connecting it to be controlled via ROS
### Initial ROS Bridge setup (the base connection to ROS)
1. Create a new game-object underneath your scene, This object is going to be used to setup the bridge between ROS and Unity (so name it sensibly) (this should not live underneath your robot model, this will be connected later).
2. Add the `Ros Connector` component to this object and configure it correctly like when you imported the model from ROS.  
   The address is probably `ws://127.0.0.1:9090` unless you changed the port or are connecting over the network

### Robot arm joints
1. To each revolute joint (ROS term) / Hinge joint (Unity term) you need to add a `JointStateReader` Component. (this will make it readable for the Joint Publisher later)
2. In each revolute joint (ROS term) / Hinge joint (Unity term) you need to check the joint name in the `Urdf Joint Revolute` component's configuration and how they are expected in ROS, if needed rename them in unity (eg the moveit controller definitions (controller.yaml))
3. Navigate to your RosBridge game-object
4. (optional) create another game-object that is going to be used for setting up the joint state publisher for easy management
5. Add the `Joint State Publisher` component. This will expose the joint states to ROS.  
   In the configuration you should setup the topic as expected by your ROS system (eg. `/joint_states` or `/<robotname>/joint_states`).  
   Under the `Joint State Readers`, set the size to the number of joints, and link each `JointStateReader` to an element.  
   The reference frame might need to be set to world as ROS expects this frame for the origin of the world.
6. TODO Subscribers for arm movement via ROS


### Robot arm calibration
In order to calibrate the arm, by default it will take the initial angle (when starting the simulation) as the 0 point. The axis for each hinge joint is used to correctly denote which way the motor moves.  
There is also the possibility to set a manual calibration in the `JointStateReader` or the `Urdf Joint Revolute` component
//TODO update docs to which of the 2 it is part of as this might change over time it is still TODO

# ROS Setup
When running the simulation the ROSBridge server needs to be running. For most of the systems that interact with the robot, the URDF also needs to be exposed inside of ROS for knowledge about the robot. So when running the simulation you could launch the ROS master with `roslaunch file_server <launchfilename>` (same as when importing the URDF in Unity). Or build your own launch file that launches the RosBridge and exposes the ROBOT URDF file.

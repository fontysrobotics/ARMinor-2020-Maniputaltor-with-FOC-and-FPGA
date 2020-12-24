# Affordable Manipulator With FPGA-driven FOC 


### Introduction 

Robotics can be prohibitively expensive and complex, preventing robotics students from learning about them. The goal of this project was to design an affordable, easy to use robot arm that would help solve these problems. The project was split into four parts: the electrical design, the mechanical design, the Field Oriented Control (FOC) implementation, and the simulation. Each of these projects' parts will be discussed in order below.


**This project is one of the deliverables of the [ROSIN EP FREROS project](https://www.rosin-project.eu/ftp/freros). It was developed for the Adaptive Robotics minor of Fontys University of Applied Sciences which aims to be one of the best ROS and Robotics education center's in the Netherlands.**

### Project structure

- **Electrical Design:** Within this folder you will find schematics, hardware documentation that was selected for the affordable manipulator.

- **Mechanical Design:** Within this folder you will find the proof of concept of the manipulator's mechanical design, alongside some design hardware selection documentation.


- **FieldOrientedControl:** Within this folder you will find the FOC implementation within FPGA boards. The files there is a simulink model of the controller and motors.

- **Simulation:** Within this folder you will find the simulation of the affordable manipulator in Unity. The project files for the communication between ROS and Unity, and the proof of concept of an GUI to interface with the simulation. 

**There is a final report available in this repository, which is meant to be the compiled version of our results and findings**

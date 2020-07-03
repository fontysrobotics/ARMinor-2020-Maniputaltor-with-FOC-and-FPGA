# Electrical Design
## General information 
The robotic arm will have 6 axes with a 100W 3-phase ac motor in each. 
It has to have a tool interface at its end with a 24V power supply and 24V I/O pins.
The robotic arm should be able to be plugged into the 230V 50Hz mains. 


## Power supplies
For the main power supplies a MEAN WELL RSP-750-48 and a MEAN WELL RS-35-5 were chosen for the 48V and 5V lines respectively.
They fit all the requirements for safety features and the RSP-750-48 also has active power factor correction.
It can continuously deliver 750 W and has active cooling.
Thus, it would be able to power the motor drivers for extended periods of time. 
Unfortunately, something that cannot be remotely tested or calculated is how well it behaves with the transient load of the motors. 


The low voltage power supply can deliver up to 35W, which would be enough for all the systems that use 5V or less. Moreover, those systems do not cause the same problems with transient loads so the power supply will handle it well.


It should be physically tested whether the power supply witll be able to handle the transient load of the 6 motors turning on or off at once. 

## FPGA 
Because each motor controller board has an individual FPGA, an FPGA with less pins and less logic gates could be chosen.
The Spartan-3A Xilinx XC3S50A-4VQG100C (Xilinx) was a suitable choice because of multiple reasons.
Fist it got a suitable price for the goal.
It also has a package with pins on the side instead of a ball grid array.
This makes it much easier to solder it by hand, which is helpful for this in-house project.
Moreover, it is from Xilinx, which was already used previously in the minor, thus the students were already familiar with its software. 


## Tool power supply 
The tool power supply is a simple implementation of a Buck convertert that steps down the 48V for the power supply to 24V.
The design goal was to be able to provide at least 600mA current, but this one can go to over 1A.


## The schematic 
The schematic was done on the free version of Eagle 9.1.3. It was not build and physically tested.
Because of the corona outbreak it was not possible to go to Fontys and build or test anything physical.
Therefore it is reccommended that the schematic is first built on a prototype board and tested carefully. 

## Communication between the main controller and motor controllers
In order to minimize the amount of cables inside the robot arm the motor drivers will be inside the axes, next to the motors and the main controller will communicate with them through the I2C protocol.
In this way there will be only 5 cables that run through the arm, moreover the signal will be more noise proof compared to the situation where the motor drivers will be inside the base and the connection from them to the motors will run through the robot arm. 
The tool I/O will also be controlled through the I2C lines.
This is possible since the tool does not require a big I/O bandwidth and the I2C standard bandwidth will be fast enough to accommodate both all of the motor controllers and the tool I/O as well.

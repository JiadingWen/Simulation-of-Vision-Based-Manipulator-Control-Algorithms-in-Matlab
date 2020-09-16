# Simulation of Vision-Based Control Algorithm for Robot Manipulator on Matlab

|![image](https://github.com/JiadingWen/01-Manipulator_Simulation/blob/master/img/ScreenShot1.gif)|![image](https://github.com/JiadingWen/01-Manipulator_Simulation/blob/master/img/ScreenShot2.gif)|
| - | :-: |

## Introduction
This project mainly simulates four control algorithms: 

1. Linear Camera Space Manipulation (**LCSM**)
2. Image-Based Visual Servo with monocular vision system (**Monocular IBVS**)
3. Image-Based Visual Servo with binocular vision system (**Binocular IBVS**)
4. Position-Based Visual Servo with binocular vision system (**Binocular PBVS**)

These algorithms are applied to a virtual PUMA robot manipulator on Matlab with Robotics Toolbox.

## Requirement
All codes have **only** been tested on 
* Windows10 1809 
* Matlab R2018b 
* Robotics Toolbox 10.3.1

There is no guarantee that the codes have good compatibility in the updated version. 
Before running code, make sure you have the Robotics Toolbox installed on Matlab.

> [Robotics Toolbox (RTB)](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) is a useful Matlab toolbox, which provides functions related to robotics. You can click the link to download RTB and install it. 

In this project, RTB is only used to visualise the robot manipulator during simulation. In other words, if you don not want to install RTB, you can comment all statements related to RTB in the code. It will not affect the operation of the code. 

But I don't recommend you to do this. Because after commenting the RTB related code, you won't be able to see a dynamic manipulator during simulation. If you can't see the manipulator, what's the fun of simulation?

## Usage
Four folders in the repository represent four control algorithms. Double-click `Main.m` in the each folder to start simulation of corresponding control algorithms. 

That‘s simple~ Enjoy playing robot!!
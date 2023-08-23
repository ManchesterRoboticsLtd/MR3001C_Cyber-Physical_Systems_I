<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/Logotipo%20Vertical%20Bco_Transparente.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/Logotipo%20Vertical%20Azul%20transparente.png">
  <img alt="Shows ITESM logo in black or white." width="160" align="right">
</picture>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/MCR2_Logo_White.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/MCR2_Logo_Black.png">
  <img alt="Shows MCR2 logo in black or white." width="150" align="right">
</picture>

# Week 3: Mini challenge

* In this folder, the student will find the neccessary files for Mini challenge 2.
* The instruction for this challenge can be found inside the presentation *MCR2_MiniChallenge_3*

### << We Encourage the students to NOT USE the files and follow the instructions during class and in the presentation to make this activity !! >>

## Description
This mini challenge is intended for the student to review the concepts introduced in the previous sessions.
* The activity involves creating a ROS node to communicate an ultrasonic sensor with the computer using ROS.
* The sensor will be read using a microcontroller (Arduino Board) as an intermediary to save the computer's processing time.

<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/c5d71f1a-48dd-495b-826d-e55c61f6d8eb" 
alt="ROS Basics" width="400" border="10"/></p>


## Instructions

* Create a "*sonar*" package into your *catkin_ws/src* folder.
* Create the Arduino *"scripts"* rqeuired to solve the problem description and instructions.
* Complete the Roslaunch and msg files inside the *"launch"*.
* Execute the nodes using the *rosalunch* command.

  - Remember to make the nodes executable using the the following commands
```
 chmod +x foo.py
```

## Rules
  * This is challenge, not a class. The students are encouraged to research, improve tune explain their algorithms by themselves.
  * MCR2(Manchester Robotics) Reserves the right to answer a question if it is determined that the question contains partially or an answer.
  * The students are welcome to ask only about the theoretical aspect of the class.
  * No remote control or any other form of human interaction with the simulator or ROS is allowed (except at the start when launching the files).
  * It is forbidden to use any other internet libraries except for standard libraries such as NumPy.
  * If in doubt about libraries, please ask any teaching assistant.
  * Improvements to the algorithms are encouraged and may be used if the students provide the reasons and a detailed explanation of the improvements.
  * All the students must respect each other and abide by the previously defined rules.
  * Manchester Robotics reserves the right to provide any form of grading. Grading and grading methodology are done by the professor in charge of the unit.

  
  ## Expected results
  Left image shows the results from this challenge. 
  
  Right image shows the plot of the results (using *rqt_plot*) where the blue line represents the "distance" signal.
  
  <p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/b34adf38-4f40-44bd-982a-67ad4ab37deb" 
alt="ROS Basics" width="800" border="10"/></p>

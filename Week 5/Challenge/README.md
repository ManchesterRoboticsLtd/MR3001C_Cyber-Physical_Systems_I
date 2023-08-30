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

# Week 4: Half-Term challenge

* In this folder, the student will find the necessary files for Half-Term Challenge.
* The instruction for this challenge can be found inside the presentation *MCR2_MiniChallenge_4*

### << We Encourage the students to NOT USE the files and follow the instructions during class and in the presentation to make this activity !! >>

## Description
This challenge is intended for the student to review the concepts introduced in the previous sessions.
* The activity consists of controlling the speed of a DC Motor.
* The motor speed, must be controlled using an external computer, a microcontroller, and a motor driver.

<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/42fcc7bd-e68d-410d-b90c-e5be420596e4" 
alt="ROS Basics" width="600" border="10"/></p>

## Instructions

* The challenge consists of creating three different nodes "/Input", "/Controller", "/Motor".
* The "/Input" and "/Controller" nodes must run on the External Computer, whilst the "/Motor" node must run in the MCU.
<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/4f76e43a-8d4b-421f-9bda-e814404f5d0d" 
alt="ROS Basics" width="600" border="10"/></p>

### Motor Node
* This node should act as a translator, to send information to the motor driver and, at the same time, estimate the speed of the motor to be sent to the controller.
    * The node must subscribe and translate the control input “𝑢(𝑡)” in the “/motor_input” topic to a PWM signal to the Motor Driver. 
    * The node must estimate the motor speed using the encoders’ information and publish the data in the topic “/motor_output”.
<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/e704a7e9-6850-4647-a0ca-d051381c20b1" 
alt="ROS Basics" width="400" border="10"/></p>

### Controller Node
* Make a node called /Controller” in the External Computing Unit, to generate a control input to the “/Motor” node.
    * The node must publish in the “/motor_input” topic and subscribe to the “/motor_output” and “/set_point” topics.
    * The output of the controller “/motor_input” must be bounded between in the interval 0 to 1 i.e., 𝑢(𝑘)∈[0,1].
    * The control node, must use a parameter file, for all the required tunning variables.
    * The controller can be  “P”, “PI” or “PID” controller (other controllers can be accepted upon agreement with the professor). 
    * The sampling time and rate must be defined by the student (Check Hints).
<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/645ae8d5-8a1b-4106-829a-767edd7d251d" 
alt="ROS Basics" width="400" border="10"/></p>

### Input Node (Set Point Node)
* The “/Input” node must publish into the previously defined topic “/set_point”.
    * The node must publish an input signal to the motor (select between step, sinusoidal, etc.).
    * The message for the “/set_point” topic must be defined by the student.
    * Must be a custom message. 
    * Must include at least 2 different variables.
    * As before, It is forbidden to use any libraries, except from NumPy for this exercise. 
    * Make the necessary plots to analyse the system in rqt_plot.
      
<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/837934f8-2309-47f7-9119-9c0cad85337e" 
alt="ROS Basics" width="250" border="10"/></p>

### More information, further instructions and some hints can be found in *MCR2_MiniChallenge_4*


  - Remember to make the nodes executable using the following commands
```
 chmod +x foo.py
```

## Rules
* This is a challenge, not a class. The students are encouraged to research, improve tune explain their algorithms by themselves.
* MCR2(Manchester Robotics) Reserves the right to answer a question if it is determined that the question contains partially or totally an answer.
* The students are welcome to ask only about the theoretical aspect of the class.
* No remote control or any other form of human interaction with the simulator or ROS is allowed (except at the start when launching the files).
* It is forbidden to use any other internet libraries with the exception of standard libraries or NumPy.
* If in doubt about libraries please ask any teaching assistant.
* Improvements to the algorithms are encouraged and may be used as long as the students provide the reasons and a detailed explanation on the improvements.
* All the students must be respectful towards each other and abide by the previously defined rules.
* Manchester Robotics reserves the right to provide any form of grading. Grading and grading methodology are done by the professor in charge of the unit.


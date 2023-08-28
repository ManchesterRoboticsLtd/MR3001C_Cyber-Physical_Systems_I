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


# Week 4: Activity: Pushbutton (Interrupts)
.
### << We Encourage the students to NOT USE the files and follow the instructions during class and in the presentation to make this activity !! >>

### Requirements
* Ubuntu in VM or dual booting
* ROS installed
* Arduino IDE
* Arduino IDE configured
  * Follow the tutorial on how to install it in the presentation:
       *MCR2_ROS_ArduinoIDE_Configuration*
* Arduino ROS Libraries
* Hackerboard/Arduino Mega/Arduino Uno

<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/05c0446a-350f-44ed-a9f5-1c5527f35084" 
alt="ROS Basics" width="400" border="10"/></p>


## Activity 1: pushbuttonV1_NoROS
### Usage
  1. This sketch verifies if the pushbutton is PRESSED (Interrupt) AND THEN RELEASED.
  2. STARTS a debounce timer 
  3. AFTER the timer has elapsed, change the status of the LED (toggle). 

### Connections
  * Connect a normally open (NO) pushbutton in a pulldown resistor configuration to the ButtonPin.
<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/12348746-825b-463b-ac97-0fcf0b508e62" 
alt="ROS Basics" width="400" border="10"/></p>


### Instructions 

* Configure the Arduino IDE according to the presentation "*MCR2_ROS_ArduinoIDE_Configuration*"
* Install the ROS Arduino libraries 
    * Open the folder "*ROS_Libraries*"
    * Open the folder according to your OS.
    * Download the .zip file. 
    * Import the libraries according to the tutorial in the following link
        [Arduino IDE Libraires](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries)

1. Open Arduino IDE (previously configured).
2. Open the program
3. Download the program to the Arduino
4. Open the Serial Monitor of the Arduino IDE.
5. Press the button and see the counter going up.

## Activity 2: pushbuttonV1_ROS

### Usage
  1. This sketch verifies if the pushbutton is PRESSED (Interrupt) AND THEN RELEASED.
  2. STARTS a debounce timer 
  3. AFTER the timer has elapsed, change the status of the LED (toggle), and the ROS publisher.
     
### Connections
  * Connect a normally open (NO) pushbutton in a pulldown resistor configuration to the ButtonPin.
<p align="center"><img src="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/12348746-825b-463b-ac97-0fcf0b508e62" 
alt="ROS Basics" width="400" border="10"/></p>

### Instructions 

* Configure the Arduino IDE according to the presentation "*MCR2_ROS_ArduinoIDE_Configuration*"
* Install the ROS Arduino libraries 
    * Open the folder "*ROS_Libraries*"
    * Open the folder according to your OS.
    * Download the .zip file. 
    * Import the libraries according to the tutorial in the following link
        [Arduino IDE Libraires](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries)

1. Open Arduino IDE (previously configured).
2. Open the program
3. Download the program to the Arduino
4. Run ROS and Rosserial in Ubuntu
5. Publish a Float32 value in the range [0,1] to the /PWM topic
6. Look at the Arduino Mega Builtin LED

### Pakcage: ros_arduino_comms
This package is just a launch file, intended to show the student how to launch the rosserial commnuication using a launch file.

1. Download the package inside your workspace (catkin_ws/src)
2. Source the files source /devel/setup.bash (if required)
3. launch the file (make sure Arduino is connected)
   
   ```
     roslaunch ros_arduino_comms pushbutton.launch
  ```



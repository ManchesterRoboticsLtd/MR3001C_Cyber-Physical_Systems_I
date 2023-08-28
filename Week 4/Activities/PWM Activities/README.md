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


# Week 4: Activity: PWM
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


## Activity 1: pwmV1_NoROS
### Usage
Increase/decrease the luminosity of the Built-in LED (Arduino Mega Pin13) by increasing the output PWM duty cycle value from 0 -> 255 and decreasing the PWM Value from 255->0 once it has reached the maximum value (255).

### Connections
  * No connections required (Arduino Mega Built-in LED in pin 13 used)

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
4. Observe the Built-in LED on pin 13.

## Activity 1: pwmV1_ROS
### Usage
Increase/decrease the luminosity of the Built-in LED (Arduino Mega Pin13) by increasing the output PWM duty cycle value from 0 -> 255.
This program remaps a float point value in the range [0,1], received from an external computer using ROS by subscribing to the topic "/PWM", to an integer analog PWM value in the range [0, 255] i.e., [0,1] -> [0,255].
The mapped value is then used to estate the duty cycle of a generated PWM signal. Such signal is then sent to the built-in LED of the Arduino Mega (For Arduino Uno change ports). 

### Connections
  * Connection to the computer using the USB cable
  * No connections required (Arduino Mega Built-in LED in pin 13 used)

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

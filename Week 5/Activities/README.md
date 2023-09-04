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


# Week 5: Activities and Examples
.
### << We Encourage the students NOT to USE the files and follow the instructions during class and in the presentation to make this activity !! >>

### Requirements
* Ubuntu in VM or dual booting
* ROS installed
* RVIZ installed

## Instructions
* Download the *"markers"* project to you *"catkin_ws/src"* folder
* Compile the project using *"catkin_make"*

## Activity 1: Marker
  ### Marker
  1. Make a new package, with the following packages 
    
      ```
      catkin_create_pkg markers rospy std_msgs tf2_ros visualization_msgs tf_conversions geometry_msgs
      ```
      
  2. Create a node called marker.py inside the scripts folder
     ```
      mkdir scripts && touch scripts/marker.py
      ```
     
  3. Give executable permission to the file
    ```
    cd ~/catkin_ws/src/markers/scripts/
    sudo chmod +x marker.py
    ```

  4. Modify the CMake file to include the newly created node
    ```
    catkin_install_python(PROGRAMS scripts/marker.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
     ```
  5. Open the file marker.py
  6. Define a new marker called sun (marker message), its publisher, and publish the marker (look at the code inside *"marker.py"*).
    ```

    #Marker Message Example
    sun = Marker()	#Declare Message
    
    #Header
    sun.header.frame_id = "sun"
    sun.header.stamp = rospy.Time.now()
    #Set Shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    sun.id = 0
    sun.type = 2
    #Add Marker
    sun.action = 0     #Action 0 add/modify, 2 delete object, 3 deletes all objects
    
    # Set the pose of the marker
    sun.pose.position.x = 0.0
    sun.pose.position.y = 0.0
    sun.pose.position.z = 0.0
    sun.pose.orientation.x = 0.0
    sun.pose.orientation.y = 0.0
    sun.pose.orientation.z = 0.0
    sun.pose.orientation.w = 1.0
    
    # Set the scale of the marker
    sun.scale.x = 2.0
    sun.scale.y = 2.0
    sun.scale.z = 2.0
    # Set the colour
    sun.color.r = 1.0
    sun.color.g = 1.0
    sun.color.b = 0.0
    sun.color.a = 1.0
    
    #Set Duration
    sun.lifetime = rospy.Duration(0) 


  8. Compile the program
  ```
    cd ~/catkin_ws
    catkin_make
  ```
  9. Start ROS
  ```
  roscore
  ```
  10. Run the node
  ```
  rosrun markers marker.py
  ```
  11. Start RViz
  ```
  rosrun rviz rviz 
  ```
  12. Add the marker 
  * Press Add
  * By topic>>/sun>> marker

## Activity 2.1: Declaring a static transform
  1. In the package *“markers”* create a launch file called *"marker.launch”*

    ```
    cd ~/catkin_ws/src/markers/ && mkdir launch
    cd launch && touch marker.launch
    ```
  2. Write and save the following inside the launch file

  ```
<?xml version="1.0" ?>
<launch>
    <node name="marker" pkg="markers" type="marker.py" output="screen"/> 

<node pkg="tf2_ros" type="static_transform_publisher" name="link1" args="2  1   1   3.1416  0   0   world   link1"/>
</launch>

  ```
  3. The Launch file Launches the previously created marker and creates a static transform 


### A more detailed description of each activity is inside the folder.

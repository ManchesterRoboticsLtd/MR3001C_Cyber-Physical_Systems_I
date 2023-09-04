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

<node pkg="tf2_ros" type="static_transform_publisher" name="link1" args="2  1   1   3.1416  0   0   sun   link1"/>
</launch>

  ```
  3. The Launch file Launches the previously created marker and creates a static transform
  4. Launch the file
   ```
   roslaunch marker marker.launch
  ```
  5. Open RVIZ in another terminal
  ```
  rosrun rviz rviz
  ```
  
  6. Change the Fixed Frame to “world”
  7. Click the button “Add” and on the “By display type” tab, select “Axes”.
  8. Repeat to Add two Axes
  9. Select one of the axes and change its “Reference Frame” to “link1”
  10. Click the button “Add” and on the “By Topic” tab, select Marker
  11. Click the button “Add” and on the “By display type” tab, select “TF”.


## Activity 2.2: Transformations
  1. In this activity, Static and Dynamic transforms will be generated in a script.
  2. In the package “markers” create a new node called “tf_act.py”
  
  ```
    cd ~/catkin_ws/src/markers/scripts/
    touch scripts/tf_act.py
  ```
  
  4. Give executable permission to the file
 
  ```
  cd ~/catkin_ws/src/markers/scripts/
  sudo chmod +x tf_act.py
  ```
  
  5. Modify the CMake file to include the newly created node to the 
 
 ```
 catkin_install_python(PROGRAMS scripts/marker.py scripts/tf_act.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  ```

  6. Open the file tf_act.py
    * Define three new frames called, inertial, sun and planet, their publishers and publish the transforms (Look at the code inside the file *"tf_act.py"*). 
  7. Compile the program
     ```
    cd ~/catkin_ws
    catkin_make
     ```
     
  8. Start ROS
   ```
  roscore
   ```
  9. Run the node
   ```
    rosrun markers tf_act.py 
   ```

  10. Start RViz
      
   ```
    rosrun rviz rviz 
   ```

  11. Add the marker 
  12. Press Add
  13. By display type>>TF

## Activity 2.2: Multiple Markers
  1. In this activity the knowledge acquired in the previous two activities will be used to create a series of planets orbiting a sun.
  2. In the package “markers” create a new node called “markers.py”
  
  ```
    cd ~/catkin_ws/src/markers/scripts/
    touch scripts/markers.py
  ```

  3. Give executable permission to the file
  
  ```
    cd ~/catkin_ws/src/markers/scripts/
    sudo chmod +x markers.py
  ```

  4. Modify the CMake file to include the newly created node to the 
    ```
     catkin_install_python(PROGRAMS scripts/marker.py scripts/markers.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    ```
  5. Follow the previous two activities to add a planet marker to the “sun” frame, make a moon rotate around the planet and an arrow pointing to the moon using transforms (Look at the code inside the file *"markers.py"*). 




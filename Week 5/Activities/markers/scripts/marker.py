#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Time

#Setup parameters, vriables and callback functions here (if required)
# Declare message
sun = Marker()

#Function to initialise a message
def init_sun():
    #Header
    sun.header.frame_id = "sun"
    sun.header.stamp = rospy.Time.now()
    #Set Shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    sun.id = 0
    sun.type = 2
    #Add Marker (Action: 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects)
    sun.action = 0
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
    # Set the color
    sun.color.r = 1.0
    sun.color.g = 1.0
    sun.color.b = 0.0
    sun.color.a = 1.0
    #Set Duration
    sun.lifetime = rospy.Duration(0) 

#Stop Condition
def stop():
    print("Stopping")


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("RVIZ_marker")

    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)
    print("The Sun is ready")

    #Setup the messages
    init_sun()

    #Setup Publishers and subscribers here
    pub_sun = rospy.Publisher('/sun', Marker, queue_size=1)

    try:
    #Run the node
        while not rospy.is_shutdown():
           
            sun.header.stamp = rospy.Time.now()

            pub_sun.publish(sun)
            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
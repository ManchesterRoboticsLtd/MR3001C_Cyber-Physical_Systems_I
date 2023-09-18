#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Time

#Define message globally
sun = Marker()

def init_sun():
    sun.header.frame_id = "sun"
    sun.header.stamp = rospy.Time.now()
    sun.id = 0
    #Set Shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    sun.type = 2
    #Set Action (Action: 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects)
    sun.action = 0
    
    #Set Pose of the object
    sun.pose.position.x = 0.0
    sun.pose.position.y = -0.5
    sun.pose.position.z = 0.0
    sun.pose.orientation.x = 0.0
    sun.pose.orientation.y = 0.0
    sun.pose.orientation.z = 0.0
    sun.pose.orientation.w = 1.0

    #Set Scale
    sun.scale.x = 0.5
    sun.scale.y = 0.5
    sun.scale.z = 0.5

    #Set Scale
    sun.color.r = 1.0
    sun.color.g = 1.0
    sun.color.b = 0.0
    sun.color.a = 1.0

    #Set Duration
    sun.lifetime = rospy.Duration(0)


def stop():
    print("Stopping")


#Decalre Main Function
if __name__=='__main__':

    #Initialise Node
    rospy.init_node("RVIZ_marker")

    #Configure Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)
    print("Configuraton Done!")

    #Setup Messages
    init_sun()

    #Setup Publisher
    pub_sun = rospy.Publisher('/sun', Marker, queue_size=1)

    while not rospy.is_shutdown():

        sun.header.stamp = rospy.Time.now()

        pub_sun.publish(sun)

        loop_rate.sleep()



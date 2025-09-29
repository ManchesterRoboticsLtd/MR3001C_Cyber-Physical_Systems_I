#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
# Setup Variables to be used
first = True   

# Declare the  process output message
controlOutput = Twist()
clock_msg = None

#Define the callback functions
def clock_callback(msg):
    global clock_msg
    clock_msg = msg

#Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("open_loop_controller")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",10))
    rospy.on_shutdown(stop)

    print("The controller is Running")

    try:

        while clock_msg == None:
            rospy.loginfo("Waiting for Gazebo")
            loop_rate.sleep()

        rospy.loginfo("Clock synchronized")
        rospy.sleep(0.5)

        #Run the node
        while not rospy.is_shutdown(): 

            if (first):

                first = False
            
            else:

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
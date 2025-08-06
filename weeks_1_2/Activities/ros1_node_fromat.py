#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


#Declare Global Variables to be used

#First = True

#Initialise messages to be published
#pub_msg = Float32()    #Change the type of message accordingly

#Initialise messages for the subscribers
#received_msg = Float32()    #Change the type of message accordingly

#Define callback functions
#def subscriber_callbackmsg(msg):
#    global received_msg
#    received_msg = msg

#Define other functions (if required)

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Control")

    #Declare Parameters to be used
    #kp = rospy.get_param(...)

    #Configure the Node
    rate = rospy.Rate(100)  #Node rate in Hz

    rospy.on_shutdown(stop) #What to do when ROS is Shutdown

    #Setup Publishers and Subscribers
    #rospy.Subscriber(...)
    #controlInput=rospy.Publisher(...)
    
    #Node Running
    print("The Node is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():

            #Wait and repeat
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
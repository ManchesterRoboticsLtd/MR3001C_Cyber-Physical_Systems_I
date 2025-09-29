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
    controlOutput.linear.x = 0.0
    controlOutput.angular.z = 0.0
    control_pub.publish(controlOutput)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("open_loop_controller")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",10))
    rospy.on_shutdown(stop)

    #Setup de publishers
    control_pub = rospy.Publisher("/puzzlebot/cmd_vel", Twist, queue_size=1)

    #Setup Subscribers
    clock_sub = rospy.Subscriber('/clock', Clock, clock_callback, queue_size=1)

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
                controlOutput.linear.x = 0.0
                controlOutput.angular.z = 0.0
                control_pub.publish(controlOutput)
                rospy.loginfo("Motion Initiated")
                first = False
            
            else:
                # Move Forward
                controlOutput.linear.x = 0.3
                controlOutput.angular.z = 0.0
                control_pub.publish(controlOutput)
                rospy.sleep(4)  # Move for 2 seconds

                # Stop
                controlOutput.linear.x = 0.0
                controlOutput.angular.z = 0.0
                control_pub.publish(controlOutput)
                rospy.sleep(1)

                # Turn
                controlOutput.linear.x = 0.0
                controlOutput.angular.z = 1.57  # 90-degree turn
                control_pub.publish(controlOutput)
                rospy.sleep(2)

                # Stop
                controlOutput.linear.x = 0.0
                controlOutput.angular.z = 0.0
                control_pub.publish(controlOutput)
                rospy.sleep(1)

                # Move Forward
                controlOutput.linear.x = 0.3
                controlOutput.angular.z = 0.0
                control_pub.publish(controlOutput)
                rospy.sleep(4)  # Move for 2 seconds

                
                # Stop
                controlOutput.linear.x = 0.0
                controlOutput.angular.z = 0.0
                control_pub.publish(controlOutput)
                rospy.sleep(1)

                rospy.loginfo("Motion Complete")
                rospy.signal_shutdown("Square Completed")


            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SetPoint_Generator")

    #Declare Variables/Parameters to be used
    Amplitude = rospy.get_param("~setpoint_Amplitude",8.0)
    Omega = rospy.get_param("~setpoint_Freq",0.1)

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("~setpoint_Rate",200))

    #Setup Publishers and Subscribers
    signal_pub=rospy.Publisher("/motor_input",Float32, queue_size=10)
    
    #Declare initial time
    init_time = rospy.get_time()

    while not rospy.is_shutdown():

        #Get time
        time = rospy.get_time()-init_time
        
        #Define the Set Point
        signal = Amplitude*np.sin(Omega*time)

        #Publish the Set Point
        signal_pub.publish(signal)

        #Wait and repeat
        rate.sleep()
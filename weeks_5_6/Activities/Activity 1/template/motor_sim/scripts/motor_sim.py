import rospy
import numpy as np
from std_msgs.msg import Float32

#Initial conditions

# Setup Variables to be used
first = True

# Declare the input Message

# Declare the  process output message

#Define the callback functions

 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Motor_Sim")
    
    #Declare Variables/Parameters to be used

    #Motor Parameters

    # Configure the Node
    loop_rate = rospy.Rate(200)
    rospy.on_shutdown(stop)

    # Setup the Subscribers
   
    #Setup de publishers
  
    print("The Motor is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True:
                #Initialise Variables Here
                first = False
        #System
            else:
                pass
            #Dynamical System Simulation        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
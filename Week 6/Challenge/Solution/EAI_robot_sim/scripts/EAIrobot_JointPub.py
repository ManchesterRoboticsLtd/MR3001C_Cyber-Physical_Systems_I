#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf_conversions
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# Declare the output Messages
contJoints = JointState()
robot_tf = TransformStamped()

q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.0)

def init_tf():
        #Define Transformations
    
    robot_tf.header.stamp = rospy.Time.now()
    robot_tf.header.frame_id = "world"
    robot_tf.child_frame_id = "base_link"
    robot_tf.transform.translation.x = 0.0
    robot_tf.transform.translation.y = 0.0
    robot_tf.transform.translation.z = 0.0
    robot_tf.transform.rotation.x = q[0]
    robot_tf.transform.rotation.y = q[1]
    robot_tf.transform.rotation.z = q[2]
    robot_tf.transform.rotation.w = q[3]


# Declare the output Messages
def init_joints():
    contJoints.header.frame_id = "base_link"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.extend(["rightWheel", "leftWheel"])
    contJoints.position.extend([0.0, 0.0])
    contJoints.velocity.extend([0.0, 0.0])
    contJoints.effort.extend([0.0, 0.0])


   #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi



    #Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")



if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("EAI_Robot_Sim")

 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/rate",100))
    rospy.on_shutdown(stop)

    #Init joints
    init_joints()
    init_tf()

    #Setup Transform Broadcasters
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    tf_broadcaster = TransformBroadcaster()


    print("The EAI Robot Joint Publisher is Running")
    try:
    #Run the node
        while not rospy.is_shutdown(): 
            t = rospy.Time.now().to_sec()

            #Update Transformations
            q_w = tf_conversions.transformations.quaternion_from_euler(0, 0.0, -0.05*t)
            robot_tf.header.stamp = rospy.Time.now()
            robot_tf.transform.translation.x = 3*np.sin(0.05*t)
            robot_tf.transform.translation.y = 3*np.cos(0.05*t)
            robot_tf.transform.translation.z = 0.0
            robot_tf.transform.rotation.x = q_w[0]
            robot_tf.transform.rotation.y = q_w[1]
            robot_tf.transform.rotation.z = q_w[2]
            robot_tf.transform.rotation.w = q_w[3]

            q_wR = tf_conversions.transformations.quaternion_from_euler(1.57, t*(3 + 0.33 * -0.05 / 2.0) / 0.124, 0.0)
            q_wL = tf_conversions.transformations.quaternion_from_euler(1.57, t*(3 - 0.33 * -0.05 / 2.0) / 0.124, 0.0)
            
            contJoints.header.stamp = rospy.Time.now()
            contJoints.position[0] = wrap_to_Pi(t*(3 + 0.33 * -0.05 / 2.0) / 0.124)
            contJoints.position[1] = wrap_to_Pi(t*(3 - 0.33 * -0.05 / 2.0) / 0.124)
                    
            joint_pub.publish(contJoints)
            tf_broadcaster.sendTransform(robot_tf)
            
            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
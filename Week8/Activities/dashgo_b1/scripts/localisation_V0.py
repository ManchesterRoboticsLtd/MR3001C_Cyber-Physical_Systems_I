#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import tf_conversions
from tf2_ros import TransformBroadcaster

#Set the parameters of the system
_wheel_r = rospy.get_param("/wheel_radius",0.062)
_robot_l = rospy.get_param("/robot_wheelbase",0.330)
_sample_time = rospy.get_param("/localisation_sample_time",0.01)
_node_rate = rospy.get_param("/localisation_node_rate",500)

#Set initial conditions of the system
pos_x = rospy.get_param("pos_x0",0.00)
pos_y = rospy.get_param("pos_y0",0.00)
pos_th = rospy.get_param("pos_th0",0.00)

# Setup Variables to be used
first = True
new_pose = False
start_time = 0.0
last_time =  0.0
current_time = 0.0

v_r = 0.0
v_l = 0.0
Vel = 0.0
Omega = 0.0
right_wheel_angle = 0.0
left_wheel_angle = 0.0

x_dot = 0.0
y_dot = 0.0
th_dot = 0.0

# Declare Messages to be used
robot_pose = PoseStamped()
robot_pose_prev = PoseStamped()
robot_odom = Odometry()
robot_tf = TransformStamped()

q = tf_conversions.transformations.quaternion_from_euler(0, 0, pos_th)

#Initialise messages (if required)
def init_tf():
    #Define Transformations
    robot_tf.header.stamp = rospy.Time.now()
    robot_tf.header.frame_id = "odom"
    robot_tf.child_frame_id = "base_link"
    robot_tf.transform.translation.x = pos_x
    robot_tf.transform.translation.y = pos_y
    robot_tf.transform.translation.z = 0.0
    robot_tf.transform.rotation.x = q[0]
    robot_tf.transform.rotation.y = q[1]
    robot_tf.transform.rotation.z = q[2]
    robot_tf.transform.rotation.w = q[3]

def init_pose_prev():
        #Transform the angle into a quaternion for the orientation
    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
    robot_pose_prev.header.frame_id = "odom"
    robot_pose_prev.header.stamp = rospy.Time.now()
    robot_pose_prev.pose.position.x = pos_x
    robot_pose_prev.pose.position.y = pos_y
    robot_pose_prev.pose.position.z = 0.0
    robot_pose_prev.pose.orientation = Quaternion(pos_orient[0], pos_orient[1], pos_orient[2], pos_orient[3])


def init_odom():
    robot_odom.header.stamp = rospy.Time.now()
    robot_odom.header.frame_id = "odom"
    robot_odom.child_frame_id = "base_link"
    robot_odom.pose.pose.position.x = 0.0
    robot_odom.pose.pose.position.y = 0.0
    robot_odom.pose.pose.position.z = 0.0
    robot_odom.pose.pose.orientation.x = 0.0
    robot_odom.pose.pose.orientation.y = 0.0
    robot_odom.pose.pose.orientation.z = 0.0
    robot_odom.pose.pose.orientation.w = 0.0
    robot_odom.pose.covariance = [0]*36
    robot_odom.twist.twist.linear.x = 0.0
    robot_odom.twist.twist.linear.y = 0.0
    robot_odom.twist.twist.linear.z = 0.0
    robot_odom.twist.twist.angular.x = 0.0
    robot_odom.twist.twist.angular.y = 0.0
    robot_odom.twist.twist.angular.z = 0.0
    robot_odom.twist.covariance = [0]*36

#Define the callback functions (if required)
#Pose Callback Function
def pos_callback(msg):
    global robot_pose, new_pose
    robot_pose = msg
    new_pose = True

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
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)

    #Init joints
    init_tf()
    init_pose_prev()
    init_odom()

    #Setup Publishers/Subscribers and Transform Broadcasters
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
    rospy.Subscriber("pose_sim", PoseStamped, pos_callback)
    tf_broadcaster = TransformBroadcaster()

    print("The Localisation is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():
            if new_pose == True:

                Vel_x = (robot_pose.pose.position.x - robot_pose_prev.pose.position.x  )/((robot_pose.header.stamp.to_nsec() - robot_pose_prev.header.stamp.to_nsec())/ 1000000000.0 )
                Vel_y = (robot_pose.pose.position.y - robot_pose_prev.pose.position.y  )/((robot_pose.header.stamp.to_nsec()- robot_pose_prev.header.stamp.to_nsec())/ 1000000000.0 )
                Vel = np.sqrt(np.power(Vel_x,2) + np.power(Vel_y,2))

                q1_inv = [robot_pose_prev.pose.orientation.x, robot_pose_prev.pose.orientation.y, robot_pose_prev.pose.orientation.z, -robot_pose_prev.pose.orientation.w]       
                q2 = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]       
                qr = tf_conversions.transformations.quaternion_multiply(q2,q1_inv)
                
                angles = tf_conversions.transformations.euler_from_quaternion(qr)
                Omega =  angles[2] / ((robot_pose.header.stamp.to_nsec() - robot_pose_prev.header.stamp.to_nsec())/ 1000000000.0 )

                robot_pose_prev = robot_pose

                new_pose = False

            #Fill the messages/tf/joints to be published 
            robot_odom.header.stamp = rospy.Time.now()
            robot_odom.pose.pose = robot_pose.pose
            robot_odom.twist.twist.linear.x = Vel
            robot_odom.twist.twist.angular.z = Omega

            #Define Transformations/Joints
            robot_tf.header.stamp = rospy.Time.now()
            robot_tf.transform.translation.x = robot_odom.pose.pose.position.x
            robot_tf.transform.translation.y = robot_odom.pose.pose.position.y
            robot_tf.transform.translation.z = 0.0
            robot_tf.transform.rotation.x = robot_odom.pose.pose.orientation.x
            robot_tf.transform.rotation.y = robot_odom.pose.pose.orientation.y
            robot_tf.transform.rotation.z = robot_odom.pose.pose.orientation.z
            robot_tf.transform.rotation.w = robot_odom.pose.pose.orientation.w

            #Publish messages
            odom_pub.publish(robot_odom)
            tf_broadcaster.sendTransform(robot_tf)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass
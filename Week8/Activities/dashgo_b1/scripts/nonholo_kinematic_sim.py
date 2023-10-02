#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
import tf_conversions

#Set the parameters of the system
_wheel_r = rospy.get_param("/wheel_radius",0.062)
_robot_l = rospy.get_param("/robot_wheelbase",0.330)
_sample_time = rospy.get_param("/sim_sample_time",0.01)
_node_rate = rospy.get_param("/sim_node_rate",500)

#Set initial conditions of the system
pos_x = rospy.get_param("pos_x0",0.00)
pos_y = rospy.get_param("pos_y0",0.00)
pos_th = rospy.get_param("pos_th0",0.00)

# Setup Variables to be used
first = True
start_time = 0.0
last_time =  0.0
current_time = 0.0

x_dot = 0.0
y_dot = 0.0
omega = 0.0
wheel_wl = 0.0
wheel_wr = 0.0

# Declare Messages to be used
cmd_vel = Twist()
robot_vel = Twist()
pose_sim = PoseStamped()
wr = Float32()
wl = Float32()

#Initialise messages (if required)
def init_wheel_speed():
    wr.data = 0.0
    wl.data = 0.0

def init_pose():
    #Transform the angle into a quaternion for the orientation
    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
    pose_sim.header.frame_id = "world"
    pose_sim.header.stamp = rospy.Time.now()
    pose_sim.pose.position.x = pos_x
    pose_sim.pose.position.y = pos_y
    pose_sim.pose.position.z = 0.0
    pose_sim.pose.orientation = Quaternion(pos_orient[0], pos_orient[1], pos_orient[2], pos_orient[3])

def init_robot_vel():
    #Declare the velocity output Message
    robot_vel.linear.x = 0.0
    robot_vel.linear.y = 0.0
    robot_vel.linear.z = 0.0
    robot_vel.angular.x = 0.0
    robot_vel.angular.y = 0.0
    robot_vel.angular.z = 0.0

def init_cmd_vel():
    # Declare the input Message
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0

    #Define the callback functions (if required)
def cmd_callback(msg):
    global cmd_vel
    cmd_vel = msg

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")
    wr.data = 0.0
    wl.data = 0.0
    wr_pub.publish(wr)
    wl_pub.publish(wl)

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Puzzlebot_sim")
 
    # Configure the Node
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)

    #Init messages to be used
    init_cmd_vel()
    init_wheel_speed()
    init_pose()
    init_robot_vel()

    # Setup the Subscribers
    rospy.Subscriber("cmd_vel", Twist, cmd_callback)

    #Setup de publishers
    wr_pub = rospy.Publisher("wr", Float32, queue_size=1)
    wl_pub = rospy.Publisher("wl", Float32, queue_size=1)
    pose_pub = rospy.Publisher("pose_sim", PoseStamped, queue_size=1)
    vel_pub = rospy.Publisher("robot_vel", Twist, queue_size=1)

    #Node Running
    print("The Robot Simulator is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():
            if first == True:
                start_time = rospy.get_time()
                last_time =  rospy.get_time()
                current_time = rospy.get_time()
                first = False
            else: 
                current_time = rospy.get_time()
                dt = current_time - last_time

                if dt >= _sample_time:

                    #Velocity Estimation
                    x_dot = cmd_vel.linear.x * np.cos(pos_th)
                    y_dot = cmd_vel.linear.x * np.sin(pos_th)
                    omega = cmd_vel.angular.z

                    #Pose estimation
                    pos_x += dt * x_dot
                    pos_y += dt * y_dot
                    pos_th += dt * omega

                    #Wheel speeds
                    wheel_wr = (cmd_vel.linear.x + _robot_l * cmd_vel.angular.z / 2.0) / _wheel_r
                    wheel_wl = (cmd_vel.linear.x - _robot_l * cmd_vel.angular.z / 2.0) / _wheel_r

                    #Fill The messages to be published

                    #Fill the Velocities
                    robot_vel.linear.x = x_dot
                    robot_vel.linear.y = y_dot
                    robot_vel.angular.z = omega

                    #Fill the estimated pose message
                    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
                    pose_sim.header.stamp = rospy.Time.now()
                    pose_sim.pose.position.x = pos_x
                    pose_sim.pose.position.y = pos_y
                    pose_sim.pose.orientation = Quaternion(pos_orient[0], pos_orient[1], pos_orient[2], pos_orient[3])
                    
                    #Fill the wheel speed message
                    wr.data = wheel_wr
                    wl.data = wheel_wl

                    last_time = rospy.get_time()

                    #Publish messages
                    wr_pub.publish(wr)
                    wl_pub.publish(wl)
                    pose_pub.publish(pose_sim)
                    vel_pub.publish(robot_vel)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass
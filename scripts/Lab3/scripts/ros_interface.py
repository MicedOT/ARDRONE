#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import time

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from std_msgs.msg import String, Bool
from position_controller import PositionController


class ROSControllerNode(object):

    #Initialize Ros Controller 
    def __init__(self):
        
        self.pub_vel = rospy.Publisher('/cmd_vel_RHC', Twist, queue_size = 32)
        self.vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', TransformStamped, self.update_vicon)
        self.desired_position_data = rospy.Subscriber('/desired_positions', PoseStamped, self.update_desired_position)
        self.start_stop_command = rospy.Subscriber('/start_stop_toggle', String,self.update_state)

        self.request_pic = rospy.Publisher('/take_pic', String, queue_size = 10)

        self.pub_errors = rospy.Publisher('/errors', PoseStamped, queue_size = 100)
        #self.pub_check = rospy.Publisher('/check_mate', String, queue_size = 100)
        
        self.run_state=False
        
        self.translation_x_old = 0.0
        self.translation_y_old = 0.0
        self.translation_z_old = 0.0

        self.z_velocity_old = 0.0

        self.translation_x_desired = 0.0
        self.translation_y_desired = 0.0
        self.translation_z_desired = 0.0

        self.translation_x = 0.0
        self.translation_y = 0.0
        self.translation_z = 0.0

        self.rotation_x=0.0
        self.rotation_y=0.0
        self.rotation_z=0.0
        self.rotation_w=1.0
        
        self.rotation_x_desired=0.0
        self.rotation_y_desired=0.0
        self.rotation_z_desired=0.0
        self.rotation_w_desired=1.0

        self.time_interval=0
        self.current_time=rospy.get_time()
        self.old_time=rospy.get_time()
        
        self.postcom = PositionController()

           
        # Run the publish commands at 100 Hz
        self.onboard_loop_frequency = 100.
        
        self.pic_taking_speed=1.
        
        # Run this ROS node at the onboard loop frequency
        self.nutjobcase = rospy.Timer(rospy.Duration(1. / self.onboard_loop_frequency), self.run_process) 

        self.wedfsj = rospy.Timer(rospy.Duration(1. / self.pic_taking_speed), self.pic_initiator)      
        
        self.postcom = PositionController()
        
    def update_state(self,engage_command):
        if(engage_command.data=="start"):
            self.run_state=True
        elif(engage_command.data=="stop"):
            self.run_state=False
            

    #Publish Errors
    def publish_errors(self):
        msg = PoseStamped()
    
        msg.header.stamp = rospy.Time.now()
        
        msg.pose.position.x = self.translation_x_desired-self.translation_x
        msg.pose.position.y = self.translation_y_desired-self.translation_y
        msg.pose.position.z = self.translation_z_desired-self.translation_z
        euler_angle=euler_from_quaternion([self.rotation_x,self.rotation_y,self.rotation_z,self.rotation_w])
        angle_yaw= euler_angle[2]
        msg.pose.orientation.x = angle_yaw

        self.pub_errors.publish(msg) 

    #Controlling Process
    def process_commands(self):
        #Create Orientation Quaternion Arrays
        self.rotation=np.array([self.rotation_x,self.rotation_y,self.rotation_z,self.rotation_w])
        self.rotation_desired=np.array([self.rotation_x_desired,self.rotation_y_desired,self.rotation_z_desired,self.rotation_w_desired])
        
        #Calulate Time Interval from previous process 
        current_time=rospy.get_time()
        self.time_interval=current_time-self.old_time
        self.old_time=current_time
        
        #Send Required Values to Position Controller
        self.postcom.update_pos_controller_values(self.translation_x, self.translation_y, self.translation_z, self.rotation, self.translation_x_desired, self.translation_y_desired, self.translation_z_desired, self.rotation_desired, self.time_interval)

        #Recieve Calculation of Commands
        returnvalue= self.postcom.calculate_commands()
        [roll,pitch,yaw,z_dot]=returnvalue

        #Publish Errors
        self.publish_errors()
        #Call function to publish commands
        self.set_vel(roll, pitch, z_dot, yaw)

    
    # Take desired position value
    def update_desired_position(self,desired_position_data):
        #Update Current Position
        self.translation_x_desired = desired_position_data.pose.position.x
        self.translation_y_desired = desired_position_data.pose.position.y
        self.translation_z_desired = desired_position_data.pose.position.z
        #Update Current Orientation
        self.rotation_x_desired = desired_position_data.pose.orientation.x
        self.rotation_y_desired = desired_position_data.pose.orientation.y
        self.rotation_z_desired = desired_position_data.pose.orientation.z       
        self.rotation_w_desired = desired_position_data.pose.orientation.w

    # Obtain value from VICON
    def update_vicon(self,vicon_data_msg):
        #Update Current Position
        self.translation_x = vicon_data_msg.transform.translation.x
        self.translation_y = vicon_data_msg.transform.translation.y
        self.translation_z = vicon_data_msg.transform.translation.z
        #Update Current Orientation
        self.rotation_x = vicon_data_msg.transform.rotation.x
        self.rotation_y = vicon_data_msg.transform.rotation.y
        self.rotation_z = vicon_data_msg.transform.rotation.z
        self.rotation_w = vicon_data_msg.transform.rotation.w

        self.current_time=vicon_data_msg.header.stamp
    
        
    #Publish velocity to /cmd_vel_RHC
    def set_vel(self, roll, pitch, z_dot, yaw_dot):
        msg = Twist()
        msg.linear.x = roll
        msg.linear.y = pitch
        msg.linear.z = z_dot
        msg.angular.z = yaw_dot
        self.pub_vel.publish(msg)
        

    pass


    def run_process(self,random):
        if(self.run_state):
            self.process_commands()

    def pic_initiator(self,random):
        sfsdfjhds="hello"
        self.request_pic.publish(sfsdfjhds)


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_controller')
    ROSControllerNode()
    rospy.spin()
    

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

    def __init__(self):
        """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
        
        self.pub_vel = rospy.Publisher('/cmd_vel_RHC', Twist, queue_size = 32)
        #self.random_test = rospy.Publisher('/random_stuff', Twist, queue_size = 32)
        self.vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', TransformStamped, self.update_vicon)
        #self.start_value = rospy.Subscriber('/start_execution', String, self.run_stuff)
        self.desired_position_data = rospy.Subscriber('/desired_positions', PoseStamped, self.update_desired_position)
        self.pub_errors = rospy.Publisher('/errors', PoseStamped, queue_size = 1000)
        #self.pub_check = rospy.Publisher('/check_mate', String, queue_size = 100)
        self.pub_reached = rospy.Publisher('/check_reach', String, queue_size = 100)
        
        self.pub_counter=0
        
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

           
        # Run the onboard controller at 100 Hz
        self.onboard_loop_frequency = 100.
        
        
        # Run this ROS node at the onboard loop frequency
        self.nutjobcase = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.process_commands)        
        
        self.postcom = PositionController()
        
        """
        new_message=String()
        new_message.data="true"
        self.pub_check.publish(new_message)
        """

       
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

    
    def process_commands(self,random):
        
        self.rotation=np.array([self.rotation_x,self.rotation_y,self.rotation_z,self.rotation_w])
        self.rotation_desired=np.array([self.rotation_x_desired,self.rotation_y_desired,self.rotation_z_desired,self.rotation_w_desired])
        
        current_time=rospy.get_time()
        self.time_interval=current_time-self.old_time
        self.old_time=current_time
        
        #postcom = PositionController(self.translation_x_old,self.translation_y_old,self.translation_z_old,self.translation_x, self.translation_y, self.translation_z, self.rotation, self.translation_x_desired, self.translation_y_desired, self.translation_z_desired, self.rotation_desired, self.z_velocity_old,self.time_interval)
        self.postcom.update_pos_controller_values(self.translation_x, self.translation_y, self.translation_z, self.rotation, self.translation_x_desired, self.translation_y_desired, self.translation_z_desired, self.rotation_desired, self.time_interval)
        returnvalue= self.postcom.member()
        
        """
        roll = returnvalue[0]
        pitch = returnvalue[1]
        yaw = returnvalue[2]
        z_dot = returnvalue[3]
        old_x = returnvalue[4]
        old_y = returnvalue[5]
        old_z = returnvalue[6]
        old_velocity_z = returnvalue[7]
        """
        #[roll,pitch,yaw,z_dot,old_x,old_y,old_z,old_velocity_z]=returnvalue
        [roll,pitch,yaw,z_dot]=returnvalue
        yaw_dot = 0

        self.publish_errors()
        self.set_vel(roll, pitch, z_dot, yaw)

        #self.store_old_values(old_x,old_y,old_z,old_velocity_z)
    
         
        
    
    # Take desired position value
    def update_desired_position(self,desired_position_data):
        self.translation_x_desired = desired_position_data.pose.position.x
        self.translation_y_desired = desired_position_data.pose.position.y
        self.translation_z_desired = desired_position_data.pose.position.z
        self.rotation_x_desired = desired_position_data.pose.orientation.x
        self.rotation_y_desired = desired_position_data.pose.orientation.y
        self.rotation_z_desired = desired_position_data.pose.orientation.z       
        self.rotation_w_desired = desired_position_data.pose.orientation.w

        
        #self.process_commands()
        #self.run_stuff()
        
        test_var=0
        """
        while(test_var==0):
            if (((self.translation_x_desired - 0.10) < self.translation_x < (self.translation_x_desired + 0.10)) and 
        ((self.translation_y_desired - 0.10) < self.translation_y < (self.translation_y_desired + 0.10)) and
        ((self.translation_z_desired - 0.10) < self.translation_z < (self.translation_z_desired + 0.10))):
                self.pub_check.publish("true")
                test_var=1
            else:
                self.pub_check.publish("false")
        """
    
    # Obtain value from VICON
    def update_vicon(self,vicon_data_msg):
        self.translation_x = vicon_data_msg.transform.translation.x
        self.translation_y = vicon_data_msg.transform.translation.y
        self.translation_z = vicon_data_msg.transform.translation.z
           
        self.rotation_x = vicon_data_msg.transform.rotation.x
        self.rotation_y = vicon_data_msg.transform.rotation.y
        self.rotation_z = vicon_data_msg.transform.rotation.z
        self.rotation_w = vicon_data_msg.transform.rotation.w

        self.current_time=vicon_data_msg.header.stamp

    def store_old_values(self,old_x,old_y,old_z,z_velocity_old):
        self.translation_x_old=old_x
        self.translation_y_old=old_y
        self.translation_z_old=old_z

        self.z_velocity_old= z_velocity_old
    
        
    
    def set_vel(self, roll, pitch, z_dot, yaw_dot):
        msg = Twist()
        msg.linear.x = roll
        msg.linear.y = pitch
        msg.linear.z = z_dot
        msg.angular.z = yaw_dot
        self.pub_vel.publish(msg)
        
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    def run_stuff(self):
        
        margin = 2.25
        while(True):
            if (((self.translation_x_desired - margin) < self.translation_x < (self.translation_x_desired + margin)) and((self.translation_y_desired - margin) < self.translation_y < (self.translation_y_desired + margin)) and((self.translation_z_desired - margin) < self.translation_z < (self.translation_z_desired + margin))):
                #self.pub_check.publish("true")
                self.pub_reached.publish("Reached")
                #print("point Reached")
                break
                
        #else:
            #self.process_commands()
                #self.pub_check.publish("false")
            
                
            #time.sleep(5)
            



    pass
    


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_controller')
    ROSControllerNode()
    #obj=ROSControllerNode()
    #obj.run_stuff()
    rospy.spin()
    

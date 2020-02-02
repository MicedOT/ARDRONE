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
        self.start_value = rospy.Subscriber('/start_execution', String, self.run_stuff)
        self.desired_position_data = rospy.Subscriber('/desired_positions', PoseStamped, self.update_desired_position)

        self.pub_check = rospy.Publisher('/check_mate', String, queue_size = 10)
        
        self.translation_x_old = 0.0
        self.translation_y_old = 0.0
        self.translation_z_old = 0.0

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
        
        print("hello")
        
        new_message=String()
        new_message.data="true"
        self.pub_check.publish(new_message)

    #translation_x = 0
    #self.translation_x = 5

    #print (self.translation_x)
    #print (translation_x)
    
    def process_commands(self):

        self.rotation=np.array([self.rotation_x,self.rotation_y,self.rotation_z,self.rotation_w])
        self.rotation_desired=np.array([self.rotation_x_desired,self.rotation_y_desired,self.rotation_z_desired,self.rotation_w_des])
        postcom = PositionController(self.translation_x_old,self.translation_y_old,self.translation_z_old,self.translation_x, self.translation_y, self.translation_z, self.rotation, self.translation_x_desired, self.translation_y_desired, self.translation_z_desired, self.rotation_desired)
        
        
        
        #postcom.translation_x = self.translation_x
        #postcom.translation_y = self.translation_y
        #postcom.translation_z = self.translation_z
        
        # roatation? 
        #postcom.rotation_x = self.rotation_x
        #postcom.rotation_y = self.rotation_y
        #postcom.rotation_z = self.rotation_z
        #postcom.rotation_w = self.rotation_w


    
        returnvalue= postcom.member()
        roll = returnvalue[0]
        pitch = returnvalue[1]
        yaw = returnvalue[2]
        z_dot = returnvalue[3]
        yaw_dot = 0


        self.set_vel(roll, pitch, z_dot, yaw_dot)

        self.store_old_values()
    
     	
	
    
    # Take desired position value
    def update_desired_position(self,desired_position_data):
        self.translation_x_desired = desired_position_data.pose.position.x
        self.translation_y_desired = desired_position_data.pose.position.y
        self.translation_z_desired = desired_position_data.pose.position.z
        self.rotation_x_desired = desired_position_data.pose.orientation.x
        self.rotation_y_desired = desired_position_data.pose.orientation.y
        self.rotation_z_desired = desired_position_data.pose.orientation.z       
        self.rotation_w_des = desired_position_data.pose.orientation.w

        
        
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

    def store_old_values(self):
        self.translation_x_old=self.translation_x
        self.translation_y_old=self.translation_y
        self.translation_z_old=self.translation_z

        
    #
    def set_vel(self, roll, pitch, z_dot, yaw_dot):
        msg = Twist()
        msg.linear.x = roll
        msg.linear.y = pitch
        msg.linear.z = z_dot
        msg.angular.x = yaw_dot
        self.pub_vel.publish(msg)
        
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    def run_stuff(self,incoming):
        while(True):
            if (((self.translation_x_desired - 0.50) < self.translation_x < (self.translation_x_desired + 0.50)) and 
        ((self.translation_y_desired - 0.50) < self.translation_y < (self.translation_y_desired + 0.50)) and
        ((self.translation_z_desired - 0.50) < self.translation_z < (self.translation_z_desired + 0.50))):
                self.pub_check.publish("true")
            else:
                self.pub_check.publish("false")
            
            self.process_commands()
            time.sleep(0.01)
            



    pass
    


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_controller')
    ROSControllerNode()
    rospy.spin()
    

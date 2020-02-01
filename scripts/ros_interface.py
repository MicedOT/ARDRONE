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
        
        self.pub_pos_des = rospy.Subscriber('/desired_positions', PoseStamped, self.pos_des)

        self.pub_check = rospy.Publisher('/check_mate', String, queue_size = 10)
        
        self.x_old = 0.0
        self.y_old = 0.0
        self.z_old = 0.0

        self.x_des = 0.0
        self.y_des = 0.0
        self.z_des = 0.0

        self.x_trans = 0.0
        self.y_trans = 0.0
        self.z_trans = 0.0
        
        print("hello")
        
        new_message=String()
        new_message.data="true"
        self.pub_check.publish(new_message)

    #x_trans = 0
    #self.x_trans = 5

    #print (self.x_trans)
    #print (x_trans)
    
    def unknow(self):

        self.rotat=(self.x_rotat,self.y_rotat,self.z_rotat,self.w_rotat)
        self.des_rotat=(self.x_rotat_des,self.y_rotat_des,self.z_rotat_des,self.w_rotat_des)
        postcom = PositionController(self.x_old,self.y_old,self.z_old,self.x_trans, self.y_trans, self.z_trans, self.rotat, self.x_des, self.y_des, self.z_des, self.des_rotat)
        
        
        
        #postcom.x_trans = self.x_trans
        #postcom.y_trans = self.y_trans
        #postcom.z_trans = self.z_trans
        
        # roatation? 
        #postcom.x_rotat = self.x_rotat
        #postcom.y_rotat = self.y_rotat
        #postcom.z_rotat = self.z_rotat
        #postcom.w_rotat = self.w_rotat


    
        returnvalue= postcom.member()
        roll = returnvalue[0]
        pitch = returnvalue[1]
        yaw = returnvalue[2]
        z_dot = returnvalue[3]
        yaw_dot = 0


        self.set_vel(roll, pitch, z_dot, yaw_dot)

        self.store_old_values()
    
     	
	
    
    # Take desired position value
    def pos_des(self,pub_pos_des):
        self.x_des = pub_pos_des.pose.position.x
        self.y_des = pub_pos_des.pose.position.y
        self.z_des = pub_pos_des.pose.position.z
        self.x_rotat_des = pub_pos_des.pose.orientation.x
        self.y_rotat_des = pub_pos_des.pose.orientation.y
        self.z_rotat_des = pub_pos_des.pose.orientation.z       
        self.w_rotat_des = pub_pos_des.pose.orientation.w

        
        self.unknow()
        self.run_stuff()
        test_var=0
        """
        while(test_var==0):
            if (((self.x_des - 0.10) < self.x_trans < (self.x_des + 0.10)) and 
        ((self.y_des - 0.10) < self.y_trans < (self.y_des + 0.10)) and
        ((self.z_des - 0.10) < self.z_trans < (self.z_des + 0.10))):
                self.pub_check.publish("true")
                test_var=1
            else:
                self.pub_check.publish("false")
        """
    
    # Obtain value from VICON
    def update_vicon(self,vicon_data_msg):
        self.x_trans = vicon_data_msg.transform.translation.x
        self.y_trans = vicon_data_msg.transform.translation.y
        self.z_trans = vicon_data_msg.transform.translation.z
           
        self.x_rotat = vicon_data_msg.transform.rotation.x
        self.y_rotat = vicon_data_msg.transform.rotation.y
        self.z_rotat = vicon_data_msg.transform.rotation.z
        self.w_rotat = vicon_data_msg.transform.rotation.w

    def store_old_values(self):
        self.x_old=self.x_trans
        self.y_old=self.y_trans
        self.z_old=self.z_trans

        
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
    def run_stuff(self):
        
        if (((self.x_des - 0.10) < self.x_trans < (self.x_des + 0.10)) and 
    ((self.y_des - 0.10) < self.y_trans < (self.y_des + 0.10)) and
    ((self.z_des - 0.10) < self.z_trans < (self.z_des + 0.10))):
            self.pub_check.publish("true")
        else:
            self.pub_check.publish("false")
    pass
    

if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_controller')
    obj=ROSControllerNode()
    print("hello")
    obj.run_stuff()
    #obj.pub_check.publish("true")
    rospy.spin()
    

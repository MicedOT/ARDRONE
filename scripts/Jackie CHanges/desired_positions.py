#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator

    def __init__(self):
        self.desired_position_counter = 0
        
        self.pub_pos_des = rospy.Publisher('/desired_positions', PoseStamped, queue_size = 100)
        self.check = rospy.Subscriber('/check_mate', String, self.send)
        self.X = np.linspace(0, 0, num=1)
        self.Y = np.linspace(0, 0, num=1)
        self.Z = np.linspace(0, 0, num=1)

        
        
    #def cord(self):
        number_of_points=100
        self.X = np.linspace(-1, 1, num=number_of_points)
        self.Y = np.linspace(-1, 1, num=number_of_points)
        self.Z = np.linspace(1, 2, num=number_of_points)

    def spiral(self):
        radius=3
        number_of_points=100
        angle = np.linspace(0, 2*np.pi, number_of_points)
        self.X=np.cos(angle)*radius
        self.Y=np.sin(angle)*radius
        self.Z = np.linspace(1, 3, number_of_points)

    def circle(self):
        radius=3
        number_of_points=100
        angle = np.linspace(0, 2*np.pi, number_of_points)
        self.X=np.cos(angle)*radius
        self.Y=np.sin(angle)*radius
        self.Z = np.linspace(2, 2, number_of_points)
        

    def send(self,message):

        msg = PoseStamped()
    
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = self.X[self.desired_position_counter]
        msg.pose.position.y = self.Y[self.desired_position_counter]
        msg.pose.position.z = self.Z[self.desired_position_counter]
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1.0
        #if(message.data=="true"):
        self.desired_position_counter = self.desired_position_counter + 1
        self.pub_pos_des.publish(msg)
        
    pass

if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    
    rospy.spin()

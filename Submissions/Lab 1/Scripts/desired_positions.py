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

    def __init__(self):
        self.desired_position_counter = 0
        self.number_of_points=2000
        self.pub_pos_des = rospy.Publisher('/desired_positions', PoseStamped, queue_size = self.number_of_points)
        self.check = rospy.Subscriber('/check_type', String, self.choose_type)
        #self.check = rospy.Subscriber('/check_mate', String, self.send)
        


        self.X = np.linspace(0, 0, num=self.number_of_points)
        self.Y = np.linspace(0, 0, num=self.number_of_points)
        self.Z = np.linspace(0, 0, num=self.number_of_points)

        # Run the desired positions at 100 Hz
        self.onboard_loop_frequency = 100.
        
        
        # Run this ROS node at the onboard loop frequency
        self.desired_send_stuff = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send)

        self.direction = 1

    #Create  Trajectory    
    def linear_trajectory(self):
        
        X1 = np.linspace(-1, 1, num=self.number_of_points/2)
        Y1 = np.linspace(-1, 1, num=self.number_of_points/2)
        Z1 = np.linspace(1, 2, num=self.number_of_points/2)
        X2 = np.linspace(1, -1, num=self.number_of_points/2)
        Y2 = np.linspace(1, -1, num=self.number_of_points/2)
        Z2 = np.linspace(2, 1, num=self.number_of_points/2)

        self.X=np.concatenate([X1,X2])
        self.Y=np.concatenate([Y1,Y2])
        self.Z=np.concatenate([Z1,Z2])

    #Create Spiral Trajectory 
    def spiral_trajectory(self):
        radius=1
        angle = np.linspace(0, 2*np.pi, (self.number_of_points))
        self.X=np.cos(angle)*radius
        self.Y=np.sin(angle)*radius
        #self.circle()
        Z1 = np.linspace(0.5, 1.5, (self.number_of_points/2))
        Z2= np.linspace(1.5, 0.5, (self.number_of_points/2))
        self.Z=np.concatenate([Z1,Z2])
        
    #Create Circular Trajectory 
    def circle(self):
        radius=1
        angle = np.linspace(0, 2*np.pi, self.number_of_points)
        self.X=np.cos(angle)*radius
        self.Y=np.sin(angle)*radius
        self.Z = np.linspace(2, 2, self.number_of_points)
        
    #Choose Path Type
    def choose_type(self,message):
        if(message.data=="linear"):
            self.linear_trajectory()
        elif(message.data=="spiral"):
            self.spiral_trajectory()

    #Publish Desired Trajectory
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

        self.desired_position_counter = (self.desired_position_counter + 1)%self.number_of_points

        
        self.pub_pos_des.publish(msg) 

    pass

if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    
    rospy.spin()

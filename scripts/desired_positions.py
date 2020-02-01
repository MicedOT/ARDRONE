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

    def __init__(object):
        object.i = 0
        
        object.pub_pos_des = rospy.Publisher('/desired_positions', PoseStamped, queue_size = 32)
        object.check = rospy.Subscriber('/check_mate', String, object.send)
        
    #def cord(object):
        object.X = np.linspace(-5, 5, num=100)
        object.Y = np.linspace(-5, 5, num=100)
        object.Z = np.linspace(2, 5, num=100)
        

    def send(object,message):
        msg = PoseStamped()
        msg.pose.position.x = object.X[object.i]
        msg.pose.position.y = object.Y[object.i]
        msg.pose.position.z = object.Z[object.i]
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1.0
        if(message=="true"):
            object.i = object.i + 1
            object.pub_pos_des.publish(msg)
        
    pass

if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    
    rospy.spin()

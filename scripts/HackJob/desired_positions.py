#!/usr/bin/env python2



from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np

from std_msgs.msg import String

class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""

    def __init__(self):
        """Initialize the ROSDesiredPositionGenerator class."""

        # Publisher
        self.pub_des_pos = rospy.Publisher('des_pos', String, queue_size=10)
        
        # Run the onboard controller at 200 Hz.
        self.onboard_loop_frequency = 200.

        self.current_point = 1
        
        # Run this ROS node at the onboard loop frequency.
        self.run_pub_des_pos = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send_des_pos)

 
        # Linear trajectory



        linear_one = np.array([[-1, -1, 1, 0]])
        linear_two = np.array([[1, 1, 2, 0]])   

        linear_waypoints = 100 #set 10 points along path
        linear_forward = np.ones((linear_waypoints,4))
        linear_back = np.ones((linear_waypoints, 4))

        self.linear_trajectory = np.ones((linear_waypoints, 4))

        for i in range(1, linear_waypoints+1):
        	linear_forward[i-1,:] = ((linear_two-linear_one)/(linear_waypoints+1)*i) + linear_one

        linear_forward = np.concatenate((linear_one, linear_forward, linear_two), axis = 0)
        linear_back = np.flipud(linear_forward)

        self.linear_trajectory = np.concatenate((linear_forward, linear_back), axis = 0)

        #self.linear_trajectory = np.array([[3, 2, 1, 0]])


        # Spiral trajectory
        origin = np.array([[1, 0, 0.5, 0]])
        radius = 1
        # Number of circular waypoints points.
        spiral_waypoints = 100
        # Vertical increment for spiral upwards (z from 0.5 to 1.5)
        height_step = (1.5-0.5)/spiral_waypoints/2
        
        circle_one = np.ones((spiral_waypoints, 4))
        circle_two = np.ones((spiral_waypoints, 4))
        self.spiral_trajectory = np.ones((spiral_waypoints, 4))

        # Calculate waypoints and store in spiral trajectory
        for i in range(1, spiral_waypoints+1):
            angle_segment = i*((2*np.pi)/spiral_waypoints)
            circle_one[i-1,:] = [origin[0,0] + radius*np.cos(angle_segment), 
                                        origin[0,1] + radius*np.sin(angle_segment), 
                                        origin[0,2]+(i-1)*height_step, 0]                                     

        for j in range(1, spiral_waypoints+1):
            circle_two[j-1,:] = [circle_one[j-1,0], circle_one[j-1,1],
                                        circle_one[spiral_waypoints-1,2]-(j-1)*height_step, 0]

        self.spiral_trajectory = np.concatenate((circle_one, circle_two), axis = 0)


        #---------------------------------------------------------------------
        # Circular trajectory
        #---------------------------------------------------------------------
        origin = np.array([[0, 1, 1.5, 0]])
        radius = 1
        # Number of circular waypoints points.
        circular_waypoints = 30
        
        self.circular_trajectory = np.ones((circular_waypoints, 4))

        # Calculate waypoints and store in spiral trajectory
        for i in range(1, circular_waypoints+1):
            angle_segment = i*((2*np.pi)/circular_waypoints)
            self.circular_trajectory[i-1,:] = [origin[0,0] + radius*np.cos(angle_segment), 
                                        origin[0,1] + radius*np.sin(angle_segment), origin[0,2], 0]                                     



    def send_des_pos(self, event):
        """Publish desired trajectory."""
        
        # Flatten desired_trajectory
        msg_array_linear = self.linear_trajectory.flatten()
        msg_array_spiral = self.spiral_trajectory.flatten()
        msg_array_circular = self.circular_trajectory.flatten()


        # Convert array to string
        self.msg_string = ' '.join(map(str, msg_array_linear))
        # Change Trajectory
        # For spiral trajectory, please uncomment the folowing line (optional: comment the above line of code): 
        #self.msg_string = ' '.join(map(str, msg_array_spiral))
        # Publish the desired trajectory msg to to topic des_pos
        self.pub_des_pos.publish(self.msg_string)

    
if __name__ == '__main__':
    """Initializing the desired_positions node and starting an instance of ROSDesiredPositionGenerator."""

    rospy.init_node('desired_positions')
    ROSDesiredPositionGenerator()
    rospy.spin()

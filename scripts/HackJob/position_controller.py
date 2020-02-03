#!/usr/bin/env python2


from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
import csv

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped, PoseArray

from std_msgs.msg import String
import math

class PositionController(object):
    """ROS interface for position controller"""

    def __init__(self):
        """Initialize the PositionController class."""
    
        # Publishers
        self.pub_data_now = rospy.Publisher('data_now', PoseStamped, queue_size=100)
        self.pub_data_des = rospy.Publisher('data_des', PoseStamped, queue_size=100)
        self.pub_yaw_rate_command = rospy.Publisher('yaw_rate_command', PoseStamped, queue_size=200)

        # Initialize message for publishing 
        # data for plotting, not subscribed by any other nodes)
        self.data_now_pos = PoseStamped()
        self.data_des_pos = PoseStamped()
        self.yaw_rate_command=PoseStamped()
         
        # Initialize current position
        self.current_trans_x = 0.
        self.current_trans_y = 0.
        self.current_trans_z = 0.

        # Initialize current quaternion
        self.current_rot_x = 0.
        self.current_rot_y = 0.
        self.current_rot_z = 0.
        self.current_rot_w = 0.

        self.current_yaw = 0

        # Initialize current desired position
        self.desired_x = 0.
        self.desired_y = 0.
        self.desired_z = 0.
        self.desired_yaw = 0.

        # Old variables
        self.x_old = 0.
        self.y_old = 0.
        self.z_old = 0.
        self.climb_rate_old = 0.

        self.x_desired_old = 0.
        self.y_desired_old = 0.
        self.z_desired_old = 0.

        self.x_err = 0.
        self.y_err = 0.
        self.z_err = 0.
        self.yaw_err = 0.


    def update_position_controller(self, dt):

        # Define gravitational constant
        g = 9.8 

        # Define rise time
        #rise_time_z = 0.8
        #rise_time_yaw = 1
        

        # Define controller tunning parameters: damping ratio & natural frequency

        # Define damping ratio
        damping_rt_x = 0.8
        damping_rt_y = 0.8
        
        # Define natural frequency
        w_n_x = 1
        w_n_y = 1


        # Calculate mass-normalized thrust.
        
        x_now = self.current_trans_x
        y_now = self.current_trans_y
        z_now = self.current_trans_z
        climb_rate_now = (z_now - self.z_old) / dt
        climb_accel = (climb_rate_now - self.climb_rate_old) / dt

        quaternion = np.array([self.current_rot_x,
                              self.current_rot_y,
                              self.current_rot_z,
                              self.current_rot_w])

        euler = euler_from_quaternion(quaternion)
        roll_angle = euler[0]
        pitch_angle = euler[1]
        yaw_angle = euler[2]

        self.current_yaw = yaw_angle
    
        # Simplify equation for normalized thrust: f = (z_dot_dot + g) / (np.cos(roll_angle) * np.cos(pitch_angle)) by
        # setting z_dot_dot (climb acceleration) zero since z_dot_dot could be noisy
        f = (0.0 + g) / (math.cos(roll_angle) * math.cos(pitch_angle))

        self.z_old = z_now
        self.climb_rate_old = climb_rate_now


        # Determine desired x and y accelerations
 
        x_rate_now = (x_now - self.x_old)/dt
        y_rate_now = (y_now - self.y_old)/dt

        # compute x and y rate
        desired_x_rate = (self.desired_x - self.x_desired_old)/dt
        desired_y_rate = (self.desired_y - self.y_desired_old)/dt

        # desired_z_rate = 0
   
        # Calculate x and y acceleration using the following equations for zero yaw
        x_accel = 2.0*damping_rt_x*w_n_x*(desired_x_rate - x_rate_now) + math.pow(w_n_x, 2)*(self.desired_x - x_now)
        y_accel = 2.0*damping_rt_y*w_n_y*(desired_y_rate - y_rate_now) + math.pow(w_n_y, 2)*(self.desired_y - y_now)

        self.x_old = x_now
        self.y_old = y_now
        self.x_desired_old = self.desired_x
        self.y_desired_old = self.desired_y


        # Calculate commands

        
        # Roll command
        # Account fo arcsin() angle range [-1,1]
        if -y_accel/f >= 1:
            roll_command = math.asin(1)
        elif -y_accel / f <= -1:
            roll_command = math.asin(-1)
        else:
            roll_command = math.asin(-y_accel/f)

        # Pitch command
        # Account for arcsin() angle range [-1, 1]
        if x_accel/(f*math.cos(roll_command)) >= 1:
            pitch_command = math.asin(1)
        elif x_accel/(f*math.cos(roll_command)) <= -1:
            pitch_command = math.asin(-1)
        else:
            pitch_command = math.asin(x_accel/(f*math.cos(roll_command)))


        # Correct for non-zero yaws
        if yaw_angle != 0:
            roll_command = (roll_command*math.cos(yaw_angle)) + (pitch_command*math.sin(yaw_angle))
            pitch_command = (-roll_command*math.sin(yaw_angle)) + (pitch_command*math.cos(yaw_angle))



        # Climb rate (vertical velocity) command
        tau_z = 0.08  #rise_time_z/2.2 
        climb_rate_command = damp_z*(1.0/tau_z)*(self.desired_z - self.current_trans_z)
        if climb_rate_command >= 2:
            climb_rate_command = 2
        
        # Yaw rate (angular velocity) command
        tau_yaw = 1.0/2.2 #rise_time_yaw/2.2
        yaw_rate_command = (1/tau_yaw)*(self.desired_yaw - yaw_angle)

        # Return roll, pitch, climb rate and yaw rate commands in commands
        commands = np.array([roll_command, pitch_command, yaw_rate_command, climb_rate_command])


        # Publish data for plotting and analysis 
        # (current positions, desired positions, errors)
	
        self.data_now_pos.pose.position.x = x_now
        self.data_now_pos.pose.position.y = y_now
        self.data_now_pos.pose.position.z = z_now
        self.data_now_pos.pose.orientation.z = yaw_angle
        self.data_now_pos.header.stamp = rospy.Time.now()

        self.data_des_pos.pose.position.x = self.desired_x
        self.data_des_pos.pose.position.y = self.desired_y
        self.data_des_pos.pose.position.z = self.desired_z
        self.data_des_pos.header.stamp = rospy.Time.now()


        self.pub_data_now.publish(self.data_now_pos)
        self.pub_data_des.publish(self.data_des_pos)
        self.pub_data_des.publish(self.yaw_rate_command)

        return commands

#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist


class PositionController(object):

    

    def __init__(self, x_old, y_old, z_old, x_trans, y_trans, z_trans, w, x_d, y_d, z_d, w_d):
        self.a = 0
        self.g = 9.81
        self.angle_yaw = 0
        self.t = 1

        self.C_x = 1
        self.w_nx = 1
        self.C_y = 1
        self.w_ny = 1
        self.x_old = x_old
        self.y_old = y_old
        self.z_old = z_old

        #self.z_velocity_old = z_velocity_old

        #self.x = a
        #self.y = a
        euler_angle = euler_from_quaternion(w)
        self.angle_roll = euler_angle[0]
        self.angle_pitch = euler_angle[1]
        self.angle_yaw = euler_angle[2]

        self.x_trans = x_trans
        self.y_trans = y_trans
        self.z_trans = z_trans
        #self.w = w
        
        self.x_d = x_d
        self.y_d = y_d
        self.z_d = z_d
        self.w_d = w_d      
        

    def member(self):

        x_velocity = (self.x_trans-self.x_old)/self.t
        y_velocity = (self.y_trans-self.y_old)/self.t
        z_velocity = (self.z_trans-self.z_old)/self.t
        

        # Desired velocity
        x_velocity_d = (self.x_d-self.x_trans)/self.t
        y_velocity_d = (self.y_d-self.y_trans)/self.t	
        z_velocity_d = (self.z_d-self.z_trans)/self.t

        z_acceleration =  (z_velocity_d-z_velocity)/self.t
        f = ((z_acceleration)+self.g) / (np.cos(self.angle_pitch)*np.cos(self.angle_roll))
        
        # Command Acceleration
        x_acceleration_command = 2*self.C_x*self.w_nx*(x_velocity_d - x_velocity) + np.power(self.w_nx,2) * (self.x_d-self.x_trans)
        y_acceleration_command = 2*self.C_y*self.w_ny*(y_velocity_d - y_velocity) + np.power(self.w_ny,2) * (self.y_d-self.y_trans)
        #z_acceleration_command = 2*C_z*w_nz(z_velocity_d - z_velocity) + w_nz^2 * (z_d-z)

        # Command Angle
        roll_command_rt = -y_acceleration_command / f

        if roll_command_rt >= 1.:
            roll_command_rt = 1.
        if roll_command_rt <= -1.:
            roll_command_rt = -1.
        roll_c = np.arcsin(roll_command_rt)

        pitch_command_rt = x_acceleration_command/(f*np.cos(roll_c))

        if pitch_command_rt >= 1.:
            pitch_command_rt = 1.
        if pitch_command_rt <= -1.:
            pitch_command_rt = -1.

        pitch_c = np.arcsin(pitch_command_rt)

        # Command Angle - inertial frame
        roll_cB = roll_c*np.cos(self.angle_yaw) + pitch_c*np.sin(self.angle_yaw)
        pitch_cB = -roll_c*np.sin(self.angle_yaw) + roll_c*(np.cos(self.angle_yaw))
        yaw_cB = 0

        #self.x_old=self.x
        #self.y_old=self.y
        #self.z_old=self.z
        
        #self.z_velocity_old=self.z_velocity 
	
        list = [roll_cB, pitch_cB, yaw_cB, z_velocity_d]

        return list


















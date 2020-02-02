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

    

    def __init__(self, position_controller_x_translationlation_old, position_controller_y_translationlation_old, position_controller_z_translationlation_old, position_controller_x_translation, position_controller_y_translation, position_controller_z_translation, rotat, position_controller_x_translation_desired, position_controller_y_translation_desired, position_controller_z_translation_desired, position_controller_rotation_desired):
        
        self.g = 9.81
        self.angle_yaw = 0
        self.t = 5

        self.damping_x = 0.8
        self.natural_frequency_x = 1
        self.damping_y = 0.8
        self.natural_frequency_y = 1
        self.position_controller_x_translationlation_old = position_controller_x_translationlation_old
        self.position_controller_y_translationlation_old = position_controller_y_translationlation_old
        self.position_controller_z_translationlation_old = position_controller_z_translationlation_old

        #self.z_velocitposition_controller_y_translationlation_old = z_velocitposition_controller_y_translationlation_old

        #self.x = a
        #self.y = a
        euler_angle = euler_from_quaternion(rotat)
        self.angle_roll = euler_angle[0]
        self.angle_pitch = euler_angle[1]
        self.angle_yaw = euler_angle[2]

        self.position_controller_x_translation = position_controller_x_translation
        self.position_controller_y_translation = position_controller_y_translation
        self.position_controller_z_translation = position_controller_z_translation
        #self.w = w
        
        self.position_controller_x_translation_desired = position_controller_x_translation_desired
        self.position_controller_y_translation_desired = position_controller_y_translation_desired
        self.position_controller_z_translation_desired = position_controller_z_translation_desired
        self.position_controller_rotation_desired = position_controller_rotation_desired      
        

    def member(self):

        x_velocity = (self.position_controller_x_translation-self.position_controller_x_translationlation_old)/self.t
        y_velocity = (self.position_controller_y_translation-self.position_controller_y_translationlation_old)/self.t
        z_velocity = (self.position_controller_z_translation-self.position_controller_z_translationlation_old)/self.t
        

        # Desired velocity
        velocity_x_desired = (self.position_controller_x_translation_desired-self.position_controller_x_translation)/self.t
        velocity_y_desired = (self.position_controller_y_translation_desired-self.position_controller_y_translation)/self.t	
        velocity_z_desired = (self.position_controller_z_translation_desired-self.position_controller_z_translation)/self.t
        
        #z_acceleration =  (velocity_z_desired-z_velocity)/self.t
        z_acceleration=0
        f = ((z_acceleration)+self.g) / (np.cos(self.angle_pitch)*np.cos(self.angle_roll))
        
        # Command Acceleration
        x_acceleration_command = 2*self.damping_x*self.natural_frequency_x*(velocity_x_desired - x_velocity) + np.power(self.natural_frequency_x,2) * (self.position_controller_x_translation_desired-self.position_controller_x_translation)
        y_acceleration_command = 2*self.damping_y*self.natural_frequency_y*(velocity_y_desired - y_velocity) + np.power(self.natural_frequency_y,2) * (self.position_controller_y_translation_desired-self.position_controller_y_translation)
        #z_acceleration_command = 2*C_z*w_nz(velocity_z_desired - z_velocity) + w_nz^2 * (position_controller_z_translation_desired-z)

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

        #self.position_controller_x_translationlation_old=self.x
        #self.position_controller_y_translationlation_old=self.y
        #self.position_controller_z_translationlation_old=self.z
        
        #self.z_velocitposition_controller_y_translationlation_old=self.z_velocity 
	print("Pass Complete")
        list = [roll_cB, pitch_cB, yaw_cB, velocity_z_desired]

        return list


















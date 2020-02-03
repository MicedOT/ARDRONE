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

    

    def __init__(self):
        
        self.g = 9.81
        self.angle_yaw = 0
        
        #print(self.t)

        self.damping_x = 0.1
        self.natural_frequency_x = 0.1
        self.damping_y = 0.1
        self.natural_frequency_y = 0.1

        self.position_controller_x_translation_old = 0
        self.position_controller_y_translation_old = 0
        self.position_controller_z_translation_old = 0

        self.position_controller_z_velocity_old = 0

    def update_pos_controller_values(self, position_controller_x_translation, position_controller_y_translation, position_controller_z_translation, rotat, position_controller_x_translation_desired, position_controller_y_translation_desired, position_controller_z_translation_desired, position_controller_rotation_desired,position_controller_time_interval):
        

        self.t = position_controller_time_interval


        #self.z_velocitposition_controller_y_translation_old = z_velocitposition_controller_y_translation_old

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

        #print(self.position_controller_x_translation," ",self.position_controller_y_translation," ",self.position_controller_z_translation," ",self.position_controller_x_translation_old," ",self.position_controller_y_translation_old," ",self.position_controller_z_translation_old," ",self.position_controller_x_translation_desired," ",self.position_controller_y_translation_desired," ",self.position_controller_z_translation_desired)
        x_velocity = (self.position_controller_x_translation-self.position_controller_x_translation_old)/self.t
        y_velocity = (self.position_controller_y_translation-self.position_controller_y_translation_old)/self.t
        z_velocity = (self.position_controller_z_translation-self.position_controller_z_translation_old)/self.t
        
    #print(self.angle_pitch," ",self.angle_roll," ",self.angle_yaw)

        # Desired velocity
        velocity_x_desired = (self.position_controller_x_translation_desired-self.position_controller_x_translation)/self.t
        velocity_y_desired = (self.position_controller_y_translation_desired-self.position_controller_y_translation)/self.t    
        velocity_z_desired = (self.position_controller_z_translation_desired-self.position_controller_z_translation)/self.t
        
        #z_acceleration =  (z_velocity-self.position_controller_z_velocity_old)/self.t
        z_acceleration=0
        f = ((z_acceleration)+self.g) / (np.cos(self.angle_pitch)*np.cos(self.angle_roll))
        
        # Command Acceleration
        x_acceleration_command = 2*self.damping_x*self.natural_frequency_x*(velocity_x_desired - x_velocity) + np.power(self.natural_frequency_x,2) * (self.position_controller_x_translation_desired-self.position_controller_x_translation)
        y_acceleration_command = 2*self.damping_y*self.natural_frequency_y*(velocity_y_desired - y_velocity) + np.power(self.natural_frequency_y,2) * (self.position_controller_y_translation_desired-self.position_controller_y_translation)
        #z_acceleration_command = 2*C_z*w_nz(velocity_z_desired - z_velocity) + w_nz^2 * (position_controller_z_translation_desired-z)

        # Command Angle

        # Roll command
        roll_command_rt = (-1*y_acceleration_command) / f

        if roll_command_rt >= 1.0:
            roll_command_rt = 1.0
        if roll_command_rt <= -1.0:
            roll_command_rt = -1.0
        roll_c = np.arcsin(roll_command_rt)


        # Pitch command
        pitch_command_rt = x_acceleration_command/(f*np.cos(roll_c))

        if pitch_command_rt >= 1.0:
            pitch_command_rt = 1.0
        if pitch_command_rt <= -1.0:
            pitch_command_rt = -1.0

        pitch_c = np.arcsin(pitch_command_rt)

        # Command Angle - inertial frame
        #roll_cB = roll_c*np.cos(self.angle_yaw) + pitch_c*np.sin(self.angle_yaw)
        #pitch_cB = -roll_c*np.sin(self.angle_yaw) + roll_c*(np.cos(self.angle_yaw))
        yaw_c = 0


        # Correct for non-zero yaws
        if self.angle_yaw!= 0:
            roll_cB = (roll_c*np.cos(self.angle_yaw)) + (pitch_c*np.sin(self.angle_yaw))
            pitch_cB = (-roll_c*np.sin(self.angle_yaw)) + (pitch_c*np.cos(self.angle_yaw))
            roll_c=roll_cB
            pitch_c=pitch_cB
            

        self.position_controller_x_translation_old = self.position_controller_x_translation
        self.position_controller_y_translation_old = self.position_controller_y_translation
        self.position_controller_z_translation_old = self.position_controller_z_translation
        
        self.position_controller_z_velocity_old = z_velocity
        #print("Pass Complete")
        """
        list = [roll_c, pitch_c, yaw_c, velocity_z_desired, self.position_controller_x_translation, self.position_controller_y_translation, self.position_controller_z_translation, z_velocity]
        """
        command_data = np.array([roll_c, pitch_c, yaw_c, velocity_z_desired, self.position_controller_x_translation, self.position_controller_y_translation, self.position_controller_z_translation, z_velocity])

        return command_data


















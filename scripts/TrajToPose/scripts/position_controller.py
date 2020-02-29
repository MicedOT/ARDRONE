#!/usr/bin/env python2


from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist


class PositionController(object):

    #Initialize Position Controller Variables
    def __init__(self):
        
        self.g = 9.81
        self.angle_yaw = 0
        
        #print(self.t)
        #Set Damping Ratios and Natural Frequencies
        self.damping_x = 0.7
        self.natural_frequency_x = 0.8
        self.damping_y = 0.7
        self.natural_frequency_y = 0.8
        self.damping_z = 0.95
        self.natural_frequency_z = 0.9

        self.damping_yaw=2
        self.natural_frequency_yaw = 3

        self.yaw_rate_const=200

        self.position_controller_x_translation_old = 0
        self.position_controller_y_translation_old = 0
        self.position_controller_z_translation_old = 0

        self.angle_yaw_old = 0

        self.position_controller_x_translation_desired_old = 0
        self.position_controller_y_translation_desired_old = 0
        self.position_controller_z_translation_desired_old = 0

        self.angle_yaw_desired_old = 0

        self.position_controller_z_velocity_old = 0

    #Update Values for Calculation Variables
    def update_pos_controller_values(self, position_controller_x_translation, position_controller_y_translation, position_controller_z_translation, rotat, position_controller_x_translation_desired, position_controller_y_translation_desired, position_controller_z_translation_desired, position_controller_rotation_desired,position_controller_time_interval):
        
        #Assign Received Values
        self.t = position_controller_time_interval

        self.position_controller_x_translation = position_controller_x_translation
        self.position_controller_y_translation = position_controller_y_translation
        self.position_controller_z_translation = position_controller_z_translation
        
        self.position_controller_x_translation_desired = position_controller_x_translation_desired
        self.position_controller_y_translation_desired = position_controller_y_translation_desired
        self.position_controller_z_translation_desired = position_controller_z_translation_desired
        self.position_controller_rotation_desired = position_controller_rotation_desired 


        #Calculate Angles from Quaternion
        euler_angle = euler_from_quaternion(rotat)
        self.angle_roll = euler_angle[0]
        self.angle_pitch = euler_angle[1]
        self.angle_yaw = euler_angle[2]

        #Calculate Desired Angles from Quaternion
        euler_angle = euler_from_quaternion(position_controller_rotation_desired)
        self.angle_roll_desired = euler_angle[0]
        self.angle_pitch_desired = euler_angle[1]
        self.angle_yaw_desired = euler_angle[2]
    
             
        
    #Calculate Commands for On Board Controller
    def calculate_commands(self):

        #Calculate 1st Derivatives of Current Positions
        x_velocity = (self.position_controller_x_translation-self.position_controller_x_translation_old)/self.t
        y_velocity = (self.position_controller_y_translation-self.position_controller_y_translation_old)/self.t
        z_velocity = (self.position_controller_z_translation-self.position_controller_z_translation_old)/self.t

        #Test Check Values
        #print(self.angle_pitch," ",self.angle_roll," ",self.angle_yaw)

        
        # Desired velocity
        """
        velocity_x_desired = (self.position_controller_x_translation_desired-self.position_controller_x_translation)/self.t
        velocity_y_desired = (self.position_controller_y_translation_desired-self.position_controller_y_translation)/self.t    
        velocity_z_desired = (self.position_controller_z_translation_desired-self.position_controller_z_translation)/self.t
        """
        velocity_x_desired = (self.position_controller_x_translation_desired-self.position_controller_x_translation_desired_old)/self.t
        velocity_y_desired = (self.position_controller_y_translation_desired-self.position_controller_y_translation_desired_old)/self.t    
        velocity_z_desired = (self.position_controller_z_translation_desired-self.position_controller_z_translation_desired_old)/self.t
        
        #Yaw Correction (Based on the Quadrant for Direction of spin)
        
        #Making all angles between 0 to 2*pi        
        std_angle_yaw=self.angle_yaw%(2*np.pi)
        std_angle_yaw_old=self.angle_yaw_old%(2*np.pi)

        #Calculate Difference
        std_diff_yaw=std_angle_yaw-std_angle_yaw_old

        if(np.absolute(std_diff_yaw)>np.pi):
            std_diff_yaw=-1.0*np.sign(std_diff_yaw)*((2*np.pi)-np.absolute(std_diff_yaw))

        #Making all angles between 0 to 2*pi        
        std_angle_yaw_desired_old=self.angle_yaw_desired_old%(2*np.pi)
        std_angle_yaw_desired=self.angle_yaw_desired%(2*np.pi)

        #Calculate Difference
        std_diff_yaw_desired=std_angle_yaw_desired-std_angle_yaw_desired_old

        if(np.absolute(std_diff_yaw_desired)>np.pi):
            std_diff_yaw_desired=-1.0*np.sign(std_diff_yaw_desired)*((2*np.pi)-np.absolute(std_diff_yaw_desired))
        
        yaw_velocity=std_diff_yaw/self.t
        velocity_yaw_desired=std_diff_yaw_desired/self.t

        #Calculate Difference
        std_diff_yaw_desired_now=std_angle_yaw_desired-std_angle_yaw
        if(np.absolute(std_diff_yaw_desired_now)>np.pi):
            std_diff_yaw_desired_now=-1.0*np.sign(std_diff_yaw_desired_now)*((2*np.pi)-np.absolute(std_diff_yaw_desired_now))

        #Calculate Mass Normalized Thrust
        z_acceleration =  (z_velocity-self.position_controller_z_velocity_old)/self.t
        #z_acceleration=0
        f = ((z_acceleration)+self.g) / (np.cos(self.angle_pitch)*np.cos(self.angle_roll))

        # Command Acceleration
        x_acceleration_command = 2*self.damping_x*self.natural_frequency_x*(velocity_x_desired - x_velocity) + np.power(self.natural_frequency_x,2) * (self.position_controller_x_translation_desired-self.position_controller_x_translation)
        y_acceleration_command = 2*self.damping_y*self.natural_frequency_y*(velocity_y_desired - y_velocity) + np.power(self.natural_frequency_y,2) * (self.position_controller_y_translation_desired-self.position_controller_y_translation)
        z_acceleration_command = 2*self.damping_z*self.natural_frequency_z*(velocity_z_desired - z_velocity) +np.power(self.natural_frequency_z,2) * (self.position_controller_z_translation_desired-self.position_controller_z_translation)

        yaw_rate_command=2*self.damping_yaw*self.natural_frequency_yaw*(velocity_yaw_desired - yaw_velocity) + np.power(self.natural_frequency_yaw,2) * (std_diff_yaw)
        
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
        # Correct for non-zero yaws
        #if self.angle_yaw!= 0:
        roll_cB = (roll_c*np.cos(self.angle_yaw)) + (pitch_c*np.sin(self.angle_yaw))
        pitch_cB = (-roll_c*np.sin(self.angle_yaw)) + (pitch_c*np.cos(self.angle_yaw))
        yaw_c = self.yaw_rate_const*(std_diff_yaw_desired_now)
        #roll_c=roll_cB
        #pitch_c=pitch_cB
            
        #Save Reference Pose, Old Pose and Velocity
        self.position_controller_x_translation_old = self.position_controller_x_translation
        self.position_controller_y_translation_old = self.position_controller_y_translation
        self.position_controller_z_translation_old = self.position_controller_z_translation

        self.angle_yaw_old=self.angle_yaw

        self.position_controller_x_translation_desired_old = self.position_controller_x_translation_desired
        self.position_controller_y_translation_desired_old = self.position_controller_y_translation_desired
        self.position_controller_z_translation_desired_old = self.position_controller_z_translation_desired

        self.angle_yaw_desired_old=self.angle_yaw_desired
        
        self.position_controller_z_velocity_old = z_velocity

        #Return Command Data
        command_data = np.array([roll_cB, pitch_cB, yaw_c, z_acceleration_command])

        return command_data


















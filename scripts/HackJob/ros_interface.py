#!/usr/bin/env python2


from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np

import rosbag

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from position_controller import PositionController

from std_msgs.msg import String

class ROSControllerNode(object):
    """ROS interface for controlling ARDrone, using position controller"""

    def __init__(self):
        """Initialize the ROSControllerNode class."""
        self.model_name = 'ARDroneCarre'

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('cmd_vel_RHC', 
                                            Twist,
                                            queue_size = 300)

        # Subscribers
        self.sub_vicon_data = rospy.Subscriber('/vicon/{0}/{0}'.format(
                                              self.model_name),
                                              TransformStamped, self.process_vicon_data)

        self.sub_des_pos = rospy.Subscriber('des_pos', String, self.process_desired_position)

        # Initialize messages for publishing
        self.cmd_vel = Twist()

        # Run the onboard controller at 200 Hz
        self.onboard_loop_frequency = 200.
        
        # Calling the position controller to pass the data
        self.pos_class = PositionController()

        # Run this ROS node at the onboard loop frequency
        self.run_pub_cmd_vel = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.process_commands)

        # Initialize time
        self.nonunix_time = 0
        self.old_time = rospy.get_time()
        self.start_time = rospy.get_time()
        self.dt = 0

        # Initialize points
        self.current_point = 1
        self.num_points_old = 0

    def process_commands(self, event):
        """Determine the motor speeds and and publishes these."""
        
        # Calculate time step
        current_time = rospy.get_time()
        self.dt = current_time - self.old_time
        
        # Get commands from the position controller
        commands = self.pos_class.update_position_controller(self.dt)
        [roll_command, pitch_command, yaw_rate_command, climb_rate_command] = commands
        
        # Set cmd_vel message values to commands
        self.cmd_vel.linear.x = roll_command
        self.cmd_vel.linear.y = pitch_command
        self.cmd_vel.linear.z = climb_rate_command
        self.cmd_vel.angular.z = yaw_rate_command
        
        # Publish commands
        self.pub_cmd_vel.publish(self.cmd_vel)
        
        # Update timestamp
        self.old_time = current_time


    def process_vicon_data(self, vicon_data_msg):

        self.pos_class.current_trans_x = vicon_data_msg.transform.translation.x
        self.pos_class.current_trans_y = vicon_data_msg.transform.translation.y
        self.pos_class.current_trans_z = vicon_data_msg.transform.translation.z

        self.pos_class.current_rot_x = vicon_data_msg.transform.rotation.x
        self.pos_class.current_rot_y = vicon_data_msg.transform.rotation.y
        self.pos_class.current_rot_z = vicon_data_msg.transform.rotation.z
        self.pos_class.current_rot_w = vicon_data_msg.transform.rotation.w


    def process_desired_position(self, pos_msg):
        
        # Get message containing desired trajectory coordinates from desired_positions
        self.des_pos_msg = np.fromstring(pos_msg.data, dtype = float, sep = ' ')
        
        # Calculate total number of waypoints
        num_points = self.des_pos_msg.size/4

        # Update number of waypoints
        if num_points != self.num_points_old:
            self.num_points_old = num_points
            self.current_point = 0
        
        # Convert 1x12 array into 3 arrays of 4 elements (x,y,z)
        trajectory = np.reshape(self.des_pos_msg, (-1, 4))

        self.pos_class.desired_x = trajectory[self.current_point - 1, 0]    # x coordinates
        self.pos_class.desired_y = trajectory[self.current_point - 1, 1]    # y coordinates
        self.pos_class.desired_z = trajectory[self.current_point - 1, 2]    # z coordinates

        self.nonunix_time += self.dt
        # setting deviation tolerance, the higher tolerance, the faster the drone navigates
        if (((self.pos_class.desired_x - 0.5) < self.pos_class.current_trans_x < (self.pos_class.desired_x + 0.5)) and
            ((self.pos_class.desired_y - 0.5) < self.pos_class.current_trans_y < (self.pos_class.desired_y + 0.5)) and
            ((self.pos_class.desired_z - 0.5) < self.pos_class.current_trans_z < (self.pos_class.desired_z + 0.5))):
                if self.current_point < num_points and self.nonunix_time >= 0.01:
                    self.current_point += 1
                    self.nonunix_time = 0

if __name__ == '__main__':
    rospy.init_node('ros_controller')
    ROSControllerNode()
    rospy.spin()


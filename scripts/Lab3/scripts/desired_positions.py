#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""

    def __init__(self):
        self.desired_position_counter = 0
        self.number_of_points=2400
        self.pub_pos_des = rospy.Publisher('/desired_positions', PoseStamped, queue_size = self.number_of_points)
        self.check = rospy.Subscriber('/check_type', String, self.choose_type)
        #self.check = rospy.Subscriber('/check_mate', String, self.send)
        


        self.X = np.linspace(0, 0, num=self.number_of_points)
        self.Y = np.linspace(0, 0, num=self.number_of_points)
        self.Z = np.linspace(0, 0, num=self.number_of_points)

        self.X_euler = np.linspace(0, 0 , self.number_of_points)
        self.Y_euler = np.linspace(0, 0 , self.number_of_points)
        self.Z_euler = np.linspace(0 ,0 , self.number_of_points)

        # Run the desired positions at 100 Hz
        self.onboard_loop_frequency = 10.
        
        
        # Run this ROS node at the onboard loop frequency
        self.desired_send_stuff = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send)

        self.direction = 1


    #Create  Trajectory
    """    
    def linear_trajectory(self):
        
        X1 = np.linspace(-1, 1, num=self.number_of_points/2)
        Y1 = np.linspace(2, 0, num=self.number_of_points/2)
        Z1 = np.linspace(1, 2, num=self.number_of_points/2)
        X2 = np.linspace(1, -1, num=self.number_of_points/2)
        Y2 = np.linspace(0, 2, num=self.number_of_points/2)
        Z2 = np.linspace(2, 1, num=self.number_of_points/2)
        
        X1 = np.linspace(0, 0, num=self.number_of_points/2)
        Y1 = np.linspace(0, 0, num=self.number_of_points/2)
        Z1 = np.linspace(1, 6, num=self.number_of_points/2)
        X2 = np.linspace(0, 0, num=self.number_of_points/2)
        Y2 = np.linspace(0, 0, num=self.number_of_points/2)
        Z2 = np.linspace(6, 1, num=self.number_of_points/2)
        
        self.X=np.concatenate([X1,X2])
        self.Y=np.concatenate([Y1,Y2])
        self.Z=np.concatenate([Z1,Z2])

        self.X_euler = np.linspace(0, 0 , self.number_of_points)
        self.Y_euler = np.linspace(0, 0 , self.number_of_points)
        self.Z_euler = np.linspace(0 ,0 , self.number_of_points)
     """
    
    def linear_trajectory(self):
        point_short=int(self.number_of_points/24)
        point_long=int(self.number_of_points/6)

        y1 = np.linspace(-2,2,num=point_long)
        y2 = np.ones(point_short)*2
        self.Y=np.concatenate([y1,y2])
        y3 = np.linspace(2,-2,num=point_long)
        self.Y=np.concatenate([self.Y,y3])
        y4 = np.ones(point_short)*-2
        self.Y=np.concatenate([self.Y,y4])
        y5 = np.linspace(-2,2,num=point_long)
        self.Y=np.concatenate([self.Y,y5])
        y6 = np.ones(point_short)*2
        self.Y=np.concatenate([self.Y,y6])
        y7 = np.linspace(2,-2,num=point_long)
        self.Y=np.concatenate([self.Y,y7])
        y8 = np.ones(point_short)*-2
        self.Y=np.concatenate([self.Y,y8])
        y9 = np.linspace(-2,2,num=point_long)
        self.Y=np.concatenate([self.Y,y9])

        x1 = np.ones(point_long)*-2
        x2 = np.linspace(-2,-1,num=point_short)
        self.X=np.concatenate([x1,x2])
        x3 = np.ones(point_long)*-1
        self.X=np.concatenate([self.X,x3])
        x4 = np.linspace(-1,0,num=point_short)
        self.X=np.concatenate([self.X,x4])
        x5 = np.ones(point_long)*0
        self.X=np.concatenate([self.X,x5])
        x6 = np.linspace(0,1,num=point_short)
        self.X=np.concatenate([self.X,x6])
        x7 = np.ones(point_long)*1
        self.X=np.concatenate([self.X,x7])
        x8 = np.linspace(1,2,num=point_short)
        self.X=np.concatenate([self.X,x8])
        x9 = np.ones(point_long)*2
        self.X=np.concatenate([self.X,x9])
 
        self.Z = np.ones(self.number_of_points)*1.3

        self.X_euler = np.linspace(0, 0 , self.number_of_points)
        self.Y_euler = np.linspace(0, 0 , self.number_of_points)
        self.Z_euler = np.linspace(0 ,0 , self.number_of_points)


    #Create Spiral Trajectory 
    def spiral_trajectory(self):

        self.circle_trajectory()
        Z1 = np.linspace(0.5, 1.5, (self.number_of_points/2))
        Z2= np.linspace(1.5, 0.5, (self.number_of_points/2))
        self.Z=np.concatenate([Z1,Z2])

        
    #Create Circular Trajectory 
    def circle_trajectory(self):
        
        radius=1
        origin=[0,1,1.5]
        x_offset=np.linspace(origin[0],origin[0],self.number_of_points)
        y_offset=np.linspace(origin[1],origin[1],self.number_of_points)
        angle = np.linspace(0, 2*np.pi, self.number_of_points)
        self.X=np.cos(angle)*radius+x_offset
        self.Y=np.sin(angle)*radius+y_offset
        self.Z = np.linspace(origin[2], origin[2], self.number_of_points)
        self.X_euler = np.linspace(0, 0, self.number_of_points)
        self.Y_euler = np.linspace(0, 0, self.number_of_points)
        #self.Z_euler = np.linspace(-np.pi/2,(3.0/2.0)*np.pi , self.number_of_points)
        start_ang=-np.pi
        end_ang=np.pi
        self.Z_euler = np.linspace(start_ang, end_ang, self.number_of_points)


        
    #Choose Path Type
    def choose_type(self,message):
        if(message.data=="linear"):
            self.linear_trajectory()
        elif(message.data=="spiral"):
            self.spiral_trajectory()
        if(message.data=="circle"):
            self.circle_trajectory()

        

    #Publish Desired Trajectory
    def send(self,message):
        msg = PoseStamped()
    
        msg.header.stamp = rospy.Time.now()
        
        msg.pose.position.x = self.X[self.desired_position_counter]
        msg.pose.position.y = self.Y[self.desired_position_counter]
        msg.pose.position.z = self.Z[self.desired_position_counter]
        
        [X_quaternion,Y_quaternion,Z_quaternion,W_quaternion]=quaternion_from_euler(self.X_euler[self.desired_position_counter],self.Y_euler[self.desired_position_counter],self.Z_euler[self.desired_position_counter])
        msg.pose.orientation.x = X_quaternion
        msg.pose.orientation.y = Y_quaternion
        msg.pose.orientation.z = Z_quaternion
        msg.pose.orientation.w = W_quaternion
        self.desired_position_counter = (self.desired_position_counter + 1)%self.number_of_points

        
        self.pub_pos_des.publish(msg) 


    pass

if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    
    rospy.spin()

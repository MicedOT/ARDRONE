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
from rrt import RRT


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""

    def __init__(self):
        self.desired_position_counter = 0
        self.number_of_points=2400
        self.pub_pos_des = rospy.Publisher('/desired_positions', PoseStamped, queue_size = self.number_of_points)
        self.check = rospy.Subscriber('/check_type', String, self.choose_type)
        #self.check = rospy.Subscriber('/check_mate', String, self.send)
        self.check = rospy.Subscriber('/ask_land', String, self.make_land)

        # Final Entry Commands
        self.pubLanddes    = rospy.Publisher('/ardrone/land',Empty)
        self.pubToggledes   = rospy.Publisher('/start_stop_toggle',String,queue_size = 5)

        self.X = np.linspace(0, 0, num=self.number_of_points)
        self.Y = np.linspace(0, 0, num=self.number_of_points)
        self.Z = np.linspace(0, 0, num=self.number_of_points)

        self.X_euler = np.linspace(0, 0 , self.number_of_points)
        self.Y_euler = np.linspace(0, 0 , self.number_of_points)
        self.Z_euler = np.linspace(0 ,0 , self.number_of_points)

        # Run the desired positions at 100 Hz
        self.onboard_loop_frequency = 100.
        
        
        # Run this ROS node at the onboard loop frequency
        self.desired_send_stuff = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send)

        self.direction = 1


        # Parameters
        self.KP = 5.0  # attractive potential gain
        #self.ETA = 100.0  # repulsive potential gain
        self.ETA = [1800.0,1800.0,1800.0,960.0]
        self.AREA_WIDTH = 10.0  # potential area width [m]

        self.show_animation = False

        #self.order_pic=[3,2,1,4]
        #self.order_pic=[1,2,3,4]
        x=np.loadtxt('/home/mason/aer1217/labs/src/aer1217_ardrone_simulator/scripts/order.txt',delimiter=None)
        self.order_pic=x.astype(int)
        
        self.x_array=[1,4.26,0.88,4.33,7.69]
        self.y_array=[1,1.23,5.48,8.04,4.24]
        self.angle_orientation=[0,-1.26,0.12,-0.717,2.11]

        self.obstacleList = [
        (6.23, 1.9, 1.7),
        (4.48, 3.44, 1.52),
        (7.51, 7.14, 1.46),
        (0.59,8.35, 1.25),
        (2.19, 7.31, 1.1),
        (1.43,2.5, 0.98),
        (5.8, 6.53 , 0.95)
    ]  # [x, y, radius]


    #Create  Trajectory
      
    def linear_trajectory(self):
        
        X1 = np.linspace(-1, 1, num=self.number_of_points/2)
        Y1 = np.linspace(2, 0, num=self.number_of_points/2)
        Z1 = np.linspace(1, 2, num=self.number_of_points/2)
        X2 = np.linspace(1, -1, num=self.number_of_points/2)
        Y2 = np.linspace(0, 2, num=self.number_of_points/2)
        Z2 = np.linspace(2, 1, num=self.number_of_points/2)

        self.X=np.concatenate([X1,X2])
        self.Y=np.concatenate([Y1,Y2])
        self.Z=np.concatenate([Z1,Z2])

        self.X_euler = np.linspace(0, 0 , self.number_of_points)
        self.Y_euler = np.linspace(0, 0 , self.number_of_points)
        self.Z_euler = np.linspace(0 ,0 , self.number_of_points)
    
    
    def snake_trajectory(self):
        point_short=int(self.number_of_points/24)
        point_long=int(self.number_of_points/6)

        y1 = np.linspace(-2,2,num=point_long)
        y2 = np.ones(point_short)*2
        y3 = np.linspace(2,-2,num=point_long)
        y4 = np.ones(point_short)*-2
        y5 = np.linspace(-2,2,num=point_long)
        y6 = np.ones(point_short)*2
        y7 = np.linspace(2,-2,num=point_long)
        y8 = np.ones(point_short)*-2
        y9 = np.linspace(-2,2,num=point_long)
        self.Y=np.concatenate([y1,y2,y3,y4,y5,y6,y7,y8,y9])

        x1 = np.ones(point_long)*-2
        x2 = np.linspace(-2,-1,num=point_short)
        x3 = np.ones(point_long)*-1
        x4 = np.linspace(-1,0,num=point_short)
        x5 = np.ones(point_long)*0
        x6 = np.linspace(0,1,num=point_short)
        x7 = np.ones(point_long)*1
        x8 = np.linspace(1,2,num=point_short)
        x9 = np.ones(point_long)*2
        self.X=np.concatenate([x1,x2,x3,x4,x5,x6,x7,x8,x9])
 
        self.Z = np.ones(self.number_of_points)*1.3

        self.X_euler = np.linspace(0, 0 , self.number_of_points)
        self.Y_euler = np.linspace(0, 0 , self.number_of_points)
        self.Z_euler = np.linspace(0 ,0 , self.number_of_points)

    #Reach Position to start snake path
    def ready(self):
        self.X = np.ones(self.number_of_points)*1
        self.Y = np.ones(self.number_of_points)*1
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
    
    #RRT Path Planning
    def rrt_planning(self):
        #initialize array
        X = []
        Y = []
        Z_euler=[]
        #set start point for orientation 
        angle_s= self.angle_orientation[0]
        #set start point for path
        x_s = self.x_array[0]
        y_s = self.x_array[0]
        
        l = len(self.order_pic)
        maxIter = 1000
        obstacleList=self.obstacleList
	    #Generate Path & Orientation based on given order
        for i in range(l):
            #get order number
            n = self.order_pic[i]
            print(n)
            #set goal point
            x_g = self.x_array[n]
            y_g = self.y_array[n]
            angle_g = self.angle_orientation[n]
            angle_g = angle_g+np.pi
            #call RRT from rrt.py
            rrt = RRT(start=[x_s, y_s], goal=[x_g, y_g], rand_area=[0, 9], obstacle_list=self.obstacleList)
            path = rrt.planning(animation=False)
            #print(path)
            smoothedPath = rrt.path_smoothing(path, maxIter, obstacleList)
            #print(smoothedPath)
            X_p,Y_p = map(list,zip(*smoothedPath))
            #Reverse X,Y coordinate for each segment
            X_p = np.flipud(X_p)
            Y_p = np.flipud(Y_p)

            m = len(X_p)-1
            
            num_point = 800
            #Store the path
            for j in range(m):
                #linaer interpolation of path
                n_x = np.linspace(X_p[j], X_p[j+1], num=num_point)
                n_y = np.linspace(Y_p[j], Y_p[j+1], num=num_point)

                X = np.concatenate([X,n_x])
                Y = np.concatenate([Y,n_y])
                     
            #linear interpolation of orientation
            n_z_euler=np.linspace(angle_s , angle_g, m*num_point)
            Z_euler=np.concatenate([Z_euler,n_z_euler])

            #update start point
            x_s = x_g
            y_s = y_g
            angle_s = angle_g
        
        Z = np.ones(len(X))*1.3
        X_euler = np.linspace(0, 0 , len(X))
        Y_euler = np.linspace(0, 0 , len(X))
        #print(X)
        
        self.number_of_points=len(X)
        return X, Y, Z, X_euler, Y_euler, Z_euler
        

        #Path = np.array([X, Y, Z, X_euler, Y_euler, Z_euler])
        #return Path


        
    def project(self):
        #Data = self.rrt_planning()
        #[self.X, self.Y, self.Z, self.X_euler, self.Y_euler, self.Z_euler] = Data
        self.desired_position_counter=0
        self.X, self.Y, self.Z, self.X_euler, self.Y_euler, self.Z_euler = self.rrt_planning()
        print(len(self.X))
        print(len(self.Y_euler))
        print(len(self.Z_euler))
        
        
    #Choose Path Type
    def choose_type(self,message):
        if(message.data=="linear"):
            self.linear_trajectory()
        elif(message.data=="spiral"):
            self.spiral_trajectory()
        if(message.data=="circle"):
            self.circle_trajectory()
        if(message.data=="snake"):
            self.snake_trajectory()
        if(message.data=="ready"):
            self.ready()
        if(message.data=="project"):
            self.project()

    def make_land(self,message):
        if(message.data=="land"):
            self.X=[1,1,1]
            self.desired_position_counter=0
            self.number_of_points=(self.X)
            self.Y=[1,1,1]
            self.Z=[0,0,0]

            self.X_euler = np.linspace(0, 0 , self.number_of_points)
            self.Y_euler = np.linspace(0, 0 , self.number_of_points)
            self.Z_euler = np.linspace(0, 0 , self.number_of_points)
            msg.data="stop"
            
            self.pubToggledes.publish(msg)
            self.pubLanddes.publish(Empty())
        

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

# Import ROS libraries
import roslib
import rospy
import numpy as np
import time

# Import class that computes the desired positions

from rrt import RRT

#start=0
#end=4

order_pic = [3,2,1,4]
x_array=[1,4.26,0.88,4.33,7.69]
y_array=[1,1.23,5.48,8.04,4.24]

obstacleList = [
        (6.23, 1.9, 1.2),
        (4.48, 3.44, 1.02),
        (7.51, 7.14, 0.96),
        (0.59,8.35, 0.75),
        (2.19, 7.31, 0.6),
        (1.43,2.5, 0.48),
        (5.8, 6.53 , 0.45)
    ]  # [x, y, radius]

def rrt_planning():
    
    X = []
    Y = []

    l = len(order_pic)
    #print(l) = 4

    x_s = x_array[0]
    y_s = x_array[0]
    for i in range(l):
        #get order from order array
        n = order_pic[i]

        x_g = x_array[n]
        y_g = y_array[n]

        rrt = RRT(start=[x_s, y_s], goal=[x_g, y_g], rand_area=[0, 9], obstacle_list=obstacleList)

        path = rrt.planning(animation=False)
        X_p,Y_p = map(list,zip(*path))

        X_p = np.flipud(X_p)
        Y_p = np.flipud(Y_p)

        print(X_p)

        l = len(X_p)-1
        num_point = 5
        
        for i in range(l):

            n_x = np.linspace(X_p[i], X_p[i+1], num=num_point)
            n_y = np.linspace(Y_p[i], Y_p[i+1], num=num_point)
            X = np.concatenate([X,n_x])
            Y = np.concatenate([Y,n_y])

        x_s = x_g
        y_s = y_g
    return X, Y

x,y = rrt_planning()

print(x)
#print(y)







import numpy as np
import cv2 as cv
import pandas as pd

from tf.transformations import euler_from_quaternion

debugrun=False

def camera_matrix():
    # K(given)
    cam_matrix = [[604.62,   0.00, 320.5], 
                 [  0.00, 604.62, 180.5],
                 [  0.00,   0.00,   1.00]]

    return cam_matrix

def distrotion_coefficients():
 # d(given)
    dist_coeffs =   [[ 0.191887,
                    -0.563680,
                    -0.003676,
                    -0.002037,
		            0.000000 ]]    
    return dist_coeffs





def eulerAnglesToRotationMatrix(roll,pitch,yaw) :
    
    R_x = np.array([[1,         0,                  0               ],
                    [0,         np.cos(roll),       -np.sin(roll)   ],
                    [0,         np.sin(roll),       np.cos(roll)    ]
                    ])
        
        
                    
    R_y = np.array([[np.cos(pitch),     0,      np.sin(pitch)   ],
                    [0,                 1,      0               ],
                    [-np.sin(pitch),    0,      np.cos(pitch)   ]
                    ])
                
    R_z = np.array([[np.cos(yaw),    -np.sin(yaw),    0 ],
                    [np.sin(yaw),    np.cos(yaw),     0 ],
                    [0,                 0,            1 ]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def image_to_frame(x,y,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w):
    quaternion = np.array([rotation_x,
                          rotation_y,
                          rotation_z,
                          rotation_w])

    euler = euler_from_quaternion(quaternion)
    roll_angle = euler[0]
    pitch_angle = euler[1]
    yaw_angle = euler[2]
    
    R =eulerAnglesToRotationMatrix(roll_angle,pitch_angle,yaw_angle)
    
    x_ground=np.tan(roll_angle)
    
    y_ground=np.tan(pitch_angle)
    
    hyp=np.sqrt(np.power(x_ground,2)+np.power(y_ground,2))
    dist_ang=hyp    
    top_angle=np.arctan(dist_ang) 
     
    
    #cg input of ball
    pixel_x = x
    pixel_y = y

    if(debugrun==True):
        print(x_ground)
        print(y_ground)
        print(top_angle)  
        print(pixel_x)
        print(pixel_y)


    pixel_delta = [[pixel_x],[pixel_y],[1]]

    k_matrix = camera_matrix()

    p_matrix = np.matrix([[0,-1,0],[-1,0,0],[0,0,-1]])
    pt_matrix = np.matrix([[0],[0.0125],[-0.025]])
    
    
    k_inv = np.linalg.inv(k_matrix)
    p_inv = np.linalg.inv(p_matrix)
    #distance = k_inv.dot(p_inv.dot(pixel_delta))
    
    distance = k_inv.dot(pixel_delta)
    distance=distance*translation_z/np.cos(top_angle)
    
    distance=pt_matrix+p_matrix.dot(distance)
    
    translation_matrix=[[translation_x],[translation_y],[translation_z]]
    
    [[delta_x],[delta_y],[delta_z]] = distance
    ball_location =  translation_matrix+ R.dot(distance)
    if(debugrun==True):
        print(k_matrix)
        print(p_matrix)
        print(np.shape(pixel_delta))
        print(distance)
        print(np.shape(distance))
        print(np.shape(ball_location))
        print(translation_matrix)
        print(np.shape(translation_matrix))
        print(ball_location)

    return ball_location




def main():
    df_cr= pd.read_csv("corresponding_center_radius.csv")
    df_p=pd.read_csv("corresponding_uav_pose.csv")


    number_of_iterations=len(df_cr)
    save_directory = '/home/ubuntu16/Desktop/Nudes/Project/processing'
    save_file= "global_coordinates.csv"
    save_pathname=save_directory+"/"+save_file
    f= open(save_pathname,"w+")
    f.write("X"+","+"Y"+","+"RADIUS"+"\n")
    for i in range(number_of_iterations):
        x=df_cr.loc[i, "x"]
        y=df_cr.loc[i, "y"]
        radius=df_cr.loc[i, "radius"]
        upper_limit=30
        if(radius>35):
            translation_x=df_p.loc[i, "position_x"]
            translation_y=df_p.loc[i, "position_y"]
            translation_z=df_p.loc[i, "position_z"]
            rotation_x=df_p.loc[i, "orientation_x"]
            rotation_y=df_p.loc[i, "orientation_y"]
            rotation_z=df_p.loc[i, "orientation_z"]
            rotation_w=df_p.loc[i, "orientation_w"]


            ball_location=image_to_frame(x,y,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w)
            ball_edge_location=image_to_frame((x+radius),y,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w)
            """
            [[global_x],[global_y],[walla]]=ball_location
            print(global_x)
            print(ball_location[1][0][0])
            print(ball_location[2])
            """        
            global_x=str(ball_location[0][0])
            global_y=str(ball_location[1][0])
            global_x=global_x.replace(']','')
            global_y=global_y.replace(']','')
            global_x=global_x.replace('[','')
            global_y=global_y.replace('[','')


            global_edge_x=str(ball_edge_location[0][0])
            global_edge_y=str(ball_edge_location[1][0])
            global_edge_x=global_edge_x.replace(']','')
            global_edge_y=global_edge_y.replace(']','')
            global_edge_x=global_edge_x.replace('[','')
            global_edge_y=global_edge_y.replace('[','')
            diff_x=float(global_x)-float(global_edge_x)
            diff_y=float(global_y)-float(global_edge_y)
            radius_calc=np.sqrt((np.power(diff_x, 2)+np.power(diff_y, 2)))
            #print(global_x)
            #print(global_y)
            f.write(str(global_x)+","+str(global_y)+","+str(radius_calc)+"\n")
            #break
    f.close()
    

main()



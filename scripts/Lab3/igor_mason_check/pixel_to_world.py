import numpy as np
import cv2 as cv
import pandas as pd

from tf.transformations import euler_from_quaternion

def camera_matrix():
    # K(given)
    cam_matrix = [[698.86,   0.00, 306.91], 
                 [  0.00, 699.13, 150.34],
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
    
    #cg input of ball
    pixel_x = x
    pixel_y = y
    #print(pixel_x)
    #print(pixel_y)

    camera_loc_x = 320
    camera_loc_y = 180

    #difference 

    diff_x = camera_loc_x - pixel_x
    diff_y = -(camera_loc_y - pixel_y)

    pixel_delta = [[pixel_x],[pixel_y],[1]]

    k_matrix = camera_matrix()

    p_matrix = np.matrix([[0,-1,0],[-1,0,0],[0,0,-1]])
    #print(k_matrix)
    #print(p_matrix)
    #print(np.shape(pixel_delta))
    k_inv = np.linalg.inv(k_matrix)
    p_inv = np.linalg.inv(p_matrix)
    #distance = k_inv.dot(p_inv.dot(pixel_delta))
    
    distance = k_inv.dot(pixel_delta)
    distance=distance*translation_z
    #print(np.shape(distance))
    distance=p_matrix.dot(distance)
    #print(distance)
    translation_matrix=[[translation_x],[translation_y],[translation_z]]
    #print(translation_matrix)
    #print(np.shape(translation_matrix))
    [[delta_x],[delta_y],[delta_z]] = distance
    ball_location =  translation_matrix+ R.dot(distance)
    #print(np.shape(ball_location))

    #print(ball_location)

    return ball_location




def main():
    df_cr= pd.read_csv("center_radius.csv")
    df_p=pd.read_csv("corresponding_uav_pose.csv")


    number_of_iterations=len(df_cr)
    save_directory = '/home/ubuntu16/MEng/New/Their Stuff/gi/scripts/Lab3/igor_mason_check'
    save_file= "global_coordinates.csv"
    save_pathname=save_directory+"/"+save_file
    f= open(save_pathname,"w+")
    f.write("X"+","+"Y"+"\n")
    for i in range(number_of_iterations):
        x=df_cr.loc[i, "x"]
        y=df_cr.loc[i, "y"]
        radius=df_cr.loc[i, "radius"]
        upper_limit=30
        if(radius>6 and radius<=30):
            translation_x=df_p.loc[i, "position_x"]
            translation_y=df_p.loc[i, "position_y"]
            translation_z=df_p.loc[i, "position_z"]
            rotation_x=df_p.loc[i, "orientation_x"]
            rotation_y=df_p.loc[i, "orientation_y"]
            rotation_z=df_p.loc[i, "orientation_z"]
            rotation_w=df_p.loc[i, "orientation_w"]


            ball_location=image_to_frame(x,y,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w)
            
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
            #print(global_x)
            #print(global_y)
            f.write(str(global_x)+","+str(global_y)+"\n")
            #break
    f.close()
    

main()



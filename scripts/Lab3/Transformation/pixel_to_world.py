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

    camera_loc_x = 320
    camera_loc_y = 180

    
    #difference 
    k_matrix = camera_matrix()

    k_inv = np.linalg.inv(k_matrix)
    pixel = [[pixel_x],[pixel_y],[1]]
    pixel_image = k_inv.dot(pixel)*translation_z
    pixel_global = R.dot(pixel_image)
    
    diff_x = camera_loc_x - pixel_global[0,:]
    diff_y =  camera_loc_y - pixel_global[1,:]
    
    #diff_x = camera_loc_x - pixel_image[0,:]
    #diff_y =  camera_loc_y - pixel_image[1,:]
    
    distance = [[diff_x],[diff_x],[0]]
  
    #print(np.shape(distance))
    translation_matrix=[[translation_x],[translation_y],[translation_z]]
    #print(np.shape(translation_matrix))
    #[[delta_x],[delta_y],[delta_z]] = distance
    ball_location =  translation_matrix+distance
    #ball_location =  translation_matrix+pixel_global
    #print(np.shape(ball_location))

    #print(ball_location)

    return ball_location

def main():
    df_cr= pd.read_csv("center_radius.csv")
    df_p=pd.read_csv("corresponding_uav_pose.csv")


    number_of_iterations=len(df_cr)
    save_directory = '/home/mason/Desktop/Transformation'
    save_file= "global_coordinates.csv"
    save_pathname=save_directory+"/"+save_file
    f= open(save_pathname,"a+")
            
    for i in range(number_of_iterations):
        x=df_cr.loc[i, "x"]
        y=df_cr.loc[i, "y"]
        radius=df_cr.loc[i, "radius"]
        if(radius!=0 and radius<=20):
            translation_x=df_p.loc[i, "position_x"]
            translation_y=df_p.loc[i, "position_y"]
            translation_z=df_p.loc[i, "position_z"]
            rotation_x=df_p.loc[i, "orientation_x"]
            rotation_y=df_p.loc[i, "orientation_y"]
            rotation_z=df_p.loc[i, "orientation_z"]
            rotation_w=df_p.loc[i, "orientation_w"]

            ball_location=image_to_frame(x,y,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w)
            

            global_x=ball_location[0][0]
            global_y=ball_location[1][0]
            f.write(str(global_x)+","+str(global_y)+"\n")
            #break
    f.close()
    

main()



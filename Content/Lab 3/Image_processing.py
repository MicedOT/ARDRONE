# sample x and y (349.642857143, 84.5318416523)

import numpy as np
from numpy.linalg import inv
import cv2
import sys

# K(given)
camera_matrix = [[698.86,   0.00, 306.91], 
                 [  0.00, 699.13, 150.34],
                 [  0.00,   0.00,   1.00]]

# d(given)
dist_coeffs =   [[ 0.191887,
                  -0.563680,
                  -0.003676,
                  -0.002037,
		   0.000000 ]]

# Array of object points in world coordinate space, sample points
tnsPoints = np.zeros((19, 3)) 
tnsPoints[ 0] = (   0.00,     0.00, 0)
tnsPoints[ 1] = (   0.00,   137.16, 0)
tnsPoints[ 2] = (   0.00,   548.64, 0)
tnsPoints[ 3] = (   0.00,   960.12, 0)
tnsPoints[ 4] = (   0.00,  1097.28, 0)
tnsPoints[ 5] = ( 548.64,   137.16, 0)
tnsPoints[ 6] = ( 548.64,   548.64, 0)
tnsPoints[ 7] = ( 548.64,   960.12, 0)
tnsPoints[ 8] = (1188.72 ,    0.00, 0)
tnsPoints[ 9] = (1188.72 ,  137.16, 0)
tnsPoints[10] = (1188.72 ,  548.64, 0)
tnsPoints[11] = (1188.72 ,  960.12, 0)
tnsPoints[12] = (1188.72 , 1097.28, 0)
tnsPoints[13] = (1828.80 ,  137.16, 0)
tnsPoints[14] = (1828.80 ,  548.64, 0)
tnsPoints[15] = (1828.80 ,  960.12, 0)
tnsPoints[16] = (2377.44 ,    0.00, 0)
tnsPoints[17] = (2377.44 ,  137.16, 0)
tnsPoints[18] = (2377.44 ,  548.64, 0)

# Array of corresponding image points, sample points
imPoints = np.zeros((19,2))
imPoints[ 0] = (306.91, 150.34)
imPoints[ 1] = (326, 156)
imPoints[ 2] = (398, 154)
imPoints[ 3] = (471, 150)
imPoints[ 4] = (494, 148)
imPoints[ 5] = (319, 172)
imPoints[ 6] = (406, 170)
imPoints[ 7] = (491, 167)
imPoints[ 8] = (270, 206)
imPoints[ 9] = (306, 206)
imPoints[10] = (421, 203)
imPoints[11] = (532, 197)
imPoints[12] = (570, 195)
imPoints[13] = (283, 266)
imPoints[14] = (446, 260)
imPoints[15] = (607, 252)
imPoints[16] = (146, 390)
imPoints[17] = (235, 387)
imPoints[18] = (499, 374)

# Solve PnP(perspective-n-Point), rvec =  O/P rotation vector, tvec = O/P translation vector
retval, rvec, tvec = cv2.solvePnP(tnsPoints,
                                  imPoints,
                                  np.asarray(camera_matrix),
                                  np.asarray(dist_coeffs))
#print('Rotation Vector:',rvec)
#print('Translation vector:', tvec)

# Converts vector to matrix
rotMat, _ = cv2.Rodrigues(rvec)
#print('Rotation Matrix',rotMat)


def groundProjectPoint(image_point, z = 0.0):
    camMat = np.asarray(camera_matrix)
# Inverse of matrices
    iRot = inv(rotMat)
    iCam = inv(camMat)

# u,v are coordinate values in pixel
    uvPoint = np.ones((3, 1))


    uvPoint[0, 0] = image_point[0]
    uvPoint[1, 0] = image_point[1]

# Matrix multiplication 
# equations https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/ 
    tempMat = np.matmul(np.matmul(iRot, iCam), uvPoint)
    #print("tempMat=",tempMat)
    tempMat2 = np.matmul(iRot, tvec)
    #print("tempMat2=",tempMat2)

    s = (z + tempMat2[2, 0]) / tempMat[2, 0]
    #print("s=",s)
    wcPoint = np.matmul(iRot, (np.matmul(s * iCam, uvPoint) - tvec))

    # wcPoint[2] will not be exactly equal to z, but very close to it
    assert int(abs(wcPoint[2] - z) * (10 ** 8)) == 0
    wcPoint[2] = z

    return wcPoint

pixel = (349.642857143, 84.5318416523)
print("Pixel: %s" % (pixel, ))
print(groundProjectPoint(pixel))


from PIL import Image
import cv2
import numpy as np

#img = np.array(Image.open('sample_tennis_01.png')) NOT WORKING
img=cv2.imread('sample_tennis_01.png')

# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# define range of blue color in HSV
lower_green = np.array([20, 90, 50])
upper_green = np.array([165, 173, 190])

# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower_green, upper_green)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(img,img, mask= mask)
# find the center
moments = cv2.moments(mask)
area = moments['m00']

#if (area > 10000):
x= moments['m10']/area
y=  moments['m01']/area

cv2.imshow('frame',img)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
print(x)
print(y)
cv2.waitKey()
    
# convert image to grayscale image
gray_image = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

# convert the grayscale image to binary image
ret,thresh = cv2.threshold(gray_image,127,255,0)

# calculate moments of binary image
M = cv2.moments(thresh)

# calculate x,y coordinate of center
cX = int(M["m10"] / M["m00"])
cY = int(M["m01"] / M["m00"])

# put text and highlight the center
cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
print(cX)
print(cY)
# display the image
cv2.imshow("Image", img)
cv2.waitKey(0)



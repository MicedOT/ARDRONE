import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

import os
import glob
import ntpath
def matcher(img_left,img_right):
    theta=0
    MIN_MATCH_COUNT = 10
    outlierrejectionmethod="ransac" #(lm,ransac)
    filtervalue=0.7		# (Extreme Strictness)0-1(Relaxed)
    numberoffeatures=1000

    debugrun=False
    plotpython=False
    testmatches=False
    drawboundingbox=False
    #load images
    #img_left = cv.imread(left_image_dir+"/"+sample_name+".png")
    #img_right = cv.imread(right_image_dir+"/"+sample_name+".png")
    
    if debugrun==True:
	    cv.imshow('Left Image',img_left)
	    cv.imshow('Right Image',img_right)
	    cv.waitKey(0)
	    cv.destroyAllWindows()

    # create a feature detector
    orb = cv.ORB_create(nfeatures=numberoffeatures)

    # find the keypoints
    kp_left = orb.detect(img_left,None)
    kp_right = orb.detect(img_right, None)

    #if debugrun==True:
        #imgkp = cv.drawKeypoints(img_left,kp_left,imgkp,color="Green",flags=0)
        #cv.imshow( "matches", imgkp)
        #cv.waitKey(0)

    # compute the descriptors
    kp_left,des_left = orb.compute(img_left,kp_left)
    kp_right,des_right = orb.compute(img_right, kp_right)

    if debugrun==True:
        if debugrun==True:print("len(kp_left) : ", len(kp_left), "    des_left.shape : ", des_left.shape)
        if debugrun==True:print("len(kp_right) : ", len(kp_right), "    des_left.shape : ", des_left.shape)


    # create a matcher
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2  
    search_params = dict(checks = 50)
    flann = cv.FlannBasedMatcher(index_params, search_params)

    # match descriptors
    matches = flann.knnMatch(des_left,des_right,k=2)

    good = []
    for m,n in matches:
        if m.distance < filtervalue*n.distance:
            good.append(m)

    if testmatches==True:
        print("Good Matches:	",len(good))


    MIN_MATCH_COUNT=10 # Defines number of good matches required to perform outlier rejection
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp_left[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp_right[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        if outlierrejectionmethod=="lm":
            M, mask = cv.findHomography(src_pts, dst_pts, cv.LEAST_MEDIAN,5.0)
        else :
            M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img_left.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts,M)    # top-left bottom-left bottom-right top-right
        if drawboundingbox==True:
            img2 = cv.polylines(img_right,[np.int32(dst)],True,(0,255,255),3, cv.LINE_AA)
        
        theta = - np.arctan2(M[0,1], M[0,0]) #* 180 / np.pi
    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None

    if debugrun==True:
        #matchesMask is a list of size good which stores a binary value of whether outliers(0) or inliers(1)
        #print(matchesMask)
        print(dst.shape)
        print(dst[0,0,0])
        print(dst[2,0,0])
        print(dst[0,0,1])
        print(dst[2,0,1])

    # draw matches
    draw_params = dict(singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers, None for No mask
                   flags = 2)
    img3 = cv.drawMatches(img_left,kp_left,img_right,kp_right,good, None,**draw_params)
    
    if plotpython==True:
        #img3 = cv.drawMatches(img_left,kp_left,img_right,kp_right,matches, None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        #not working by self
        #install tkinter very buggy still colour changed
        plt.imshow(img3)
        plt.show()
    #cv.imshow( "matches", img3)
    #cv.waitKey(0)
    midpoint_x=int((dst[0,0,0]+dst[2,0,0])/2)
    midpoint_y=int((dst[0,0,1]+dst[2,0,1])/2)
    return [theta,midpoint_x,midpoint_y]

#"""
sight_directory="/home/ubuntu16/Desktop/Nudes/Project/sights"
image_directory="/home/ubuntu16/Desktop/Nudes/Project/images"
images = [f for f in glob.glob(image_directory + "/*.png",)]
sorted_images=sorted(images)
sights = [f for f in glob.glob(sight_directory + "/*.png",)]
sorted_sights=sorted(sights)

save_directory = '/home/ubuntu16/Desktop/Nudes/Project/processing'
save_file= "pic_orientation.csv"
save_pathname=save_directory+"/"+save_file
id_file= "pic_id.csv"
id_pathname=save_directory+"/"+id_file
f= open(save_pathname,"w+")
f.write("id"+","+"frame"+","+"X"+","+"Y"+","+"orientation"+"\n")
g= open(id_pathname,"w+")
g.write("id"+","+"name"+"\n")
identification_no=0
check=True
for image_filename in sorted_images:
    img2 = cv.imread(image_filename,0) # trainImage
    identification_no=0
    check=True
    frame_name=ntpath.basename(image_filename)
    for sight_filename in sorted_sights:
        try:
         
            
            img1 = cv.imread(sight_filename,0)          # queryImage
            [theta,mx,my]=matcher(img1,img2)
            if(theta!=0):                
                check=False
                mx=str(mx)
                my=str(my)
                theta=str(theta)
                f.write(str(identification_no)+","+frame_name+","+mx+","+my+","+theta+"\n")
                break
        except Exception:
            print("well done") 
        
        identification_no=identification_no+1 
    if(check==True):        
        f.write("-1"+","+frame_name+","+"0"+","+"0"+","+"0"+"\n")
        
    
f.close()
identification_no=0
for sight_filename in sorted_sights:
    sight_name=ntpath.basename(sight_filename)
    g.write(str(identification_no)+","+sight_name+"\n")
    identification_no=identification_no+1 
g.close()



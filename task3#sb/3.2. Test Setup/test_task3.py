###############################################################################
## Author: Team Supply Bot
## Edition: eYRC 2019-20
## Instructions: Do Not modify the basic skeletal structure of given APIs!!!
###############################################################################


######################
## Essential libraries
######################
import cv2
import numpy as np
import os
import math
import csv
import copy


def find_correct_contours(contour, name, im_shape):
    detected = []
    for cont in contour:
        area = cv2.contourArea(cont)
        equi_diameter = np.sqrt(4*area/np.pi)
        center, radius = cv2.minEnclosingCircle(cont)
        try:
            ratio = (2*radius-equi_diameter)/equi_diameter
        except:
            pass
        #print(ratio)
        if name == 'G' or name == 'R':
            if 0.06 < ratio < 0.25 and 7 < equi_diameter < 15:  #9.5 to 12
                detected.append(cont)
                #print("detected")
        elif name == 'W':
            H, W, CH = im_shape
            x, y = center
            if 0.4*W < x < 0.6*W and 0.4*H < y < 0.6*H:
                if 0.06 < ratio < 0.25 and  7< equi_diameter < 15:  #9.5 to 12
                    detected.append(cont)
                
    return detected

def enquire(contourG, contourR, contourW):
    res = 1
    if not contourG:
        print("Green not found")
        res = 0
    if not contourR:
        print("Red not found")
        res = 0
    if not contourW:
        print("White not found")
        res = 0
    return res
    

def get_centre(contour):
    M = cv2.moments(contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [cX, cY]





############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    ###########################
    angle = -1
    
    pic = cv2.cvtColor(ip_image, cv2.COLOR_BGR2HSV)
    
    LLG = (25, 62, 110)#GREEN HSV=(60,255,255)
    ULG = (76, 255, 255)
    LLR = ( 164, 150, 150)#RED HSV=(0,255,255)
    ULR = ( 180, 255, 255)
    LLW = ( 0, 0, 0)#WHITE HSV=(0,0,255)
    ULW = ( 180, 41, 255)
    
    maskG = cv2.inRange(pic, LLG, ULG)
    maskR = cv2.inRange(pic, LLR, ULR)
    maskW = cv2.inRange(pic, LLW, ULW)
    
    contourG,h=cv2.findContours(maskG, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourR,h=cv2.findContours(maskR, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourW,h=cv2.findContours(maskW, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    contourG = find_correct_contours(contourG, 'G', ip_image.shape)
    contourW = find_correct_contours(contourW, 'W', ip_image.shape)
    contourR = find_correct_contours(contourR, 'R', ip_image.shape)

    cv2.drawContours(ip_image, contourG, -1, (255, 0, 0), 2)
    cv2.drawContours(ip_image, contourR, -1, (255, 255, 0), 2)
    cv2.drawContours(ip_image, contourW, -1, (0, 255, 255), 2)


    if enquire(contourG, contourR, contourW):
        cG = get_centre(contourG[0])
        cR = get_centre(contourR[0])
        cW = get_centre(contourW[0])
        '''
        cG, rG = cv2.minEnclosingCircle(contourG[0])
        cR, rR = cv2.minEnclosingCircle(contourR[0])
        cW, rW = cv2.minEnclosingCircle(contourW[0])
        
        mG = (cG[0]-cW[0])/(cG[1]-cW[1])
        mR = (cR[0]-cW[0])/(cR[1]-cW[1])
        aG = math.degrees(math.atan(mG))
        aR = math.degrees(math.atan(mR))
        if cG[1]>cW[1]:
            aG=180+aG
        if cR[1]>cW[1]:
            aR=180+aR
        if aG > 180:
            aG=aG-360
        if aR > 180:
            aR=aR-360
        #print(aG, aR)
        if aG > aR:
            angle = aG-aR
        if aR > aG:
            angle = aR-aG
        angle = abs(angle)
        if angle > 180:
            angle = 360-angle
        angle = round(angle,2)
        '''
        angle = math.degrees(math.atan2(cG[1]-cW[1], cG[0]-cW[0]) - math.atan2(cR[1]-cW[1], cR[0]-cW[0]))
        if angle < 0:
            angle = angle + 360
        if angle > 180:
            angle = 360 - angle
        angle = round(angle,2)
        ip_image = cv2.putText(ip_image, "Angle: " + str(angle), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
    else:
        print("One of three contour not found")
    cv2.imshow("Result", ip_image)
    op_image = ip_image
    return op_image
    
    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Modify the image name as per instruction
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(1) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    while(ret):
        ret, frame = cap.read()
        ## display to see if the frame is correct
        #cv2.imshow("window", frame)
        cv2.waitKey(int(1000/fps));
        ## calling the algorithm function
        op_image = process(frame)
        cv2.imwrite("SB#4277_task3I.jpg",op_image)


    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

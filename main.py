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




########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Images'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))




############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    
    pic = cv2.cvtColor(ip_image, cv2.COLOR_BGR2HSV)
    
    LLG = (55, 250, 250)#GREEN HSV=(60,255,255)
    ULG = (65, 255, 255)
    LLR = ( 0, 250, 250)#RED HSV=(0,255,255)
    ULR = ( 0, 255, 255)
    LLW = ( 0,   0, 250)#WHITE HSV=(0,0,255)
    ULW = ( 0,   0, 255)
    
    maskG = cv2.inRange(pic, LLG, ULG)
    maskR = cv2.inRange(pic, LLR, ULR)
    maskW = cv2.inRange(pic, LLW, ULW)

    contourG,h=cv2.findContours(maskG,cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourR,h=cv2.findContours(maskR,cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourW,h=cv2.findContours(maskW,cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    
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
    angle = -1
    if aG > aR:
        angle = aG-aR
    if aR > aG:
        angle = aR-aG
    angle = abs(angle)
    if angle > 180:
        angle = 360-angle
    angle = round(angle,2)
    print(angle)
    
    ## Your Code goes here
    ###########################
    #cv2.imshow("window", ip_image)
    #cv2.waitKey(0);
    return angle




    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    line = []
    ## Reading 1 image at a time from the Images folder
    for image_name in os.listdir(images_folder_path):
        ## verifying name of image
        print(image_name)
        ## reading in image 
        ip_image = cv2.imread(images_folder_path+"/"+image_name)
        ## verifying image has content
        print(ip_image.shape)
        ## passing read in image to process function
        A = process(ip_image)
        ## saving the output in  a list variable
        line.append([str(i), image_name , str(A)])
        ## incrementing counter variable
        i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'angles.csv', 'w', newline='') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

import cv2
import numpy as np
import os

def partA():
    n=1
    vdo = cv2.VideoCapture('../Videos/RoseBloom.mp4')
    while(True):
        ret, frame = vdo.read()
        if n == 150:  # 25fps X 6 sec = 150th frame
            cv2.imwrite('../Generated/frame_as_6.jpg',frame)
            break
        n=n+1
    vdo.release()
    cv2.destroyAllWindows()

def partB():
    rose = cv2.imread('../Generated/frame_as_6.jpg',1)
    rose[:,:,0]=0
    rose[:,:,1]=0
    cv2.imwrite('../Generated/frame_as_6_red.jpg',rose)

partA()
partB()

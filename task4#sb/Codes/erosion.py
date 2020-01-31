from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse

erosion_size = 0
max_elem = 2
max_kernel_size = 21
title_trackbar_element_type = 'Element:\n 0: Rect \n 1: Cross \n 2: Ellipse'
title_trackbar_kernel_size = 'Kernel size:\n 2n +1'
title_erosion_window = 'Erosion Demo'
title_dilatation_window = 'Dilation Demo'

def erosion(val):
    LLW = ( 0, 0, 167)#WHITE HSV=(0,0,255)
    ULW = ( 180, 71, 255)

    erosion_size = cv.getTrackbarPos(title_trackbar_kernel_size, title_erosion_window)
    erosion_type = 0
    val_type = cv.getTrackbarPos(title_trackbar_element_type, title_erosion_window)
    if val_type == 0:
        erosion_type = cv.MORPH_RECT
    elif val_type == 1:
        erosion_type = cv.MORPH_CROSS
    elif val_type == 2:
        erosion_type = cv.MORPH_ELLIPSE

    element = cv.getStructuringElement(erosion_type, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    erosion_dst = cv.erode(pic, element)
    maskW = cv.inRange(erosion_dst, LLW, ULW)
    contourW,hW = cv.findContours(maskW, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    bgr = cv.cvtColor(erosion_dst, cv.COLOR_HSV2BGR)
    cv.drawContours(bgr, contourW, -1, (0, 255, 255), 1)
    cv.imshow(title_erosion_window, bgr)

def dilatation(val):
    dilatation_size = cv.getTrackbarPos(title_trackbar_kernel_size, title_dilatation_window)
    dilatation_type = 0
    val_type = cv.getTrackbarPos(title_trackbar_element_type, title_dilatation_window)
    if val_type == 0:
        dilatation_type = cv.MORPH_RECT
    elif val_type == 1:
        dilatation_type = cv.MORPH_CROSS
    elif val_type == 2:
        dilatation_type = cv.MORPH_ELLIPSE

    element = cv.getStructuringElement(dilatation_type, (2*dilatation_size + 1, 2*dilatation_size+1), (dilatation_size, dilatation_size))
    dilatation_dst = cv.dilate(src, element)
    cv.imshow(title_dilatation_window, dilatation_dst)

parser = argparse.ArgumentParser(description='Code for Eroding and Dilating tutorial.')
parser.add_argument('--input', help='Path to input image.', default='LinuxLogo.jpg')
args = parser.parse_args()

src = cv.imread(cv.samples.findFile(args.input))
if src is None:
    print('Could not open or find the image: ', args.input)
    exit(0)

pic = cv.cvtColor(src, cv.COLOR_BGR2HSV)

cv.namedWindow(title_erosion_window)
cv.createTrackbar(title_trackbar_element_type, title_erosion_window , 0, max_elem, erosion)
cv.createTrackbar(title_trackbar_kernel_size, title_erosion_window , 0, max_kernel_size, erosion)

#cv.namedWindow(title_dilatation_window)
#cv.createTrackbar(title_trackbar_element_type, title_dilatation_window , 0, max_elem, dilatation)
#cv.createTrackbar(title_trackbar_kernel_size, title_dilatation_window , 0, max_kernel_size, dilatation)

erosion(0)
#dilatation(0)
cv.waitKey()

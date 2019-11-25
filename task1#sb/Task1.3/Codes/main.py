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
import cv2.aruco as aruco
from aruco_lib import *
import copy



########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Videos'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))

def blur_edge(img, d=150):
    h, w  = img.shape[:2]
    img_pad = cv2.copyMakeBorder(img, d, d, d, d, cv2.BORDER_WRAP)
    img_blur = cv2.GaussianBlur(img_pad, (2*d+1, 2*d+1), -1)[d:-d,d:-d]
    y, x = np.indices((h, w))
    dist = np.dstack([x, w-x-1, y, h-y-1]).min(-1)
    w = np.minimum(np.float32(dist)/d, 1.0)
    return img*w + img_blur*(1-w)

def motion_kernel(angle, d, sz=65):
    kern = np.ones((1, d), np.float32)
    c, s = np.cos(angle), np.sin(angle)
    A = np.float32([[c, -s, 0], [s, c, 0]])
    sz2 = sz // 2
    A[:,2] = (sz2, sz2) - np.dot(A[:,:2], ((d-1)*0.5, 0))
    kern = cv2.warpAffine(kern, A, (sz, sz), flags=cv2.INTER_CUBIC)
    return kern




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
	img = np.float32(ip_image)/255.0
	channels = []
	results = []
	channels = cv2.split(img)
	i=0
	for channel in channels:
		i=i+1
		channel = np.float32(channel)/255.0
	#	cv2.imwrite(str(i)+'before_blur_edge_forchannel.jpg',channel * 255)
		channel = blur_edge(channel) 
	#	cv2.imwrite(str(i)+'blur_edge_for_channel.jpg',channel * 255)
		IMG = cv2.dft(channel, flags=cv2.DFT_COMPLEX_OUTPUT)
		ang = np.deg2rad( 90 ) #90
		d = 20                 #20
		noise = 10**(-0.1*25)  #25
		psf = motion_kernel(ang, d)
	#	cv2.imshow('psf', psf)
		psf /= psf.sum()
		psf_pad = np.zeros_like(channel)
		kh, kw = psf.shape
		psf_pad[:kh, :kw] = psf
		PSF = cv2.dft(psf_pad, flags=cv2.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
		PSF2 = (PSF**2).sum(-1)
		iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
		RES = cv2.mulSpectrums(IMG, iPSF, 0)
		res = cv2.idft(RES, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT )
		res = np.roll(res, -kh//2, 0)
		res = np.roll(res, -kw//2, 1)
		res = res*255.0
#		cv2.imshow('output',res)
		results.append(res)
#		cv2.imwrite(str(i) + 'deconvuled_eyantra_perchannel.jpg',res * 255)
	
	ip_image = cv2.merge(results)
	cv2.imwrite('bot.jpg',ip_image * 255)
	id_list = []
	detect_aruco = {}
	deconvulted = cv2.imread("bot.jpg")
	detect_aruco = detect_Aruco(deconvulted)
	if detect_aruco:
		img = mark_Aruco(deconvulted,detect_aruco)
		robot_state = calculate_Robot_State(img,detect_aruco)
#	id_list = calculate_Robot_State(deconvul
	print(id_list)	
	return ip_image, id_list


    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main(val):
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(images_folder_path+"/"+"aruco_bot.mp4")
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## getting the frame sequence
    frame_seq = int(val)*fps
    ## setting the video counter to frame sequence
    cap.set(1,frame_seq)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    ## display to see if the frame is correct
    cv2.imshow("window", frame)
    cv2.waitKey(0);
    ## calling the algorithm function
    op_image, aruco_info = process(frame)
    ## saving the output in  a list variable
    line = [str(i), "Aruco_bot.jpg" , str(aruco_info[0]), str(aruco_info[3])]
    ## incrementing counter variable
    i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'output.csv', 'w') as writeFile:
        print("About to write csv")
        writer = csv.writer(writeFile)
        writer.writerow(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main(input("time value in seconds:"))

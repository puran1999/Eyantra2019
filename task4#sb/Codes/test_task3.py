##############################################################################
## Team SB#4277
###############################################################################
## References:
## ArUCO Detection: aruco_lib provided by Eyantra Team
## Erosion: https://github.com/opencv/opencv/blob/3.4/samples/python/tutorial_code/imgProc/erosion_dilatation/morphology_1.py 
## Pyserial: https://pythonhosted.org/pyserial/index.html
## Port Detection: Answer from https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
## Tabular Formating: Answer from https://stackoverflow.com/questions/9535954/printing-lists-as-tabular-data
###############################################################################

######################
## Essential libraries
######################
import cv2
import numpy as np
#import os
import math
#import csv
#import copy
import time
from collections import defaultdict
from prettytable import PrettyTable

from aruco_lib import *
from port_detection import *

########################
## Setting color ranges
########################
LLG = (25, 62, 110)
ULG = (76, 255, 255)
LLR = ( 164, 150, 150)
ULR = ( 180, 255, 255)
LLW = ( 0, 0, 170)
ULW = ( 180, 50, 255)

###################################################
## Function to pick the required contours from the 
## detected ones.
###################################################
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
        if name == 'G' or name == 'R':
            if 0.06 < ratio < 0.25 and 7 < equi_diameter < 15: 
                detected.append(cont)
        elif name == 'IW':
            H, W, CH = im_shape
            x, y = center
            if 0.4*W < x < 0.6*W and 0.4*H < y < 0.6*H:
                if 0.06 < ratio < 0.25 and  7 < equi_diameter < 15:
                    detected.append(cont)
        elif name == 'OW':
            dist_min = 140
            dist_max = 220
            X, Y = tuple(im_shape)
            x, y = tuple(get_centre(cont))
            dist = math.sqrt((x - X)**2 + (y - Y)**2)
            if dist_max > dist > dist_min:
                detected.append(cont)

    return detected

#######################
## Function to get 
## centre of contour
#######################
def get_centre(contour):
    M = cv2.moments(contour)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return [cX, cY]
    except ZeroDivisionError:
        return [0,0]

############################
## Function to get 
## nodes from white contours
#############################
def get_nodes(pic):
    erosion_size = 3
    erosion_type = cv2.MORPH_CROSS
    element = cv2.getStructuringElement(erosion_type, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    erosion_dst = cv2.erode(pic, element)
    maskW = cv2.inRange(erosion_dst, LLW, ULW)
    contourW,hW = cv2.findContours(maskW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contourW

#############################
## Function to get calculate
## angles wrt white centre
#############################
def calculate_angle(pt1, pt2, centre):
    angle = math.degrees(math.atan2(pt1[1] - centre[1], pt1[0] - centre[0]) -
    math.atan2(pt2[1] - centre[1], pt2[0] - centre[0]))
    if angle < 0:
        angle = angle + 360
    if angle > 180:
        angle = angle - 360
    angle = round(angle,2)
    return angle


######################################
## Function to process the image and
## return node sequence to follow
######################################
def process(ip_image):
    pic = cv2.cvtColor(ip_image, cv2.COLOR_BGR2HSV)
    table = PrettyTable(['Node Type','Node Number',])

    maskG = cv2.inRange(pic, LLG, ULG)
    maskR = cv2.inRange(pic, LLR, ULR)
    maskW = cv2.inRange(pic, LLW, ULW)
    
    contourG,h=cv2.findContours(maskG, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourR,h=cv2.findContours(maskR, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourW,h=cv2.findContours(maskW, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    copyW = get_nodes(pic)
    
    contourG = find_correct_contours(contourG, 'G', ip_image.shape)
    contourW = find_correct_contours(contourW, 'IW', ip_image.shape)
    contourR = find_correct_contours(contourR, 'R', ip_image.shape)

    det_aruco_list = detect_Aruco(ip_image)
    #print(det_aruco_list)
    aruco_centre = []
    if det_aruco_list:
        robot_state = calculate_Robot_State(ip_image,det_aruco_list)
        ip_image = mark_Aruco(ip_image, det_aruco_list)
        keys = robot_state.keys()
        for key in keys:
            aruco_data = robot_state.get(key)
            aruco_centre = aruco_data[1:3]
            #print(aruco_centre)
    else:
        print("ArUCO not found!!!")
 
    cv2.drawContours(ip_image, contourG, -1, (255, 0, 0), 2)
    cv2.drawContours(ip_image, contourR, -1, (255, 0, 0), 2)

    if len(contourW) > 0:
        cW = get_centre(contourW[0])
        xw, yw = tuple(cW)
        #cv2.circle(ip_image,(xw, yw),140,(0,255,0),1)
        #cv2.circle(ip_image,(xw, yw),220,(0,255,0),1)
        ip_image = cv2.line(ip_image,(xw,yw),(xw,yw),(0,0,255),2)
        contOW = find_correct_contours(copyW, 'OW', cW)
        #cv2.drawContours(ip_image, contOW, -1, (0, 0, 255), 2)
        node_centers = []
        pos_node_angles = {}
        neg_node_angles = {}
        nodes = defaultdict(list)
        green_nodes_data = defaultdict(list)
        red_nodes_data = defaultdict(list)
        for cont in contOW:
            node_centers.append(get_centre(cont))
        for node in node_centers:
            if node != None:
                cv2.circle(ip_image,tuple(node),1,(0,0,255),2)
                if len(aruco_centre) > 0:
                    node_angle = calculate_angle(node, aruco_centre, cW)
                    if abs(node_angle) > 10:
                        if node_angle > 0:
                            pos_node_angles[node_angle] = node
                        else:
                            neg_node_angles[node_angle] = node
                        #ip_image = cv2.putText(ip_image, str(node_angle), tuple(node), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
        nodes[1].append(aruco_centre)
        nodes[1].append(0.0)
        node_num = len(nodes)
        for angle, centre in sorted(pos_node_angles.items()):
            node_num = node_num + 1
            nodes[node_num].append(centre)
            nodes[node_num].append(angle)
        node_num = len(nodes)
        for angle, centre in sorted(neg_node_angles.items()):
            node_num = node_num + 1
            nodes[node_num].append(centre)
            nodes[node_num].append(angle)
        for num, data in nodes.items():
            ip_image = cv2.putText(ip_image, str(num), tuple(data[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))

        if len(contourR) > 0:
            cR = []
            red_nodes = []
            for cont in contourR:
                cR.append(get_centre(cont))
            for centre in cR:
                min_dist = 0
                red_node = 0
                for num, data in nodes.items():
                    dist = math.sqrt((centre[0] - data[0][0])**2 + (centre[1] -
                    data[0][1])**2)
                    if num == 1:
                        min_dist = dist
                        red_node = num
                    elif dist < min_dist:
                        red_node = num
                        min_dist = dist
                red_nodes.append(red_node)

            for node in red_nodes:
                #print("Medical supply at: " + str(node))
                table.add_row(['Medical Aid', node])
                for num, data in sorted(nodes.items()):
                    if num == node:
                        red_nodes_data[num] = data
                        
            print(red_nodes_data) 

            if len(aruco_centre) > 0:
                for centre in cR:
                    angleR = calculate_angle(centre, aruco_centre, cW)
                    ip_image = cv2.putText(ip_image, str(angleR), tuple(centre), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
        else:
            print("Red coins not found!!!")

        if len(contourG) > 0:
            cG = []
            green_nodes = []
            for cont in contourG:
                cG.append(get_centre(cont))
            for centre in cG:
                min_dist = 0
                green_node = 0
                for num, data in nodes.items():
                    dist = math.sqrt((centre[0] - data[0][0])**2 + (centre[1] -
                    data[0][1])**2)
                    if num == 1:
                        min_dist = dist
                        green_node = num
                    elif dist < min_dist:
                        green_node = num
                        min_dist = dist
                green_nodes.append(green_node)

            for node in green_nodes:
                #print("Food supply at: " + str(node))
                table.add_row(['Food Supply', node])
                for num, data in sorted(nodes.items()):
                    if num == node:
                        green_nodes_data[num] = data
                        
            print(green_nodes_data) 
            if len(aruco_centre) > 0:
                for centre in cG:
                    angleG = calculate_angle(centre, aruco_centre, cW)
                    ip_image = cv2.putText(ip_image, str(angleG), tuple(centre), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
        else:
            print("Green coins not found!!!")
        node_seq = []
        red_angle_seq = []
        green_angle_seq = defaultdict(int)
        for node, data in red_nodes_data.items():
            red_angle_seq.append(abs(data[1]))
        red_angle_seq.sort()
        print(red_angle_seq)
        for angle in red_angle_seq:
            for node, data in red_nodes_data.items():
                if angle == abs(data[1]):
                    node_seq.append(node)
        if len(node_seq) > 0:
            last_red = red_nodes_data[node_seq[-1]][1]
            print(last_red)
            for node, data in green_nodes_data.items():
                if (last_red > 0 and data[1] > 0) or (last_red < 0 and data[1] < 0):
                    green_angle_seq[round(abs(abs(last_red) - abs(data[1])), 2)] = node
                elif (last_red > 0 and data[1] < 0) or (last_red < 0 and data[1] > 0):
                    diff = round(abs(last_red) + abs(data[1]), 2)
                    if diff > 180:
                        green_angle_seq[round(360 - diff, 2)] = node
                    else: 
                        green_angle_seq[round(diff, 2)] = node
            print(green_angle_seq)
            for angle, node in sorted(green_angle_seq.items()):
                node_seq.append(node)
        else:
            green_angle_seq = []
            for node, data in green_nodes_data.items():
                green_angle_seq.append(abs(data[1]))
            green_angle_seq.sort()
            print(green_angle_seq)
            for angle in green_angle_seq:
                for node, data in green_nodes_data.items():
                    if angle == abs(data[1]):
                        node_seq.append(node)

        node_seq.append(0)
        #print(node_seq)
        '''
        command_seq = []
        i = 1
        while i < len(node_seq):
            src = node_seq[i - 1]
            dest = node_seq[i]
            diff = dest - src
            if abs(diff) < 5:
                command_seq.append(diff)
            else:
                command_seq.append(abs(diff) - 9)
            i = i + 1

        print(command_seq)

        print("#######################################")
        '''
    else:
        print("White centre not found!!!")

    print(table)
    cv2.imshow("Result", ip_image)
    return node_seq
    
    
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
    while(ret):
        ret, frame = cap.read()
        ## display to see if the frame is correct
        cv2.imshow("window", frame)
        cv2.waitKey(int(1000/fps));
        ## calling the algorithm function
        node_seq = process(frame)
        print(node_seq)
        ports = serial_ports()
        for port in ports:
            try:
                s = serial.Serial(port)
                for node in node_seq:
                    response = None
                    while True:
                        s.write(str(node).encode("utf-8"))
                        print(str(node) + " sent to Bot")
                        time.sleep(1)
                        s.reset_input_buffer()
                        s.reset_output_buffer()
                        response = s.read(1)
                        if response is not None:
                            print(response)
                            break
                s.close()
            except (OSError, serial.SerialException):
                pass
            

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

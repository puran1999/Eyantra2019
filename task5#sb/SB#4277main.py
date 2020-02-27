'''
* Team Id : 4277
* Author List : Sudhanshu Dubey, Puran Singh
* Filename: SB#4277main.py
* Theme: Supply Bot -- Specific to eYRC
* Functions: find_correct_contours(list, string, list), get_centre(list),
*            get_nodes(image), calculate_angle(list, list, list), process(image),
*            main(), show(boolean, VideoCapture Object, float), communication()
* Global Variables: node_sequence, Lower_limit_Green, Upper_limit_Green, Lower_limit_Red, 
*                    Upper_limit_Red, Lower_limit_White, Upper_limit_White
'''

######################
## Essential libraries
######################
import cv2
import numpy as np
import math
import time
from collections import defaultdict
from prettytable import PrettyTable
from threading import Thread

from aruco_lib import *
from port_detection import *

# Global variables defining the colour ranges
Lower_limit_Green = (25, 62, 110)
Upper_limit_Green = (76, 255, 255)
Lower_limit_Red = (164, 150, 150)
Upper_limit_Red = (180, 255, 255)
Lower_limit_White = (0, 0, 170)
Upper_limit_White = (180, 50, 255)

# Global variable to store the node sequence that the bot should travel.
node_sequence = []
loop = 0

'''
* Function Name: find_correct_contours
* Input: contour -> The list of contours from which the required ones are to be
*                 picked
*        name -> The keyword specifying the type of contour
*        im_shape -> Shape of the image or centre of frame.
* Output: detected -> List of contours that pass the required tests
* Logic: Function to pick the required contours from the detected ones.
* Example Call: detected_contours = find_correct_contours(contours, 'G',
* im_shape)
'''
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

'''
* Function Name: get_centre
* Input: contour -> Contour for which centre is to be found 
* Output: Returns list of centre coordinate in the [x, y] format 
* Logic: Function to get the centre coordinates of a contour. It uses moments
*        to find the centre. 
* Example Call: centre = get_centre(contour_name) 
'''
def get_centre(contour):
    M = cv2.moments(contour)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return [cX, cY]
    except ZeroDivisionError:
        return [0,0]

'''
* Function Name: get_nodes
* Input: pic -> Image from which nodes are to be found 
* Output: contourW -> list of contours classified as nodes 
* Logic: Function to get the list of nodes. It erodes the pic and then detects
*        remaining white contours.
* Example Call: node_contour = get_nodes(image) 
'''
def get_nodes(pic):
    erosion_size = 3
    erosion_type = cv2.MORPH_CROSS
    element = cv2.getStructuringElement(erosion_type, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    erosion_dst = cv2.erode(pic, element)
    maskW = cv2.inRange(erosion_dst, Lower_limit_White, Upper_limit_White)
    contourW,hW = cv2.findContours(maskW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contourW

'''
* Function Name: calculate_angle
* Input: pt1 -> point 1
*        pt2 -> point 2
*        centre -> contre point of angle 
* Output: angle -> Calculated angles between point 1, point 2 w.r.t centre 
* Logic: Function to calculate angle between given 2 points w.r.t given centre.
* Example Call: angle = calculate_angle(pointA, pointB, centre) 
'''
def calculate_angle(pt1, pt2, centre):
    angle = math.degrees(math.atan2(pt1[1] - centre[1], pt1[0] - centre[0]) -
    math.atan2(pt2[1] - centre[1], pt2[0] - centre[0]))
    if angle < 0:
        angle = angle + 360
    if angle > 180:
        angle = angle - 360
    angle = round(angle,2)
    return angle

'''
* Function Name: process
* Input: ip_image -> Frame of the video to be processed
* Output: node_seq -> A list of nodes to be followed by the bot
* Logic: The function to process the frame and calculate the sequence of nodes
*        to follow.
* Example Call: seq = process(frame) 
'''
def process(ip_image):
    global loop
    pic = cv2.cvtColor(ip_image, cv2.COLOR_BGR2HSV)
    table = PrettyTable(['Node Type','Node Number',])
    node_seq = []

    maskG = cv2.inRange(pic, Lower_limit_Green, Upper_limit_Green)
    maskR = cv2.inRange(pic, Lower_limit_Red, Upper_limit_Red)
    maskW = cv2.inRange(pic, Lower_limit_White, Upper_limit_White)
    
    contourG,h=cv2.findContours(maskG, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourR,h=cv2.findContours(maskR, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourW,h=cv2.findContours(maskW, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    copyW = get_nodes(pic)
    
    contourG = find_correct_contours(contourG, 'G', ip_image.shape)
    contourW = find_correct_contours(contourW, 'IW', ip_image.shape)
    contourR = find_correct_contours(contourR, 'R', ip_image.shape)

    det_aruco_list = detect_Aruco(ip_image)
    aruco_centre = []
    if det_aruco_list:
        robot_state = calculate_Robot_State(ip_image,det_aruco_list)
        ip_image = mark_Aruco(ip_image, det_aruco_list)
        keys = robot_state.keys()
        for key in keys:
            aruco_data = robot_state.get(key)
            aruco_centre = aruco_data[1:3]
            
        if len(contourW) > 0:
            cW = get_centre(contourW[0])
            xw, yw = tuple(cW)
            ip_image = cv2.line(ip_image,(xw,yw),(xw,yw),(0,0,255),2)
            contOW = find_correct_contours(copyW, 'OW', cW)
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
                    if len(aruco_centre) > 0:
                        node_angle = calculate_angle(node, aruco_centre, cW)
                        if abs(node_angle) > 10:
                            if node_angle > 0:
                                pos_node_angles[node_angle] = node
                            else:
                                neg_node_angles[node_angle] = node
                           
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
                    table.add_row(['Medical Aid', node])
                    for num, data in sorted(nodes.items()):
                        if num == node:
                            red_nodes_data[num] = data

                if len(aruco_centre) > 0:
                    for centre in cR:
                        angleR = calculate_angle(centre, aruco_centre, cW)                    
            else:
                if loop != 1:
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
                    table.add_row(['Food Supply', node])
                    for num, data in sorted(nodes.items()):
                        if num == node:
                            green_nodes_data[num] = data
                        
                if len(aruco_centre) > 0:
                    for centre in cG:
                        angleG = calculate_angle(centre, aruco_centre, cW)
            else:
                if loop != 1:
                    print("Green coins not found!!!")
                    
            node_seq = []
            red_angle_seq = []
            green_angle_seq = defaultdict(int)
            for node, data in red_nodes_data.items():
                red_angle_seq.append(abs(data[1]))
            red_angle_seq.sort()
            for angle in red_angle_seq:
                for node, data in red_nodes_data.items():
                    if angle == abs(data[1]):
                        node_seq.append(node)
            if len(node_seq) > 0:
                last_red = red_nodes_data[node_seq[-1]][1]
                for node, data in green_nodes_data.items():
                    if (last_red > 0 and data[1] > 0) or (last_red < 0 and data[1] < 0):
                        green_angle_seq[round(abs(abs(last_red) - abs(data[1])), 2)] = node
                    elif (last_red > 0 and data[1] < 0) or (last_red < 0 and data[1] > 0):
                        diff = round(abs(last_red) + abs(data[1]), 2)
                        if diff > 180:
                            green_angle_seq[round(360 - diff, 2)] = node
                        else: 
                            green_angle_seq[round(diff, 2)] = node
                for angle, node in sorted(green_angle_seq.items()):
                    node_seq.append(node)
            else:
                green_angle_seq = []
                for node, data in green_nodes_data.items():
                    green_angle_seq.append(abs(data[1]))
                green_angle_seq.sort()
                for angle in green_angle_seq:
                    for node, data in green_nodes_data.items():
                        if angle == abs(data[1]):
                            node_seq.append(node)

            node_seq.append(0)
        else:
            if loop != 1:
                print("White centre not found!!!")
    else:
        if loop != 1:
            print("ArUCO not found!!!")

    if loop != 1:
        loop = 1
        print(table)
    cv2.imshow("Result", ip_image)
    return node_seq

'''
* Function Name: communication
* Input: 
* Output: Sends node numbers to bot sequentially. 
* Logic: The function to communicate with the bot using serial module. 
* Example Call: seq = communication() 
'''   
def communication():
    #time.sleep(20)
    ports = serial_ports()
    for port in ports:
        try:
            s = serial.Serial(port)
            for node in node_sequence:
                response = None
                while True:
                    s.write(str(node).encode("utf-8"))
                    print(str(node) + " sent to bot")
                    time.sleep(5)
                    s.reset_input_buffer()
                    s.reset_output_buffer()
                    if node == 0:
                        _thread.exit()
                    #while response != '0':
                    response = s.read(1).decode("utf-8")
                    print(response)
                    if response is not None:
                        #print(response)
                        break
            s.close()
        except (OSError, serial.SerialException):
            pass
'''
* Function Name: show
* Input: ret -> retrieval value telling if the frame is available or not.
*        cap -> VideoCapture object containing the frame
*        fps -> Frame per second rate
* Output: Displays live capture of the frames.
* Logic: The function to capture frame, display them and pass to process
*        function
* Example Call: show(ret, cap, fps)
''' 
def show(ret, cap, fps):
    while(ret):
        ret, frame = cap.read()
        ## display to see if the frame is correct
        cv2.imshow("window", frame)
        cv2.waitKey(int(1000/fps));
        ## calling the algorithm function
        global node_sequence
        node_sequence = process(frame)
'''
* Function Name: main
* Input:
* Output:
* Logic: The main function to read the frames and call show and communication
*        functions in seperate threads.
* Example Call: main() 
''' 
def main():
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
    show_thread = Thread(target = show, args= (ret,cap,fps))
    com_thread = Thread( target = communication)
    show_thread.start()
    while True:
        if len(node_sequence) > 0:
            print(node_sequence)
            com_thread.start()
            break
            

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

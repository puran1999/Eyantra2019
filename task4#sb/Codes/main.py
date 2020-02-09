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
from collections import defaultdict
from prettytable import PrettyTable

from aruco_lib import *
from port_detection import *

########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Images'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))
count = 0
LLG = (25, 62, 110)#GREEN HSV=(60,255,255)
ULG = (76, 255, 255)
LLR = ( 164, 150, 150)#RED HSV=(0,255,255)
ULR = ( 180, 255, 255)
LLW = ( 0, 0, 170)#WHITE HSV=(0,0,255)
ULW = ( 180, 50, 255)
#    LLO = (161, 103, 0)
#    ULO = (180, 255, 255) 
LLP = (122, 67, 109)
ULP = (151, 135, 255) 
#    LLB = (104, 44, 169)
#    ULB = (121, 118, 255) 

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
            if 0.06 < ratio < 0.25 and 7 < equi_diameter < 15:  #9.5 to 12
                detected.append(cont)
        elif name == 'IW':
            H, W, CH = im_shape
            x, y = center
            if 0.4*W < x < 0.6*W and 0.4*H < y < 0.6*H:
                if 0.06 < ratio < 0.25 and  7 < equi_diameter < 15:  #9.5 to 12
                    detected.append(cont)
        elif name == 'OW':
            inner = 50
            outer = 245
            dist_min = 140
            dist_max = 240
            X, Y = tuple(im_shape)
            x, y = tuple(get_centre(cont))
            dist = math.sqrt((x - X)**2 + (y - Y)**2)
            #if W - outer < x < W + outer and H - outer < y < H + outer:
            if dist_max > dist > dist_min:
                detected.append(cont)

    return detected

def get_centre(contour):
    M = cv2.moments(contour)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return [cX, cY]
    except ZeroDivisionError:
        return [0,0]

def is_inside(OuterContour, InnerContour):
    inside = []
    for cont in InnerContour:
        if cv2.pointPolygonTest(OuterContour, tuple(cont[0][0]), False) > 0:
            inside.append(cont)
    return inside

def is_outside(OuterContour, InnerContour):
    outside = []
    for cont in InnerContour:
        if cv2.pointPolygonTest(OuterContour, tuple(cont[0][0]), False) < 0:
            outside.append(cont)
    return outside


def find_approx_contour(contour):
    approx = []
    for cont in contour:
        accuracy=0.005*cv2.arcLength(cont,True)
        approx.append(cv2.approxPolyDP(cont,accuracy,True))
    return approx

def get_largest_contour(contour):
    maxArea = cv2.contourArea(contour[0])
    largestContour = contour[0]
    for cont in contour:
        if(cv2.contourArea(cont) > maxArea):
            largestContour = cont
    return largestContour

def get_nodes(pic):
    erosion_size = 3
    erosion_type = cv2.MORPH_CROSS
    element = cv2.getStructuringElement(erosion_type, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    erosion_dst = cv2.erode(pic, element)
    maskW = cv2.inRange(erosion_dst, LLW, ULW)
    contourW,hW = cv2.findContours(maskW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contourW

def calculate_angle(pt1, pt2, centre):
    angle = math.degrees(math.atan2(pt1[1] - centre[1], pt1[0] - centre[0]) -
    math.atan2(pt2[1] - centre[1], pt2[0] - centre[0]))
    if angle < 0:
        angle = angle + 360
    if angle > 180:
        angle = angle - 360
    angle = round(angle,2)
    return angle

############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    angle = -1
    pic = cv2.cvtColor(ip_image, cv2.COLOR_BGR2HSV)
    t = PrettyTable(['Node Type', 'Node Number'])

    maskG = cv2.inRange(pic, LLG, ULG)
    maskR = cv2.inRange(pic, LLR, ULR)
    maskW = cv2.inRange(pic, LLW, ULW)
    maskP = cv2.inRange(pic, LLP, ULP)
#    maskB = cv2.inRange(pic, LLB, ULB)

    contourG,hG=cv2.findContours(maskG, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourR,hR=cv2.findContours(maskR, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourW,hW=cv2.findContours(maskW, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contourP,hP=cv2.findContours(maskP, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#    contourB,hB=cv2.findContours(maskB, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    copyW = get_nodes(pic)

    #contOW =  is_inside(contourP, contOW)
    contourG = find_correct_contours(contourG, 'G', ip_image.shape)
    contourW = find_correct_contours(contourW, 'IW', ip_image.shape)
    contourR = find_correct_contours(contourR, 'R', ip_image.shape)
   
    det_aruco_list = detect_Aruco(ip_image)
    aruco_centre = []
    if det_aruco_list:
        aruco_id = det_aruco_list.keys()
        for aid in aruco_id:
            corners = det_aruco_list.get(aid)
        cv2.circle(ip_image,tuple(corners[0]),1,(0,255,255),2)
        cv2.circle(ip_image,tuple(corners[1]),1,(255,0,255),2)
        robot_state = calculate_Robot_State(ip_image,det_aruco_list)
        ip_image = mark_Aruco(ip_image, det_aruco_list)
        keys = robot_state.keys()
        for key in keys:
            aruco_data = robot_state.get(key)
            aruco_centre = aruco_data[1:3]
    
    cv2.drawContours(ip_image, contourG, -1, (255, 0, 0), 2)
    cv2.drawContours(ip_image, contourR, -1, (255, 255, 0), 2)
    cv2.drawContours(ip_image, contourW, -1, (0, 255, 255), 2)
   #cv2.drawContours(ip_image, contourP, -1, (0, 255, 0), 2)
    #print(contOW)
#    cv2.drawContours(ip_image, contourB, -1, (255, 155, 10), 2)
#    for cont,hier in zip(contourO,hO[0]):
#        if hier[3] != -1 and hier[2] == -1:
#            print(hier)
#            cv2.drawContours(ip_image, cont, -1, (255, 0, 255), 2)
    if len(contourW) > 0:
        cW = get_centre(contourW[0])
        xw, yw = tuple(cW)
        cv2.circle(ip_image,(xw, yw),140,(0,255,0),1)
        cv2.circle(ip_image,(xw, yw),240,(0,255,0),1)
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
                #cv2.circle(ip_image,tuple(node),1,(0,0,255),2)
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
                    #print(str(num) + ":" + str(dist))
                red_nodes.append(red_node)
            
            for node in red_nodes:
                #print("Medical supply at: " + str(node))
                t.add_row(['Medical Aid', node])
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
                    #print(str(num) + ":" + str(dist))
                green_nodes.append(green_node)

            for node in green_nodes:
                #print("Food supply at: " + str(node))
                t.add_row(['Food Supply', node])
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
        #node_seq.append(1)
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
    print(t)
    cv2.imshow("Result", ip_image)
    #cv2.waitKey(0);
    #cv2.imwrite("../Generated/generated" + str(count + 1) + ".jpg", ip_image)
    ## Your Code goes here
    ###########################
    return node_seq


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
        node_seq = process(ip_image)
        print(node_seq)
        ports = serial_ports()
        print(ports)
        for port in ports:
            try:
                s = serial.Serial(port)
                for node in node_seq:
                    num = 0
                    while num < 500:
                        s.write(str(node).encode("utf-8"))
                        num = num + 1
                    print(str(node) + " sent to Bot")
                    response = s.read(1)
                    print(response)
                s.close()
            except (OSError, serial.SerialException):
                pass

        '''
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
    '''



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

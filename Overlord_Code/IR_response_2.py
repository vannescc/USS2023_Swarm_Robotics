#seems to work well, inacurate but otherwise fine

#import imp
import os
import time
import numpy
from math import atan2, degrees, radians, pi, cos, sin, sqrt, atan, tan
import cv2
import sys
import tags
import apriltag
import socket
import random
import termios
import csv
from select import select
import signal

UDP_IP = "172.24.87.204"#"172.25.255.118"#"192.168.7.2"
# UDP_IP = "172.24.52.22"
UDP_PORT = 5005#8080
UDP_IP_OVERLORD = "172.24.87.109"
UDP_PORT_OVERLORD = 8080
socksender = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
socksender.connect((UDP_IP,UDP_PORT))
socksender.setblocking(0)

sockreceiver = socket.socket(socket.AF_INET,
                             socket.SOCK_DGRAM)
sockreceiver.bind((UDP_IP_OVERLORD, UDP_PORT_OVERLORD))
sockreceiver.setblocking(0)

global botHeight
botHeight = 17.3 #cm
global cameraHeight
cameraHeight = 112 #cm
global IRoffset
IRoffset = 4 #cm

#roughly 194 pixels/50 cm with current setup on 5/23/2023 = 3.88 #200/50 = 4
global pix_cm
pix_cm = 4 #subject to small fluxuations

class TimeOutException(Exception):
    pass

def alarm_handler(signum, frame):
    print("Alarm signal Received")
    raise TimeOutException()

def mark_debug(frame, x_b, y_b, angles, x_p, y_p, x_n, y_n, ax_p, ay_p, R, c_x, c_y, m_x, m_y, f_x, f_y, c_x_f, c_y_f, R_f, intersections, point_f, x_b_par, y_b_par):
    out = frame
    dimensions = out.shape
    height = dimensions[0]
    width = dimensions[1]
    #print([height,width])
    cv2.rectangle(out, (0,0), (width-1,height-1), (0,0,255), 1) #image border
    cv2.rectangle(out, (40,40), (width-41,height-41), (0,0,255), 1) #abort border
    cv2.rectangle(out, (80,80), (width-81,height-81), (0,0,255), 1) #valid border
    cv2.rectangle(out, (120,120), (width-121,height-121), (0,255,0), 1) #Salgorithm border
    if(point_f != None):
        cv2.circle(out, point_f, 5, (255, 0, 0), -1)
    if(R_f != None):
        circ_debug(frame, (int(c_x_f), int(c_y_f)), int(abs(R_f)), intersections)
    
    #print(calc_dist((m_x, m_y), (x_b[0], y_b[0])))
    
    cv2.circle(out, (int(m_x), int(m_y)), 5, (255, 0, 0), -1)
    """
    if(RL != None):
        cv2.circle(out, (int(RL*sin(radians(angles[0]-90)) + x_b), int(RL*cos(radians(angles[0]-90)) + y_b)), int(RL), (0, 0, 255), 1)
    if(RR != None):
        cv2.circle(out, (int(RR*sin(radians(angles[0]+90)) + x_b), int(RR*cos(radians(angles[0]+90)) + y_b)), int(RR), (0, 0, 255), 1)
    """
    if(f_x != None and f_y != None):
        cv2.circle(out, (int(f_x), int(f_y)), 5, (0, 0, 255), -1)

    for i in range(len(x_b)):
        cv2.circle(out, (int(x_b[i]), int(y_b[i])), 5, (0, 255, 0), -1)
        cv2.circle(out, (int(x_b_par[i]), int(y_b_par[i])), 5, (0, 255, 255), -1)
        if(x_p[i] != None and y_p[i] != None):
            cv2.line(out, (int(x_b[i]), int(y_b[i])), (int(x_p[i]), int(y_p[i])), (0, 0, 255), 1)
    if(x_n != None and y_n != None):
        cv2.circle(out, (int(x_n), int(y_n)), 15, (255, 0, 0), 1)
        if(ax_p != None and ay_p != None):
            cv2.line(out, (int(x_n), int(y_n)), (int(ax_p), int(ay_p)), (0, 0, 255), 1)
    if(R != None and c_x != None and x_n != None):
        cv2.circle(out, (int(c_x), int(c_y)), abs(int(R)), (255, 0, 0), 1)
#         cv2.line(out, (int(x_n), int(y_n)), (int(c_x), int(c_y)), (0, 0, 255), 1)
        
    cv2.imshow("DEBUG", out)

def mark_debug_basic(frame, x, y, x_p, y_p, x_c, y_c, t_p, t_ang, x_IR, y_IR, x_c_val, y_c_val):
    out = frame
    if(t_p != None):
        cv2.circle(out, (int(t_p[0]), int(t_p[1])), 10, (0, 0, 255), 1)
        if(t_ang != None):
            cv2.line(out, (int(t_p[0]), int(t_p[1])), (int(t_p[0] + sin(radians(t_ang))*40), int(t_p[1] + cos(radians(t_ang))*40)), (0, 0, 255), 1)
        
    for i in range(len(x)):
        cv2.circle(out, (int(x[i]), int(y[i])), 5, (0, 255, 0), -1)
        if((len(x_p) == len(x)) and x_p[i] != None and y_p[i] != None):
            cv2.line(out, (int(x[i]), int(y[i])), (int(x_p[i]), int(y_p[i])), (0, 0, 255), 1)
    
    for i in range(len(x_c)):
        cv2.putText(out, str(i), (int(x_c[i]), int(y_c[i])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
        if(i < len(x_c) - 1):
            cv2.line(out, (int(x_c[i]), int(y_c[i])), (int(x_c[i+1]), int(y_c[i+1])), (255, 0, 0), 2)
            cv2.line(out, (int(x_c_val[i]), int(y_c_val[i])), (int(x_c_val[i+1]), int(y_c_val[i+1])), (255, 0, 0), 2)
        else:
            cv2.line(out, (int(x_c[i]), int(y_c[i])), (int(x_c[0]), int(y_c[0])), (255, 0, 0), 2)
            cv2.line(out, (int(x_c_val[i]), int(y_c_val[i])), (int(x_c_val[0]), int(y_c_val[0])), (255, 0, 0), 2)
    
    if(x_IR != None):
        cv2.circle(out, (int(x_IR), int(y_IR)), 5, (255, 0, 0), -1)
    
    """
    kernel = numpy.ones((3,3), numpy.uint8)
    dilated_img = cv2.dilate(linesFrame, kernel, iterations = 4)
    
    edges = cv2.Canny(image=frame, threshold1=100, threshold2=200)
    edgesLines = cv2.Canny(image=dilated_img, threshold1=100, threshold2=200)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = numpy.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 3, 0.04)
    dst = cv2.dilate(dst, None)
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = numpy.uint8(dst)
    
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray, numpy.float32(centroids), (5, 5), (-1, -1), criteria)
    
    res = numpy.hstack((centroids, corners))
    res = numpy.int0(res)
    frame[res[:, 1], res[:, 0]] = [0, 0, 255]
    frame[res[:, 3], res[:, 2]] = [0, 255, 0]
    
    if(T > 1):
        ret, thresh = cv2.threshold(edgesLines, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours[3], -1, (0, 255, 0), 3)
        maxArea = 0
        maxcnt = None
        for cnt in contours:
             area = cv2.contourArea(cnt)
             if(area > maxArea):
                maxArea = area
                maxcnt = cnt
#         cnt = contours[0]
        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        x_temp, y_temp, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(out, (x_temp, y_temp), (x_temp + w, y_temp + h), (0, 0, 255), 2)
    
    
    minLineLength = 50
    maxLineGap = 10
    lines = cv2.HoughLinesP(edges, 1, pi/180, 100, minLineLength, maxLineGap)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(linesFrame, (x1, y1), (x2, y2), (255, 255, 255), 4)
    """     
    """
    if(T > 1):
        lines = cv2.HoughLinesP(edgesLines, 1, pi/180, 100, minLineLength, maxLineGap)
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
    """
    
    
#     cv2.imshow('Canny Edge Detection', edges)
#     cv2.imshow('Lines', out)
    cv2.imshow("DEBUG", out)
#     cv2.imshow("Lines", linesFrame)
#     cv2.imshow("Lines Negative", linesFrameNeg)
#     cv2.imshow("Lines Filled", dilated_img)
    
def calc_slope(x1, y1, x2, y2):
    #if (x2 - x1 == 0):
    #    return ((float(y1) - float(y1)) / 0.1)
    return ((float(y1) - float(y2)) / (float(x1) - float(x2)))

def calc_dist(target, bot):
    return abs(sqrt(float((pow((target[0] - bot[0]), 2) + pow((target[1] - bot[1]), 2)))))

def calc_ang_between(p1, p2): #fixed it, something about this coordinate system messes with trig functions
    return float(degrees(atan2((p2[0] - p1[0]), (p2[1] - p1[1]))))
#     return float(degrees(atan2((p2[0] - p1[0]), (p2[1] - p1[1]))))
def calc_ang_between_radians(p1, p2): #fixed it, something about this coordinate system messes with trig functions
    return float(atan2((p2[0] - p1[0]), (p2[1] - p1[1])))

def norm_angle(angle):
    while(angle < -180):
        angle = angle + 360
    while (angle > 180):
        angle = angle - 360
    return angle

#takes a point at the top of a robot and the border of the camera's image and returns the point where the robot sits accounting for the hiehgt of the robot
def account_for_parallax(point, cameraBorder):
    x, y = point
    camera_x = (cameraBorder[0][0] + cameraBorder[1][0])/2
    camera_y = (cameraBorder[0][1] + cameraBorder[1][1])/2
    dist_x_b = x - camera_x
    dist_y_b = y - camera_y
    dist_x_g = dist_x_b - dist_x_b * (botHeight/cameraHeight)
    dist_y_g = dist_y_b - dist_y_b * (botHeight/cameraHeight)
    return (dist_x_g + camera_x, dist_y_g + camera_y)

def offset_corners(offset_corners_x, offset_corners_y, offset):
    corners_x = []
    corners_y = []
    tempX = None
    tempY = None
    dist = None
    rise = None
    slope = None
    for i in range(len(offset_corners_x)):
        if(i < len(offset_corners_x) - 1):
            dist = calc_dist((offset_corners_x[i], offset_corners_y[i]), (offset_corners_x[i+1], offset_corners_y[i+1]))
            rise = offset_corners_y[i] - offset_corners_y[i+1]
            run = offset_corners_x[i] - offset_corners_x[i+1]
        else:
            dist = calc_dist((offset_corners_x[i], offset_corners_y[i]), (offset_corners_x[0], offset_corners_y[0]))
            rise = offset_corners_y[i] - offset_corners_y[0]
            run = offset_corners_x[i] - offset_corners_x[0]
        tempY = offset_corners_y[i] + rise/dist*offset
        tempX = offset_corners_x[i] + run/dist*offset
#         print([i, calc_dist((offset_corners_x[i], offset_corners_y[i]), (tempX, tempY))])
        if(i > 0):
            dist = calc_dist((offset_corners_x[i], offset_corners_y[i]), (offset_corners_x[i-1], offset_corners_y[i-1]))
            rise = offset_corners_y[i] - offset_corners_y[i-1]
            run = offset_corners_x[i] - offset_corners_x[i-1]
        else:
            dist = calc_dist((offset_corners_x[i], offset_corners_y[i]), (offset_corners_x[len(offset_corners_x) - 1], offset_corners_y[len(offset_corners_x) - 1]))
            rise = offset_corners_y[i] - offset_corners_y[len(offset_corners_x) - 1]
            run = offset_corners_x[i] - offset_corners_x[len(offset_corners_x) - 1]
        tempY = tempY + rise/dist*offset
        tempX = tempX + run/dist*offset
        """
        if(i % 2 == 0):
            if(i < len(offset_corners_x) - 1):
                if(offset_corners_x[i] < offset_corners_x[i+1]):
                    slope = calc_slope(offset_corners_x[i], offset_corners_y[i], offset_corners_x[i+1], offset_corners_y[i+1])
                else:
                    slope = calc_slope(offset_corners_x[i+1], offset_corners_y[i+1], offset_corners_x[i], offset_corners_y[i])
            else:
                if(offset_corners_x[i] < offset_corners_x[0]):
                    slope = calc_slope(offset_corners_x[i], offset_corners_y[i], offset_corners_x[0], offset_corners_y[0])
                else:
                    slope = calc_slope(offset_corners_x[0], offset_corners_y[0], offset_corners_x[i], offset_corners_y[i])
        theta = atan(1/slope)
        tempX = offset_corners_x[i] + sin(theta)*10
        tempY = offset_corners_y[i] + cos(theta)*10
        """
        """
        else:
            tempX = offset_corners_x[i] + sin(theta)*10
            tempY = offset_corners_y[i] - cos(theta)*10
        """
#         theta = atan(-1/slope)
#         tempX = tempX + sin(theta)*10
#         tempY = tempY + cos(theta)*10
        corners_x.append(tempX)
        corners_y.append(tempY)
        
            
#         if(offset_corners_x[i] > center)#continue here on wednesday, maybe use the lines between the corners and not the center
    """
    for i in range(len(offset_corners_x)):
        dist = calc_dist((offset_corners_x[i], offset_corners_y[i]), (center[0], center[1]))
        corners_x.append((offset_corners_x[i] - center[0]) / dist * offset + offset_corners_x[i])
        corners_y.append((offset_corners_y[i] - center[1]) / dist * offset + offset_corners_y[i])
    """
    return (corners_x, corners_y)

def send_to_bot_s(ang, speed, T):
    Message = ""
    Message = Message + str(2) + ", " + str(ang) + ", " + str(speed) + ", " + str(T)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

def send_to_bot_s2(ang1, speed, T, ang2):
    Message = ""
    Message = Message + str(3) + ", " + str(ang1) + ", " + str(speed) + ", " + str(T) + ", " + str(ang2)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

def send_to_bot_IR_collect(num_data_points):
    Message = ""
    Message = Message + str(4) + ", " + str(num_data_points)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

def send_to_bot_IR_send():
    Message = ""
    Message = Message + str(5)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue
        
def send_to_bot_stop():
    Message = ""   
    Message = Message + str(0) + ", " + str(0) + ", " + str(0) + ", " + str(0)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

def dist_to_border(x_b, y_b, angle, border): #chat GPT helped with this function
    x_p = x_b + sin(radians(angle))*40
    y_p = y_b + cos(radians(angle))*40
    c_slopes = []
    maxDist = 0
    for i in range(len(border[0])):
        if(i < len(border[0]) - 1):
            rise = border[1][i] - border[1][i+1]
            run = border[0][i] - border[0][i+1]
        else:
            rise = border[1][i] - border[1][0]
            run = border[0][i] - border[0][0]
        c_slopes.append(rise/run)
    if(x_b != None and y_b != None and x_p != None and y_p != None and x_p != x_b):
        slope_b = (y_p-y_b)/(x_p-x_b)
        dist = []
        for i in range(len(border[0])):
            x = ((border[1][i] - y_b + (slope_b * x_b) - (c_slopes[i] * border[0][i])) / (slope_b - c_slopes[i]))
            y = slope_b * (x - x_b) + y_b
            if(numpy.sign(x_b-x_p) == numpy.sign(x_b-x)):
                dist.append(calc_dist((x_b, y_b), (x, y)))
#             dist.append(calc_dist((x_b, y_b), (x, y)))
        if len(dist) == 0:
            maxDist = 0
        else:
            maxDist = min(dist)
            
    elif(x_b != None and y_b != None and x_p != None and y_p != None):
        dist = []
        for i in range(len(border[0])):
            x = x_b
            y = c_slopes[i] * (x - border[0][i]) + border[1][i]
            if(numpy.sign(y_b-y_p) == numpy.sign(y_b-y)):
                dist.append(calc_dist((x_b, y_b), (x, y)))
        if len(dist) == 0:
            maxDist = 0
        else:
            maxDist = min(dist)
        
#     print(maxDist)
    return maxDist

def rand_dist_in_border_at_angle(x_b, y_b, angle, border):
    return random.randrange(int(dist_to_border(x_b, y_b, angle, border)))

"""
def dist_to_border(x_b, y_b, angle, border): #chat GPT helped with this function
    x_p = x_b + sin(radians(angle))*40
    y_p = y_b + cos(radians(angle))*40
    c_slopes = []
    maxDist = 0
    for i in range(len(border[0])):
        if(i < len(border[0]) - 1):
            rise = border[1][i] - border[1][i+1]
            run = border[0][i] - border[0][i+1]
        else:
            rise = border[1][i] - border[1][0]
            run = border[0][i] - border[0][0]
        c_slopes.append(rise/run)
    if(x_b != None and y_b != None and x_p != None and y_p != None and x_p != x_b):
        slope_b = (y_p-y_b)/(x_p-x_b)
        dist = []
        for i in range(len(border[0])):
            x = ((border[1][i] - y_b + (slope_b * x_b) - (c_slopes[i] * border[0][i])) / (slope_b - c_slopes[i]))
            y = slope_b * (x - x_b) + y_b
            if(numpy.sign(x_b-x_p) == numpy.sign(x_b-x)):
                dist.append(calc_dist((x_b, y_b), (x, y)))
#             dist.append(calc_dist((x_b, y_b), (x, y)))
        if len(dist) == 0:
            maxDist = 0
        else:
            maxDist = min(dist)
            
    elif(x_b != None and y_b != None and x_p != None and y_p != None):
        dist = []
        for i in range(len(border[0])):
            x = x_b
            y = c_slopes[i] * (x - border[0][i]) + border[1][i]
            if(numpy.sign(y_b-y_p) == numpy.sign(y_b-y)):
                dist.append(calc_dist((x_b, y_b), (x, y)))
        if len(dist) == 0:
            maxDist = 0
        else:
            maxDist = min(dist)
        
#     print(maxDist)
    return maxDist
"""

def get_AoA(intersection, angle, border):
    temp_c_1 = [None] * 2
    temp_c_2 = [None] * 2
    for i in range(len(border[0])):
        temp_c_1 = (border[0][i], border[1][i])
        if(i < len(border[0]) - 1):
            temp_c_2 = (border[0][i+1], border[1][i+1])
        else:
            temp_c_2 = (border[0][0], border[1][0])
        if(((intersection[0] > temp_c_1[0] and intersection[0] < temp_c_2[0]) or (intersection[0] < temp_c_1[0] and intersection[0] > temp_c_2[0])) and ((intersection[1] > temp_c_1[1] and intersection[1] < temp_c_2[1]) or (intersection[1] < temp_c_1[1] and intersection[1] > temp_c_2[1]))):
            AoA = norm_AoA(calc_ang_between(temp_c_1, temp_c_2) - angle)
            return AoA
            
def norm_AoA(angle):
    while True:
        if(angle > 90):
            angle = angle - 180
        elif(angle < -90):
            angle = angle + 180
        else:
            return abs(angle)
        
def point_from_border_at_dist_and_ang(AoA, dist, border): #naive implimentation, not good
    border_point_1 = (border[0][0], border[1][0])
    border_point_2 = (border[0][1], border[1][1])
    x_total = 0
    y_total = 0
    num_points = 0
    for i in range(len(border[0])):
        x_total = x_total + border[0][i]
        y_total = y_total + border[1][i]
        num_points = num_points + 1
    center = (x_total/num_points, y_total/num_points)
    intersection = ((border_point_1[0] + border_point_2[0])/2, (border_point_1[1] + border_point_2[1])/2)
    intersection_ang = calc_ang_between(border_point_1, border_point_2) + AoA
    point = [None] * 2
    point[0] = intersection[0] + sin(radians(intersection_ang)) * dist
    point[1] = intersection[1] + cos(radians(intersection_ang)) * dist
    if(calc_dist(point, center) > calc_dist(intersection, center)):
        intersection_ang = calc_ang_between(border_point_1, border_point_2) - AoA
        point = [None] * 2
        point[0] = intersection[0] + sin(radians(intersection_ang)) * dist
        point[1] = intersection[1] + cos(radians(intersection_ang)) * dist
    
    angle = intersection_ang - 180
    return (point, angle)

def point_in_border(point, border):
    for i in range(len(border[0])):
        if(i < len(border[0]) - 1):
            edge = numpy.array((border[0][i+1] - border[0][i], border[1][i+1] - border[1][i]))
        else:
            edge = numpy.array((border[0][0] - border[0][i], border[1][0] - border[1][i]))
        point_vec = numpy.array((border[0][i], border[1][i])) - numpy.array(point)
        if(numpy.cross(edge, point_vec) <= 0):
            return False
    return True

def rand_point_in_border(border):
    min_x = min(border[0])
    max_x = max(border[0])
    min_y = min(border[1])
    max_y = max(border[1])
    while True:
        rand_point = ((random.uniform(min_x, max_x), random.uniform(min_y, max_y)))
        if(point_in_border(rand_point, border)):
            return rand_point

def main():
    MOVEMODE = 1
    DEBUG = True
    nCorners = 4
    x = [None] * (nCorners + 1)
    y = [None] * (nCorners + 1)
    x_b = None
    y_b = None
    x_b_par = None
    y_b_par = None
    x_IR = None
    y_IR = None
    x_p = [None] * (nCorners + 1)
    y_p = [None] * (nCorners + 1)
    angles = [None] * (nCorners + 1)
    ids = [None] * (nCorners + 1)
    x_c_offset = [None] * (nCorners + 1)
    y_c_offset = [None] * (nCorners + 1)
    x_c = [None] * (nCorners)
    y_c = [None] * (nCorners)
    x_c_val = [None] * (nCorners)
    y_c_val = [None] * (nCorners)
    c_last_id = [None] * nCorners
    c_last_x_offset = [None] * nCorners
    c_last_y_offset = [None] * nCorners
    CornerOffset = 12
    ValOffset = -20
    border_dist = None
    border_point = [None] * 2
    AoA = None
    CollectTime = 100
    t_AoA = 90
    Distances = []
    MAXDIST = 40
    distance_index = 0
    for i in range(MAXDIST + 1):
        Distances.append((MAXDIST - i) * pix_cm)
#     t_dist = 30*pix_cm
    t_p = None
    t_p_IR_off = None
    t_ang = None
    t_p_f = None
    t_ang_f = None
    Move_ready = True
    Collect_ready = False
    Reverse_ready = False
    frame = tags.get_frame()
    dimensions = frame.shape
    height = dimensions[0]
    width = dimensions[1]
#     SalgBorder = [[120, 120], [width-121, height-121]]
#     validBorder = [[80,80], [width-81, height-81]]
#     abortBorder = [[40,40], [width-41, height-41]]
    cameraBorder = [[0,0], [width-1, height-1]]
    UPDATETIMESTEP = 0.3
    cam_update_time = time.time() + UPDATETIMESTEP
    Waiting_for_update = False
    
    
     # read for tag centers until all tags have been found and localized
    while (any(elem is None for elem in x) or any(elem is None for elem in y)):
        frame = tags.get_frame()
        results = tags.find_tags(frame)
        centers = tags.extract_centers_new(results, nCorners + 1)#get_bot_centers(results, nCorners + 1)
        angles = tags.get_bot_angles(results, nCorners + 1)
        ids = tags.extract_ids(results)
        if(len(centers) == nCorners + 1):#not any(-1 in item for item in centers)):
            for i in range(nCorners + 1):
                if(centers[i][0] == -1):
                    continue
                x[i] = centers[i][0]
                y[i] = centers[i][1]
#                 x_b_par[i], y_b_par[i] = account_for_parallax((x_b[i], y_b[i]), cameraBorder)
                if(angles[i] != None):
                    x_p[i] = centers[i][0] + sin(radians(angles[i]))*40
                    y_p[i] = centers[i][1] + cos(radians(angles[i]))*40
    tempId = [None]*2
    tempX = None
    tempy = None
    lastId = -1
    x_c_offset = []
    y_c_offset = []
    already_added = []
    center = [None] * 2
    x_b = x[0]
    y_b = y[0]
    x_b_par, y_b_par = account_for_parallax((x_b, y_b), cameraBorder)
    x_IR = x_b_par + sin(radians(angles[0])) * IRoffset * pix_cm
    y_IR = y_b_par + cos(radians(angles[0])) * IRoffset * pix_cm
    
    while(len(x_c_offset) < nCorners):
        for i in range(len(ids)):
            if(ids[i] == 0 or i in already_added):
                continue
            elif(ids[i] != lastId):
                x_c_offset.append(x[i])
                y_c_offset.append(y[i])
                already_added.append(i)
            lastId = ids[i]
            
    c_last_id = ids
    c_last_x_offset = x_c_offset
    c_last_y_offset = y_c_offset
    
    center[0] = sum(x_c_offset)/len(x_c_offset)
    center[1] = sum(y_c_offset)/len(y_c_offset)
    
    x_c, y_c = offset_corners(x_c_offset, y_c_offset, CornerOffset)
    x_c_val, y_c_val = offset_corners(x_c_offset, y_c_offset, ValOffset)
    
    border_dist = dist_to_border(x_IR, y_IR, angles[0], (x_c, y_c))
    border_point[0] = x_IR + sin(radians(angles[0])) * border_dist
    border_point[1] = y_IR + cos(radians(angles[0])) * border_dist
    AoA = get_AoA(border_point, angles[0], (x_c, y_c))
    
    if(MOVEMODE == 1):
        t_p, t_ang = point_from_border_at_dist_and_ang(t_AoA, Distances[distance_index], (x_c, y_c))
    elif(MOVEMODE == 2):
        try:
            t_ang = random.randrange(-180, 180, 1)
            t_dist = rand_dist_in_border_at_angle(x_b_par, y_b_par, t_ang, (x_c_val, y_c_val))
            t_p = [None]*2
            t_p[0] = x_b_par + sin(radians(t_ang))*t_dist
            t_p[1] = y_b_par + cos(radians(t_ang))*t_dist
        except:
            if(dist_to_border(x_IR, y_IR, angles[0], (x_c, y_c)) < 10 or not point_in_border((x_IR, y_IR), (x_c, y_c))):
                Reverse_ready = True
            #the t_p does nothing, but its needed so the program doesn't crash
            t_p = center
#             t_p = rand_point_in_border((x_c_val, y_c_val))
            t_ang = calc_ang_between((x_b_par, y_b_par), t_p)
            t_dist = calc_dist((x_b_par, y_b_par), t_p)
        """
        if(point_in_border((x_b_par, y_b_par), (x_c_val, y_c_val))):
            t_ang = random.randrange(-180, 180, 1)
            t_dist = rand_dist_in_border_at_angle(x_b_par, y_b_par, t_ang, (x_c_val, y_c_val))
            t_p = [None]*2
            t_p[0] = x_b_par + sin(radians(t_ang))*t_dist
            t_p[1] = y_b_par + cos(radians(t_ang))*t_dist
        else:
            Reverse_ready = True
            #the t_p does nothing, but its needed so the program doesn't crash
            t_p = rand_point_in_border((x_c_val, y_c_val))
            t_ang = calc_ang_between((x_b_par, y_b_par), t_p)
            t_dist = calc_dist((x_b_par, y_b_par), t_p)
#             print(t_p)
#             print("oopsie")
        """
        
    t_p_IR_off = (t_p[0] - sin(radians(t_ang)) * IRoffset * pix_cm, t_p[1] - cos(radians(t_ang)) * IRoffset * pix_cm)
    
    frame_mod = frame
    dimensions = frame.shape
    height = dimensions[0]
    width = dimensions[1]
    done = False
    while not done:
        Missreading = False
        if (DEBUG == True):
            mark_debug_basic(frame, x, y, x_p, y_p, x_c, y_c, t_p_f, t_ang_f, x_IR, y_IR, x_c_val, y_c_val)
            #print("Code Location: Orienting Bots")
#             for i in range(len(x_b)):
                #print("bot#:" + str(i))
                #print("location: " + str(x_b[i]) + ", " + str(y_b[i]))
                #print("Target: " + str(x_t[i]) + ", " + str(y_t[i]))
                #print("------")
#                 mark_debug(frame_mod, x_b, y_b, angles, x_p, y_p, x_n, y_n, ax_p, ay_p, R, c_x, c_y, m_x, m_y, f_x, f_y, c_x_f, c_y_f, R_f, intersections, point_f, x_b_par, y_b_par)
#               mark_debug_angle(frame_mod, centers, results, nBots)
            #print("----------------")
            if cv2.waitKey(1) & 0xFF == ord('1'):
                break
        
        
        x_temp = []
        y_temp = []
        angles_temp = [None] * (nCorners + 1)
        x_p_temp = []
        y_p_temp = []
        signal.signal(signal.SIGALRM, alarm_handler)
        signal.alarm(1)
        try:
            frame = tags.get_frame()
            results = tags.find_tags(frame)
            corners = tags.get_two_corners(results, nCorners + 1)
            centers = tags.extract_centers_new(results, nCorners + 1)#get_bot_centers(results, nCorners + 1)
            ids = tags.extract_ids(results)
            if(len(results) == nCorners + 1):
                angles_temp = tags.get_bot_angles(results, nCorners + 1)
#                 ids = tags.extract_ids(results)
#                 print(ids)
        except TimeOutException as ex:
            print(ex)
        signal.alarm(0)
        
        """
        for i in range(nCorners + 1):
            if newAngles[i] != None:
                angles[i] = newAngles[i]
        """
        
        if(len(centers) == nCorners + 1):
            for i in range(nCorners + 1):
                if (centers[i][0] == -1):
                    continue
                x_temp.append(centers[i][0])
                y_temp.append(centers[i][1])
#                 x_b_par[i], y_b_par[i] = account_for_parallax((x_b[i], y_b[i]), cameraBorder)
                if(angles_temp[i] != None):
                    x_p_temp.append(centers[i][0] + sin(radians(angles[i]))*40)
                    y_p_temp.append(centers[i][1] + cos(radians(angles[i]))*40)
        
        """
        if(len(x_temp) == nCorners + 1 and len(x_p_temp)):
            x = x_temp
            y = y_temp
            angles = angles_temp
            if(angles[0] == None):
                print("oopsies")
            x_p = x_p_temp
            y_p = y_p_temp
        elif(ids[0] != 0):
            Missreading = True
            print("Missreading")
        """
        if(ids[0] == 0):
            x = x_temp
            y = y_temp
            if(angles_temp[0] != None):
                angles = angles_temp
            if(angles[0] == None):
                print("oopsies")
                print(angles_temp[0])
            x_p = x_p_temp
            y_p = y_p_temp
        else:
            Missreading = True
            print(angles_temp[0])
            print(time.time())
        tempId = [None]*2
        tempX = None
        tempy = None
        lastId = -1
        already_added = []
        x_b = x[0]
        y_b = y[0]
        x_b_par, y_b_par = account_for_parallax((x_b, y_b), cameraBorder)
        x_IR = x_b_par + sin(radians(angles[0])) * IRoffset * pix_cm
        y_IR = y_b_par + cos(radians(angles[0])) * IRoffset * pix_cm
        """
        x_c_offset = []
        y_c_offset = []
        while(len(x_c_offset) < nCorners):
            for i in range(len(ids)):
                if(ids[i] == 0 or i in already_added):
                    continue
                elif(ids[i] != lastId):
                    x_c_offset.append(x[i])
                    y_c_offset.append(y[i])
                    already_added.append(i)
                lastId = ids[i]
                
        tempX = None
        tempY = None
        for i in range(2):
            if(calc_dist((x_c_offset[i], y_c_offset[i]), (c_last_x_offset[i], c_last_y_offset[i])) > calc_dist((x_c_offset[i+2], y_c_offset[i+2]), (c_last_x_offset[i], c_last_y_offset[i]))):
                tempX = x_c_offset[i]
                tempY = y_c_offset[i]
                x_c_offset[i] = x_c_offset[i+2]
                y_c_offset[i] = y_c_offset[i+2]
                x_c_offset[i+2] = tempX
                y_c_offset[i+2] = tempY
                
        c_last_id = ids
        c_last_x_offset = x_c_offset
        c_last_y_offset = y_c_offset
        
        center[0] = sum(x_c_offset)/len(x_c_offset)
        center[1] = sum(y_c_offset)/len(y_c_offset)
        
        x_c, y_c = offset_corners(x_c_offset, y_c_offset, CornerOffset)
        """
        border_dist = dist_to_border(x_IR, y_IR, angles[0], (x_c, y_c))
        border_point[0] = x_IR + sin(radians(angles[0])) * border_dist
        border_point[1] = y_IR + cos(radians(angles[0])) * border_dist
        AoA = get_AoA(border_point, angles[0], (x_c, y_c))
        
#         print(dist_to_border(x_b_par, y_b_par, angles[0], (x_c, y_c))/pix_cm)
        
        if(MOVEMODE == 1): #t_ang mean different things depending on what movement mode is in effect
            t_p, t_ang = point_from_border_at_dist_and_ang(t_AoA, Distances[distance_index], (x_c, y_c))
        elif(MOVEMODE == 2):
            try:
                t_ang = random.randrange(-180, 180, 1)
                t_dist = rand_dist_in_border_at_angle(x_b_par, y_b_par, t_ang, (x_c_val, y_c_val))
                t_p = [None]*2
                t_p[0] = x_b_par + sin(radians(t_ang))*t_dist
                t_p[1] = y_b_par + cos(radians(t_ang))*t_dist
                if(dist_to_border(x_IR, y_IR, angles[0], (x_c, y_c)) < 10 or (not point_in_border((x_IR, y_IR), (x_c, y_c))) or Missreading):
                    Reverse_ready = True
            except:
                if(dist_to_border(x_IR, y_IR, angles[0], (x_c, y_c)) < 10 or (not point_in_border((x_IR, y_IR), (x_c, y_c))) or Missreading):
                    Reverse_ready = True
                #the t_p does nothing, but its needed so the program doesn't crash
                t_p = center
#                 t_p = rand_point_in_border((x_c_val, y_c_val))
                t_ang = calc_ang_between((x_b_par, y_b_par), t_p)
                t_dist = calc_dist((x_b_par, y_b_par), t_p)
            """
            if(point_in_border((x_b_par, y_b_par), (x_c_val, y_c_val))):
                t_ang = random.randrange(-180, 180, 1)
                t_dist = rand_dist_in_border_at_angle(x_b_par, y_b_par, t_ang, (x_c_val, y_c_val))
                t_p = [None]*2
                t_p[0] = x_b_par + sin(radians(t_ang))*t_dist
                t_p[1] = y_b_par + cos(radians(t_ang))*t_dist
            else:
                Reverse_ready = True
                #the t_p does nothing, but its needed so the program doesn't crash
                t_p = rand_point_in_border((x_c_val, y_c_val))
                t_ang = calc_ang_between((x_b_par, y_b_par), t_p)
                t_dist = calc_dist((x_b_par, y_b_par), t_p)
#                 print(t_p)
#                 print("oopsie")
            """
        t_p_IR_off = (t_p[0] - sin(radians(t_ang)) * IRoffset * pix_cm, t_p[1] - cos(radians(t_ang)) * IRoffset * pix_cm)
#         print(AoA)
        
        if(Move_ready == True):
            t_p_f = t_p
            t_ang_f = t_ang
            """
            dist = calc_dist((x_IR, y_IR), t_p)/pix_cm
            ang1 = norm_angle(calc_ang_between((x_IR, y_IR), t_p) - angles[0])
            ang2 = norm_angle(t_ang - calc_ang_between((x_IR, y_IR), t_p))
            """
            if(MOVEMODE == 1):
                dist = calc_dist((x_b_par, y_b_par), t_p_IR_off)/pix_cm
                ang1 = norm_angle(calc_ang_between((x_b_par, y_b_par), t_p_IR_off) - angles[0])
                ang2 = norm_angle(t_ang - calc_ang_between((x_b_par, y_b_par), t_p_IR_off))
                send_to_bot_s2(ang1, 13, dist/13, ang2)
            elif(MOVEMODE == 2):
                if(Reverse_ready == True):
                    send_to_bot_s(0, -13, 2)
                    Reverse_ready = False
                else:
                    dist = calc_dist((x_b_par, y_b_par), t_p)/pix_cm
                    ang1 = norm_angle(calc_ang_between((x_b_par, y_b_par), t_p) - angles[0])
                    send_to_bot_s(ang1, 13, dist/13)
            Move_ready = False
        
        if(Collect_ready == True):
            send_to_bot_IR_collect(1000)
            distance_index = distance_index + 1
            Collect_ready = False
        
        if(Waiting_for_update and time.time() >= cam_update_time):
            Waiting_for_update = False
            if(MOVEMODE == 1 and (abs(norm_angle(t_ang - angles[0])) > 3 or calc_dist(t_p, (x_IR, y_IR)) > 3)):
                print([abs(norm_angle(t_ang - angles[0])), calc_dist(t_p, (x_IR, y_IR))])
                Move_ready = True
            else:
#                 print("hi")
                Collect_ready = True
        """      
        if(Receiving_data == True):
            send_to_bot_IR_send()
            sockreceiver.setblocking(1)
            file = open('TempFileName.csv', 'w')
            writer = csv.writer(file)

            data = sockreceiver.recv(4096)
            print(data)
            numRows = int(data.decode('utf'))
            print("received message: %s" % numRows)
            data = sockreceiver.recv(4096)
            numCol = int(data.decode('utf'))
            print("received message: %s" % numCol)
            Results = [[]*numCol]*numRows
            for i in range(numRows):
                data = sockreceiver.recv(4096) # buffer size is 1024 bytes
                message = data.decode('utf').split(',')
                floatMessage = []*len(message)
                for f in message:
                    floatMessage.append(float(f))
                Results[i] = floatMessage
                writer.writerow(Results[i])
                print("received message: %s" % floatMessage)
            file.close()
            sockreceiver.setblocking(0)
        """
        
        try:
            data = sockreceiver.recv(4096)#1024)
            print(data)
            if(data.decode('utf-8') == "Done Moving"):
                Waiting_for_update = True
                cam_update_time = time.time() + UPDATETIMESTEP
                """
                if(abs(t_ang - angles[0]) > 5 or calc_dist(t_p, (x_IR, y_IR)) > 2):
                    Move_ready = True
                else:
                    Collect_ready = True
                """
#                 print("done")
                """
                ReadingUDP = False
                Moving = False
                if(ABORTING):
                    x = 1
                    ABORTING = False
                elif(TESTMODE == 1):
                    x = 1
                elif(TESTMODE == 2):
                    f_dist = calc_dist((s_x,s_y),(x_b_par[0],y_b_par[0]))/pix_cm
                    f_angle = norm_angle(s_ang - angles[0])
                    writer.writerow([testType, testNum, t_dist, f_dist, t_angle, f_angle])
                """
            elif(data.decode('utf-8') == "Done Collecting"):
                if(Missreading == False):
                    send_to_bot_IR_send()
                    sockreceiver.setblocking(1)
                    file = open('IR_data_90_1.csv', 'a')
                    writer = csv.writer(file)

                    data = sockreceiver.recv(4096)
                    print(data)
                    numRows = int(data.decode('utf'))
                    print("received message: %s" % numRows)
                    data = sockreceiver.recv(4096)
                    numCol = int(data.decode('utf'))
                    print("received message: %s" % numCol)
                    Results = [[]*numCol]*numRows
                    for i in range(numRows):
                        data = sockreceiver.recv(4096) # buffer size is 1024 bytes
                        message = data.decode('utf').split(',')
                        floatMessage = []*len(message)
                        for f in message:
                            floatMessage.append(float(f))
                        Results[i] = floatMessage
                        Results[i].append(border_dist/pix_cm)
                        Results[i].append(AoA)
                        writer.writerow(Results[i])
                        print("received message: %s" % floatMessage)
                    file.close()
                    sockreceiver.setblocking(0)
                Move_ready = True
        except Exception as e:
            garbageVarIHatePython = 1
        
        if(len(x_c_offset) < 4):
            print("problem")
        dr,dw,de = select([sys.stdin], [], [], 0)
        if(dr != []):
            send_to_bot_stop()
#             writer.writerow(row)
            file.close()
            done = True

if __name__ == "__main__":
    main()
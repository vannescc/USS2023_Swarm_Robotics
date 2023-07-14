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
UDP_PORT = 5005
UDP_IP_OVERLORD = "172.24.87.109"
UDP_PORT_OVERLORD = 8080
socksender = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
socksender.connect((UDP_IP,UDP_PORT))
socksender.setblocking(0)

sockreceiver = socket.socket(socket.AF_INET,
                             socket.SOCK_DGRAM)
sockreceiver.bind((UDP_IP_OVERLORD, UDP_PORT_OVERLORD))
sockreceiver.setblocking(0) #allows us to write this program as non-blocking

global botHeight
botHeight = 17.3 #cm
global cameraHeight
cameraHeight = 112 #cm

#this has since been depricated and is not used in later programs, instead we use the pnp transformation from the opencv2 library
#roughly 194 pixels/50 cm with current setup on 5/23/2023 = 3.88 #200/50 = 4
global pix_cm
pix_cm = 4 #subject to small fluxuations

class TimeOutException(Exception):
    pass

def alarm_handler(signum, frame):
    print("Alarm signal Received")
    raise TimeOutException()


def send_signal_stop(bot_id):
    print(str(0) + "," + str(0) + "," + str(bot_id), flush=True)

def send_signal_start(bot_id):
    print(str(0) + "," + str(1) + "," + str(bot_id), flush=True)

def send_signal_OOB(bot_id):
    print(str(0) + "," + str(2) + "," + str(bot_id), flush=True)

def send_pos_home(x, y, bot_id):
    print(str(1) + "," + str(0) + "," + str(x) + "," + str(y) + "," + str(bot_id), flush=True)

def send_pos_current(x, y, bot_id):
    print(str(1) + "," + str(1) + "," + str(x) + "," + str(y) + "," + str(bot_id), flush=True)

def send_pos_target(x, y, bot_id):
    print(str(1) + "," + str(2) + "," + str(x) + "," + str(y) + "," + str(bot_id), flush=True)

#TODO: add send vector if needed

def send_angle(theta, bot_id):
    print(str(3) + "," + str(theta) + "," + str(bot_id), flush=True)

def calc_offset(x_bot, y_bot, x_tar, y_tar, x_c1, y_c1, x_c2, y_c2):
    theta1 = float(degrees(atan2((y_tar - y_bot), (x_tar - x_bot))))
    #theta2 is angle bot is pointing
    theta2 = float(degrees(atan2((y_c1 - y_c2), (x_c1 - x_c2))))
    offset = theta1 - theta2
    if (offset > 180.0):   
        offset = offset - 360
    elif (offset < -180.0):
        offset = offset + 360
    #offset = float(degrees(atan2((m_bot - m_target), (1 + (m_bot * m_target)))))
    #if (offset < 0):
    #    offset = -1 * (180 + offset)
    return offset * -1

#calculates the clope of a line defined by two points
def calc_slope(x1, y1, x2, y2):
    #if (x2 - x1 == 0):
    #    return ((float(y1) - float(y1)) / 0.1)
    return ((float(y1) - float(y2)) / (float(x2) - float(x1)))

#calculates the straight line distance between two points
def calc_dist(target, bot):
    return abs(sqrt(float((pow((target[0] - bot[0]), 2) + pow((target[1] - bot[1]), 2)))))

#prints data to an image overlay on what the camera can see
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
    
#always assumes 2 "bots" one of which is actally a robot
def calc_mes_pos_ang(centers, angles, AoA, dist):
    ret = [None] * 3
    if ( (centers[1][0] != None) and (centers[1][1] != None)):
        if (angles[1] != None): 
            ret[0] = centers[1][0] + sin(radians(angles[1]-AoA)) * dist
            ret[1] = centers[1][1] + cos(radians(angles[1]-AoA)) * dist
            ret[2] = float(degrees(atan2((centers[1][0] - ret[0]), (centers[1][1] - ret[1]))))
    return ret

#sends the robot two speeds in cm/s corresponding to their respective wheels and a time to apply these speeds
def send_to_bot(s_l, s_r, T):
    Message = ""
    Message = Message + str(1) + ", " + str(s_l) + ", " + str(s_r) + ", " + str(T)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

#sends the robot an angle to turn, and then a speed to travel straight at after it has finished turning, and a time to travel straight
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

#tells the robot to stop it's current opperation and to wait for further instruction
def send_to_bot_abort():
    Message = ""
    Message = Message + str(-1) + ", " + str(-1) + ", " + str(-1) + ", " + str(-1)
    data = bytes(Message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

#tells the robot to stop running
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

#sends the robot a string
def send_mess_to_bot(message):
    data = bytes(message, 'utf-8')
#     sock.sendto(data, (UDP_IP, UDP_PORT))
    sent = False
    while(not sent):
        try:
            socksender.sendall(data)
            sent = True
        except Exception as e:
            continue

#generates a random distance from the given point at the given angle that will remain within the border
def rand_dist_in_border_at_angle(x_b, y_b, angle, border): #chat GPT helped with this function
    x_p = x_b + sin(radians(angle))*40
    y_p = y_b + cos(radians(angle))*40
    if(x_b != None and y_b != None and x_p != None and y_p != None and x_p != x_b):
        slope = (y_p-y_b)/(x_p-x_b)
        dist = []
        x = border[0][0]
        y = slope * (x-x_b) + y_b
        if(y >= border[0][1] and y <= border[1][1]):
            if(numpy.sign(x_b-x_p) == numpy.sign(x_b-x)):
                dist.append(calc_dist((x_b, y_b), (x, y)))
        x = border[1][0]
        y = slope * (x-x_b) + y_b
        if(y >= border[0][1] and y <= border[1][1]):
            if(numpy.sign(x_b-x_p) == numpy.sign(x_b-x)):
                dist.append(calc_dist((x_b, y_b), (x, y)))
        y = border[0][1]
        if(slope != 0):
            x = (y-y_b)/slope + x_b
            if(x >= border[0][0] and x <= border[1][0]):
                if(numpy.sign(x_b-x_p) == numpy.sign(x_b-x)):
                    dist.append(calc_dist((x_b, y_b), (x, y)))
        y = border[1][1]
        if(slope != 0):
            x = (y-y_b)/slope + x_b
            if(x >= border[0][0] and x <= border[1][0]):
                if(numpy.sign(x_b-x_p) == numpy.sign(x_b-x)):
                    dist.append(calc_dist((x_b, y_b), (x, y)))
        if len(dist) == 0:
            maxDist = 0
        else:
            maxDist = min(dist)
    elif(x_b != None and y_b != None and x_p != None and y_p != None):
        if(y_p > y_b):
            maxDist = border[1][1] - y_b
        else:
            maxDist = y_b - border[0][1]
        """
        slope = (y_p-y)/(x_p-x)
        posdist = [0]*2
        dist = [0]*4
        dist[0] = sqrt((border[0][0] * slope)**2 + (border[0][0]-x)**2)
        dist[1] = sqrt((border[0][1] / slope)**2 + (border[0][1]-y)**2)
        dist[2] = sqrt((border[1][0] * slope)**2 + (border[1][0]-x)**2)
        dist[3] = sqrt((border[1][1] / slope)**2 + (border[1][1]-y)**2)
        for val in dist:
            if val > 0:
                posdist.append(val)
        maxDist = max(posdist)
        """
    if(int(maxDist) <= 0):
        return 0
    else:
        return random.randrange(int(maxDist))

#calculates how the robot would have to move to reach the center of the border and then tells the robot to move there
def move_to_mid(x_b, y_b, angle, height, width):
    theta = float(degrees(atan2((width/2 - x_b), (height/2 - y_b))))
    ang = theta - angle
    dist = calc_dist((x_b, y_b), (width/2, height/2))/pix_cm
    send_to_bot_s(ang, 13, dist/13)

#returns true if the point is within the border and false if it isn't
def within_border(x_b, y_b, border):
    return not (x_b >= border[1][0] or x_b <= border[0][0] or y_b <= border[0][1] or y_b >= border[1][1])

#returns a random point within the border
def rand_point_in_border(border):
    x = random.randrange(border[0][0], border[1][0], 1)
    y = random.randrange(border[0][1], border[1][1], 1)
    #print (x,y)
    return (x,y)

#calculates the angle to turn from point 1 to point point 2
def calc_ang_between(p1, p2): #fixed it, something about this coordinate system messes with trig functions
    return float(degrees(atan2((p2[0] - p1[0]), (p2[1] - p1[1]))))
#     return float(degrees(atan2((p2[0] - p1[0]), (p2[1] - p1[1]))))
#returns the angle between the two points in radians
def calc_ang_between_radians(p1, p2): #fixed it, something about this coordinate system messes with trig functions
    return float(atan2((p2[0] - p1[0]), (p2[1] - p1[1])))

#normalizes the given angle between 180 and -180
#takes degrees and returns degrees
def norm_angle(angle):
    while(angle < -180):
        angle = angle + 360
    while (angle > 180):
        angle = angle - 360
    return angle

#normalizes the angle in radians between 2pi and 0
#takes radians and returns radians
def norm_angle_radians(angle):
    while(angle < 0):
        angle = angle + 2*pi
    while (angle > 2*pi):
        angle = angle - 2*pi
    return angle

#this function takes 2 points on a circle, the first is the start point this point also has an associated angle in degrees, and the second is the target point.
#this function also takes the radius and center of the circle and a rectangular boundry to remain within
#this function returns a 0 if there is no path along the circle that connects the two points withput being interupted by the boundry
#it returns a 1 if the points can only connected in the direction defined by the start angle, returns a 2 if the points can only be connected in the opposite direction
#and will return a 3 if the points can be connected in either direction i.e. the circle doesn't intersect the boundry
def possible_circle_connections(start_point, start_angle, end_point, circle_center, circle_radius, boundry):
    s_x, s_y = start_point
    s_a_t = norm_angle_radians(radians(start_angle))
    e_x, e_y = end_point
    c_x, c_y = circle_center
    r = circle_radius
    (x1,y1),(x2,y2) = boundry
    s_a = norm_angle_radians(calc_ang_between_radians(circle_center, start_point))
    e_a = norm_angle_radians(calc_ang_between_radians(circle_center, end_point))
    intersections = []
    for x in [x1,x2]:
        dist = abs(c_x - x)
        if(dist > r):
            continue
        elif(dist == r):
            intersections.append([x,c_y])
        else:
            dy = sqrt(r**2 - dist**2)
            if(c_y + dy <= y2):
                intersections.append([x, c_y + dy])
            if(c_y - dy >= y1):
                intersections.append([x, c_y - dy])
    for y in [y1,y2]:
        dist = abs(c_y - y)
        if(dist > r):
            continue
        elif(dist == r):
            intersections.append([y,c_x])
        else:
            dx = sqrt(r**2 - dist**2)
            if(c_x + dx <= x2):
                intersections.append([y, c_x + dx])
            if(c_x - dx >= x1):
                intersections.append([y, c_x - dx])
        
    if(intersections == []):
        return 3
    
    intersection_angles = []
    for point in intersections:
        intersection_angles.append(norm_angle_radians(calc_ang_between_radians(circle_center, point)))
        
    if(norm_angle_radians(s_a + pi/2 - s_a_t) <= 0.1): #clockwise #still broken, pick up here on tuesday 6/6/2023
        return 0
        angle_diffs = []
        for angle in intersection_angles:
            angle_diffs.append(norm_angle_radians(angle - s_a))
        if(min(angle_diffs) > norm_angle_radians(e_a - s_a)):
            return 1
        elif(max(angle_diffs) < norm_angle_radians(e_a - s_a)):
            return 2
        else:
            return 0
    else: #counter clockwise
#         return 0
        angle_diffs = []
        for angle in intersection_angles:
            angle_diffs.append(norm_angle_radians(s_a - angle))
        if(max(angle_diffs) < norm_angle_radians(s_a - e_a)):
            return 2
        elif(min(angle_diffs) > norm_angle_radians(e_a - s_a)):
            return 1
        else:
            return 0

#returns a list of points where the circle intersects the boundry
def circ_intersections(circle_center, circle_radius, boundry):
    c_x, c_y = circle_center
    r = circle_radius
    (x1,y1),(x2,y2) = boundry
    intersections = []
    for x in [x1,x2]:
        dist = abs(c_x - x)
        if(dist > r):
            continue
        elif(dist == r):
            intersections.append([x,c_y])
        else:
            dy = sqrt(r**2 - dist**2)
            if(c_y + dy <= y2):
                intersections.append([x, c_y + dy])
            if(c_y - dy >= y1):
                intersections.append([x, c_y - dy])
    for y in [y1,y2]:
        dist = abs(c_y - y)
        if(dist > r):
            continue
        elif(dist == r):
            intersections.append([c_x, y])
        else:
            dx = sqrt(r**2 - dist**2)
            if(c_x + dx <= x2):
                intersections.append([c_x + dx, y])
            if(c_x - dx >= x1):
                intersections.append([c_x - dx, y])
    print(intersections)
    return intersections

#shows the circle and points on the image of the camera feed
def circ_debug(frame, center, R, points):
    out = frame
    dimensions = out.shape
    height = dimensions[0]
    width = dimensions[1]
    cv2.circle(out, center, R, (0, 0, 255), 1)
    for point in points:
        cv2.circle(out, (int(point[0]), int(point[1])), 5, (0, 0, 255), -1)
#     print(points)
    cv2.imshow("DEBUG", out)
    
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
    
def main():
    FILENAME = "SPIG_par_data_6.csv"#format: testType, testNum, target distance, distance traveled, target angle, angle turned
    file = open(FILENAME, 'a')
    writer = csv.writer(file)
    print("enter test mode")
    TESTMODE = int(input())
    if(TESTMODE == 1): #this is being tabled for now due to issues generating the random point to move to, these can be fixed but require non-trivial amounts of effort
        nBots = 1
    elif(TESTMODE == 2):
        nBots = 1 
    else:
        nBots = 2 
    DEBUG = True
    x_b = [None] * nBots
    y_b = [None] * nBots
    x_p = [None] * nBots
    y_p = [None] * nBots
    angles = [None] * nBots
    ax_p = None
    ay_p = None
    a_n = None
    x_n = None
    y_n = None
    a_t_n = None
    d_x = None
    d_y = None
    theta = None
    d_theta = None #this stays in radians
    s_l = None
    s_r = None
    b = 12.5#11.5 #wheel to wheel width of robot in cm
    T = 10
    Moving = False
    R = None
    c_x = None
    c_y = None
    T_STEP = 1
    start_time = time.time()
    V_step_time = start_time + T_STEP
    V = 0
    l_b = [0,0]
    c_b = [0,0]
    AoA = 90
    dist = 110
    frame = tags.get_frame()
    dimensions = frame.shape
    height = dimensions[0]
    width = dimensions[1]
    SalgBorder = [[120, 120], [width-121, height-121]]
    validBorder = [[80,80], [width-81, height-81]]
    abortBorder = [[40,40], [width-41, height-41]]
    cameraBorder = [[0,0], [width-1, height-1]]
    m_ang = None
    m_dist = None
    m_x = None
    m_y = None
    f_x = None
    f_y = None
    UDPTIMESTEP = 10
    UDPTIME = time.time() + UDPTIMESTEP
    sent_message = False
    testType = TESTMODE
    testNum = 0
    t_dist = None
    f_dist = None
    t_angle = None
    f_angle = None
    s_x = None
    s_y = None
    s_ang = None
    RR = None
    RL = None
    ABORTING = False
    c_x_f = None
    c_y_f = None
    R_f = None
    intersections = []
    point = None
    point_f = None
    x_b_par = [None] * nBots
    y_b_par = [None] * nBots
    m_x_par = None
    m_y_par = None
    m_ang_par = None
    m_dist_par = None
    f_x_par = None
    f_y_par = None

    #if (DEBUG == True):
        #print("Code Location: Begin")
        #print("----------------")

    # read for tag centers until all tags have been found and localized
    while (any(elem is None for elem in x_b) or any(elem is None for elem in y_b)):
        frame = tags.get_frame()
        results = tags.find_tags(frame)
        centers = tags.get_bot_centers(results, nBots)
        angles = tags.get_bot_angles(results, nBots)
        if (len(centers) == nBots):
            for i in range(nBots) :
                if (centers[i][0] == -1):
                    continue
                x_b[i] = centers[i][0]
                y_b[i] = centers[i][1]
                x_b_par[i], y_b_par[i] = account_for_parallax((x_b[i], y_b[i]), cameraBorder)
                if(angles[i] != None):
                    x_p[i] = centers[i][0] + sin(radians(angles[i]))*40
                    y_p[i] = centers[i][1] + cos(radians(angles[i]))*40
    if(TESTMODE == 1):
        """
        ret = calc_mes_pos_ang(centers, angles, AoA, dist)
        x_n = ret[0]
        y_n = ret[1]
        a_n = ret[2]
        """
        """
        RL1 = (-x_b[0]+80)/(sin(radians(angles[0]-90))-1) #left wall
        RL2 = (-x_b[0]+(width-81))/(sin(radians(angles[0]-90))+1) #right wall
        RL3 = (-y_b[0]+80)/(cos(radians(angles[0]-90))-1) #top wall
        RL4 = (-y_b[0]+(height-81))/(cos(radians(angles[0]-90))+1) #bottom wall
        RR1 = (-x_b[0]+80)/(sin(radians(angles[0]+90))-1) #left wall
        RR2 = (-x_b[0]+(width-81))/(sin(radians(angles[0]+90))+1) #right wall
        RR3 = (-y_b[0]+80)/(cos(radians(angles[0]+90))-1) #top wall
        RR4 = (-y_b[0]+(height-81))/(cos(radians(angles[0]+90))+1) #bottom wall
        RL = numpy.array([RL1, RL2, RL3, RL4])
        RL = RL[numpy.isfinite(RL)].min()
        RR = numpy.array([RR1, RR2, RR3, RR4])
        RR = RR[numpy.isfinite(RR)].min()
        """
        looping = True
        while looping:
            x_n,y_n = rand_point_in_border(validBorder)
            d_x = x_n - x_b[0]
            d_y = y_n - y_b[0]
            R = ((d_x**2 + d_y**2)/((d_x*float(cos(radians(angles[0]))))-(d_y*float(sin(radians(angles[0]))))))/2
            c_x = x_b[0] + float(sin(radians(angles[0]+90)))*R
            c_y = y_b[0] + float(cos(radians(angles[0]+90)))*R
            if(possible_circle_connections((x_b[0], y_b[0]), angles[0], (x_n, y_n), (int(c_x), int(c_y)), int(abs(R)), abortBorder) == 1):
                looping = False

    if(x_n != None):
        d_x = x_n - x_b[0]
        d_y = y_n - y_b[0]
        theta = angles[0]
        if(abs(d_y*cos(radians(theta)) - d_x*sin(radians(theta))) <= 0.01): #
            R = None
            d_theta = None
        else:
            R = .5*((d_x**2 + d_y**2)/(d_y*cos(radians(theta))-d_x*sin(radians(theta))))
            d_theta = 2 * atan((d_y-d_x*tan(radians(theta/2)))/(d_x+d_y*tan(radians(theta/2)))) - radians(theta)
            c_x = x_b[0] + sin(radians(theta+90))*R
            c_y = y_b[0] + cos(radians(theta+90))*R
            
    if(a_n != None):
        ax_p = x_n + sin(radians(a_n))*40
        ay_p = y_n + cos(radians(a_n))*40
    if(within_border(x_b[0], y_b[0], SalgBorder)):
        m_ang = random.randrange(-180, 180, 1)
        m_dist = rand_dist_in_border_at_angle(x_b[0], y_b[0], m_ang, validBorder)
    else:
        point = rand_point_in_border(validBorder)
        m_ang = calc_ang_between((x_b[0], y_b[0]), point)
        m_dist = calc_dist((x_b[0], y_b[0]), point)
    #print(m_dist)
    if(m_ang != None):
        m_x = x_b[0] + sin(radians(m_ang))*m_dist
        m_y = y_b[0] + cos(radians(m_ang))*m_dist
        m_x_par, m_y_par = account_for_parallax((m_x, m_y), cameraBorder)
        m_ang_par = calc_ang_between((x_b_par[0], y_b_par[0]), (m_x_par, m_y_par))
        m_dist_par = calc_dist((x_b_par[0], y_b_par[0]), (m_x_par, m_y_par))
    else:
        m_x = 0
        m_y = 0
        m_x_par, m_y_par = account_for_parallax((m_x, m_y), cameraBorder)
    frame_mod = frame
    done = False
    while not done:
        if (DEBUG == True):
            #print("Code Location: Orienting Bots")
            for i in range(len(x_b)):
                #print("bot#:" + str(i))
                #print("location: " + str(x_b[i]) + ", " + str(y_b[i]))
                #print("Target: " + str(x_t[i]) + ", " + str(y_t[i]))
                #print("------")
                mark_debug(frame_mod, x_b, y_b, angles, x_p, y_p, x_n, y_n, ax_p, ay_p, R, c_x, c_y, m_x, m_y, f_x, f_y, c_x_f, c_y_f, R_f, intersections, point_f, x_b_par, y_b_par)
#               mark_debug_angle(frame_mod, centers, results, nBots)
            #print("----------------")
            if cv2.waitKey(1) & 0xFF == ord('1'):
                break

        signal.signal(signal.SIGALRM, alarm_handler)
        signal.alarm(1)
        try:
            frame = tags.get_frame()
            results = tags.find_tags(frame)
            corners = tags.get_two_corners(results, nBots)
            centers = tags.get_bot_centers(results, nBots)
            newAngles = tags.get_bot_angles(results, nBots)
        except TimeOutException as ex:
            print(ex)
        signal.alarm
        
        for i in range(nBots):
            if newAngles[i] != None:
                angles[i] = newAngles[i]
        if(len(centers) == nBots):
            for i in range(nBots) :
                if (centers[i][0] == -1):
                    continue
                x_b[i] = centers[i][0]
                y_b[i] = centers[i][1]
                x_b_par[i], y_b_par[i] = account_for_parallax((x_b[i], y_b[i]), cameraBorder)
                if(angles[i] != None):
                    x_p[i] = centers[i][0] + sin(radians(angles[i]))*40
                    y_p[i] = centers[i][1] + cos(radians(angles[i]))*40
        if(TESTMODE == 1):
            """
            ret = calc_mes_pos_ang(centers, angles, AoA, dist)
            x_n = ret[0]
            y_n = ret[1]
            a_n = ret[2]
            """
            """
            RL1 = (-x_b[0]+80)/(sin(radians(angles[0]-90))-1) #left wall
            RL2 = (-x_b[0]+(width-81))/(sin(radians(angles[0]-90))+1) #right wall
            RL3 = (-y_b[0]+80)/(cos(radians(angles[0]-90))-1) #top wall
            RL4 = (-y_b[0]+(height-81))/(cos(radians(angles[0]-90))+1) #bottom wall
            RR1 = (-x_b[0]+80)/(sin(radians(angles[0]+90))-1) #left wall
            RR2 = (-x_b[0]+(width-81))/(sin(radians(angles[0]+90))+1) #right wall
            RR3 = (-y_b[0]+80)/(cos(radians(angles[0]+90))-1) #top wall
            RR4 = (-y_b[0]+(height-81))/(cos(radians(angles[0]+90))+1) #bottom wall
            RL = numpy.array([RL1, RL2, RL3, RL4])
            RL = RL[numpy.isfinite(RL)].min()
            RR = numpy.array([RR1, RR2, RR3, RR4])
            RR = RR[numpy.isfinite(RR)].min()
            """
            looping = True
            while looping:
                x_n,y_n = rand_point_in_border(validBorder)
                d_x = x_n - x_b[0]
                d_y = y_n - y_b[0]
                R = ((d_x**2 + d_y**2)/((d_x*float(cos(radians(angles[0]))))-(d_y*float(sin(radians(angles[0]))))))/2
                c_x = x_b[0] + float(sin(radians(angles[0]+90)))*R
                c_y = y_b[0] + float(cos(radians(angles[0]+90)))*R
                if(possible_circle_connections((x_b[0], y_b[0]), angles[0], (x_n, y_n), (int(c_x), int(c_y)), int(abs(R)), abortBorder) == 1):
                    looping = False
#         if(x_n != None and y_n != None and x_b[0] != None and y_b[0] != None and (calc_dist([x_n, y_n], [x_b[0],y_b[0]]) <= 15)):
#             send_mess_to_bot("stop")
#         else:
#             send_mess_to_bot("go")
        if(within_border(x_b[0], y_b[0], SalgBorder)):
            m_ang = random.randrange(-180, 180, 1)
            m_dist = rand_dist_in_border_at_angle(x_b[0], y_b[0], m_ang, validBorder)
        else:
            point = rand_point_in_border(validBorder)
            m_ang = calc_ang_between((x_b[0], y_b[0]), point)
            m_dist = calc_dist((x_b[0], y_b[0]), point)
        #print(m_dist)
        if(m_ang != None):
            m_x = x_b[0] + sin(radians(m_ang))*m_dist
            m_y = y_b[0] + cos(radians(m_ang))*m_dist
            m_x_par, m_y_par = account_for_parallax((m_x, m_y), cameraBorder)
            m_ang_par = calc_ang_between((x_b_par[0], y_b_par[0]), (m_x_par, m_y_par))
            m_dist_par = calc_dist((x_b_par[0], y_b_par[0]), (m_x_par, m_y_par))
        else:
            m_x = 0
            m_y = 0
            m_x_par, m_y_par = account_for_parallax((m_x, m_y), cameraBorder)
#         m_x = x_b[0] + sin(radians(angles[0]))*m_dist
#         m_y = y_b[0] + cos(radians(angles[0]))*m_dist
        if(Moving == False and TESTMODE == 2):
            f_x = m_x_par
            f_y = m_y_par
            point_f = point
            s_x = x_b_par[0]
            s_y = y_b_par[0]
            s_ang = angles[0]
            testNum += 1
            t_dist = m_dist_par/pix_cm
            t_angle = norm_angle(m_ang_par-angles[0])
            send_to_bot_s(t_angle, 13, t_dist/13)
            Moving = True
            sent_message = True
            
            UDPTIME = time.time() + UDPTIMESTEP
#         send_to_bot(90, 135, 10)
        if(x_n != None and y_n != None):
            d_x = x_n - x_b[0]
            d_y = y_n - y_b[0]
            theta = angles[0]
            if(abs(d_y*float(cos(radians(theta))) - d_x*float(sin(radians(theta)))) <= 0.0001): #
                print("Problem")
                R = None
                d_theta = None
            else:
                R = ((d_x**2 + d_y**2)/((d_x*float(cos(radians(theta))))-(d_y*float(sin(radians(theta))))))/2
                d_theta = 2 * atan((d_x-d_y*tan(radians(theta/2)))/(d_y+d_x*tan(radians(theta/2)))) - radians(theta)
                c_x = x_b[0] + float(sin(radians(theta+90)))*R
                c_y = y_b[0] + float(cos(radians(theta+90)))*R
                if(Moving == False and TESTMODE == 1):
                    f_x = x_n
                    f_y = y_n
                    c_x_f = c_x
                    c_y_f = c_y
                    R_f = abs(R)
                    intersections = circ_intersections((c_x_f, c_y_f), R_f, abortBorder)
#                     print(possible_circle_connections((x_b, y_b), (f_x, f_y), (c_x_f, c_y_f), R_f, abortBorder))
#                     print(possible_circle_connections((x_b, y_b), (f_x,f_y), (c_x,c_y), R, abortBorder))
                    Moving = True
                    sent_message = True
                    UDPTIME = time.time() + UDPTIMESTEP
                    if(R > 0):
                        s_r = 13
                        T = d_theta*((R/pix_cm+b/2))/s_r
                        s_l = d_theta*((R/pix_cm-b/2))/T   
                    else:
                        s_l = 13
                        T = d_theta*((R/pix_cm-b/2))/s_l
                        s_r = d_theta*((R/pix_cm+b/2))/T
                    send_to_bot(s_l, s_r, abs(T))
                    
        if(a_n != None):
            ax_p = x_n + sin(radians(a_n))*40
            ay_p = y_n + cos(radians(a_n))*40
        if(time.time() >= V_step_time):
            l_b = c_b
            c_b = [x_b[0], y_b[0]]
            V = (calc_dist(l_b, c_b)/pix_cm)/T_STEP
            V_step_time = time.time() + T_STEP
        try:
            data = sockreceiver.recv(4096)#1024)
            print(data)
            if(data.decode('utf-8') == "Done Moving"):
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
#                 file.close()
#                 file = open(FILENAME, 'a')
#                 writer = csv.writer(file)
        except Exception as e:
            garbageVarIHatePython = 1
        frame_mod = frame
        if(not ABORTING and not within_border(x_b[0], y_b[0], abortBorder)):
            send_to_bot_abort()
            move_to_mid(x_b_par[0], y_b_par[0], angles[0], height, width)
            ABORTING = True
            print("ABORTING")
        dr,dw,de = select([sys.stdin], [], [], 0)
        if(dr != []):
            send_to_bot_stop()
#             writer.writerow(row)
            file.close()
            done = True
"""
        if(sent_message == True and time.time() >= UDPTIME):
            ReadingUDP = True
            while ReadingUDP:
                data, addr = sock.recvfrom(1024)
                print(data)
                if(data.decode('utf-8') == "Done Moving"):
                    ReadingUDP = False
                    Moving = False
                    f_dist = calc_dist((s_x,s_y),(x_b[0],y_b[0]))/pix_cm
                    f_angle = norm_angle(angles[0]-s_ang)
                    writer.writerow([testType, testNum, t_dist, f_dist, t_angle, f_angle])
                    file.close()
                    file = open(FILENAME, 'a')
                    writer = csv.writer(file)
"""
        

#         cv2.putText(frame_mod, str(tags.get_bot_angles(results, nBots)[0]), (25, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
#         cv2.putText(frame_mod, str(a_n), (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
#         cv2.putText(frame_mod, str(R/pix_cm), (25, 125), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
#         cv2.putText(frame_mod, str(degrees(d_theta)), (25, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
#         cv2.putText(frame_mod, str(calc_dist((x_b[0], y_b[0]), (x_b[1], y_b[1]))), (25, 175), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
#         cv2.putText(frame_mod, str(V), (25, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
#         print(within_border(x_b[0], y_b[0], validBorder))
        
    
if __name__ == "__main__":
    main()

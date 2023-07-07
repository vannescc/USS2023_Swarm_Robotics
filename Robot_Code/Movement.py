import sys
import time
from angleToIntercept import *

import socket

import math
import numpy
import Adafruit_BBIO.UART as UART
import serial
from time import sleep
import rcpy.servo as servo
import rcpy.gpio as gpio
import rcpy.adc as adc

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

import smbus
from colorCheck import *

import csv

global WHEELCIRC
WHEELCIRC = 21.7 #cm
global ROTATETIME
ROTATETIME = 2 #s
global WIDTH
WIDTH = 12.5 #11.5 #cm


################################################
#             CONTROL FUNCTIONS
################################################

# Using the circumference of the wheel base we can calculate
# the length of time required to turn to change our pose
# by the value of angle
# float: angle The angle we want to turn at
# int: max_speed the speed at which the wheels will spin
def time_to_turn(angle, max_speed):
    circumference = (12.3 / 2) * 2 * numpy.pi
    distance = circumference / 360 * angle
    return distance / max_speed

# A function that cuts off the range so that the wheels only spin
# forward. It takes a bounded_setting between [-1 1].
# The right servo is encoded with a 1 and the left with a 0.
def cutoff(servo, bounded_setting):
    if servo:
        if bounded_setting > 0.0:
            return -0.05
        else:
            return bounded_setting
    else:
        if bounded_setting < 0.0:
            return 0.05
        else:
            return bounded_setting

# A function for normalizing the motor setting to the range [-1 1].
# The servo motors have a safe range of operation between those
# two points 
def norm(unbounded_setting):
    if abs(unbounded_setting) <= 1.0:
        return unbounded_setting
    else:
        return unbounded_setting / abs(unbounded_setting)
        
# A function for finding the second wheel speed when turning
# at a desired radius. Because the function returns the inner wheel
# speed this function is safe to call with the current wheel speed of
# the outer wheel without having to worry about going outside the bounds
# of the safe servo range [-1 1].
# r is the radius of the circle that the robot should turn at.
# v1 is the speed of the outer wheel (faster wheel)
# returns v2 which is the inner wheel speed (slower wheel)
# r = ((v1+v2)/(v1-v2) - 1)(d/2)
# (r * 2/d) + 1 = v1+v2/v1-v2
# ((r*2/d) + 1)v1 - ((r*2/d) + 1)v2 = v1 + v2
# ((r*2/d) + 1)v1 - v1 = ((r*2/d) + 1)v2 + v2
# ((r*2/d) + 1)v1 - v1 = (((r*2/d) + 1) + 1)v2
# (((r*2/d) + 1)v1 - v1) / (((r*2/d) + 1) + 1) = v2
def turn_at_radius(r, v1):
    width = 11.5 # width of the robot wheels in cm
    if r < width:
        print("Turning radius too tight: {} cm".format(r))
        print("Going Straight")
        return v1
    
    v2 = ((((r * 2 / width) + 1) * v1) - v1) / (((r * 2 / width) + 1) + 1)
    
    return v2

# Function for calculating the speed based on the wheel diameter
# and the length of time it took to travel 4.34 centimeters which
# is 1/5 the circumference of the wheel
def calculate_speed(t_Prev, t_Now):
    distance = 4.34 # 2.17 cm is how far the wheel travels in a single state
    delta_t = t_Now - t_Prev
    return distance / delta_t

# Simple funtion to calculate the error from the measured speed
def calculate_error(measured_speed, ideal_speed):
    return (ideal_speed - measured_speed)

# Simple function for calculating the integral of the error
def calculate_ierror(integral_error, curr_error, tPrev, tNow):
    return ((integral_error + curr_error) * (tNow - tPrev))

# Simple function for calculating the derivative of the error
def calculate_derror(curr_error, prev_error, tPrev, tNow):
    return ((curr_error - prev_error) / (tNow - tPrev))

# Function that implements pid control. Currently P works sufficiently and the
# other two components have not been implemented.
# wheel_id is 1 for the right servo and 0 for the left
def pid_control(p, i, d, error, integral_error, derivative_error, motor_setting, max_speed, wheel_id):
    # Since the error has units of cm/s we need to convert to us for sending to the servos
    proportional_component = convert_speed_to_duty(p * error, max_speed)
    integral_component = convert_speed_to_duty(i * integral_error, max_speed)
    derivative_component = convert_speed_to_duty(d * derivative_error, max_speed)

    # We need to normalize the output to a range of [-1 1]
    output_motor_setting = cutoff(wheel_id, norm(proportional_component + integral_component + derivative_component + motor_setting))
    print("Output Motor Setting: {}, Proportional: {}, Integral: {}, Derivative: {} Input Motor Setting: {}".format(output_motor_setting, proportional_component, integral_component, derivative_component, motor_setting))
    return output_motor_setting

# Function for converting between an input speed from control to an output duty
# cycle for the servos
# the left servo is encoded as a 0, and the right servo is encoded as a 1
def convert_speed_to_duty(servo, speed, LUTS): #could be better
    targetRPM = speed*60/WHEELCIRC
    LowerRPM = sys.float_info.min
    HigherRPM = sys.float_info.max
    LowerDuty = -1.5
    HigherDuty = 1.5
    Duty = 0
    if(speed != 0):
        if not servo:
            for row in LUTS[servo]:
                if row[2] < targetRPM:
                    LowerRPM = row[2]
                    LowerDuty = row[0]
                else:
                    HigherRPM = row[2]
                    HigherDuty = row[0]
                    #print([LowerRPM, HigherRPM])
                    break
        else:
            for row in reversed(LUTS[servo]):
                if row[2] < targetRPM:
                    LowerRPM = row[2]
                    LowerDuty = row[0]
                else:
                    HigherRPM = row[2]
                    HigherDuty = row[0]
                    #print([LowerRPM, HigherRPM])
                    break
        if(HigherDuty != LowerDuty):
            slope = (HigherRPM - LowerRPM)/(HigherDuty - LowerDuty)
            Duty = (targetRPM - LowerRPM)/slope + LowerDuty
        else:
            Duty = HigherDuty
        print(Duty)
    return Duty
    
def getLUTS():
    leftFile = open('LeftServoResponseNew.csv', 'r')
    leftReader = csv.reader(leftFile)
    leftListReader = list(leftReader)
    leftFile.close()
    
    rightFile = open('RightServoResponseNew.csv', 'r')
    rightReader = csv.reader(rightFile)
    rightListReader = list(rightReader)
    rightFile.close()
    
    LUTS = [[[]*len(leftListReader[0])]*len(leftListReader)]*2
    LUTS[0] = leftListReader
    LUTS[1] = rightListReader
    for i in range(len(LUTS)):
        for j in range(len(LUTS[i])):
            for k in range(len(LUTS[i][j])):
                LUTS[i][j][k] = float(LUTS[i][j][k])
            if i == 0 and LUTS[i][j][0] < 0:
                LUTS[i][j][2] = -LUTS[i][j][2]
            elif i == 1 and LUTS[i][j][0] > 0:
                    LUTS[i][j][2] = -LUTS[i][j][2]
    return LUTS
            
        
    #return norm(speed / max_speed)
    
# Function that takes a new speed for the two motor
# and updates the necessary variables
# it then calls the servo.set() function to update
# the pulse being sent to the servos
def update_motors(left_new_speed, right_new_speed, LUTS):
    global left_motor_setting, right_motor_setting
    global left_motor_speed, right_motor_speed

    left_motor_speed = left_new_speed
    right_motor_speed = right_new_speed
    left_motor_setting = convert_speed_to_duty(0,left_motor_speed, LUTS)#21.924028091423587)
    right_motor_setting = convert_speed_to_duty(1, right_motor_speed, LUTS)#-20.9995597347149)
    if(left_motor_setting > 1.5):
        left_motor_setting = 1.5
    elif(left_motor_setting < -1.5):
        left_motor_setting = -1.5
    if(right_motor_setting > 1.5):
        right_motor_setting = 1.5
    elif(right_motor_setting < -1.5):
        right_motor_setting = -1.5
    leftServo.set(left_motor_setting)
    rightServo.set(right_motor_setting)
    
def p_controller(ideal_speed, speed_setting, measured_speed):
    if(ideal_speed < 0):
        measured_speed = -measured_speed
    return (speed_setting + ((ideal_speed - measured_speed) *.5))
    
def pi_controller(ideal_speed, measured_speed, cumulative_error, time_since_last):
    if(True):#ideal_speed != 0):
        if(ideal_speed < 0):
            measured_speed = - measured_speed
        Kp = 0.5
        Ki = 1
        error = ideal_speed - measured_speed
        i_error_now = (cumulative_error + error) * time_since_last
        return [(ideal_speed + error * Kp + i_error_now * Ki), i_error_now]
    else:
        error = ideal_speed - measured_speed
        i_error_now = (cumulative_error + error) * time_since_last
        return [ideal_speed, i_error_now]
    
def rotate(angle, LUTS):
    dist = float(angle/360*math.pi*WIDTH)
    T = abs(float(dist/13))
    if(angle < 0):
        left_rot_speed = 13
        right_rot_speed = -13
    else:
        left_rot_speed = -13
        right_rot_speed = 13
    return [time.time() + T,  left_rot_speed, right_rot_speed]
    
def readData(origData): #takes the byte string from udp communication assuming a byte string of one int and three float values separated by commas and returns these values as a list of one int and three floats
    message = origData.decode('utf-8').split(", ")
    retMessage = []*len(message)
    print(message)
    for i in range(len(message)):
        if i == 0:
            retMessage.append(int(message[i]))
        else:
            retMessage.append(float(message[i]))
    return retMessage

################################################
#       END CONTROL FUNCTION DECLARATIONS
################################################

def main():
    LUTS = getLUTS()
    # Set up GPIO pins and configure Wheel Encoders
    left_encoder_pin = 25
    right_encoder_pin = 17
    encoder_chip = 1
    left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
    right_encoder = gpio.Input(encoder_chip, right_encoder_pin)
    
    ################################################
    #                  SERVO SETUP
    ################################################
    
    # Enable servo and calibrate light sensor
    servo.enable()
    
    # Motor speeds
    LEFT_MAX = 13#114.3#56#80#21.924028091423587#40
    RIGHT_MAX = -13#-128.6#-63#-90#-20.9995597347149#-40
    left_motor_speed = 13 #cm/s
    left_motor_setting = convert_speed_to_duty(0, left_motor_speed, LUTS)#21.924028091423587)
    right_motor_speed = 13 #cm/s
    right_motor_setting = convert_speed_to_duty(1, right_motor_speed, LUTS)#-20.9995597347149)
    
    # Set up clocks to periodically update the motors
    period = 0.02
    leftServo.set(0.0)
    rightServo.set(0.0)
    clk0 = leftServo.start(period)
    clk1 = rightServo.start(period)
    
    
    
    ################################################
    #              END SERVO SETUP
    ################################################
    
    # Time variables
    left_tPrev = time.time()
    left_tNow = left_tPrev
    right_tPrev = time.time()
    right_tNow = left_tPrev
    tPrev = time.time()
    
    # State variables
    left_start_state = left_encoder.get()
    left_state = left_start_state
    left_prev_state = None
    right_start_state = right_encoder.get()
    right_state = right_start_state
    right_prev_state = None
    control_current_state = "black"
    
    # Error variables
    left_prev_error = 0
    left_curr_error = 0
    left_integral_error = 0
    left_derivative_error = 0
    
    right_prev_error = 0
    right_curr_error = 0
    right_integral_error = 0
    right_derivative_error = 0
    
    # Chase variables
    chasing = False
    chasing_time = 0
    
    #p controller variables
    p_control_on = True
    left_last_bouncy = False
    left_curr_bouncy = False
    left_speed_measuring = False
    left_speed = None
    right_last_bouncy = False
    right_curr_bouncy = False
    right_speed_measuring = False
    right_speed = None
    UPDATETIMESTEP = 0.5
    updateTime = time.time() + UPDATETIMESTEP
    REPORTTIMESTEP = 0.05
    reportTime = time.time() + REPORTTIMESTEP
    
    left_ideal_speed = None
    right_ideal_speed = None
    left_pi_speed = None
    right_pi_speed = None
    
    left_i_error = 0
    right_i_error = 0
    
    left_rot_speed = None
    right_rot_speed = None
    
    # Characterizations
    # Right max Speed: 20.9995597347149 cm / s
    # Left max Speed: 21.924028091423587 cm / s
    # Wheel Circumference: 21.7 cm
    # Wheel Base 12.3 cm
    UART.setup("UART1")
    
    #ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200) #9600 is baudrate for PI 115200 is for beaglebone
    # Main control loop
    
    #"172.24.52.22"#"172.25.255.118"#"192.168.8.1"#"0.0.0.0"#"127.0.0.1"
    UDP_IP = "172.24.87.204"#"172.25.255.118"#"192.168.7.2"
    #UDP_IP = "172.24.52.22"
    UDP_PORT = 5005#5005#8080
    UDP_IP_OVERLORD = "172.24.87.109"
    UDP_PORT_OVERLORD = 8080
    
    DoneMovingMessage = b"Done Moving"
    DoneCollectingMessage = b"Done Collecting"
    
    sockreceiver = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sockreceiver.bind((UDP_IP, UDP_PORT))
    sockreceiver.setblocking(0)
    socksender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socksender.connect((UDP_IP_OVERLORD, UDP_PORT_OVERLORD))
    socksender.setblocking(0)
    Done = False
    Moving = False
    tStop = None
    tStopRot = None
    tStopRot2 = None
    WAITACCSTEP = 0.5
    WaitAccTime = time.time() + WAITACCSTEP
    waiting_for_acc = False
    
    READTIMESTEP = 0.02
    readTime = time.time() + READTIMESTEP
    IR_PIN = adc.ADC(0)
    Reading = False
    num_IR_points = 0
    IR_readings = []
    Sending = False
    IR_reading_index = 0
    #IR_FILE = "IR_readings"
    
    while not Done:
        """
        data = [None]
        while any (elem is None for elem in data):
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            Message = readData(data)
            print("received message: %s" % Message)#floatMessage)
        """
        if(p_control_on):
            if(left_speed_measuring == True):
                if(left_prev_state == 1 and left_state == 0):
                    left_tPrev = left_tNow
                    left_tNow = time.time()
                    left_speed = (WHEELCIRC / 5) / (left_tNow - left_tPrev)
                    #print(left_tNow - left_tPrev)
            elif(left_prev_state == 1 and left_state == 0):
                left_tNow = time.time()
                left_speed_measuring = True
                            
            #right state things
            if(right_speed_measuring == True):
                if(right_prev_state == 1 and right_state == 0):
                    right_tPrev = right_tNow
                    right_tNow = time.time()
                    right_speed = (WHEELCIRC / 5) / (right_tNow - right_tPrev)
            elif(right_prev_state == 1 and right_state == 0):
                right_tNow = time.time()
                right_speed_measuring = True
                
            left_prev_state = left_state
            left_state = left_encoder.get()
            right_prev_state = right_state
            right_state = right_encoder.get()
            
            if(left_speed != None and right_speed != None and left_ideal_speed != None and right_ideal_speed != None and left_ideal_speed != 0):
                left_pi_speed, left_i_error = pi_controller(left_ideal_speed, left_speed, left_i_error, left_tPrev - left_tNow)
                right_pi_speed, right_i_error = pi_controller(right_ideal_speed, right_speed, right_i_error, right_tPrev - right_tNow)
                print([left_pi_speed, right_pi_speed])
                print("updating")
                update_motors(left_pi_speed, right_pi_speed, LUTS)
                left_speed = None
                right_speed = None
                """
            if(left_speed != None and right_speed != None and left_ideal_speed != None and right_ideal_speed != None and time.time() >= updateTime):
                left_p_speed = p_controller(left_ideal_speed, left_p_speed, left_speed)
                right_p_speed = p_controller(right_ideal_speed, right_p_speed, right_speed)
                update_motors(left_p_speed, right_p_speed, LUTS)
                print([left_p_speed, right_p_speed])
                updateTime = time.time() + UPDATETIMESTEP
                print("updating")
                """
            
        
        if(time.time() >= reportTime):
            #print([left_speed, right_speed])
            reportTime = time.time() + UPDATETIMESTEP
        
        
        if(not Moving and not Reading and not Sending):
            data = None
            addr = None
        
        try:
            data, addr = sockreceiver.recvfrom(4096)#1024) # buffer size is 1024 bytes
            Message = readData(data)
            print("received message: %s" % Message)#floatMessage)
        except Exception as e:
            garbageVarIHatePython = 1
        
        # We wait for a short amount of time while sending 0.0 pulse
        # before jumping in to the control loop
#        print("Press enter to continue")
#        input()
        if(data != None):
            testMode = Message[0]
            if testMode == 0: #do things to stop
                Moving = False
                Done = True
            elif testMode == -1: #do things to abort
                print("hi")
                update_motors(0, 0, LUTS)
                Moving = False
                tStopRot = None
                tStop = None
            elif testMode == 1: #do things to move in curve #super broken rn #depricated as of tuesday 6/6/2023
                if(Moving == False):
                    tStop = time.time() + Message[3]
                    #update_motors(25, 25)
                    left_ideal_speed = Message[1]
                    left_p_speed = left_ideal_speed
                    right_ideal_speed = Message[2]
                    right_p_speed = right_ideal_speed
                    update_motors(Message[1], Message[2], LUTS)
                    Looping = True
                    while Looping == True:
                        if(time.time() >= tStop):
                            left_speed_measuring = False
                            right_speed_measuring = False
                            update_motors(0, 0, LUTS)
                            Looping = False
                Moving = False
            elif testMode == 2: #do things to move straight
                Moving = True
                if(tStopRot == None):
                    tStopRot, left_rot_speed, right_rot_speed = rotate(Message[1], LUTS)
                    left_ideal_speed = left_rot_speed
                    left_p_speed = left_ideal_speed
                    right_ideal_speed = right_rot_speed
                    right_p_speed = right_ideal_speed
                    update_motors(left_rot_speed, right_rot_speed, LUTS)
                elif(time.time() >= tStopRot):
                    if(tStop == None):
                        left_rot_speed = None
                        right_rot_speed = None
                        left_speed_measuring = False
                        left_speed = None
                        right_speed_measuring = False
                        right_speed = None
                        left_i_error = 0
                        right_i_error = 0
                        left_ideal_speed = Message[2]
                        left_p_speed = left_ideal_speed
                        right_ideal_speed = Message[2]
                        right_p_speed = right_ideal_speed
                        update_motors(Message[2], Message[2], LUTS)
                        tStop = time.time() + Message[3]
                    elif(time.time() >= tStop):
                        Moving = False
                        tStopRot = None
                        tStop = None
                        left_speed_measuring = False
                        left_speed = None
                        right_speed_measuring = False
                        right_speed = None
                        left_i_error = 0
                        right_i_error = 0
                        update_motors(0, 0, LUTS)
                        waiting_for_acc = True
                        sending = True
                        while sending:
                            try:
                                print("sending")
                                socksender.sendall(DoneMovingMessage)
                                sending = False
                            except Exception as e:
                                garbageVarIHatePython = 1
                                continue
                        print("sent")
                        """
                        while(waiting_for_acc == True):
                            sock.sendto(DoneMovingMessage, addr)
                            WaitAccTime = time.time() + WAITACCSTEP
                            while(time.time() < WaitAccTime):
                                try:
                                    data, addr = sock.recvfrom(4096)
                                    if(data.decode('utf-8') == "ACCDONE"):
                                        waiting_for_acc = False
                                except Exception as e:
                                    garbageVarIHatePython = 1
                                    continue
                        """
                        waiting_for_acc = True
                        left_ideal_speed = 0#None
                        left_p_speed = left_ideal_speed
                        right_ideal_speed = 0#None
                        right_p_speed = right_ideal_speed
            elif testMode == 3: #do things to move straight and then turn
                Moving = True
                if(tStopRot == None):
                    tStopRot, left_rot_speed, right_rot_speed = rotate(Message[1], LUTS)
                    left_ideal_speed = left_rot_speed
                    left_p_speed = left_ideal_speed
                    right_ideal_speed = right_rot_speed
                    right_p_speed = right_ideal_speed
                    update_motors(left_rot_speed, right_rot_speed, LUTS)
                elif(time.time() >= tStopRot):
                    if(tStop == None):
                        left_rot_speed = None
                        right_rot_speed = None
                        left_speed_measuring = False
                        left_speed = None
                        right_speed_measuring = False
                        right_speed = None
                        left_i_error = 0
                        right_i_error = 0
                        left_ideal_speed = Message[2]
                        left_p_speed = left_ideal_speed
                        right_ideal_speed = Message[2]
                        right_p_speed = right_ideal_speed
                        update_motors(Message[2], Message[2], LUTS)
                        tStop = time.time() + Message[3]
                    elif(time.time() >= tStop):
                        if(tStopRot2 == None):
                            right_rot_speed = None
                            left_speed_measuring = False
                            left_speed = None
                            right_speed_measuring = False
                            right_speed = None
                            left_i_error = 0
                            right_i_error = 0
                            tStopRot2, left_rot_speed, right_rot_speed = rotate(Message[4], LUTS)
                            left_ideal_speed = left_rot_speed
                            left_p_speed = left_ideal_speed
                            right_ideal_speed = right_rot_speed
                            right_p_speed = right_ideal_speed
                            update_motors(left_rot_speed, right_rot_speed, LUTS)
                        elif(time.time() >= tStopRot2):
                            Moving = False
                            tStopRot = None
                            tStopRot2 = None
                            tStop = None
                            left_speed_measuring = False
                            left_speed = None
                            right_speed_measuring = False
                            right_speed = None
                            left_i_error = 0
                            right_i_error = 0
                            update_motors(0, 0, LUTS)
                            waiting_for_acc = True
                            sending_loc = True
                            while sending_loc:
                                try:
                                    print("sending")
                                    socksender.sendall(DoneMovingMessage)
                                    sending_loc = False
                                except Exception as e:
                                    garbageVarIHatePython = 1
                                    continue
                            print("sent")
                            waiting_for_acc = True
                            left_ideal_speed = 0#None
                            left_p_speed = left_ideal_speed
                            right_ideal_speed = 0#None
                            right_p_speed = right_ideal_speed
                        """
                        while(waiting_for_acc == True):
                            sock.sendto(DoneMovingMessage, addr)
                            WaitAccTime = time.time() + WAITACCSTEP
                            while(time.time() < WaitAccTime):
                                try:
                                    data, addr = sock.recvfrom(4096)
                                    if(data.decode('utf-8') == "ACCDONE"):
                                        waiting_for_acc = False
                                except Exception as e:
                                    garbageVarIHatePython = 1
                                    continue
                        """
            elif testMode == 4:
                update_motors(0, 0, LUTS)
                #print("collecting")
                if(Reading == False):
                    Reading = True
                    IR_readings = []
                    readTime = time.time() + READTIMESTEP
                    num_IR_points = 0
                else:
                    if(time.time() >= readTime):
                        readTime = time.time() + READTIMESTEP
                        if(num_IR_points < Message[1]):
                            print(["collecting", num_IR_points])
                            curr_readings = str(IR_PIN.get_raw()) + ', ' + str(IR_PIN.get_voltage())
                            #[IR_PIN.get_raw(), IR_PIN.get_voltage()]
                            IR_readings.append(curr_readings)
                            num_IR_points = num_IR_points + 1
                        else:
                            sending_loc = True
                            while sending_loc:
                                try:
                                    socksender.sendall(DoneCollectingMessage)
                                    sending_loc = False
                                except Exception as e:
                                    garbageVarIHatePython = 1
                                    continue
                            Reading = False
            elif testMode == 5:
                print("Sending")
                #print(len(IR_readings))
                Sending = True
                if(IR_reading_index < len(IR_readings)):
                    sending_loc = True
                    while sending_loc:
                        try:
                            #print(IR_reading_index)
                            if(IR_reading_index == 0):
                                socksender.setblocking(1)
                                #print(bytes(str(len(IR_readings)), 'utf-8'))
                                socksender.sendall(bytes(str(len(IR_readings)), 'utf-8'))
                                socksender.sendall(bytes(str(2), 'utf-8'))
                            socksender.sendall(bytes(str(IR_readings[IR_reading_index]), 'utf-8'))
                            IR_reading_index = IR_reading_index + 1
                            sending_loc = False
                        except Exception as e:
                            garbageVarIHatePython = 1
                            continue
                else:
                    socksender.setblocking(0)
                    IR_readings = []
                    IR_reading_index = 0
                    Sending = False
                    print("done sending data")
                        
                
            """
        Moving = True
        while Moving:
            testMode = Message[0]
            if testMode == 0: #do things to stop
                Moving = False
                Done = True
            elif testMode == -1: #do things to abort
                print("hi")
                Moving = False
            elif testMode == 1: #do things to move in curve
                tStop = time.time() + Message[3]
                #update_motors(25, 25)
                update_motors(Message[1], Message[2], LUTS)
                Looping = True
                while Looping == True:
                    if(time.time() >= tStop):
                        update_motors(0, 0, LUTS)
                        Looping = False
                Moving = False
            elif testMode == 2: #do things to move straight
                rotate(Message[1], LUTS)
                update_motors(Message[2], Message[2], LUTS)
                tStop = time.time() + Message[3]
                Looping = True
                while Looping:
                    if(time.time() >= tStop):
                        update_motors(0, 0, LUTS)
                        Looping = False
                Moving = False
                sock.sendto(DoneMovingMessage, addr)
                """
        
    """
    x = input()
    
    while(x != 'exit'):
        x = input()
    """
    
    """
    while True:
        # Get Readings from encoders for comparison
        left_reading = left_encoder.get()
        right_reading = right_encoder.get()
    
        # Check if the wheel has spun
        if left_state != left_reading:
            left_state = left_reading
            # If the wheel has spun two states then we know the distance
            if left_state == left_start_state:
                # Update time
                left_tPrev = left_tNow
                left_tNow = time.time()
                
                # Calculate Speed and error
                speed = calculate_speed(left_tPrev, left_tNow)
                left_prev_error = left_curr_error
                left_curr_error = calculate_error(speed, left_motor_speed)
                left_integral_error = calculate_ierror(left_integral_error, left_curr_error, left_tPrev, left_tNow)
                left_derivative_error = calculate_derror(left_curr_error, left_prev_error, left_tPrev, left_tNow)
                print("Left Speed: {}, Left Setting: {}, Left Error: {}, Left Ierror: {} Left Derror: {}".format(speed, left_motor_setting, left_curr_error, left_integral_error, left_derivative_error))
                # Control
                #left_motor_setting = pid_control(0.5, 1, 0.006443, left_curr_error, left_integral_error, left_derivative_error, left_motor_setting, LEFT_MAX, 0)
                #left_motor_setting = pid_control(0.8833, 1.408, 0.006443, left_curr_error, left_integral_error, left_derivative_error, left_motor_setting, 21.924028091423587, 0)
                #left_motor_setting = pid_control(.1, .2, .01, left_curr_error, left_integral_error, left_derivative_error, left_motor_setting, 21.924028091423587, 0)
                leftServo.set(left_motor_setting)
    
        if right_state != right_reading:
            right_state = right_reading
            if right_state == right_start_state:
                right_tPrev = right_tNow
                right_tNow = time.time()
                
                speed = calculate_speed(right_tPrev, right_tNow)
                left_prev_error = left_curr_error
                right_curr_error = calculate_error(speed, right_motor_speed)
                right_integral_error = calculate_ierror(right_integral_error, right_curr_error, right_tPrev, right_tNow)
                right_derivative_error = calculate_derror(right_curr_error, right_prev_error, right_tPrev, right_tNow)
                print("Right Speed: {}, Right Setting: {}, Right Error: {}, Right Ierror: {} Right Derror: {}".format(speed, right_motor_setting, right_curr_error, right_integral_error, right_derivative_error))
                
                #right_motor_setting = pid_control(0.5, 1, 0.006443, right_curr_error, right_integral_error, right_derivative_error, right_motor_setting, RIGHT_MAX, 1)
                #right_motor_setting = pid_control(0.8833, 1.408, 0.006443, right_curr_error, right_integral_error, right_derivative_error, right_motor_setting, -20.9995597347149, 1)
                #right_motor_setting = pid_control(.1, .2, .01, right_curr_error, right_integral_error, right_derivative_error, right_motor_setting, -20.9995597347149, 1)
                rightServo.set(right_motor_setting)
    """
    
    clk0.stop()
    clk1.stop()
    servo.disable()

if __name__ == "__main__":
    main()

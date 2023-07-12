#performs a test to generate the look up tables that are used to controll the speed of the robots servos
import time
from angleToIntercept import * 

import socket

import numpy
import Adafruit_BBIO.UART as UART
import serial
from time import sleep
import rcpy.servo as servo
import rcpy.gpio as gpio

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

import smbus
from colorCheck import *
import csv

left_encoder_pin = 25
right_encoder_pin = 17
encoder_chip = 1
left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
right_encoder = gpio.Input(encoder_chip, right_encoder_pin)
#with 5 gaps in the wheel, the encoder will change its reading from 0-1 5 times per rotation
servo.enable()

period = 0.02
leftServo.set(0)
rightServo.set(0)
clk0 = leftServo.start(period)
clk1 = rightServo.start(period)
GAPS = 5
gapCount = 0
#time_per_ex = 10 #in seconds
right_curr = right_encoder.get()
right_prev = right_curr
left_curr = left_encoder.get()
left_prev = left_curr
startTime = 0
#endTime = startTime + time_per_ex
testDuties = [-1.5, -1.4, -1.3, -1.2, -1.1, -1, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.33, -0.3, -0.22, -0.2, -0.11, -0.1, -0.09, -0.08, -0.07, -0.06, -0.05, -0.04, -0.03, -0.02, -0.01, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.2, 0.22, 0.3, 0.33, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5]
results = [[3]] * len(testDuties)
testNum = 0
expTime = 0
timeStep = 0.005
lastTimeCheck = time.time()
nextTimeCheck = lastTimeCheck + timeStep
lastQuantum = False
currQuantum = False
for i in testDuties:
    gapCount = 0
    rightServo.set(i)
    startTime = 0
    expTime = 0
    while(startTime == 0):
        print("hi")
        if(right_prev == 1 and right_curr == 0):
            startTime = time.time()
            #endTime = startTime + time_per_ex
            print('starting')
        right_prev = right_curr
        right_curr = right_encoder.get()
    print(i)
    while(gapCount < 10):
        if(right_prev == 1 and right_curr == 0):
            gapCount = gapCount + 1
        right_prev = right_curr
        right_curr = right_encoder.get()
    expTime = time.time() - startTime
    results[testNum] = [i, (testDuties[testNum]*.6) + 1.5,(gapCount/GAPS)/(expTime/60)]
    testNum = testNum + 1
rightServo.set(0)

file = open('RightServoResponseNew.csv', 'w')

writer = csv.writer(file)

"""
header = ['hello', 'my', 'name', 'is', 'jeff']
header2 = ['and', 'my', 'name', 'is', 'kelly']
writer.writerow(header)
writer.writerow(header2)
"""

for i in range(len(testDuties)):
    writer.writerow(results[i])
file.close()


results = [[3]] * len(testDuties)
testNum = 0
expTime = 0
timeStep = 0.005
lastTimeCheck = time.time()
nextTimeCheck = lastTimeCheck + timeStep
lastQuantum = False
currQuantum = False
for i in testDuties:
    gapCount = 0
    leftServo.set(i)
    startTime = 0
    expTime = 0
    while(startTime == 0):
        if(left_prev == 1 and left_curr == 0):
            startTime = time.time()
            #endTime = startTime + time_per_ex
            print('starting')
        left_prev = left_curr
        left_curr = left_encoder.get()
    print(i)
    while(gapCount < 10):
        if(left_prev == 1 and left_curr == 0):
            gapCount = gapCount + 1
        left_prev == left_curr
        left_curr = left_encoder.get()
    expTime = time.time() - startTime
    results[testNum] = [i, (testDuties[testNum]*.6) + 1.5,(gapCount/GAPS)/(expTime/60)]
    testNum = testNum + 1
leftServo.set(0)

file = open('LeftServoResponseNew.csv', 'w')

writer = csv.writer(file)
"""
header = ['hello', 'my', 'name', 'is', 'jeff']
header2 = ['and', 'my', 'name', 'is', 'kelly']
writer.writerow(header)
writer.writerow(header2)
"""

for i in range(len(testDuties)):
    writer.writerow(results[i])
file.close()



clk0.stop()
clk1.stop()
servo.disable()
"""
x = input()

while(x != 'exit'):
    print(x)
    leftServo.set(float(x))
    rightServo.set(-float(x))
    x = input()
while prev_state == curr_state:
    curr_state = right_encoder.get()
print(prev_state)
print(curr_state)
clk0.stop()
clk1.stop()
servo.disable()
"""
"""
servo.enable()

period = 0.02
leftServo.set(0)
rightServo.set(0)
clk0 = leftServo.start(period)
clk1 = rightServo.start(period)

x = input()

while(x != 'exit'):
    print(x)
    leftServo.set(float(x))
    rightServo.set(-float(x))
    x = input()
"""


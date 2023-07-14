#unit test for the IR sensor
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

IR_PIN = adc.ADC(0)

#IR_encoder = gpio.Input(IR_CHIP, IR_PIN)

READTIMESTEP = 0.2
readTime = time.time() + READTIMESTEP

while True:
    if(time.time() > readTime):
        print([IR_PIN.get_raw(), IR_PIN.get_voltage()])
        readTime = time.time() + READTIMESTEP

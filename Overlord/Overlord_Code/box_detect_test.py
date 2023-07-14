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

def main():
    while True:
        frame = tags.get_frame()
        out = frame
        """
        edges = cv2.Canny(image=frame, threshold1=100, threshold2=200)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = numpy.float32(gray)
        """
#         dst = cv2.cornerHarris(gray, 2, 3, 0.04)
#         dst = cv2.dilate(dst, None)
#         ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
        """
        ret, thresh = cv2.threshold(edges, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        """
#         dst = numpy.uint8(dst)
        
#         ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
        
#         criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
#         corners = cv2.cornerSubPix(gray, numpy.float32(centroids), (5, 5), (-1, -1), criteria)
        
#         res = numpy.hstack((centroids, corners))
#         res = numpy.int0(res)
#         frame[res[:, 1], res[:, 0]] = [0, 0, 255]
#         frame[res[:, 3], res[:, 2]] = [0, 255, 0]
        """
        minLineLength = 50
        maxLineGap = 20
        lines = cv2.HoughLinesP(edges, 1, pi/180, 100, minLineLength, maxLineGap)
        for x1, y1, x2, y2 in lines[0]:
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        """
        
#         cv2.imshow('Canny Edge Detection', edges)
#         cv2.imshow('Lines', out)
        cv2.imshow("DEBUG", out)
    
if __name__ == "__main__":
    main()

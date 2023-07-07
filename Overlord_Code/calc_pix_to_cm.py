import os
import time
from math import atan2, degrees, radians, pi, cos, sin, sqrt, atan, tan
import cv2
import sys
import os
import tags
import apriltag
import socket

def calc_dist(c):
    return abs(sqrt(float((pow((c[0][0] - c[1][0]), 2) + pow((c[0][1] - c[1][1]), 2)))))

def mark_debug(frame, c):
    out = frame
    for i in range(2):
        cv2.circle(out, (int(c[i][0]), int(c[i][1])), 5, (0, 255, 0), -1)
        #cv2.circle(out, (int(x_t[i]), int(y_t[i])), 15, (255, 0, 0), 1)
    cv2.imshow("DEBUG", out)

def main():
    c = [[None]*2]*2
    frame = None
    while (any(elem is None for elem in c[0]) or any(elem is None for elem in c[1])):
        frame = tags.get_frame()
        results = tags.find_tags(frame)
        centers = tags.get_bot_centers(results, 2)
#         angles = tags.get_bot_angles(results, 2)
        if (len(centers) == 2):
            for i in range(2) :
                if (centers[i][0] == -1):
                    continue
                c[i][0] = centers[i][0]
                c[i][1] = centers[i][1]
    frame_mod = frame
    print(c)
    mark_debug(frame, c)
    
if __name__ == "__main__":
    main()
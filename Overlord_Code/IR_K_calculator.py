import random
from statistics import mean as mu
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
import csv
# import seaborn as sns; sns.set()
import pandas as pd
# from random import randrange

import math
# from numpy.random import normal
from numpy.random import seed
from scipy.stats import norm
# from tabulate import tabulate

from scipy.integrate import simps
from numpy import trapz
from matplotlib.animation import FuncAnimation
from random import randint
from csv import reader

file = open('IR_data_90_2.csv', 'r')

def trainingMean(mu, d):
    # ------------ distance column ---------------------
#     d = np.array([4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21])
    print("Distance array shape: ", end="")
#     print(d.shape)

    # ------------ A matrix ---------------------
    n = len(d)
    column_1 = np.full(n, 1)
    A = (np.matrix([column_1, np.log(d)])).T
    print("A: ")
    print(A)
    print(A.T)
    # ------------ A matrix ---------------------
    print("Shape of A: ", end="")
#     print(A.shape)
    # ------------ v matrix ---------------------
    y = np.log(mu)
    print("Shape of v: ", end="")
#     print(y.shape)

    # ------------ Start to find solutoin -------
    c = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), y)
    c = (c.tolist())[0]
    print("c: ")
    print(c)
    k1 = np.exp(c[0])
    k2 = c[1]
    print("k1: ", end="")
    print(k1)
    print("k2: ", end="")
    print(k2)
    return [k1, k2]

def trainingStd(std, d):
    # ------------ distance column ---------------------
#     d = np.array([4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21])
    print("Distance array shape: ", end="")
#     print(d.shape)

    # ------------ A matrix ---------------------
    n = len(d)
    column_1 = np.full(n, 1)
    A = (np.matrix([column_1, d])).T
    print("A: ")
    print(A)
    # ------------ A matrix ---------------------
    print("Shape of A: ", end="")
#     print(A.shape)
    # ------------ v matrix ---------------------
    y = np.log(std)
    print("Shape of v: ", end="")
#     print(y.shape)

    # ------------ Start to find solutoin -------
    c = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), y)
    c = (c.tolist())[0]
    print("c: ")
    print(c)
    k1 = np.exp(c[0])
    k2 = c[1]
    print("k1: ", end="")
    print(k1)
    print("k2: ", end="")
    print(k2)
    return [k1, k2]


if __name__ == "__main__":
    mean = []
    std = []
#     file = open('IR_data_90_1.csv', 'r')
    reader = csv.reader(file)
    
    listReader = list(reader)
#     print(listReader)
    
    data = []
    measurement = []
    d = []
#     distance_index = -1
    
    distance = 0
    for row in listReader:
        for elem in row:
            elem = float(elem)
#             print(elem)
#         print(row[2])
        if row[2] != distance:
            if(measurement != []):
                data_row = []
                data_row.append(float(distance))
                print(measurement)
                data_row.append(float(np.array(measurement).mean()))
                data_row.append(float(np.array(measurement).std()))
                data.append(data_row)
                d.append(float(distance))
            distance = row[2]
            measurement = []
#             distance_index = distance_index + 1
        measurement.append(float(row[0]))
    
    data_row = []
    data_row.append(float(listReader[len(listReader)-1][2]))
    data_row.append(float(np.array(measurement).mean()))
    data_row.append(float(np.array(measurement).std()))
    data.append(data_row)
    d.append(float(listReader[len(listReader)-1][2]))
    
#     result = data.reverse()
    
    
    last_mean = data[0][1]
    d = []
    data_new = []
    copying = False
    for i in reversed(range(len(data))):
        copying = False
        if(data[i][0] > 8):
            copying = True
        if(copying):
            d.append(data[i][0])
            data_new.append(data[i])
        last_mean = data[i][1]
    data = data_new
    
    """
    d = []
    for i in range(len(data)):
        if(data[i][0] > 9):
            d.append(data[i][0])
    """
    print("printing d")
    print(d)
        
            
#     exclude_index = 0
    """
    for i in range(len(d)):
        if d[i] < 10:
            exclude_index = i
    """
#     d = d[exclude_index + 1:len(d) + 1]
            
    d_min = 1000
    d_max = 0
    for elem in d:
        if elem < d_min:
            d_min = elem
        if elem > d_max:
            d_max = elem

    mean = []
    std = []
    
    for row in data:
        mean.append(float(row[1]))
        std.append(float(row[2]))
    k = trainingMean(np.array(mean), d)    # For the mean
    j = trainingStd(std, d)                # For the std

#     d = np.array([4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21])
    # Plotting Mean estimation
    fig = plt.figure(figsize=(14, 9))
    x_distance = np.arange(d_min,d_max,0.1)
    y_estimate = k[0]*(x_distance**k[1])

    plt.scatter(d, mean, color='orange')
    plt.plot(x_distance, y_estimate, color='green')

    # Plotting std estimation
    fig = plt.figure(figsize=(14, 9))
    x_distance = np.arange(d_min, d_max, 0.1)
    y_estimate = j[0] * ( np.exp(j[1]*x_distance))

    plt.scatter(d, std, color='orange')
    plt.plot(x_distance, y_estimate, color='green')
    plt.show()
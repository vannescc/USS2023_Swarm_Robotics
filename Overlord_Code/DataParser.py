import matplotlib.pyplot as plt
import numpy
import socket
import csv

file = open('SPI_par_data_5.csv', 'r')
reader = csv.reader(file)

listReader = list(reader)
 
numRows = len(listReader)
testNum = []*numRows
target_distance = []* numRows
distance_traveled = []*numRows
target_angle = []* numRows
angle_turned = []*numRows
distance_offset = []*numRows
angle_offset = []*numRows

for i in range(numRows):
    testNum.append(int(listReader[i][1]))
    target_distance.append(float(listReader[i][2]))
    distance_traveled.append(float(listReader[i][3]))
    target_angle.append(float(listReader[i][4]))
    angle_turned.append(float(listReader[i][5]))

#normalize angle loop
for i in range(numRows):
    if(target_angle[i] < 0 and angle_turned[i] > 0):
        angle_turned[i] = angle_turned[i] - 360
    elif(target_angle[i] > 0 and angle_turned[i] < 0):
        angle_turned[i] = angle_turned[i] + 360
#analysis loop
for i in range(numRows):
    distance_offset.append(target_distance[i] - distance_traveled[i])
    angle_offset.append(target_angle[i] - angle_turned[i])
fig,ax = plt.subplots()
ax.scatter(target_distance, distance_offset)
ax.set(xlabel='Target Distance(dcm)', ylabel='Distance Offset(cm)', title='Distance Offset vs Target Distance PI par')
ax.grid()
fig.savefig('Distance Offset vs Target Distance PI par.png')
plt.show()

file.close()
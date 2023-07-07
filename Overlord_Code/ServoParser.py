import matplotlib.pyplot as plt
import numpy
import socket
import csv

file = open('RightServoResponseNew.csv', 'r')
reader = csv.reader(file)

listReader = list(reader)
 
numRows = len(listReader)
duty = []*numRows
p_width = []* numRows
RPM = []*numRows

for i in range(numRows):
    duty.append(float(listReader[i][0]))
    p_width.append(float(listReader[i][1]))
    RPM.append(float(listReader[i][2]))

fig,ax = plt.subplots()
ax.plot(duty, RPM)
ax.set(xlabel='duty', ylabel='RPM', title='Right Servo Response New duty vs RPM')
ax.grid()
fig.savefig('Right Servo Response New duty vs RPM.png')
plt.show()

file.close()
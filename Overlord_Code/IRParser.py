import matplotlib.pyplot as plt
import numpy
import socket
import csv

file = open('IR_data_90_2.csv', 'r')
reader = csv.reader(file)

listReader = list(reader)
 
numRows = len(listReader)
ints = []*numRows
volts = []* numRows
distance = []*numRows
AoA = []*numRows

for i in range(numRows):
    ints.append(float(listReader[i][0]))
    volts.append(float(listReader[i][1]))
    distance.append(float(listReader[i][2]))
    AoA.append(float(listReader[i][3]))

fig,ax = plt.subplots()
ax.scatter(distance, ints)
ax.set(xlabel='IRval', ylabel='Distance(cm)', title='IR val vs Distance')
ax.grid()
fig.savefig('IR val vs Distance.png')
plt.show()

file.close()
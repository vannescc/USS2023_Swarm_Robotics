import matplotlib.pyplot as plt
import numpy
import socket
import csv
import statistics as stat
import math

file = open('IR_data_7_6.csv', 'r')
reader = csv.reader(file)

listReader = list(reader)

numRows = len(listReader)
readDist = []
actualDist = []

desired_dist = 30


actDists = []
readDists = []

bins = []
heights = [0] * 51
for i in range(51):
    bins.append(i)

for i in range(numRows):
    readDist.append(float(listReader[i][0]))
    actualDist.append(float(listReader[i][1]))
    if(actualDist[i] >= desired_dist - 0.5 and actualDist[i] < desired_dist + 0.5):
        if(readDist[i] > 50):
            readDist[i] = 50
        readDists.append(readDist[i])
        if(actualDist[i] not in actDists):
            actDists.append(actualDist[i])

for i in range(len(readDists)):
    tempVal = round(readDists[i])
#     if tempVal > 50:
#         tempVal = 50
    heights[tempVal] += 1
    
# print(heights)


print(actDists)

mean = stat.mean(readDists)
tempvar = 0
for i in range(len(readDists)):
    tempvar += ((readDists[i] - mean)**2)
sigma = math.sqrt(tempvar/len(readDists))

s = numpy.random.normal(mean, sigma, 1000)

count, bins2, ignored = plt.hist(s, 50, density=True)
fig, ax = plt.subplots()
ax.bar(bins, heights, width=1, edgecolor="white", linewidth=0.7)
plt.plot(bins2, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (bins2 - mean)**2 / (2 * sigma**2) ) * len(readDists), linewidth=2, color='r')
plt.show()


file.close()
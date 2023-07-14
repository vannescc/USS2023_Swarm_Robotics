import matplotlib.pyplot as plt
import numpy
import socket
import csv
import statistics as stat
import math
from scipy.stats import norm

# def prob(mean, sigma)

file = open('IR_data_7_6.csv', 'r')
reader = csv.reader(file)

listReader = list(reader)

numRows = len(listReader)
readDist = []
actualDist = []

desired_dist = 6


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

zStar = stat.mean(readDists)

phit = norm.pdf(bins, zStar, sigma)
# print(phit[6]*len(readDists))

plong = []
for i in range(51):
    if(i != 50):
        plong.append(0)
    else:
        plong.append(1)

# print(phit)
tempval = 0
for i in range(51):
#     eta = 1/(phit[i])
#     eihit = eta * phit[i]
#     if(phit[i] == 0):
    if(False):
        eihit = 0
    else:
#         eta = 1/(phit[i] + float(plong[i]))
#         eihit = eta * phit[i]
        eihit = phit[i]/(phit[i] + float(plong[i]) + .001)
#     print(eihit)
#     print(phit)
    tempVal += eihit
whit = tempVal/51
for j in range(1):
    sum_eihit = 0
    for i in range(51):
#         eta = 1/(phit[i])
#         eihit = eta * phit[i]
#         if(phit[i] == 0):
        if(False):
            eihit = 0
        else:
#             eta = 1/(phit[i] + float(plong[i]))
#             eihit = eta * phit[i]
            eihit = phit[i]/(phit[i] + float(plong[i]) + .001)
        sum_eihit += eihit
    sum_eihit2 = 0
    for i in range(51):
#         eta = 1/(phit[i])
#         eihit = eta * phit[i]
#         if(phit[i] == 0):
        if(False):
            eihit = 0
        else:
#             eta = 1/(phit[i] + float(plong[i]))
#             eihit = eta * phit[i]
            eihit = phit[i]/(phit[i] + float(plong[i]) + .001)
        sum_eihit2 += eihit * (i -zStar)**2
#     print(sum_eihit)
    sigma = numpy.sqrt((1/sum_eihit) * sum_eihit2)
    phit = norm.pdf(bins, zStar, sigma)
    tempval = 0
    for i in range(51):
#         eta = 1/(phit[i])
#         eihit = eta * phit[i]
#         if(phit[i] == 0):
        if(False):
            eihit = 0
        else:
#             eta = 1/(phit[i] + float(plong[i]))
#             eihit = eta * phit[i]
            eihit = phit[i]/(phit[i] + float(plong[i]) + .001)
        print(eihit)
        tempVal += eihit
    whit = tempVal/51


# s = numpy.random.normal(mean, sigma, 1000)

# count, bins2, ignored = plt.hist(s, 50, density=True)

        

    
        
fig, ax = plt.subplots()
ax.bar(bins, heights, width=1, edgecolor="white", linewidth=0.7)
# plt.plot(bins, (norm.pdf(bins, mean, sigma) + plong) * len(readDists))
# plt.plot(bins, (norm.pdf(bins, zStar, sigma) + plong) * len(readDists))
plt.plot(bins, (norm.pdf(bins, zStar, sigma)) * len(readDists)) #whit
print(sigma)
# plt.plot(bins2, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (bins2 - mean)**2 / (2 * sigma**2) ) * len(readDists), linewidth=2, color='r')
plt.show()


file.close()

import matplotlib as plt
import numpy
import socket
import csv

UDP_IP = "172.24.87.109"
UDP_PORT = 5005

file = open('RightServoResponseNew.csv', 'w')
writer = csv.writer(file)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
data, addr = sock.recvfrom(1024)
numRows = int(data.decode('utf'))
print("received message: %s" % numRows)
data, addr = sock.recvfrom(1024)
numCol = int(data.decode('utf'))
print("received message: %s" % numCol)
Results = [[]*numCol]*numRows
for i in range(numRows):
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    message = data.decode('utf').split(',')
    floatMessage = []*len(message)
    for f in message:
        floatMessage.append(float(f))
    Results[i] = floatMessage
    writer.writerow(Results[i])
    print("received message: %s" % floatMessage)
duty = []*numRows
p_width = []* numRows
RPM = []*numRows
"""
for i in range(numRows):
    duty[i] = Results[i][0]
    p_width = Results[i][1]
    RPM = Results[i][2]
fig,ax = plt.subplots()
ax.plot(p_width, RPM)

plt.show()
"""
file.close()
import socket
import time
"""
UDP_IP = "172.25.255.118"  #"192.168.8.1"#"172.25.249.107"#"10.33.50.33"#"0.0.0.0"#"127.0.0.1"
UDP_PORT = 5005#8080
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
#sock.bind((UDP_IP, UDP_PORT))
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
sock.setblocking(0)
# time.sleep(10)
while True:
    time.sleep(1)
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("Hi")
    if(data.decode('utf-8') == "received your message"):
        print("received Message: %s" % data)
sock.close()
"""
# sock = socket.socket(socket.AF_INET, # Internet
#                      socket.SOCK_DGRAM) # UDP
sock = socket.socket()
sock.bind(("172.25.251.2", 8080))
sock.listen()
conn, addr = sock.accept()
conn.sendall(b"hello")
messagereceived = False
while True:
    data = conn.recv(1024) # buffer size is 1024 bytes
    if(not messagereceived):
        print("Hi")
    else:
        print("yo")
    if(data.decode('utf-8') == "received your message"):
        print("received Message: %s" % data)
        messagereceived = True
    else:
        print(data.decode('utf-8'))
    if(data.decode('utf-8') == ""):
        print("nothing to see here")

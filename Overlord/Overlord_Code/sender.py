import socket

UDP_IP = "172.24.87.204"#"172.25.255.118"#"192.168.8.1"#"172.25.249.107"#"10.33.50.33"#"0.0.0.0"#"127.0.0.1"
UDP_PORT = 5005#8080
intmess = 178
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

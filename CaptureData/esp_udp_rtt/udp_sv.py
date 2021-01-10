import socket

UDP_IP = "192.168.0.7"
UDP_PORT = 3333

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(4) # buffer size is 1024 bytes
    print("received message: %s" % data)
    sock.sendto(data, addr)
    print("resent")
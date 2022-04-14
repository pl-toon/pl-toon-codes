import sys
import socket
import struct
import time
import subprocess

if len(sys.argv) < 2:
	filename = "default.csv"
else:
    filename = sys.argv[1]

#creacion de archivo CSV
if '-a' in sys.argv:
    mode = 'a'
else:
    mode = 'w'

fileOut = open(filename, mode)
fileOut.close()

#UDP
UDP_IP = "192.168.1.100" # ip del computador que recibe datos (mismo que el que corre este script)
UDP_PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

t0 = int(round(time.time() * 1000))     # tiempo en ms

n_pkt = 0

if '-g' in sys.argv:
    the_other_process = subprocess.Popen('python matplot_realtime.py '+filename, shell=True)

while True:
    try:
        data, addr = sock.recvfrom(1024) # buffer size es 1024 bytes
        n_double = int( len(data)/8 )
        n_double_str = 'd'*n_double
        values = struct.unpack(n_double_str, data)
    
        timestamp = int(round(time.time() * 1000)) - t0

        # id,timestamp(local),distance,velocity,extras...
        fileOut = open(filename, 'a')
        print(int(values[0]), file = fileOut, end=',')  # escribe ID de carro
        print(timestamp, file=fileOut, end='')          # escribe el timestamp
        for v in values[1:]:
            print(',' + str(v), file=fileOut, end='')
        print('', file=fileOut)
        fileOut.close()

        n_pkt += 1
        print('Packets Received: ' + str(n_pkt), end='\r')
    except KeyboardInterrupt:
        break
print('\nFin.')

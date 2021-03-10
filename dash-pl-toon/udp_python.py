"""
_____________________________________________________
|   -Conexion de UDP con la base de datos en MySQL  |
|    -Crear archivo CSV                             |
|---------------------------------------------------|

"""
#Librerias
import sys
import paho.mqtt.client as mqtt
import pymysql
import csv
import socket
#Ajustables
file_name = "C:\\Users\\sapet\\Dropbox\\git\\dash_tren2\\prueba_4.csv"  # archivo csv

UDP_IP = "192.168.100.12" # ip del computador que recibe datos (mismo que el que corre este script)
UDP_PORT = 5555
#UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#creacion de archivo CSV
texto = open(file_name,'w')
texto.write('time_event,input,referencia,error,kp,ki,kd,output_PID'+'\n')
texto.close()

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    testo = str(data.decode('utf-8'))
    lista = testo.split(",")
    texto = open(file_name,"a")
    texto.write(testo+'\n')
    texto.close()
    print(testo)
    
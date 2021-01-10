import serial
import sys

if len(sys.argv) < 2:
	print("Please specify the port to open.\nExample: python read_serial.py 'ttyUSB0'")
	sys.exit(0)

port = '/dev/'
port += sys.argv[1]

esp = serial.Serial(port, 115200)

if esp.is_open:
	while True:
		data = esp.readline()
		print(data.decode('utf-8'), end='')



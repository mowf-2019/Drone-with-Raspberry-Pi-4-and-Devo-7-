#!/usr/bin/python3

import time
import serial

ser = serial.Serial(
	port = '/dev/ttyUSB0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0
)

print("Raspberry's sending : ")
a=100.6
try:
    while True:
    	ser.write(b'data=%f\r'%a)
    	ser.flush()
#     	print("hehe")
    	time.sleep(1)
except KeyboardInterrupt:
	ser.close()
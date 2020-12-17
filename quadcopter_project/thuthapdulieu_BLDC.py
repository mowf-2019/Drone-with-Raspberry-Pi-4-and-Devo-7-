import os
import time
import math
import numpy as np
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
import serial
from gpiozero import MCP3202
pi = pigpio.pi()
pot = MCP3202(channel=0)

ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

# ser = serial.Serial(
#     port = '/dev/ttyUSB0',
#     baudrate = 115200,
#     parity = serial.PARITY_NONE,
#     stopbits = serial.STOPBITS_ONE,
#     bytesize = serial.EIGHTBITS,
#     timeout = 0
# )

time1 = 0
time_pre = 0
x=1000
pi.set_servo_pulsewidth(ESC1, x)
pi.set_servo_pulsewidth(ESC2, x)
pi.set_servo_pulsewidth(ESC3, x)
pi.set_servo_pulsewidth(ESC4, x)
time.sleep(5)
def ConvertVolts(data,places):
  volts = (data * 9.7) / 2.3
  volts = round(volts,places)
  return volts

def Rps2Pulse4(rps):  #Motor4
    pulse = rps/0.6154 + 779
    if (pulse<500):
        pulse=500
    if (pulse>2000):
        pulse=2000
    return pulse

def my_callback(gpio,level,tick):  #tick (us)
    global time1, time_pre
    time1 = tick - time_pre
    time_pre=tick
  
pi.callback(5, pigpio.RISING_EDGE, my_callback)

end=0
# while True:
#     dt=(time.time() - end)
#     if (dt >= 0.005):
#         end=time.time()
#         if time1 != 0:
#             omega=math.pi/(time1/1000000.0)
#         else:
#             omega=0
#         
#         pi.set_servo_pulsewidth(ESC4, x)
#         x += 0.5
#         ser.write(b'%f %f\r\n'%(omega,x))
#     if x>1800:
#         break
# pi.set_servo_pulsewidth(ESC4, 1000)
# print('stop')
# ser.flush()
i=0
while True:
    pi.set_servo_pulsewidth(ESC4, Rps2Pulse4(500))
    lipo_volt = ConvertVolts(pot.value*3.3,2)
    if time1 != 0:
        omega=math.pi/(time1/1000000.0)
    else:
        omega=0
    print(omega,lipo_volt)
    time.sleep(1)
    i += 1
    if i==15:
        break
pi.set_servo_pulsewidth(ESC4, 1000)
        
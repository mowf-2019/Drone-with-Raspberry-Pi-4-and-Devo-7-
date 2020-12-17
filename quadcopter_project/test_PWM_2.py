import os
import time
import math
# os.system("sudo pigpiod -s 1") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall ppigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
from gpiozero import MCP3202


ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

pi = pigpio.pi()
pot = MCP3202(channel=0)

pi.set_servo_pulsewidth(ESC1, 1000)
pi.set_servo_pulsewidth(ESC2, 1000)
pi.set_servo_pulsewidth(ESC3, 1000)
pi.set_servo_pulsewidth(ESC4, 1000)
time.sleep(5)

def ConvertVolts(data,places):
  volts = (data * 9.7) / 2.3
  volts = round(volts,places)
  return volts

def Rps2Pulse1(rps):  #Motor1
    pulse = rps/0.7856 + 1115
    return pulse

def Rps2Pulse2(rps):  #Motor2
    pulse = rps/0.7782 + 1125
    return pulse

def Rps2Pulse3(rps):  #Motor3
    pulse = rps/0.7938 + 1121
    return pulse

def Rps2Pulse4(rps):  #Motor4
    pulse = rps/0.7635 + 1118
    return pulse

while True:
    for i in range(0,10):
        pi.set_servo_pulsewidth(ESC1, 1000 + i*80)
        pi.set_servo_pulsewidth(ESC2, 1000 + i*80)
        pi.set_servo_pulsewidth(ESC3, 1000 + i*80)
        pi.set_servo_pulsewidth(ESC4, 1000 + i*80)
        
        lipo_volt = ConvertVolts(pot.value*3.3,2)
        print(pot.value*3.3,lipo_volt)
        time.sleep(3)
    pi.set_servo_pulsewidth(ESC1, 1000)
    pi.set_servo_pulsewidth(ESC2, 1000)
    pi.set_servo_pulsewidth(ESC3, 1000)
    pi.set_servo_pulsewidth(ESC4, 1000)
    break


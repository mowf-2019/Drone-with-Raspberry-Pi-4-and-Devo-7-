import RPi.GPIO as GPIO
import time
import numpy as np

ELEV = 12
AILE = 16
THRO = 20
RUDD = 21
# inPINS = [20,21,4,14,15,18,17,27,22,23]
inPINS = [ELEV, AILE, THRO, RUDD]

def getTimex():
    return time.time()

GPIO.setmode (GPIO.BCM)
GPIO.setup(inPINS, GPIO.IN)

upTimes=0
downTimes=0
deltaTimes=0
deltaTimes_pre=0

def my_callback1(channel):
  global upTimes, downTimes, deltaTimes, deltaTimes_pre
  i = inPINS.index(channel)
  v = GPIO.input(inPINS[i])
  
  if (v==0):
    downTimes=getTimex()
  else:
    upTimes=getTimex()

  deltaTimes=(downTimes-upTimes)*1000000
  if (deltaTimes<0):
    deltaTimes=deltaTimes_pre
  deltaTimes_pre=deltaTimes

# GPIO.add_event_detect(inPINS[0], GPIO.BOTH, callback=my_callback1)
# GPIO.add_event_detect(inPINS[1], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[2], GPIO.BOTH, callback=my_callback1)
# GPIO.add_event_detect(inPINS[3], GPIO.BOTH, callback=my_callback1)

roll_setpoint_fil =0
pitch_setpoint_fil =0
yaw_setpoint_fil =0
throttle_fil =0
try:
  while True:
    throttle_fil = throttle_fil*0.8 + deltaTimes*0.2
    print(throttle_fil)
    time.sleep(0.1)
except KeyboardInterrupt:
  GPIO.cleanup()
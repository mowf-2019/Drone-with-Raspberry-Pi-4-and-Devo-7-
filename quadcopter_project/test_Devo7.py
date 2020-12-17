import RPi.GPIO as GPIO
import time
import numpy as np

ELEV = 12
AILE = 16
THRO = 20
RUDD = 21
# inPINS = [20,21,4,14,15,18,17,27,22,23]
inPINS = [ELEV, AILE, THRO, RUDD]
smoothingWindowLength=4

min_ch0=600
max_ch0=1000
min_ch1=600
max_ch1=1000
min_ch2=600
max_ch2=1000
min_ch3=600
max_ch3=1000
roll_setpoint_fil=0
pitch_setpoint_fil=0
yaw_setpoint_fil=0
throttle_fil=0

def calculateSetpoint(ch0,ch1,ch2,ch3):
    global throttle_fil, roll_setpoint_fil, pitch_setpoint_fil, yaw_setpoint_fil
    r_min = -30
    r_max = 30
    p_min = -30
    p_max = 30
    y_min = -30
    y_max = 30
    t_min = 0
    t_max = 600
    if (ch0<min_ch0):
        ch0=min_ch0
    elif (ch0>max_ch0):
        ch0=max_ch0
    if (ch1<min_ch1):
        ch1=min_ch1
    elif (ch1>max_ch1):
        ch1=max_ch1
    if (ch2<min_ch2):
        ch2=min_ch2
    elif (ch2>max_ch2):
        ch2=max_ch2
    if (ch3<min_ch3):
        ch3=min_ch3
    elif (ch3>max_ch3):
        ch3=max_ch3
    roll_setpoint = (ch3 - min_ch3) * (r_max - r_min) / (max_ch3 - min_ch3) + r_min
    pitch_setpoint = (ch0 - min_ch0) * (p_max - p_min) / (max_ch0 - min_ch0) + p_min
    yaw_setpoint = (ch1 - min_ch1) * (y_max - y_min) / (max_ch1 - min_ch1) + y_min
    throttle = (ch2 - min_ch2) * (t_max - t_min) / (max_ch2 - min_ch2) + t_min
    roll_setpoint_fil = roll_setpoint_fil*0.8 + roll_setpoint*0.2
    pitch_setpoint_fil = pitch_setpoint_fil*0.8 + pitch_setpoint*0.2
    yaw_setpoint_fil = yaw_setpoint_fil*0.8 + yaw_setpoint*0.2
    throttle_fil = throttle_fil*0.8 + throttle*0.2
    return (roll_setpoint_fil,pitch_setpoint_fil,yaw_setpoint_fil,throttle_fil)

def getTimex():
    return time.time()

GPIO.setmode (GPIO.BCM)
GPIO.setup(inPINS, GPIO.IN)

upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]

def my_callback1(channel):
  i = inPINS.index(channel)
  v = GPIO.input(inPINS[i])
  #GPIO.output(outPINS[0], v) # mirror input state to output state directly (forward servo value only) - don't set PWM then for this pin
  if (v==0):
    downTimes[i].append(getTimex())
    if len(downTimes[i])>smoothingWindowLength: del downTimes[i][0]
  else:
    upTimes[i].append(getTimex())
    if len(upTimes[i])>smoothingWindowLength: del upTimes[i][0]
#   deltaTimes[i].append( (downTimes[i][-1]-upTimes[i][-2])/(upTimes[i][-1]-downTimes[i][-1]) )
  deltaTimes[i].append( (downTimes[i][-1]-upTimes[i][-2]))
  if len(deltaTimes[i])>smoothingWindowLength: del deltaTimes[i][0]

GPIO.add_event_detect(inPINS[0], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[1], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[2], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[3], GPIO.BOTH, callback=my_callback1)

try:
  while True:
    ovl = deltaTimes[0][-smoothingWindowLength:] # output first pin PWM
    ovl1 = deltaTimes[1][-smoothingWindowLength:] # output first pin PWM
    ovl2 = deltaTimes[2][-smoothingWindowLength:] # output first pin PWM
    ovl3 = deltaTimes[3][-smoothingWindowLength:] # output first pin PWM
    ov = sorted(ovl)[len(ovl) // 2] #ov = np.mean(ovl)
    ov1 = sorted(ovl1)[len(ovl1) // 2] #ov = np.mean(ovl)
    ov2 = sorted(ovl2)[len(ovl2) // 2]#ov2 = np.mean(ovl2)
    ov3 = sorted(ovl3)[len(ovl3) // 2] #ov = np.mean(ovl)
    (roll_setpoint,pitch_setpoint,yaw_setpoint,throttle)=calculateSetpoint(round(ov*10000,2),round(ov1*10000,2),round(ov2*10000,2),round(ov3*10000,2))
#     print (roll_setpoint,pitch_setpoint,yaw_setpoint,throttle)
#     print(roll_setpoint,pitch_setpoint,yaw_setpoint,throttle)
    print(deltaTimes[3])
    time.sleep(0.1)
except KeyboardInterrupt:
  GPIO.cleanup()
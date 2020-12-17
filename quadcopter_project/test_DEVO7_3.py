import os
import time
import numpy as np
# os.system("sudo pigpiod -s 1") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall ppigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio


pi = pigpio.pi()


ELEV = 5
AILE = 6
THRO = 13
RUDD = 12
GEAR = 16
AUX1 = 20
AUX2 = 21
# inPINS = [20,21,4,14,15,18,17,27,22,23]
down1=0
up1=0
timer1=0
def my_callback1(gpio,level1,tick1):
      global down1, up1, timer1
      if (level1==0):
        down1=tick1
      else:
        up1=tick1
      timer1=(down1-up1)
      if timer1<1200:
          timer1=1200
      elif timer1>1900:
          timer1=1900
down2=0
up2=0
timer2=0
def my_callback2(gpio,level2,tick2):
      global down2, up2, timer2
      if (level2==0):
        down2=tick2
      else:
        up2=tick2
      timer2=(down2-up2)
      if timer2<1200:
          timer2=1200
      elif timer2>1900:
          timer2=1900
down3=0
up3=0
timer3=0
def my_callback3(gpio,level3,tick3):
      global down3, up3, timer3
      if (level3==0):
        down3=tick3
      else:
        up3=tick3
      timer3=(down3-up3)
      if timer3<1200:
          timer3=1200
      elif timer3>1900:
          timer3=1900
down4=0
up4=0
timer4=0
def my_callback4(gpio,level4,tick4):
      global down4, up4, timer4
      if (level4==0):
        down4=tick4
      else:
        up4=tick4
      timer4=(down4-up4)
      if timer4<1200:
          timer4=1200
      elif timer4>1900:
          timer4=1900
down5=0
up5=0
timer5=0
def my_callback5(gpio,level5,tick5):
      global down5, up5, timer5
      if (level5==0):
        down5=tick5
      else:
        up5=tick5
      timer5=(down5-up5)
      if timer5<1200:
          timer5=1200
      elif timer5>1900:
          timer5=1900

  
pi.callback(ELEV, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AILE, pigpio.EITHER_EDGE, my_callback2)
pi.callback(THRO, pigpio.EITHER_EDGE, my_callback3)
pi.callback(RUDD, pigpio.EITHER_EDGE, my_callback4)
pi.callback(GEAR, pigpio.EITHER_EDGE, my_callback5)


while True:
    print(timer1,timer2,timer3,timer4,timer5)
    time.sleep(1)
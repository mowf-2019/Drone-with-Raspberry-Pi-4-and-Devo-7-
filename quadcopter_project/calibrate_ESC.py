import time
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio


ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

THRO = 5

pi = pigpio.pi()
# pi.set_servo_pulsewidth(ESC1, 2000)
# pi.set_servo_pulsewidth(ESC2, 2000)
# pi.set_servo_pulsewidth(ESC3, 2000)
# pi.set_servo_pulsewidth(ESC4, 2000)
# time.sleep(3)

pi.set_servo_pulsewidth(ESC1, 1000)
pi.set_servo_pulsewidth(ESC2, 1000)
# pi.set_servo_pulsewidth(ESC3, 1000)
pi.set_servo_pulsewidth(ESC4, 1000)
time.sleep(3)

upTimes=0
downTimes=0
deltaTimes=0
deltaTimes_pre_4=0
throttle =0

def my_callback1(gpio,level,tick):
  global upTimes, downTimes, deltaTimes,deltaTimes_pre_4, throttle
  
  if (level==0):
    downTimes=tick
  else:
    upTimes=tick

  deltaTimes=(downTimes-upTimes)
  if gpio==THRO:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_4
      deltaTimes_pre_4=deltaTimes
      if deltaTimes<1200:
          deltaTimes=1200
      elif deltaTimes>1900:
          deltaTimes=1900
      throttle=(deltaTimes - 1200) * (1000 - 0) / (1900 - 1200) + 0

pi.callback(THRO, pigpio.EITHER_EDGE, my_callback1)

print("Done")
while True:
    pi.set_servo_pulsewidth(ESC1, throttle + 1000)
    pi.set_servo_pulsewidth(ESC2, throttle + 1000)
    pi.set_servo_pulsewidth(ESC3, throttle + 1000)
    pi.set_servo_pulsewidth(ESC4, throttle + 1000)
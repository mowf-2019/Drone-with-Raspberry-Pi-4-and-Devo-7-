import os
import time
import math
import numpy as np
import RPi.GPIO as GPIO
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
from gpiozero import MCP3202
import L3G4200D_lib
import ADXL345_lib
import HMC5883L_lib
import Kalman

# Kp_roll=0.7
# Kd_roll=0
# Ki_roll=0.1

Kp_roll=1
Kd_roll=0.1
Ki_roll=0.05

Kp_pitch=Kp_roll
Kd_pitch=Kd_roll
Ki_pitch=Ki_roll

Kp_yaw=3
Kd_yaw=0
Ki_yaw=0.01

ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

ELEV = 5
AILE = 6
THRO = 13
RUDD = 12
GEAR = 16
AUX1 = 20
AUX2 = 21

inPINS = [ELEV, AILE, THRO, RUDD, GEAR, AUX1, AUX2]

pi = pigpio.pi()
pot = MCP3202(channel=0)

l3g4200d = L3G4200D_lib.L3G4200D()
adxl345 = ADXL345_lib.ADXL345()
hmc5883l = HMC5883L_lib.HMC5883L()
kalmanX = Kalman.KalmanAngle()
kalmanY = Kalman.KalmanAngle()

pi.set_servo_pulsewidth(ESC1, 1000)
pi.set_servo_pulsewidth(ESC2, 1000)
pi.set_servo_pulsewidth(ESC3, 1000)
pi.set_servo_pulsewidth(ESC4, 1000)
time.sleep(3)

#Calculate lipo voltage
def ConvertVolts(data,places):
  volts = (data * 9.76) / 2.24
  volts = round(volts,places)
  return volts

#Convert speed (rps) to pulse
def Rps2Pulse1(rps):  #Motor1
    pulse = rps/0.6053 + 770
    if (pulse<1000):
        pulse=1000
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def Rps2Pulse2(rps):  #Motor2
    pulse = rps/0.6194 + 774
    if (pulse<1000):
        pulse=1000
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def Rps2Pulse3(rps):  #Motor3
    pulse = rps/0.5871 + 759
    if (pulse<1000):
        pulse=1000
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def Rps2Pulse4(rps):  #Motor4
    pulse = rps/0.6154 + 779
    if (pulse<1000):
        pulse=1000
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def PID_roll(gyroX,roll_setpoint,dt):
    global roll_i, last_roll_error
    roll_error = gyroX - roll_setpoint
    if throttle > 0:
        roll_i += roll_error
    elif throttle ==0:
        roll_i=0
    if roll_i > 30:
        roll_i=30
    elif roll_i <-30:
        roll_i=-30
    pid_roll_output = Kp_roll*roll_error + Ki_roll*roll_i*dt + Kd_roll*(roll_error - last_roll_error)/dt
    if (pid_roll_output > 50):
        pid_roll_output = 50
    elif (pid_roll_output < -50):
        pid_roll_output = -50
    last_roll_error=roll_error
    return pid_roll_output
    
def PID_pitch(gyroY,pitch_setpoint,dt):
    global pitch_i, last_pitch_error
    pitch_error = gyroY - pitch_setpoint
    if throttle > 0:
        pitch_i += pitch_error
    elif throttle==0:
        pitch_i =0
    if pitch_i > 100:
        pitch_i=100
    elif pitch_i <-100:
        pitch_i=-100
    pid_pitch_output = Kp_pitch*pitch_error + Ki_pitch*pitch_i*dt + Kd_pitch*(pitch_error - last_pitch_error)/dt
    if (pid_pitch_output > 100):
        pid_pitch_output = 100
    elif (pid_pitch_output < -100):
        pid_pitch_output = -100
    last_pitch_error=pitch_error
    return pid_pitch_output

def PID_yaw(gyroZ,yaw_setpoint,dt):
    global yaw_i, last_yaw_error
    yaw_error = gyroZ - yaw_setpoint
    if throttle > 300:
        yaw_i += yaw_error
    elif throttle==0:
        yaw_i = 0
    if yaw_i > 30:
        yaw_i=30
    elif yaw_i <-30:
        yaw_i=-30
    
    pid_yaw_output = Kp_yaw*yaw_error + Ki_yaw*yaw_i + Kd_yaw*(yaw_error - last_yaw_error)
    if (pid_yaw_output > 50):
        pid_yaw_output = 50
    elif (pid_yaw_output < -50):
        pid_yaw_output = -50
    last_yaw_error=yaw_error
    return pid_yaw_output

end=0
roll_i=0.0
last_roll_error=0.0
pitch_i=0.0
last_pitch_error=0.0
yaw_i=0.0
last_yaw_error=0.0
throttle=0.0

class BUTTERWORTH:
    def __init__(self, sampling, cutoff, order, primer):

        self.n = int(round(order / 2))
        self.A = []
        self.d1 = []
        self.d2 = []
        self.w0 = []
        self.w1 = []
        self.w2 = []

        a = math.tan(math.pi * cutoff / sampling)
        a2 = math.pow(a, 2.0)

        for ii in range(0, self.n):
            r = math.sin(math.pi * (2.0 * ii + 1.0) / (4.0 * self.n))
            s = a2 + 2.0 * a * r + 1.0
            self.A.append(a2 / s)
            self.d1.append(2.0 * (1 - a2) / s)
            self.d2.append(-(a2 - 2.0 * a * r + 1.0) / s)

            self.w0.append(primer / (self.A[ii] * 4))
            self.w1.append(primer / (self.A[ii] * 4))
            self.w2.append(primer / (self.A[ii] * 4))

    def filter(self, input):
        for ii in range(0, self.n):
            self.w0[ii] = self.d1[ii] * self.w1[ii] + self.d2[ii] * self.w2[ii] + input
            output = self.A[ii] * (self.w0[ii] + 2.0 * self.w1[ii] + self.w2[ii])
            self.w2[ii] = self.w1[ii]
            self.w1[ii] = self.w0[ii]

        return output
    
def read_sensor(dt):
    global gyroX, gyroY, gyroZ, roll, pitch, yaw
    (Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
    gyroX = gyroX*0.8 + Xrate*0.2
    gyroY = gyroY*0.8 + Yrate*0.2
    gyroZ = gyroZ*0.8 + Zrate*0.2
#     gyroX = bfx.filter(Xrate)
#     gyroY = bfy.filter(Yrate)
#     gyroZ = bfz.filter(Zrate)
    adxl345.readNormalize()
        
    accX_angle = math.atan(adxl345.nYAxis/math.sqrt(math.pow(adxl345.nXAxis,2) + math.pow(adxl345.nZAxis,2))) * 180/math.pi
    accY_angle = math.atan(-1*adxl345.nXAxis/math.sqrt(math.pow(adxl345.nYAxis,2) + math.pow(adxl345.nZAxis,2))) * 180/math.pi
    roll =0.98*(roll + gyroX*dt) + 0.02*(accX_angle - off_roll)
    pitch=0.98*(pitch + gyroY*dt) + 0.02*(accY_angle - off_pitch)
          
    heading=hmc5883l.readheading(roll*math.pi/180,pitch*math.pi/180) - off_heading
    if heading>=180 and heading<360:
        yaw = heading - 360
    else:
        yaw = heading


upTimes=0
downTimes=0
deltaTimes=0
deltaTimes_pre_1=0
deltaTimes_pre_2=0
deltaTimes_pre_3=0
deltaTimes_pre_4=0
deltaTimes_pre_5=0
roll_setpoint=0
pitch_setpoint=0
yaw_setpoint=0
gear=0

def my_callback1(gpio,level,tick):
  global upTimes, downTimes, deltaTimes, deltaTimes_pre_1, deltaTimes_pre_2, deltaTimes_pre_3, deltaTimes_pre_4, deltaTimes_pre_5, throttle, roll_setpoint, pitch_setpoint, yaw_setpoint, gear
  
  if (level==0):
    downTimes=tick
  else:
    upTimes=tick

  deltaTimes=(downTimes-upTimes)
  if gpio==ELEV:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_1
      deltaTimes_pre_1=deltaTimes
      pitch_setpoint=round((deltaTimes-1500 + 500)*(20+20) / (500 + 500) - 20,2)
      if abs(pitch_setpoint)<0.1:
          pitch_setpoint=0
  if gpio==RUDD:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_2
      deltaTimes_pre_2=deltaTimes
      roll_setpoint=round((deltaTimes-1500 + 500)*(20+20) / (500 + 500) - 20,2)
      if abs(roll_setpoint)<0.1:
          roll_setpoint=0
  if gpio==AILE:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_3
      deltaTimes_pre_3=deltaTimes
      yaw_setpoint=round((deltaTimes-1500 + 500) * (20 +20) / (500 + 500) - 20,2)
      if abs(yaw_setpoint)<0.1:
          yaw_setpoint=0
  if gpio==THRO:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_4
      deltaTimes_pre_4=deltaTimes
      if deltaTimes<1200:
          deltaTimes=1200
      elif deltaTimes>1900:
          deltaTimes=1900
      throttle=(deltaTimes - 1200) * (600 - 0) / (1900 - 1200) + 0
  if gpio==GEAR:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_5
      deltaTimes_pre_5=deltaTimes
      if deltaTimes < 1300:
          gear=0
      elif deltaTimes > 1700:
          gear=1
  
pi.callback(ELEV, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AILE, pigpio.EITHER_EDGE, my_callback1)
pi.callback(THRO, pigpio.EITHER_EDGE, my_callback1)
pi.callback(RUDD, pigpio.EITHER_EDGE, my_callback1)
pi.callback(GEAR, pigpio.EITHER_EDGE, my_callback1)

print ("Waiting for calibration")
l3g4200d.calibrate(2000)
(Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
gyroX=0
gyroY=0
gyroZ=0
bfx = BUTTERWORTH(200, 10, 8, gyroX)
bfy = BUTTERWORTH(200, 10, 8, gyroY)
bfz = BUTTERWORTH(200, 10, 8, gyroZ)

off_heading=0
off_pitch=0
off_roll=0
roll=0
pitch=0
for i in range(0,500):
    off_heading+=hmc5883l.readheading(0,0)
#     adxl345.readNormalize()
#     off_roll += math.atan(adxl345.nYAxis/math.sqrt(math.pow(adxl345.nXAxis,2) + math.pow(adxl345.nZAxis,2))) * 180/math.pi
#     off_pitch += math.atan(-1*adxl345.nXAxis/math.sqrt(math.pow(adxl345.nYAxis,2) + math.pow(adxl345.nZAxis,2))) * 180/math.pi
# off_roll /= 500
# off_pitch /= 500
off_heading /= 500
off_roll = 1.26938
off_pitch = -1.56524



lipo_volt_pre=0
print ("Calibrate done")
end2=0

pid_roll_setpoint=0
pid_pitch_setpoint=0
while True:
    dt=(time.time() - end)
    dt2=(time.time() - end2)

    if (dt >= 0.005):   #Sample rate 100Hz
        end = time.time()
        
        read_sensor(dt)

        lipo_volt = ConvertVolts(pot.value*5,2)
#         if gear==1:
#             pid_roll_setpoint -= roll*10
#             pid_pitch_setpoint -= pitch*10
#             pid_roll_setpoint /= 3
#             pid_pitch_setpoint /=3
#         else:
#             pid_roll_setpoint=0
#             pid_pitch_setpoint=0
        
        if gear==1:
            pid_roll_setpoint = -(roll-roll_setpoint)*2
            pid_pitch_setpoint = -(pitch-pitch_setpoint)*2
#             pid_roll_setpoint /= 3
#             pid_pitch_setpoint /=3
        else:
            pid_roll_setpoint=0
            pid_pitch_setpoint=0
        
        PIDroll=PID_roll(gyroX,pid_roll_setpoint,dt)
        PIDpitch=PID_pitch(gyroY,pid_pitch_setpoint,dt)
        PIDyaw=PID_yaw(gyroZ,0,dt)
#         esc_1_rps = throttle + PIDpitch + PIDroll - PIDyaw
#         esc_2_rps = throttle + PIDpitch - PIDroll + PIDyaw
#         esc_3_rps = throttle - PIDpitch - PIDroll - PIDyaw
#         esc_4_rps = throttle - PIDpitch + PIDroll + PIDyaw
        
        esc_1_rps = throttle + PIDpitch
        esc_2_rps = throttle + PIDpitch
        esc_3_rps = throttle - PIDpitch
        esc_4_rps = throttle - PIDpitch

        off_rps1 = (12.6-lipo_volt)*15
        off_rps2 = (12.6-lipo_volt)*15
        off_rps3 = (12.6-lipo_volt)*15
        off_rps4 = (12.6-lipo_volt)*15
        
        if gear < 0.5:
            pi.set_servo_pulsewidth(ESC1, 1000)
            pi.set_servo_pulsewidth(ESC2, 1000)
            pi.set_servo_pulsewidth(ESC3, 1000)
            pi.set_servo_pulsewidth(ESC4, 1000)
        else:
            pi.set_servo_pulsewidth(ESC1, Rps2Pulse1(esc_1_rps+off_rps1))
            pi.set_servo_pulsewidth(ESC2, Rps2Pulse2(esc_2_rps+off_rps2))
            pi.set_servo_pulsewidth(ESC3, Rps2Pulse3(esc_3_rps+off_rps3))
            pi.set_servo_pulsewidth(ESC4, Rps2Pulse4(esc_4_rps+off_rps4))
            
    if dt2>0.5:
#         print(lipo_volt,yaw,esc_1_rps,esc_2_rps,esc_3_rps,esc_4_rps)
        print(roll,pitch,yaw,PIDpitch,throttle)
        end2=time.time()



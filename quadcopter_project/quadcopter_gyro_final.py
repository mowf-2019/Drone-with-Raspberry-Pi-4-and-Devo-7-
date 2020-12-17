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

#PID parameters
# Kp_roll=2
# Kd_roll=1
# Ki_roll=0.0001
# 
# Kp_pitch=2
# Kd_pitch=1
# Ki_pitch=0.0001
# 
# Kp_yaw=2
# Kd_yaw=0.5
# Ki_yaw=0

Kp_roll=2
Kd_roll=18
Ki_roll=0.08

Kp_pitch=2
Kd_pitch=1
Ki_pitch=0.0001

Kp_yaw=2
Kd_yaw=0.5
Ki_yaw=0

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
  volts = (data * 9.7) / 2.3
  volts = round(volts,places)
  return volts

#Convert speed (rps) to pulse
def Rps2Pulse1(rps):  #Motor1
    pulse = rps/0.7856 + 1127
    if (pulse<500):
        pulse=500
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def Rps2Pulse2(rps):  #Motor2
    pulse = rps/0.7782 + 1125
    if (pulse<500):
        pulse=500
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def Rps2Pulse3(rps):  #Motor3
    pulse = rps/0.7938 + 1130
    if (pulse<500):
        pulse=500
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def Rps2Pulse4(rps):  #Motor4
    pulse = rps/0.7635 + 1118
    if (pulse<500):
        pulse=500
    if (pulse>2000):
        pulse=2000
    return round(pulse)

def PID_roll(roll,roll_setpoint,dt):
    global roll_i, last_roll_error
    roll_error = roll - roll_setpoint
    if throttle !=0:
        roll_i += roll_error
    elif throttle ==0:
        roll_i=0
    pid_roll_output = Kp_roll*roll_error + Ki_roll*roll_i + Kd_roll*(roll_error - last_roll_error)
    if (pid_roll_output > 100):
        pid_roll_output = 100
    elif (pid_roll_output < -100):
        pid_roll_output = -100
    last_roll_error=roll_error
    return pid_roll_output
    
def PID_pitch(pitch,pitch_setpoint,dt):
    global pitch_i, last_pitch_error
    pitch_error = pitch - pitch_setpoint
    if throttle !=0:
        pitch_i += pitch_error
    elif throttle==0:
        pitch_i =0
    pid_pitch_output = Kp_pitch*pitch_error + Ki_pitch*pitch_i + Kd_pitch*(pitch_error - last_pitch_error)
    if (pid_pitch_output > 100):
        pid_pitch_output = 100
    elif (pid_pitch_output < -100):
        pid_pitch_output = -100
    last_pitch_error=pitch_error
    return pid_pitch_output

def PID_yaw(yaw,yaw_setpoint,dt):
    global yaw_i, last_yaw_error
    yaw_error = yaw - yaw_setpoint
    yaw_i += yaw_error
    pid_yaw_output = Kp_yaw*yaw_error + Ki_yaw*yaw_i + Kd_yaw*(yaw_error - last_yaw_error)
    if (pid_yaw_output > 100):
        pid_yaw_output = 100
    elif (pid_yaw_output < -100):
        pid_yaw_output = -100
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

        #Calculate roll, pitch, yaw angle
adxl345.readNormalize()
pitch_raw = -(math.atan2(adxl345.nXAxis, math.sqrt(adxl345.nYAxis*adxl345.nYAxis + adxl345.nZAxis*adxl345.nZAxis))*180.0)/math.pi;
roll_raw  = (math.atan2(adxl345.nYAxis, adxl345.nZAxis)*180.0)/math.pi;
kalmanX.setAngle(roll_raw)
kalmanY.setAngle(pitch_raw)

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
      pitch_setpoint=round((deltaTimes-1500 + 500)*(10+10) / (500 + 500) - 10,2)
      if abs(pitch_setpoint)<0.1:
          pitch_setpoint=0
  if gpio==RUDD:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_2
      deltaTimes_pre_2=deltaTimes
      roll_setpoint=round((deltaTimes-1500 + 500)*(10+10) / (500 + 500) - 10,2)
      if abs(roll_setpoint)<0.1:
          roll_setpoint=0
  if gpio==AILE:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_3
      deltaTimes_pre_3=deltaTimes
      yaw_setpoint=round((deltaTimes-1500 + 500) * (10 +10) / (500 + 500) - 10,2)
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
l3g4200d.calibrate(100)
off_heading=0
off_pitch=0
off_roll=0
for i in range(0,100):
    off_heading+=hmc5883l.readheading(0,0)
    adxl345.readNormalize()
    (Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
    off_pitch += -(math.atan2(adxl345.nXAxis, math.sqrt(adxl345.nYAxis*adxl345.nYAxis + adxl345.nZAxis*adxl345.nZAxis))*180.0)/math.pi;
    off_roll  += (math.atan2(adxl345.nYAxis, adxl345.nZAxis)*180.0)/math.pi;
off_pitch /= 100
off_roll /= 100
off_heading /= 100

gyroX_pre=0
gyroY_pre=0
gyroZ_pre=0
lipo_volt_pre=0
while lipo_volt_pre<10:
    lipo_volt_pre= ConvertVolts(pot.value*3.3,2)
print ("Calibrate done")
end2=0
while True:
    dt=(time.time() - end)
    dt2=(time.time() - end2)

    if (dt >= 0.01):   #Sample rate 100Hz
        end = time.time()
        #Calculate roll, pitch, yaw angle
        (Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
        gyroX = gyroX_pre*0.8 + 0.2*Xrate #LPF
        gyroY = gyroY_pre + 0.1*(Yrate - gyroY_pre) #LPF
        gyroZ = gyroZ_pre + 0.1*(Zrate - gyroZ_pre) #LPF
        gyroX_pre=gyroX
        gyroY_pre=gyroY
        gyroZ_pre=gyroZ
        adxl345.readNormalize()

        pitch_raw = -(math.atan2(adxl345.nXAxis, math.sqrt(adxl345.nYAxis*adxl345.nYAxis + adxl345.nZAxis*adxl345.nZAxis))*180.0)/math.pi;
        roll_raw  = (math.atan2(adxl345.nYAxis, adxl345.nZAxis)*180.0)/math.pi;
        roll = kalmanX.getAngle(roll_raw,Xrate,dt) - off_roll
        pitch = kalmanY.getAngle(pitch_raw,Yrate,dt) - off_pitch
        if abs(roll) > 90 or abs(pitch) >90:
            roll=0
            pitch=0
            
        heading=hmc5883l.readheading(roll*math.pi/180,pitch*math.pi/180) - off_heading
        if heading>=180 and heading<360:
            yaw = heading - 360
        else:
            yaw = heading

        lipo_volt = ConvertVolts(pot.value*3.3,2)
        
#         esc_1_rps = throttle + PID_pitch(gyroY,pitch_setpoint,dt) + PID_roll(gyroX,roll_setpoint,dt) - PID_yaw(-yaw,0,dt)
#         esc_2_rps = throttle + PID_pitch(gyroY,pitch_setpoint,dt) - PID_roll(gyroX,roll_setpoint,dt) + PID_yaw(-yaw,0,dt)
#         esc_3_rps = throttle - PID_pitch(gyroY,pitch_setpoint,dt) - PID_roll(gyroX,roll_setpoint,dt) - PID_yaw(-yaw,0,dt)
#         esc_4_rps = throttle - PID_pitch(gyroY,pitch_setpoint,dt) + PID_roll(gyroX,roll_setpoint,dt) + PID_yaw(-yaw,0,dt)
        xc=PID_roll(gyroX,roll_setpoint,dt)
        esc_1_rps = throttle + xc# - PID_yaw(-yaw,0,dt)
        esc_2_rps = throttle - xc# + PID_yaw(-yaw,0,dt)
        esc_3_rps = throttle - xc# - PID_yaw(-yaw,0,dt)
        esc_4_rps = throttle + xc# + PID_yaw(-yaw,0,dt)
        off_rps1 = (esc_1_rps/300)*(11-lipo_volt)*30
        off_rps2 = (esc_2_rps/300)*(11-lipo_volt)*30
        off_rps3 = (esc_3_rps/300)*(11-lipo_volt)*30
        off_rps4 = (esc_4_rps/300)*(11-lipo_volt)*30
        
        if gear < 0.5:
            pi.set_servo_pulsewidth(ESC1, 1000)
            pi.set_servo_pulsewidth(ESC2, 1000)
            pi.set_servo_pulsewidth(ESC3, 1000)
            pi.set_servo_pulsewidth(ESC4, 1000)
        else:
            pi.set_servo_pulsewidth(ESC1, Rps2Pulse1(round(esc_1_rps+off_rps1)))
            pi.set_servo_pulsewidth(ESC2, Rps2Pulse2(round(esc_2_rps+off_rps2)))
            pi.set_servo_pulsewidth(ESC3, Rps2Pulse3(round(esc_3_rps+off_rps3)))
            pi.set_servo_pulsewidth(ESC4, Rps2Pulse4(round(esc_4_rps+off_rps4)))
            
    if dt2>0.5:
        print(lipo_volt,xc,esc_1_rps,esc_2_rps,esc_3_rps,esc_4_rps)
        end2=time.time()
        
    if throttle==0 and yaw_setpoint>8 and roll_setpoint<8 and pitch_setpoint>8:
        break
os.system("sudo shutdown -h now")


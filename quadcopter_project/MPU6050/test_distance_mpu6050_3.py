import os
import smbus
import time
import math
import serial
import MPU6050_lib
import GY_86_lib
import MS5611_lib
import Rotation_lib
import Butterworth_lib
import PID_lib
import RPS2PULSE as R2P
import LinearAcc_lib
import numpy as np
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio

ELEV = 13
AILE = 6
THRO = 5
RUDD = 16
GEAR = 12
AUX1 = 20
AUX2 = 21
inPINS = [ELEV, AILE, THRO, RUDD, GEAR, AUX1, AUX2]
pi = pigpio.pi()

ser = serial.Serial(
  port = '/dev/ttyUSB0',
  baudrate = 115200,
  parity = serial.PARITY_NONE,
  stopbits = serial.STOPBITS_ONE,
  bytesize = serial.EIGHTBITS,
  timeout = 0
)

motion_rate = 200
adc_frequency = 1000
FULL_FIFO_BATCHES=20
sampling_rate=500
GRAV_ACCEL = 9.80665

upTimes=0
downTimes=0
deltaTimes=0
deltaTimes_pre_1=0
deltaTimes_pre_2=0
deltaTimes_pre_3=0
deltaTimes_pre_4=0
deltaTimes_pre_5=0
deltaTimes_pre_6=0
throttle =0
roll_setpoint=0
pitch_setpoint=0
yaw_setpoint=0
gear=0
MIX=0

def my_callback1(gpio,level,tick):
  global upTimes, downTimes, deltaTimes, deltaTimes_pre_1, deltaTimes_pre_2, deltaTimes_pre_3, deltaTimes_pre_4, deltaTimes_pre_5, deltaTimes_pre_6, throttle, roll_setpoint, pitch_setpoint, yaw_setpoint, gear, MIX
  
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
      if abs(pitch_setpoint)<0.5:
          pitch_setpoint=0
  if gpio==RUDD:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_2
      deltaTimes_pre_2=deltaTimes
      roll_setpoint=round((deltaTimes-1500 + 500)*(20+20) / (500 + 500) - 20,2)
      if abs(roll_setpoint)<0.5:
          roll_setpoint=0
  if gpio==AILE:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_3
      deltaTimes_pre_3=deltaTimes
      yaw_setpoint=round((deltaTimes-1500 + 500) * (20 +20) / (500 + 500) - 20,2)
      if abs(yaw_setpoint)<0.5:
          yaw_setpoint=0
  if gpio==THRO:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_4
      deltaTimes_pre_4=deltaTimes
      if deltaTimes<1200:
          deltaTimes=1200
      elif deltaTimes>1900:
          deltaTimes=1900
      throttle=(deltaTimes - 1200) * (700 - 0) / (1900 - 1200) + 0
  if gpio==GEAR:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_5
      deltaTimes_pre_5=deltaTimes
      if deltaTimes < 1300:
          gear=0
      elif deltaTimes > 1700:
          gear=1
  if gpio==AUX1:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_6
      deltaTimes_pre_6=deltaTimes
      if deltaTimes <= 1300:
          MIX=0
      elif deltaTimes > 1300 and deltaTimes <1700:
          MIX=1
      elif deltaTimes >= 1700:
          MIX=2
  
pi.callback(ELEV, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AILE, pigpio.EITHER_EDGE, my_callback1)
pi.callback(THRO, pigpio.EITHER_EDGE, my_callback1)
pi.callback(RUDD, pigpio.EITHER_EDGE, my_callback1)
pi.callback(GEAR, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AUX1, pigpio.EITHER_EDGE, my_callback1)

mpu6050 = MPU6050_lib.MPU6050(0x69, 1, 1)
gy86 = GY_86_lib.MPU6050(0x68, 1, 1)
MS5611 = MS5611_lib.MS5611(0x77)
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.flushFIFO()
# gy86.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.enableFIFOOverflowISR()

#Cablirate gyro and acceleromoter
qax = 0.0
qay = 0.0
qaz = 0.0
qrx = 0.0
qry = 0.0
qrz = 0.0
yaw_offset = 0

sigma_dt = 0.0
loops = 0
while sigma_dt < 1.0:
    time.sleep(FULL_FIFO_BATCHES / sampling_rate)
#     MS5611.readValue()
    nfb = mpu6050.numFIFOBatches()
    ax, ay, az, rx, ry, rz, dt = mpu6050.readFIFO(nfb)
    yaw_offset += gy86.readheading(0,0) #No tilt compensate
    loops += 1
    sigma_dt += dt
    qax += ax
    qay += ay
    qaz += az
    qrx += rx
    qry += ry
    qrz += rz
    
    
qax /= loops
qay /= loops
qaz /= loops
qrx /= loops
qry /= loops
qrz /= loops
yaw_offset /= loops

mpu6050.setGyroOffsets(qrx, qry, qrz)
# mpu6050.setAccelOffsets(qax, qay, qaz)
qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
lin = LinearAcc_lib.linear_Acc()
# pa_offset = -1.2178853590450544
# ra_offset = 1.0457224208647498
pa_offset = pa
ra_offset = ra
ya = 0.0


roll=0
pitch=0
yaw=0
end=0
end2=0
pressure1=0
status=0

acc_pre=0
velo=0
counterAccX= 0
velo_pre=0
x=0
start = 0

alpha = 0.15
beta = 0.95
gamma = 0.9
xAcc_Filtered=0
accFilt=0
velocity_Filtered=0

while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > (1/motion_rate):
        end = time.time()
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
        
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
        
        R = np.transpose(lin.quatern2rotMat(lin.AHRS(np.array([qrx, qry, qrz])*(math.pi/180), np.array([qax, qay, qaz]), elapse_time)))

        tcAcc = [R[0][0]*qax + R[0][1]*qay + R[0][2]*qaz, R[1][0]*qax + R[1][1]*qay + R[1][2]*qaz, R[2][0]*qax + R[2][1]*qay + R[2][2]*qaz]
        
        tcAcc = [tcAcc[0]-0, tcAcc[1]-0, tcAcc[2]-1]
        linAcc = np.array(tcAcc) * 9.81
        
        xAcc_Filtered = linAcc[0] - ((1-alpha)*xAcc_Filtered + alpha*linAcc[0])
        accFilt = (1-beta)*accFilt + beta*xAcc_Filtered
        if accFilt>-0.2 and accFilt<0.2:
            accFilt = 0
            
        if gear == 1:
            velo += (acc_pre + (accFilt - acc_pre)/2)*elapse_time
            acc_pre=accFilt
            
            velocity_Filtered = (1-gamma)*velocity_Filtered + gamma*velo
                
            x += (velo_pre + (velocity_Filtered - velo_pre)/2)*elapse_time
            velo_pre = velocity_Filtered
        
        ser.write(b'%f %f %f %f %f\r\n'%(qax*9.81,linAcc[0],accFilt, velocity_Filtered, x))
        

#     if elapse_time2>1:
#         end2=time.time()
#         print(qax, qay, qaz, linAcc)


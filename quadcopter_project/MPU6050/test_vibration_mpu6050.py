import os
import smbus
import time
import math
import serial
import MPU6050_lib
import GY_86_lib
import Rotation_lib
import Butterworth_lib
import PID_lib
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio

# ser = serial.Serial(
#   port = '/dev/ttyUSB0',
#   baudrate = 115200,
#   parity = serial.PARITY_NONE,
#   stopbits = serial.STOPBITS_ONE,
#   bytesize = serial.EIGHTBITS,
#   timeout = 0
# )

motion_rate = 200
adc_frequency = 1000
FULL_FIFO_BATCHES=20
sampling_rate=500
motion_rate = 200

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
pi.set_servo_pulsewidth(ESC1, 1000)
pi.set_servo_pulsewidth(ESC2, 1000)
pi.set_servo_pulsewidth(ESC3, 1000)
pi.set_servo_pulsewidth(ESC4, 1000)
time.sleep(3)

upTimes=0
downTimes=0
deltaTimes=0
deltaTimes_pre_1=0
deltaTimes_pre_2=0
deltaTimes_pre_3=0
deltaTimes_pre_4=0
deltaTimes_pre_5=0
throttle =0
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

mpu6050 = MPU6050_lib.MPU6050(0x69, 1, 1)
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)

#Cablirate gyro 
qrx = 0.0
qry = 0.0
qrz = 0.0

sigma_dt = 0.0
loops = 0
while sigma_dt < 1.0:
    time.sleep(FULL_FIFO_BATCHES / sampling_rate)
    nfb = mpu6050.numFIFOBatches()
    ax, ay, az, rx, ry, rz, dt = mpu6050.readFIFO(nfb)
    loops += 1
    sigma_dt += dt
    qrx += rx
    qry += ry
    qrz += rz
    print(nfb)
    
qrx /= loops
qry /= loops
qrz /= loops

mpu6050.setGyroOffsets(qrx, qry, qrz)
end=0
end2=0
# mpu6050.flushFIFO()

while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > (1/motion_rate):
        end = time.time()
        
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
#         if nfb > 20:
#             mpu6050.flushFIFO()

        
        esc_1_rps =  + 1000
        esc_2_rps =  + 1000
        esc_3_rps =  + 1000
        esc_4_rps = throttle + 1000
        if esc_1_rps > 1800:
            esc_1_rps=1800
        if esc_2_rps > 1800:
            esc_2_rps=1800
        if esc_3_rps > 1800:
            esc_3_rps=1800
        if esc_4_rps > 1800:
            esc_4_rps=1800
            
        if esc_1_rps < 1000:
            esc_1_rps=1000
        if esc_2_rps < 1000:
            esc_2_rps=1000
        if esc_3_rps < 1000:
            esc_3_rps=1000
        if esc_4_rps < 1000:
            esc_4_rps=1000

        pi.set_servo_pulsewidth(ESC1, esc_1_rps)
        pi.set_servo_pulsewidth(ESC2, esc_2_rps)
        pi.set_servo_pulsewidth(ESC3, esc_3_rps)
        pi.set_servo_pulsewidth(ESC4, esc_4_rps)
        
#         ser.write(b'%f %f %f %f %f %f\r\n'%(qax, qay, qaz, qrx, qry, qrz))
    if elapse_time2 > 1:
        end2= time.time()
        print(qax, qay, qaz, qrx, qry, qrz, nfb)

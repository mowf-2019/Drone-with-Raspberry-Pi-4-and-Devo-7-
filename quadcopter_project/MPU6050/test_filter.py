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
import numpy as np
from threading import Thread
# import Tracking_Thread_lib
import OpticalFlow_Thread_lib_3 as OpticalFlow_Thread_lib
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
from gpiozero import MCP3202


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


mpu6050 = MPU6050_lib.MPU6050(0x69, 1, 1)
gy86 = GY_86_lib.MPU6050(0x68, 1, 1)
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

sigma_dt = 0.0
loops = 0
while sigma_dt < 1.0:
    time.sleep(FULL_FIFO_BATCHES / sampling_rate)
#     MS5611.readValue()
    nfb = mpu6050.numFIFOBatches()
    ax, ay, az, rx, ry, rz, dt = mpu6050.readFIFO(nfb)
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

mpu6050.setGyroOffsets(qrx, qry, qrz)
qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
ra_offset = ra
pa_offset = pa


time.sleep(3)
mpu6050.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
ya = 0.0

roll_1=0
pitch_1=0
roll = 0
pitch =0
yaw=0
end=0
end2=0
pressure1=0
status=0
run=1
yaw_offset = 0
yaw_error=0
roll_offset = 0
pitch_offset = 0
x_pos = 0
y_pos = 0
pa_target=0
ra_target=0
counter=0
yaw_i_gain=0
init_pressure=0
actual_pressure=0
h=0
yaw1_angle=0

r_qrx = 0
p_qrx = 0

while (run):
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > (1/motion_rate):
        end = time.time()
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
            
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
        ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
        r_qrx += qrx*motion_dt
        p_qrx += qry*motion_dt
        roll_1 = 0.98*(roll_1 + qrx*motion_dt) + 0.02*(ra)
        pitch_1 = 0.98*(pitch_1 + qry*motion_dt) + 0.02*(pa)
        roll = roll_1 - roll_offset
        pitch = pitch_1 - pitch_offset
        
        ser.write(b'%f %f %f %f %f %f\r\n'%(roll, r_qrx, ra, pitch, p_qrx, pa))
            

    if elapse_time2>1:
        end2=time.time()
        print(roll,pitch)


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

bfx = Butterworth_lib.BUTTERWORTH(motion_rate, 5, 8, qax)
bfy = Butterworth_lib.BUTTERWORTH(motion_rate, 5, 8, qay)
bfz = Butterworth_lib.BUTTERWORTH(motion_rate, 5, 8, qaz)

bfvelox = Butterworth_lib.BUTTERWORTH(motion_rate, 0.1, 8, 0)
bfveloy = Butterworth_lib.BUTTERWORTH(motion_rate, 0.1, 8, 0)
bfveloz = Butterworth_lib.BUTTERWORTH(motion_rate, 0.1, 8, 0)

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
velo_fil_pre=0
velo_fil=0
y=0
y_pre =0
max_accX =0 

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
        
        accX_fil = bfx.filter(linAcc[0])
        accY_fil = bfy.filter(linAcc[1])
        accZ_fil = bfz.filter(linAcc[2])

        if abs(accX_fil) < 0.001 or start == 1:
            
            if accX_fil>-0.01 and accX_fil<0.01:
                accX_fil = 0
            velo += (acc_pre + (accX_fil - acc_pre)/2)*elapse_time
            acc_pre=accX_fil
            
            if (accX_fil<0.5) and (accX_fil > -0.5):
                counterAccX += 1
            else:
                counterAccX = 0
            if counterAccX > 50:
                velo =0
                counterAccX =0
                
            x += (velo_pre + (velo - velo_pre)/2)*elapse_time
            velo_pre = velo
            start =1
        
        ser.write(b'%f %f %f %f\r\n'%(qax*9.81,accX_fil,velo, x))
        

#     if elapse_time2>1:
#         end2=time.time()
#         print(qax, qay, qaz, linAcc)


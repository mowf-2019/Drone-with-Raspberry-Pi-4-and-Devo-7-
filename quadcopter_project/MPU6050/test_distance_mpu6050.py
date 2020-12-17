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
# pa_offset = -1.2178853590450544
# ra_offset = 1.0457224208647498
pa_offset = pa
ra_offset = ra
ya = 0.0

egx, egy, egz = Rotation_lib.RotateVector(qax, qay, qaz, -pa, -ra, -ya)
eax = egx
eay = egy
eaz = egz

bfx = Butterworth_lib.BUTTERWORTH(motion_rate, 10, 8, egx)
bfy = Butterworth_lib.BUTTERWORTH(motion_rate, 10, 8, egy)
bfz = Butterworth_lib.BUTTERWORTH(motion_rate, 10, 8, egz)


roll=0
pitch=0
yaw=0
end=0
end2=0
pressure1=0
status=0

pa=0
ra=0
ya=0
apa=0
ara=0
aya=0
apa_increment=0
ara_increment=0
aya_increment=0
qvx_input=0
qvy_input=0
qvz_input=0
qdx_input=0
qdy_input=0
qdz_input=0

while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > (1/motion_rate):
        end = time.time()
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
        
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
        
        urp, urr, ury = Rotation_lib.Body2EulerRates(qry, qrx, qrz, pa, ra)
        pa += urp * motion_dt
        ra += urr * motion_dt
        ya += ury * motion_dt

        upa, ura = Rotation_lib.GetRotationAngles(qax, qay, qaz)

        atau_fraction = 0.5
        pa = atau_fraction * pa + (1 - atau_fraction) * upa
        ra = atau_fraction * ra + (1 - atau_fraction) * ura
        
        apa += qry * motion_dt
        ara += qrx * motion_dt
        aya += qrz * motion_dt
        
        upa, ura = Rotation_lib.GetAbsoluteAngles(qax, qay, qaz)
        
        apa = atau_fraction * apa + (1 - atau_fraction) * upa
        ara = atau_fraction * ara + (1 - atau_fraction) * ura

        apa_increment += qry * motion_dt
        ara_increment += qrx * motion_dt
        aya_increment += qrz * motion_dt
        
        eax, eay, eaz = Rotation_lib.RotateVector(qax, qay, qaz, -pa, -ra, -ya)
        egx = bfx.filter(eax)
        egy = bfy.filter(eay)
        egz = bfz.filter(eaz)
        
        qgx, qgy, qgz = Rotation_lib.RotateVector(egx, egy, egz, pa, ra, ya)
        
        qvx_increment = (qax - qgx) * GRAV_ACCEL * motion_dt
        qvy_increment = (qay - qgy) * GRAV_ACCEL * motion_dt
        qvz_increment = (qaz - qgz) * GRAV_ACCEL * motion_dt

        qvx_input += qvx_increment
        qvy_input += qvy_increment
        qvz_input += qvz_increment

        qdx_increment = qvx_input * motion_dt
        qdy_increment = qvy_input * motion_dt
        qdz_increment = qvz_input * motion_dt

        qdx_input += qdx_increment
        qdy_input += qdy_increment
        qdz_input += qdz_increment
        ser.write(b'%f %f %f %f %f %f\r\n'%(qax,qay,qaz,qgx,qgy,qgz))
        

    if elapse_time2>1:
        end2=time.time()
        print(qax, qay, qaz, qdx_input, qdy_input)


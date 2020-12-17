import math
import MPU6050_lib
import GY_86_lib
import serial
import time

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
motion_rate = 200

mpu6050 = MPU6050_lib.MPU6050(0x69, 1, 1)
gy86 = GY_86_lib.MPU6050(0x68, 1, 1)
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.flushFIFO()
gy86.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)

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
    nfb = mpu6050.numFIFOBatches()
    ax, ay, az, rx, ry, rz, dt = mpu6050.readFIFO(nfb)
    loops += 1
    sigma_dt += dt
    qrx += rx
    qry += ry
    qrz += rz
    
qrx /= loops
qry /= loops
qrz /= loops

mpu6050.setGyroOffsets(qrx, qry, qrz)

while True:
    nfb = mpu6050.numFIFOBatches()
    qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
    qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
    z=gy86.noTiltConpensate()
    ser.write(b'%f %f %f %f %f %f %f %f\r\n'%(qax, qay, qaz, qrx, qry, qrz, nfb, z))
    time.sleep(1/motion_rate)
import L3G4200D_lib
import time 
import math
import serial
import numpy as np
import os
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
time.sleep(1)
import pigpio

pi = pigpio.pi()
#Config serial port
ser = serial.Serial(
  port = '/dev/ttyUSB0',
  baudrate = 115200,
  parity = serial.PARITY_NONE,
  stopbits = serial.STOPBITS_ONE,
  bytesize = serial.EIGHTBITS,
  timeout = 0
)

l3g4200d = L3G4200D_lib.L3G4200D()
print ("Waiting for calibration")
l3g4200d.calibrate(100)
print ("Calibrate done")
gyroX=0
gyroY=0
gyroZ=0
end=0
gyro =0
while True:
        dt=(pi.get_current_tick() - end)
#         if (dt >= 0.005):   #Sample rate 100Hz
        end = pi.get_current_tick()
#         (Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
#         gyroX=gyroX*0.8 + Xrate*0.2
#         gyroY=gyroY*0.8 + Yrate*0.2
#         gyroZ=gyroZ*0.8 + Zrate*0.2

        gyro=l3g4200d.read_FIFO_data()

        gyro=sum(gyro)/32
        gyroX=gyro[0]
        gyroY=gyro[1]
        gyroZ=gyro[2]
#             x,y,z=l3g4200d.readNormalize()
        ser.write(b'%f %f %f %f\r\n'%(gyroX,gyroY,gyroZ,dt))
#         gyro=l3g4200d.read_FIFO_data()
#         gyro=l3g4200d.read_FIFO_data()
#         gyroX=sum(gyro)/32
# #         gyroX=sum(gyro[])
#         print (gyroX[1])
# #         print (roll)
#         time.sleep(1)


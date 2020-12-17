import L3G4200D_lib
import ADXL345_lib
import Kalman
import time 
import math

l3g4200d = L3G4200D_lib.L3G4200D()
adxl345 = ADXL345_lib.ADXL345()
kalmanX = Kalman.KalmanAngle()
kalmanY = Kalman.KalmanAngle()

print ("Waiting for calibration")
l3g4200d.calibrate(100)
print ("Calibrate done")
 
dt=0.01
end = time.time()
 
while True:
        dt=(time.time() - end)
        end = time.time()
        (Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
        adxl345.readNormalize()
        pitch = -(math.atan2(adxl345.nXAxis, math.sqrt(adxl345.nYAxis*adxl345.nYAxis + adxl345.nZAxis*adxl345.nZAxis))*180.0)/math.pi;
        roll  = (math.atan2(adxl345.nYAxis, adxl345.nZAxis)*180.0)/math.pi;
        
        roll_filter = kalmanX.getAngle(roll,Xrate,dt)
#         print (l3g4200d.nXAxis,roll)
#         print (roll)
        print(roll,roll_filter)
        time.sleep(0.01)
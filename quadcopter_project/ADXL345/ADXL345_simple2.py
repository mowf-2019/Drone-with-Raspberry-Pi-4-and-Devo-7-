import ADXL345_lib
import time 
import math

adxl345 = ADXL345_lib.ADXL345()

    
while True:
    
        adxl345.readNormalize()
        pitch = -(math.atan2(adxl345.nXAxis, math.sqrt(adxl345.nYAxis*adxl345.nYAxis + adxl345.nZAxis*adxl345.nZAxis))*180.0)/math.pi;
        roll  = (math.atan2(adxl345.nYAxis, adxl345.nZAxis)*180.0)/math.pi;
#         print ("Normalize acceleration = %d" %normAcc)
        print (adxl345.nXAxis, roll)
#         print (roll)
        time.sleep(1)

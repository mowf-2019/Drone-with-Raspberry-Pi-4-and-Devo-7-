import time
from i2clibraries import i2c_adxl345

adxl345 = i2c_adxl345.i2c_adxl345(1) #choosing which i2c port to use, RPi2 model B uses port 1

while True:
    print("%f %f %f",adxl345.getAxes)
    time.sleep(1)
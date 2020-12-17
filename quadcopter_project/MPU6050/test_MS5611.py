import MS5611_lib
import time
import math

MS5611 = MS5611_lib.MS5611(0x77)

time1 = 0
while time1 < 1000:
    MS5611.readValue()
#     if pressure_abc is not None:
#         pressure_abc1 = round(pressure_abc/100,3)
    time.sleep(0.005)
    time1 += 1
    

end=0
end2 = 0
altitude = 0
pressure = 0
pressure1 = 0.1
pressure_abc=0.1
time2=0
while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > 0.005:
        end = time.time()
        pressure = MS5611.readValue()
        
        if pressure is not None:
            pressure1 = round(pressure/100,3)
            if time2 <= 300:
                pressure_abc = pressure1
                time2 += 1
            
        altitude = (44330.0 * (1.0 - pow(pressure1 / pressure_abc, 0.1902949)))
    
    if elapse_time2 > 1:
        end2 = time.time()
        print(pressure1, pressure_abc, altitude, elapse_time)
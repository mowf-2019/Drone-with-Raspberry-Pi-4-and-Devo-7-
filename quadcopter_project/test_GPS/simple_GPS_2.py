import serial
import time

end =0
lat_gps_actual = 0
lon_gps_actual = 0
lat_gps_previous=0
lon_gps_previous=0
l_lat_gps=0
l_lon_gps=0
gps_add_counter=0
end=0
elapse_time =0

# ser2 = serial.Serial(
#   port = '/dev/ttyUSB0',
#   baudrate = 115200,
#   parity = serial.PARITY_NONE,
#   stopbits = serial.STOPBITS_ONE,
#   bytesize = serial.EIGHTBITS,
#   timeout = 0
# )

port="/dev/ttyS0"
ser=serial.Serial(port, baudrate=57600, timeout=0.5)
while True:
    elapse_time = time.time() - end
    if elapse_time > 1/200:
        end = time.time()
        if(ser.in_waiting):
            newdata=ser.readline()
            newdata = newdata.decode('utf-8')
            if newdata[0:6] == "$GPGGA": #and newdata[17] == "A":
                print(newdata)
#         newdata = newdata.decode('utf-8')
#         print(newdata)
#         ser2.write(b'%f\r\n'%(elapse_time))
#         print(elapse_time)
#         print(elapse_time)

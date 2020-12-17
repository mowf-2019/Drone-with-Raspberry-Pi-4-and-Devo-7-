import smbus        #import SMBus module of I2C
import math
import time

class MPU6050:
    
    #some MPU6050 Registers and their Address
    Device_Address = 0x69   # MPU6050 magnetometer device address


    def __init__(self):
            self.bus = smbus.SMBus(1)   # or bus = smbus.SMBus(0) for older version boards
            #Enable measurement mode
            self.bus.write_byte_data(self.Device_Address, 0x6B, 0x00)

            #Set range 16G, full res: scale factor 4mg/LSB
            self.bus.write_byte_data(self.Device_Address, 0x1B, 0x08)
            self.bus.write_byte_data(self.Device_Address, 0x1C, 0x10)
            
            

    def read_raw_data(self, addr=0x3B):
        
            #Read raw 16-bit value
            accX_high = self.bus.read_byte_data(self.Device_Address, addr)
            accX_low = self.bus.read_byte_data(self.Device_Address, addr+1)
            accY_high = self.bus.read_byte_data(self.Device_Address, addr+2)
            accY_low = self.bus.read_byte_data(self.Device_Address, addr+3)
            accZ_high = self.bus.read_byte_data(self.Device_Address, addr+4)
            accZ_low = self.bus.read_byte_data(self.Device_Address, addr+5)
            
            gyroX_high = self.bus.read_byte_data(self.Device_Address, addr+8)
            gyroX_low = self.bus.read_byte_data(self.Device_Address, addr+9)
            gyroY_high = self.bus.read_byte_data(self.Device_Address, addr+10)
            gyroY_low = self.bus.read_byte_data(self.Device_Address, addr+11)
            gyroZ_high = self.bus.read_byte_data(self.Device_Address, addr+12)
            gyroZ_low = self.bus.read_byte_data(self.Device_Address, addr+13)
            #concatenate higher and lower value
            self.accX = ((accX_high << 8) | accX_low)
            self.accY = ((accY_high << 8) | accY_low)
            self.accZ = ((accZ_high << 8) | accZ_low)
            
            self.gyroX = ((gyroX_high << 8) | gyroX_low)
            self.gyroY = ((gyroY_high << 8) | gyroY_low)
            self.gyroZ = ((gyroZ_high << 8) | gyroZ_low)

            #to get signed value from module
            if(self.accX > 32768):
                self.accX = self.accX - 65536
            if(self.accY > 32768):
                self.accY = self.accY - 65536
            if(self.accZ > 32768):
                self.accZ = self.accZ - 65536
                
            if(self.gyroX > 32768):
                self.gyroX = self.gyroX - 65536
            if(self.gyroY > 32768):
                self.gyroY = self.gyroY - 65536
            if(self.gyroZ > 32768):
                self.gyroZ = self.gyroZ - 65536
            return (self.accX*0.000244 ,self.accY*0.000244 ,self.accZ*0.000244 ,self.gyroX*0.015267 ,self.gyroY*0.015267 ,self.gyroZ*0.015267)
        
mpu6050 = MPU6050()

end = 0
elapse_time=0
end2 = 0
elapse_time2=0

while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > 0.005:
        end = time.time()
        ax, ay, az, gx, gy, gz = mpu6050.read_raw_data()

    if elapse_time2 > 1:
        end2 = time.time()
        print(ax,ay,az,gx,gy,gz)

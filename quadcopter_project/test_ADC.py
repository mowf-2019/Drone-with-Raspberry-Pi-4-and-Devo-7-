import spidev
import time
import os

CLK  = 23    #GPIO11
MISO = 21    #GPIO9
MOSI = 19    #GPIO10
CS   = 24    #GPIO8

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

# Function to read SPI data from MCP3202 chip
# Channel must be an integer 0-2 , 2channels
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data

def ConvertVolts(data,places):
  volts = (data * 3.3) / float(1023)
  volts = round(volts,places)
  return volts

while True:
    value=ReadChannel(0)
    value_vol=ConvertVolts(value,2)
    print(value,value_vol)
    time.sleep(1)
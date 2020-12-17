from gpiozero import MCP3202
import time

# CLK  = 23    #GPIO11
# MISO = 21    #GPIO9
# MOSI = 19    #GPIO10
# CS   = 24    #GPIO8

pot = MCP3202(channel=0)

def Cal_battery(value,places):
  volts = (value*5)*9.57/2.295
  volts = round(volts,places)
  return volts

while True:
    print(pot.value*5, Cal_battery(pot.value,2))
    
    time.sleep(1)
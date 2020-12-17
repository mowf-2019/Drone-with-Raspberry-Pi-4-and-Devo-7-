import RPi.GPIO as IO          #calling header file which helps us use GPIO’s of PI
import time                            #calling time to provide delays in program
import pigpio

IO.setwarnings(False)           #do not show any warnings

IO.setmode (IO.BCM)         #we are programming the GPIO by BCM pin numbers. (PIN35 as ‘GPIO19’)

IO.setup(17,IO.OUT)
IO.setup(27,IO.OUT)
IO.setup(19,IO.OUT)
IO.setup(26,IO.OUT)

p1 = IO.PWM(17,50)
p2 = IO.PWM(27,50)
p3 = IO.PWM(19,50)
p4 = IO.PWM(26,50)

p1.start(5)
p2.start(5)
p3.start(5)
p4.start(5)

time.sleep(2)

def ESCcontrol(pulse1,pulse2,pulse3,pulse4):
    duty1 = (pulse1 * 100.0)/20000.0
    duty2 = (pulse2 * 100.0)/20000.0
    duty3 = (pulse3 * 100.0)/20000.0
    duty4 = (pulse4 * 100.0)/20000.0
    print(duty4)
    p1.ChangeDutyCycle(duty1)
    p2.ChangeDutyCycle(duty2)
    p3.ChangeDutyCycle(duty3)
    p4.ChangeDutyCycle(duty4)

while True:
#     for i in range(1000,2000):
#         ESCcontrol(1000,1000,1000,i)
#         time.sleep(0.1)
#         if i==1500:
#             break
#     ESCcontrol(1000,1000,1000,1000)
#     break
    ESCcontrol(1000,1000,1000,1500)
    time.sleep(10)
    ESCcontrol(1000,1000,1000,1000)
    break

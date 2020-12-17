import Control_Quadcopter_lib
import Tracking_Thread_lib
from threading import Thread
import time

#Create Class
Tracking = Tracking_Thread_lib.Tracking_Thread()
Control = Control_Quadcopter_lib.Control_Quadcopter()
#Create Thread
TrackingThread = Thread(target=Tracking.run)
ControlThread = Thread(target=Control.run) 
#Start Thread 
TrackingThread.start()
ControlThread.start()

while True:
    end3 = 0
    if time.time() - end3 > 1:
        end3 = time.time()
        print(Control.Observe())

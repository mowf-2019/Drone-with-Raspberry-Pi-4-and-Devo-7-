from threading import Thread
import time
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import sys
sys.path.append('/usr/local/python')
import cv2

global cycle
cycle = 0.0



class Hello5Program:  
    def __init__(self):
        self._running = True
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.camera.vflip = True
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        time.sleep(0.1)
    def terminate(self):  
        self._running = False  

    def run(self):
        global cycle
        while self._running:
            for self.frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                self.image = self.frame.array
                # show the frame
                cv2.imshow("Frame", self.image)
                key = cv2.waitKey(1) & 0xFF
                # clear the stream in preparation for the next frame
                self.rawCapture.truncate(0)
                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    break
            time.sleep(5) #Five second delay
            cycle = cycle + 1.0
            print ("5 Second Thread cycle+1.0 - ", cycle)
            
#Create Class
FiveSecond = Hello5Program()
#Create Thread
FiveSecondThread = Thread(target=FiveSecond.run) 
#Start Thread 
FiveSecondThread.start()

Exit = False #Exit flag
while Exit==False:
 cycle = cycle + 0.1 
 print ("Main Program increases cycle+0.1 - ", cycle)
 time.sleep(1) #One second delay
 if (cycle > 5): Exit = True #Exit Program

FiveSecond.terminate()
print ("Goodbye :)")
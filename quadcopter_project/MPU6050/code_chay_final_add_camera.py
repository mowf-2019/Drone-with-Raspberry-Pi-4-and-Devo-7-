import os
import smbus
import time
import math
import serial
import MPU6050_lib
import GY_86_lib
import Rotation_lib
import Butterworth_lib
import PID_lib
import numpy as np
import sys
sys.path.append('/usr/local/python')
import cv2


from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
from threading import Thread

PID_RA_P_GAIN = 1
PID_RA_I_GAIN = 0
PID_RA_D_GAIN = 0
PID_PA_P_GAIN = PID_RA_P_GAIN
PID_PA_I_GAIN = PID_RA_I_GAIN
PID_PA_D_GAIN = PID_RA_D_GAIN
PID_YA_P_GAIN = 0
PID_YA_I_GAIN = 0
PID_YA_D_GAIN = 0
PID_A_I_TH = 100

PID_PR_P_GAIN = 3
PID_PR_I_GAIN = 0.5
PID_PR_D_GAIN = 0
PID_RR_P_GAIN = PID_PR_P_GAIN
PID_RR_I_GAIN = PID_PR_I_GAIN
PID_RR_D_GAIN = PID_PR_D_GAIN
PID_YR_P_GAIN = 2
PID_YR_I_GAIN = 0.5
PID_YR_D_GAIN = 0
PID_R_I_TH = 100
#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw angle PID - note the different PID class for yaw.
#-------------------------------------------------------------------------------------------
pa_pid = PID_lib.PID(PID_PA_P_GAIN, PID_PA_I_GAIN, PID_PA_D_GAIN, PID_A_I_TH)
ra_pid = PID_lib.PID(PID_RA_P_GAIN, PID_RA_I_GAIN, PID_RA_D_GAIN, PID_A_I_TH)
ya_pid = PID_lib.YAW_PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN, PID_A_I_TH)

#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw rotation rate PIDs
#-------------------------------------------------------------------------------------------
pr_pid = PID_lib.PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN, PID_R_I_TH)
rr_pid = PID_lib.PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN, PID_R_I_TH)
yr_pid = PID_lib.PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN, PID_R_I_TH)


ser = serial.Serial(
  port = '/dev/ttyUSB0',
  baudrate = 115200,
  parity = serial.PARITY_NONE,
  stopbits = serial.STOPBITS_ONE,
  bytesize = serial.EIGHTBITS,
  timeout = 0
)

motion_rate = 200
adc_frequency = 1000
FULL_FIFO_BATCHES=20
sampling_rate=500
motion_rate = 200

ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

ELEV = 5
AILE = 6
THRO = 13
RUDD = 12
GEAR = 16
AUX1 = 20
AUX2 = 21

inPINS = [ELEV, AILE, THRO, RUDD, GEAR, AUX1, AUX2]
pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC1, 1000)
pi.set_servo_pulsewidth(ESC2, 1000)
pi.set_servo_pulsewidth(ESC3, 1000)
pi.set_servo_pulsewidth(ESC4, 1000)
time.sleep(3)

upTimes=0
downTimes=0
deltaTimes=0
deltaTimes_pre_1=0
deltaTimes_pre_2=0
deltaTimes_pre_3=0
deltaTimes_pre_4=0
deltaTimes_pre_5=0
throttle =0
roll_setpoint=0
pitch_setpoint=0
yaw_setpoint=0
gear=0

def my_callback1(gpio,level,tick):
  global upTimes, downTimes, deltaTimes, deltaTimes_pre_1, deltaTimes_pre_2, deltaTimes_pre_3, deltaTimes_pre_4, deltaTimes_pre_5, throttle, roll_setpoint, pitch_setpoint, yaw_setpoint, gear
  
  if (level==0):
    downTimes=tick
  else:
    upTimes=tick

  deltaTimes=(downTimes-upTimes)
  if gpio==ELEV:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_1
      deltaTimes_pre_1=deltaTimes
      pitch_setpoint=round((deltaTimes-1500 + 500)*(20+20) / (500 + 500) - 20,2)
      if abs(pitch_setpoint)<0.1:
          pitch_setpoint=0
  if gpio==RUDD:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_2
      deltaTimes_pre_2=deltaTimes
      roll_setpoint=round((deltaTimes-1500 + 500)*(20+20) / (500 + 500) - 20,2)
      if abs(roll_setpoint)<0.1:
          roll_setpoint=0
  if gpio==AILE:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_3
      deltaTimes_pre_3=deltaTimes
      yaw_setpoint=round((deltaTimes-1500 + 500) * (20 +20) / (500 + 500) - 20,2)
      if abs(yaw_setpoint)<0.1:
          yaw_setpoint=0
  if gpio==THRO:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_4
      deltaTimes_pre_4=deltaTimes
      if deltaTimes<1200:
          deltaTimes=1200
      elif deltaTimes>1900:
          deltaTimes=1900
      throttle=(deltaTimes - 1200) * (800 - 0) / (1900 - 1200) + 0
  if gpio==GEAR:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_5
      deltaTimes_pre_5=deltaTimes
      if deltaTimes < 1300:
          gear=0
      elif deltaTimes > 1700:
          gear=1
  
pi.callback(ELEV, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AILE, pigpio.EITHER_EDGE, my_callback1)
pi.callback(THRO, pigpio.EITHER_EDGE, my_callback1)
pi.callback(RUDD, pigpio.EITHER_EDGE, my_callback1)
pi.callback(GEAR, pigpio.EITHER_EDGE, my_callback1)

# vs = PiVideoStream().start()
# time.sleep(2.0)
# fps = FPS().start()

class Hello5Program:  
    def __init__(self):
        self._running = True
        self.start = True

    def terminate(self):  
        self._running = False 
    def run(self):
        global cycle
        if self.start == True:
            self.camera = PiCamera()
#             self.camera.resolution = (640, 480)
            self.camera.resolution = (640, 480)
            self.camera.framerate = 20
            self.camera.vflip = True
            self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
            self.tracker = cv2.TrackerTLD_create()
            self.initBB = None
            time.sleep(1)
            self.start = None
        while self._running:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                W  = 640   # float
                H = 480  # float
                
                if self.initBB is not None:
                    # grab the new bounding box coordinates of the object
                    (success, box) = self.tracker.update(image)

                    # check to see if the tracking was a success
                    if success:
                        (x, y, w, h) = [int(v) for v in box]
                        cv2.rectangle(image, (x, y), (x + w, y + h),
                            (0, 255, 0), 2)


                # show the output frame
            #     fps = cap.get(cv2.CAP_PROP_FPS)
            #     print('fps:', fps)  # float
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
#                 print(time.time()-end)
#                 end = time.time()

                # if the 's' key is selected, we are going to "select" a bounding
                # box to track
                if key == ord("s"):
                    # select the bounding box of the object we want to track (make
                    # sure you press ENTER or SPACE after selecting the ROI)
                    self.initBB = cv2.selectROI("Frame", image, fromCenter=False,
                        showCrosshair=True)

                    # start OpenCV object tracker using the supplied bounding box
                    # coordinates, then start the FPS throughput estimator as well
                    self.tracker.init(image, self.initBB)

                # if the `q` key was pressed, break from the loop
                elif key == ord("q"):
                    break
                # show the frame

                # clear the stream in preparation for the next frame
                self.rawCapture.truncate(0)

#Create Class
FiveSecond = Hello5Program()
#Create Thread
FiveSecondThread = Thread(target=FiveSecond.run) 
#Start Thread 
FiveSecondThread.start()


mpu6050 = MPU6050_lib.MPU6050(0x69, 1, 1)
gy86 = GY_86_lib.MPU6050(0x68, 1, 1)
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.flushFIFO()
gy86.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
# mpu6050.enableFIFOOverflowISR()


#Cablirate gyro and acceleromoter
qax = 0.0
qay = 0.0
qaz = 0.0
qrx = 0.0
qry = 0.0
qrz = 0.0

sigma_dt = 0.0
loops = 0
while sigma_dt < 1.0:
    time.sleep(FULL_FIFO_BATCHES / sampling_rate)
    nfb = mpu6050.numFIFOBatches()
    ax, ay, az, rx, ry, rz, dt = mpu6050.readFIFO(nfb)
    loops += 1
    sigma_dt += dt
    qax += ax
    qay += ay
    qaz += az
    qrx += rx
    qry += ry
    qrz += rz
    
qax /= loops
qay /= loops
qaz /= loops
qrx /= loops
qry /= loops
qrz /= loops

mpu6050.setGyroOffsets(qrx, qry, qrz)
# mpu6050.setAccelOffsets(qax, qay, qaz)
qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
pa_offset = pa
ra_offset = ra
ya = 0.0

roll=0
pitch=0
end=0
end2=0


while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    
    if elapse_time > (0.005):
        end = time.time()
        
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
        ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
        roll =0.98*(roll + qrx*motion_dt) + 0.02*(ra - ra_offset)
        pitch=0.98*(pitch + qry*motion_dt) + 0.02*(pa - pa_offset)
        
        if gear == 1:
            #======================================================================================
            # Angle PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pa_pid.Compute(pitch, pitch_setpoint, motion_dt, gear)
            pr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ra_pid.Compute(roll, roll_setpoint, motion_dt, gear)
            rr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ya_pid.Compute(0, 0, motion_dt, gear)
            yr_target = p_out + i_out + d_out

            #======================================================================================
            # Rotation rate PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pr_pid.Compute(qry, pr_target, motion_dt, gear)
            pr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = rr_pid.Compute(qrx, rr_target, motion_dt, gear)
            rr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = yr_pid.Compute(qrz, 0, motion_dt, gear)
            yr_out = p_out + i_out + d_out
        else:
            pr_target=0
            rr_target=0
            yr_target=0
            pr_out=0
            rr_out=0
            yr_out=0
            
        pr_out /= 2
        rr_out /= 2
        yr_out /= 2
        
        esc_1_rps = throttle - pr_out - rr_out + yr_out + 1000
        esc_2_rps = throttle - pr_out + rr_out - yr_out + 1000
        esc_3_rps = throttle + pr_out + rr_out + yr_out + 1000
        esc_4_rps = throttle + pr_out - rr_out - yr_out + 1000
        
#         esc_1_rps = throttle - pr_out + 1000
#         esc_2_rps = throttle - pr_out + 1000
#         esc_3_rps = throttle + pr_out + 1000
#         esc_4_rps = throttle + pr_out + 1000
        if esc_1_rps > 1900:
            esc_1_rps=1900
        if esc_2_rps > 1900:
            esc_2_rps=1900
        if esc_3_rps > 1900:
            esc_3_rps=1900
        if esc_4_rps > 1900:
            esc_4_rps=1900
            
        if esc_1_rps < 1000:
            esc_1_rps=1000
        if esc_2_rps < 1000:
            esc_2_rps=1000
        if esc_3_rps < 1000:
            esc_3_rps=1000
        if esc_4_rps < 1000:
            esc_4_rps=1000
    
        pi.set_servo_pulsewidth(ESC1, esc_1_rps)
        pi.set_servo_pulsewidth(ESC2, esc_2_rps)
        pi.set_servo_pulsewidth(ESC3, esc_3_rps)
        pi.set_servo_pulsewidth(ESC4, esc_4_rps)
        z=gy86.readheading(roll*(math.pi)/180,pitch*(math.pi)/180)

#     if elapse_time2>1:
#         end2=time.time()
#         print(esc_1_rps, esc_2_rps, esc_3_rps, esc_4_rps, roll, pitch, nfb, elapse_time)

        
        ser.write(b'%f %f %f %f %f\r\n'%(qrx,qry,roll,pitch,elapse_time))
    #     print(ax,ay,az)
#         time.sleep(1/motion_rate)

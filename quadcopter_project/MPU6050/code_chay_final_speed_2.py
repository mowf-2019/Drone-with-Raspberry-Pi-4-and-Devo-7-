import os
import smbus
import time
import math
import serial
import MPU6050_lib
import GY_86_lib
import MS5611_lib
import Rotation_lib
import Butterworth_lib
import PID_lib
import RPS2PULSE as R2P

from threading import Thread
import Tracking_Thread_lib
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
from gpiozero import MCP3202

# file = open("/home/pi/data_log.csv", "a")
# i=0
# if os.stat("/home/pi/data_log.csv").st_size == 0:
#         file.write("roll,pitch,yaw,pressure,throttle\n")

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

PID_PR_P_GAIN = 0.7
PID_PR_I_GAIN = 0.15
PID_PR_D_GAIN = 0
PID_RR_P_GAIN = PID_PR_P_GAIN
PID_RR_I_GAIN = PID_PR_I_GAIN
PID_RR_D_GAIN = PID_PR_D_GAIN
PID_YR_P_GAIN = 1
PID_YR_I_GAIN = 0.2
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

# ser = serial.Serial(
#   port = '/dev/ttyUSB0',
#   baudrate = 115200,
#   parity = serial.PARITY_NONE,
#   stopbits = serial.STOPBITS_ONE,
#   bytesize = serial.EIGHTBITS,
#   timeout = 0
# )

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
deltaTimes_pre_6=0
throttle =0
roll_setpoint=0
pitch_setpoint=0
yaw_setpoint=0
gear=0
MIX=0

def my_callback1(gpio,level,tick):
  global upTimes, downTimes, deltaTimes, deltaTimes_pre_1, deltaTimes_pre_2, deltaTimes_pre_3, deltaTimes_pre_4, deltaTimes_pre_5, deltaTimes_pre_6, throttle, roll_setpoint, pitch_setpoint, yaw_setpoint, gear, MIX
  
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
      throttle=(deltaTimes - 1200) * (700 - 0) / (1900 - 1200) + 0
  if gpio==GEAR:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_5
      deltaTimes_pre_5=deltaTimes
      if deltaTimes < 1300:
          gear=0
      elif deltaTimes > 1700:
          gear=1
  if gpio==AUX1:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_6
      deltaTimes_pre_6=deltaTimes
      if deltaTimes <= 1300:
          MIX=0
      elif deltaTimes > 1300 and deltaTimes <1700:
          MIX=1
      elif deltaTimes >= 1700:
          MIX=2
  
pi.callback(ELEV, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AILE, pigpio.EITHER_EDGE, my_callback1)
pi.callback(THRO, pigpio.EITHER_EDGE, my_callback1)
pi.callback(RUDD, pigpio.EITHER_EDGE, my_callback1)
pi.callback(GEAR, pigpio.EITHER_EDGE, my_callback1)
pi.callback(AUX1, pigpio.EITHER_EDGE, my_callback1)


#Create Class
Tracking = Tracking_Thread_lib.Tracking_Thread()
#Create Thread
TrackingThread = Thread(target=Tracking.run) 
#Start Thread 
TrackingThread.start()


#Calculate lipo voltage
pot = MCP3202(channel=0)
def LipoVolts():
    lipo_volt = pot.value*5
    volts = (lipo_volt * 9.76) / 2.24
    volts = round(volts,2)
    return volts

mpu6050 = MPU6050_lib.MPU6050(0x69, 1, 1)
gy86 = GY_86_lib.MPU6050(0x68, 1, 1)
MS5611 = MS5611_lib.MS5611(0x77)

#Cablirate gyro and acceleromoter
qax = 0.0
qay = 0.0
qaz = 0.0
qrx = 0.0
qry = 0.0
qrz = 0.0
yaw_offset = 0

sigma_dt = 0.0
loops = 0
while sigma_dt < 1.0:
    time.sleep(0.005)
    ax, ay, az, rx, ry, rz = mpu6050.readRaw()
    yaw_offset += gy86.readheading(1,1) #No tilt compensate
    loops += 1
    sigma_dt += 0.005
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
yaw_offset /= loops

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
pressure1=0
qrx_filter=0
qry_filter=0
qrz_filter=0
while True:
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > (0.01):
        end = time.time()
        qax, qay, qaz, qrx, qry, qrz = mpu6050.readRaw()
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
#         qrx_filter = qrx_filter*0.8 + qrx*0.2
#         qry_filter = qry_filter*0.8 + qry*0.2
#         qrz_filter = qrz_filter*0.8 + qrz*0.2
        qrx_filter = qrx
        qry_filter = qry
        qrz_filter = qrz
        ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
        roll = 0.98*(roll + qrx_filter*elapse_time) + 0.02*(ra - ra_offset)
        pitch = 0.98*(pitch + qry_filter*elapse_time) + 0.02*(pa - pa_offset)
        yaw = gy86.readheading(roll*(math.pi)/180,pitch*(math.pi)/180) - yaw_offset
#         pressure = MS5611.readValue()
        lipo_volt = LipoVolts()
        
#         if pressure is not None:
#             pressure1 = round(pressure/100,3)
        
        if gear == 1:
            #======================================================================================
            # Angle PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pa_pid.Compute(pitch, pitch_setpoint, elapse_time, gear, throttle)
            pr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ra_pid.Compute(roll, roll_setpoint, elapse_time, gear, throttle)
            rr_target = p_out + i_out + d_out

#             [p_out, i_out, d_out] = ya_pid.Compute(0, 0, motion_dt, gear)
#             yr_target = p_out + i_out + d_out
            yaw_error = ya_pid.Error(yaw*(math.pi/180), yaw_setpoint*(math.pi/180))
            yr_target = yaw_error

            #======================================================================================
            # Rotation rate PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pr_pid.Compute(qry_filter, pr_target, elapse_time, gear, throttle)
            pr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = rr_pid.Compute(qrx_filter, rr_target, elapse_time, gear, throttle)
            rr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = yr_pid.Compute(qrz_filter, 0, elapse_time, gear, throttle)
            yr_out = p_out + i_out + d_out
            
            if lipo_volt < 9:
                off_rps = 0
            else:
                off_rps = (12.6-lipo_volt)*15
            
            esc_1_rps = throttle - pr_out - rr_out + yr_out + off_rps
            esc_2_rps = throttle - pr_out + rr_out - yr_out + off_rps
            esc_3_rps = throttle + pr_out + rr_out + yr_out + off_rps
            esc_4_rps = throttle + pr_out - rr_out - yr_out + off_rps
        else:
            pr_target=0
            rr_target=0
            yr_target=0
            pr_out=0
            rr_out=0
            yr_out=0
            esc_1_rps=0
            esc_2_rps=0
            esc_3_rps=0
            esc_4_rps=0
            

        pi.set_servo_pulsewidth(ESC1, R2P.Rps2Pulse_M1(esc_1_rps))
        pi.set_servo_pulsewidth(ESC2, R2P.Rps2Pulse_M2(esc_2_rps))
        pi.set_servo_pulsewidth(ESC3, R2P.Rps2Pulse_M3(esc_3_rps))
        pi.set_servo_pulsewidth(ESC4, R2P.Rps2Pulse_M4(esc_4_rps))
#         ser.write(b'%f %f %f %f %f %f\r\n'%(qax,qay,qaz,qrx,qry,qrz))
        

    if elapse_time2>1:
        end2=time.time()
#         print(R2P.Rps2Pulse_M1(esc_1_rps), R2P.Rps2Pulse_M2(esc_2_rps), R2P.Rps2Pulse_M3(esc_3_rps), R2P.Rps2Pulse_M4(esc_4_rps), roll, pitch,elapse_time)
        print(roll,pitch,ra,pa,yaw,throttle,ra_offset,pa_offset,lipo_volt,elapse_time)
        
    
    #     print(ax,ay,az)
#         time.sleep(1/motion_rate)

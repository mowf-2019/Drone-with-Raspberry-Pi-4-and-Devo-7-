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
import numpy as np
from threading import Thread
import Tracking_Thread_lib_2
import OpticalFlow_Thread_lib
# os.system("sudo pigpiod -s 5") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall pigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
from gpiozero import MCP3202

# file = open("/media/pi/SAVE/data_log.csv", "a")
# i=0
# if os.stat("/media/pi/SAVE/data_log.csv").st_size == 0:
#         file.write("roll,pitch,MIX,x_position,y_position\n")

#PID parameters for angle PIDs
PID_RA_P_GAIN = 1
PID_RA_I_GAIN = 0
PID_RA_D_GAIN = 0
PID_PA_P_GAIN = PID_RA_P_GAIN
PID_PA_I_GAIN = PID_RA_I_GAIN
PID_PA_D_GAIN = PID_RA_D_GAIN
PID_YA_P_GAIN = 0
PID_YA_I_GAIN = 0
PID_YA_D_GAIN = 0
PID_A_I_TH = 50

#PID parameters for rotation rate PIDs
PID_PR_P_GAIN = 0.5
PID_PR_I_GAIN = 0.6
PID_PR_D_GAIN = 0
PID_RR_P_GAIN = PID_PR_P_GAIN
PID_RR_I_GAIN = PID_PR_I_GAIN
PID_RR_D_GAIN = PID_PR_D_GAIN
PID_YR_P_GAIN = 0.7
PID_YR_I_GAIN = 0.4
PID_YR_D_GAIN = 0
PID_R_I_TH = 50

#PID for position PIDs
PID_X_P_GAIN = 5
PID_X_I_GAIN = 0
PID_X_D_GAIN = 0
PID_Y_P_GAIN = 5
PID_Y_I_GAIN = 0
PID_Y_D_GAIN = 0
PID_X_I_TH = 0

#-------------------------------------------------------------------------------------------
# Start the position (x, y) PID - note the different PID class for yaw.
#-------------------------------------------------------------------------------------------
x_pid = PID_lib.PID(PID_X_P_GAIN, PID_X_I_GAIN, PID_X_D_GAIN, PID_X_I_TH)
y_pid = PID_lib.PID(PID_Y_P_GAIN, PID_Y_I_GAIN, PID_Y_D_GAIN, PID_X_I_TH)

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

ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

ELEV = 13
AILE = 6
THRO = 5
RUDD = 16
GEAR = 12
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
      if abs(pitch_setpoint)<0.5:
          pitch_setpoint=0
  if gpio==RUDD:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_2
      deltaTimes_pre_2=deltaTimes
      roll_setpoint=round((deltaTimes-1500 + 500)*(20+20) / (500 + 500) - 20,2)
      if abs(roll_setpoint)<0.5:
          roll_setpoint=0
  if gpio==AILE:
      if deltaTimes<0:
          deltaTimes=deltaTimes_pre_3
      deltaTimes_pre_3=deltaTimes
      yaw_setpoint=round((deltaTimes-1500 + 500) * (20 +20) / (500 + 500) - 20,2)
      if abs(yaw_setpoint)<0.5:
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
Tracking = Tracking_Thread_lib_2.Tracking_Thread()
#Create Thread
TrackingThread = Thread(target=Tracking.run)
#Start Thread 
TrackingThread.start()


# OpticalFlow = OpticalFlow_Thread_lib.Tracking_Thread()
# OpticalFlowThread = Thread(target=OpticalFlow.run)
# time.sleep(5)
# OpticalFlowThread.start()

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
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.flushFIFO()
# gy86.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
mpu6050.enableFIFOOverflowISR()

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
#     MS5611.readValue()
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
qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
ra_offset = ra
pa_offset = pa

mgx, mgy, mgz = gy86.Compass_readRaw()
max_mgx = mgx
min_mgx = mgx
max_mgy = mgy
min_mgy = mgy
max_mgz = mgz
min_mgz = mgz
mpu6050.flushFIFO()

yaw = 0.0
# print("rotate around Z axis")
# while abs(yaw) < 720:
#     time.sleep(10 / sampling_rate)
# 
#     nfb = mpu6050.numFIFOBatches()
#     ax, ay, az, gx, gy, gz, dt = mpu6050.readFIFO(nfb)
#     ax, ay, az, gx, gy, gz = mpu6050.scaleSensors(ax, ay, az, gx, gy, gz)
# 
#     yaw += gz * dt
# 
#     mgx, mgy, mgz = gy86.Compass_readRaw()
#     max_mgx = mgx if mgx > max_mgx else max_mgx
#     max_mgy = mgy if mgy > max_mgy else max_mgy
#     min_mgx = mgx if mgx < min_mgx else min_mgx
#     min_mgy = mgy if mgy < min_mgy else min_mgy
#     print("yaw angle =", yaw)
# print("X, Y was calibrated")
# time.sleep(5)
# mpu6050.flushFIFO()
# time.sleep(FULL_FIFO_BATCHES/sampling_rate)
# rotation = 0.0
# print("rotate around X+Y axis")
# while abs(rotation) < 720:
#     time.sleep(10 / sampling_rate)
# 
#     nfb = mpu6050.numFIFOBatches()
#     ax, ay, az, gx, gy, gz, dt = mpu6050.readFIFO(nfb)
#     ax, ay, az, gx, gy, gz = mpu6050.scaleSensors(ax, ay, az, gx, gy, gz)
# 
#     rotation += math.pow(math.pow(gx, 2) + math.pow(gy, 2), 0.5) * dt
# 
#     mgx, mgy, mgz = gy86.Compass_readRaw()
#     max_mgz = mgz if mgz > max_mgz else max_mgz
#     min_mgz = mgz if mgz < min_mgz else min_mgz
#     print("rotation =", rotation)
# print("Z was calibrated")
# 
# mgx_offset = (max_mgx + min_mgx) / 2
# mgy_offset = (max_mgy + min_mgy) / 2
# mgz_offset = (max_mgz + min_mgz) / 2
# mgx_gain = 1 / (max_mgx - min_mgx)
# mgy_gain = 1 / (max_mgy - min_mgy)
# mgz_gain = 1 / (max_mgz - min_mgz)

mgx_offset = 735.54
mgy_offset = 169.74
mgz_offset = -433.32
mgx_gain = 0.0012337758476040073
mgy_gain = 0.0011957717510881524
mgz_gain = 0.005602868668758404

print(mgx_offset,mgy_offset,mgz_offset,mgx_gain,mgy_gain,mgz_gain)
gy86.setCompassOffset(mgx_offset,mgy_offset,mgz_offset,mgx_gain,mgy_gain,mgz_gain)

time.sleep(3)
mpu6050.flushFIFO()
time.sleep(FULL_FIFO_BATCHES/sampling_rate)
ya = 0.0

roll_1=0
pitch_1=0
roll = 0
pitch =0
yaw=0
end=0
end2=0
pressure1=0
status=0
run=1
yaw_offset = 0
yaw_error=0
roll_offset = 0
pitch_offset = 0
x_pos = 0
y_pos = 0

while (run):
    elapse_time = time.time() - end
    elapse_time2 = time.time() - end2
    if elapse_time > (1/motion_rate):
        end = time.time()
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
        
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
        ra, pa = Rotation_lib.GetRotationAngles(qax, qay, qaz)
        roll_1 = 0.98*(roll_1 + qrx*motion_dt) + 0.02*(ra)
        pitch_1 = 0.98*(pitch_1 + qry*motion_dt) + 0.02*(pa)
        roll = roll_1 - roll_offset
        pitch = pitch_1 - pitch_offset
        heading = gy86.readheading(roll*(math.pi)/180,pitch*(math.pi)/180)
        
        if abs(roll)>45 or abs(pitch)>45:
            run=0
        
#         pressure = MS5611.readValue()
        lipo_volt = LipoVolts()
        
#         if pressure is not None:
#             pressure1 = round(pressure/100,3)
        
        
        if gear == 1 and run == 1:
            if MIX == 2:
                Tracking.enable_position_mode()
            else:
                Tracking.disable_position_mode()
            x_pos, y_pos = Tracking.position()
            
            #======================================================================================
            # Position PIDs
            #======================================================================================
            [p_out, i_out, d_out] = x_pid.Compute(x_pos, 0, motion_dt, gear, throttle)
            pa_target = p_out + i_out + d_out
            if abs(pa_target) > 10:
                pa_target = np.sign(pa_target)*10
            
            [p_out, i_out, d_out] = y_pid.Compute(y_pos, 0, motion_dt, gear, throttle)
            ra_target = p_out + i_out + d_out
            if abs(ra_target) > 10:
                ra_target = np.sign(ra_target)*10
            
            #======================================================================================
            # Angle PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pa_pid.Compute(pitch, pa_target + pitch_setpoint, motion_dt, gear, throttle)
            pr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ra_pid.Compute(roll, ra_target + roll_setpoint, motion_dt, gear, throttle)
            rr_target = p_out + i_out + d_out

#             [p_out, i_out, d_out] = ya_pid.Compute(0, 0, motion_dt, gear)
#             yr_target = p_out + i_out + d_out
            if yaw_offset == 0:
                yaw_offset = heading
            if roll_offset == 0:
                roll_offset = roll
            if pitch_offset == 0:
                pitch_offset = pitch
            yaw_input = heading - yaw_offset
            yaw_error = ya_pid.Error(yaw_input, yaw_setpoint)
            yr_target = -yaw_error*0.5
            if pr_target > 50:
                pr_target = 50
            if rr_target > 50:
                rr_target = 50
                
            #======================================================================================
            # Rotation rate PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pr_pid.Compute(qry, pr_target, motion_dt, gear, throttle)
            pr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = rr_pid.Compute(qrx, rr_target, motion_dt, gear, throttle)
            rr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = yr_pid.Compute(qrz, 0, motion_dt, gear, throttle)
            yr_out = p_out + i_out + d_out
            
            if pr_out > 50:
                pr_out = 50
            if rr_out > 50:
                rr_out = 50
            if yr_out > 50:
                yr_out = 50
            
            if lipo_volt < 9 or lipo_volt > 15:
                off_rps = 0
            else:
                off_rps = (12.6-lipo_volt)*15
#             off_rps=0
            
            esc_1_rps = throttle - pr_out - rr_out + yr_out + off_rps
            esc_2_rps = throttle - pr_out + rr_out - yr_out + off_rps
            esc_3_rps = throttle + pr_out + rr_out + yr_out + off_rps
            esc_4_rps = throttle + pr_out - rr_out - yr_out + off_rps
        else:
            pa_target=0
            ra_target=0
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
#         file.write(str(roll)+","+str(pitch)+","+str(MIX)+","+str(x_pos)+","+str(y_pos)+"\n")
#         ser.write(b'%f %f\r\n'%(roll, elapse_time))
        

    if elapse_time2>1:
        end2=time.time()
#         print(R2P.Rps2Pulse_M1(esc_1_rps), R2P.Rps2Pulse_M2(esc_2_rps), R2P.Rps2Pulse_M3(esc_3_rps), R2P.Rps2Pulse_M4(esc_4_rps), roll, pitch,elapse_time)
#         print(roll,pitch,throttle,roll_setpoint,pitch_setpoint,nfb,elapse_time,lipo_volt,yr_target)
        print(roll,pitch,nfb,throttle,lipo_volt,MIX,x_pos,y_pos, pa_target, ra_target)
        
    
    #     print(ax,ay,az)
#         time.sleep(1/motion_rate)
file.flush()
time.sleep(5)
file.close()
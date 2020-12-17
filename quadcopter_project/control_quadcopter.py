import os
import time
import math
import numpy as np
import RPi.GPIO as GPIO
# os.system("sudo pigpiod -s 1") #Run once to start the pigpio daemon, sample rate 1us
# os.system("sudo killall ppigpiod") #To stop the pigpio daemon
time.sleep(1)
import pigpio
import L3G4200D_lib
import ADXL345_lib
import Kalman
import serial

#PID parameters
Kp_roll=1
Kd_roll=2
Ki_roll=0

Kp_pitch=1
Kd_pitch=2
Ki_pitch=0

Kp_yaw=0
Kd_yaw=0
Ki_yaw=0

#Config serial port
ser = serial.Serial(
	port = '/dev/ttyUSB0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0
)

ESC1 = 17
ESC2 = 27
ESC3 = 19
ESC4 = 26

ELEV = 12
AILE = 16
THRO = 20
RUDD = 21

inPINS = [ELEV, AILE, THRO, RUDD]

pi = pigpio.pi()

l3g4200d = L3G4200D_lib.L3G4200D()
adxl345 = ADXL345_lib.ADXL345()
kalmanX = Kalman.KalmanAngle()
kalmanY = Kalman.KalmanAngle()

pi.set_servo_pulsewidth(ESC1, 1000)
pi.set_servo_pulsewidth(ESC2, 1000)
pi.set_servo_pulsewidth(ESC3, 1000)
pi.set_servo_pulsewidth(ESC4, 1000)
time.sleep(5)

print ("Waiting for calibration")
l3g4200d.calibrate(100)
print ("Calibrate done")

#Convert speed (rps) to pulse 
def Rps2Pulse1(rps):  #Motor1
    pulse = rps/0.7856 + 1115
    if (pulse<0):
        pulse=0
    if (pulse>2000):
        pulse=2000
    return pulse

def Rps2Pulse2(rps):  #Motor2
    pulse = rps/0.7782 + 1125
    if (pulse<0):
        pulse=0
    if (pulse>2000):
        pulse=2000
    return pulse

def Rps2Pulse3(rps):  #Motor3
    pulse = rps/0.7938 + 1121
    if (pulse<0):
        pulse=0
    if (pulse>2000):
        pulse=2000
    return pulse

def Rps2Pulse4(rps):  #Motor4
    pulse = rps/0.7635 + 1118
    if (pulse<0):
        pulse=0
    if (pulse>2000):
        pulse=2000
    return pulse

def PID_roll(angle,dt):
    global E1_roll, E2_roll, LastOutput_roll
    E = angle
    alpha = 2*dt*Kp_roll + Ki_roll*dt*dt + 2*Kd_roll
    beta = dt*dt*Ki_roll - 4*Kd_roll - 2*dt*Kp_roll
    gamma = 2*Kd_roll
    Output = (alpha*E + beta*E1_roll + gamma*E2_roll + 2*dt*LastOutput_roll)/(2*dt)
    LastOutput_roll = Output
    E2_roll=E1_roll
    E1_roll=E
    return Output
    
def PID_pitch(angle,dt):
    global E1_pitch, E2_pitch, LastOutput_pitch
    E = angle
    alpha = 2*dt*Kp_pitch + Ki_pitch*dt*dt + 2*Kd_pitch
    beta = dt*dt*Ki_pitch - 4*Kd_pitch - 2*dt*Kp_pitch
    gamma = 2*Kd_pitch
    Output = (alpha*E + beta*E1_pitch + gamma*E2_pitch + 2*dt*LastOutput_pitch)/(2*dt)
    LastOutput_pitch = Output
    E2_pitch=E1_pitch
    E1_pitch=E
    return Output

def PID_yaw(angle,dt):
    global E1_yaw, E2_yaw, LastOutput_yaw
    E = angle
    alpha = 2*dt*Kp_yaw + Ki_yaw*dt*dt + 2*Kd_yaw
    beta = dt*dt*Ki_yaw - 4*Kd_yaw - 2*dt*Kp_yaw
    gamma = 2*Kd_yaw
    Output = (alpha*E + beta*E1_yaw + gamma*E2_yaw + 2*dt*LastOutput_yaw)/(2*dt)
    LastOutput_yaw = Output
    E2_yaw=E1_yaw
    E1_yaw=E
    return Output

end=0.0
E1_roll=0.0
E2_roll=0.0
E1_pitch=0.0
E2_pitch=0.0
E1_yaw=0.0
E2_yaw=0.0
LastOutput_roll=0.0
LastOutput_pitch=0.0
LastOutput_yaw=0.0
throttle=0.0

while True:
    dt=(pi.get_current_tick() - end)/1000000
    
    if (dt >= 0.001):   #Sample rate 1000Hz
        end = pi.get_current_tick()
        #Calculate roll, pitch, yaw angle
        (Xrate,Yrate,Zrate)=l3g4200d.readNormalize()
        adxl345.readNormalize()
        pitch_raw = -(math.atan2(adxl345.nXAxis, math.sqrt(adxl345.nYAxis*adxl345.nYAxis + adxl345.nZAxis*adxl345.nZAxis))*180.0)/math.pi;
        roll_raw  = (math.atan2(adxl345.nYAxis, adxl345.nZAxis)*180.0)/math.pi;
        roll = kalmanX.getAngle(roll_raw,Xrate,dt) - 1.5   #Subtract offset
        pitch = kalmanY.getAngle(pitch_raw,Yrate,dt) + 2.2
        if ser.inWaiting() >0:
            throttle=float(ser.readline())
#         print(type(throttle))
        esc_1_rps = throttle + PID_pitch(pitch,dt) + PID_roll(roll,dt)# - PID_yaw(yaw,dt)
        esc_2_rps = throttle + PID_pitch(pitch,dt) - PID_roll(roll,dt)# + PID_yaw(yaw,dt)
        esc_3_rps = throttle - PID_pitch(pitch,dt) - PID_roll(roll,dt)# - PID_yaw(yaw,dt)
        esc_4_rps = throttle - PID_pitch(pitch,dt) + PID_roll(roll,dt)# + PID_yaw(yaw,dt)

        pi.set_servo_pulsewidth(ESC1, Rps2Pulse1(esc_1_rps))
        pi.set_servo_pulsewidth(ESC2, Rps2Pulse2(esc_2_rps))
        pi.set_servo_pulsewidth(ESC3, Rps2Pulse3(esc_3_rps))
        pi.set_servo_pulsewidth(ESC4, Rps2Pulse4(esc_4_rps))
        ser.write(b'%f %f %f %f %f %f %f\r\n'%(roll, pitch, esc_1_rps, esc_2_rps, esc_3_rps, esc_4_rps, dt))
        ser.flush()


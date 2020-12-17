import serial
import time
import numpy as np
import math

class GPS:
    def __init__(self):
        port="/dev/ttyS0"
        self.ser=serial.Serial(port, baudrate=57600, timeout=0.5)
        self.lat_gps_actual = 0
        self.lon_gps_actual = 0
        self.lat_gps_previous=0
        self.lon_gps_previous=0
        self.l_lat_gps=0
        self.l_lon_gps=0
        self.gps_add_counter=0

        self.new_line_found=0
        self.new_gps_data_counter=0
        self.new_gps_data_available=0
        self.waypoint_set=0

        self.gps_lat_rotating_mem = np.zeros(35)
        self.gps_lat_total_avarage=0
        self.gps_lon_rotating_mem = np.zeros(35)
        self.gps_lon_total_avarage=0

        self.gps_rotating_mem_location=0

        self.gps_lat_error_previous=0
        self.gps_lon_error_previous=0

        self.pos_status=0
        self.gps_lon_error = 0
        self.gps_lat_error = 0
        self.gps_roll_adjust_north = 0
        self.gps_pitch_adjust_north = 0
        self.gps_roll_adjust = 0
        self.gps_pitch_adjust = 0
        self.l_lon_waypoint = 0
        self.l_lat_waypoint = 0
        self.l_lon_gps = 0
        self.l_lat_gps = 0
        self.lat_gps_add = 0
        self.lon_gps_add = 0
        
        self.lat_gps_loop_add = 0
        self.lon_gps_loop_add = 0
        
        self.flight_mode=0
        self.waypoint_set=0
        self.gps_p_gain = 2
        self.gps_d_gain = 6
    def read_GPS_PID(self):
        return self.gps_pitch_adjust_north, self.gps_roll_adjust_north
#         return self.lat_gps_add, self.lon_gps_add
    def set_mode(self, mode):
        self.flight_mode=mode

    def read_GPS(self):
        
        if(self.ser.in_waiting):
            self.newdata = self.ser.readline()
            self.newdata = self.newdata.decode('utf-8')
#             print(self.newdata)
            self.new_line_found = 1
        #     print(newdata)
        if self.new_line_found==1:
            self.new_line_found = 0
            if self.newdata[0:6] == "$GPRMC" and self.newdata[17] == "V":
                self.pos_status = 0
            if self.newdata[0:6] == "$GPRMC" and self.newdata[17] == "A":
                self.lat_gps_actual = int(self.newdata[21]) *  10000000
                self.lat_gps_actual += int(self.newdata[22]) * 1000000
                self.lat_gps_actual += int(self.newdata[24]) * 100000
                self.lat_gps_actual += int(self.newdata[25]) * 10000
                self.lat_gps_actual += int(self.newdata[26]) * 1000
                self.lat_gps_actual += int(self.newdata[27]) * 100
                self.lat_gps_actual += int(self.newdata[28]) * 10
                self.lat_gps_actual /= 6                                         
                self.lat_gps_actual += int(self.newdata[19]) * 100000000
                self.lat_gps_actual += int(self.newdata[20]) * 10000000
                self.lat_gps_actual /= 10
                        #############
                self.lon_gps_actual = int(self.newdata[35]) *  10000000
                self.lon_gps_actual += int(self.newdata[36]) * 1000000
                self.lon_gps_actual += int(self.newdata[38]) * 100000
                self.lon_gps_actual += int(self.newdata[39]) * 10000
                self.lon_gps_actual += int(self.newdata[40]) * 1000
                self.lon_gps_actual += int(self.newdata[41]) * 100
                self.lon_gps_actual += int(self.newdata[42]) * 10
                self.lon_gps_actual /= 6
                self.lon_gps_actual += int(self.newdata[32]) * 1000000000
                self.lon_gps_actual += int(self.newdata[33]) * 100000000
                self.lon_gps_actual += int(self.newdata[34]) * 10000000
                self.lon_gps_actual /= 10
                if self.lat_gps_previous == 0 and self.lon_gps_previous == 0: 
                    self.lat_gps_previous = self.lat_gps_actual
                    self.lon_gps_previous = self.lon_gps_actual

                self.lat_gps_loop_add = (float)(self.lat_gps_actual - self.lat_gps_previous) / 10.0
                self.lon_gps_loop_add = (float)(self.lon_gps_actual - self.lon_gps_previous) / 10.0
#                 print(round(self.lat_gps_actual,3), round(self.lon_gps_actual,3))
                self.l_lat_gps = self.lat_gps_previous
                self.l_lon_gps = self.lon_gps_previous

                self.lat_gps_previous = self.lat_gps_actual
                self.lon_gps_previous = self.lon_gps_actual

                self.gps_add_counter = 4
                self.new_gps_data_counter = 9
                self.lat_gps_add = 0
                self.lon_gps_add = 0
                self.new_gps_data_available = 1
                self.pos_status=1
              
        #After 5 program loops 4 x 5ms = 20ms the gps_add_counter is 0.
        if (self.gps_add_counter >= 0):
            self.gps_add_counter -= 1
#         print(self.gps_add_counter)
        if self.gps_add_counter == 0 and self.new_gps_data_counter > 0:
            self.new_gps_data_available = 1
            self.new_gps_data_counter -=1
            self.gps_add_counter = 4

            self.lat_gps_add += self.lat_gps_loop_add
            if (abs(self.lat_gps_add) >= 1):
                self.l_lat_gps += (int)(self.lat_gps_add)
                self.lat_gps_add -= (int)(self.lat_gps_add)
                  
            self.lon_gps_add += self.lon_gps_loop_add
            if (abs(self.lon_gps_add) >= 1):
                self.l_lon_gps += (int)(self.lon_gps_add)
                self.lon_gps_add -= (int)(self.lon_gps_add)
#         print(self.new_gps_data_available, self.flight_mode, self.waypoint_set)
        if (self.new_gps_data_available==1):
#             print("Helo")
#             self.new_gps_data_available = 0
            
            if self.flight_mode >= 1 and self.waypoint_set == 0:
                self.waypoint_set = 1
                self.l_lat_waypoint = self.l_lat_gps
                self.l_lon_waypoint = self.l_lon_gps

            if self.flight_mode >= 1 and self.waypoint_set == 1:
                self.gps_lon_error = self.l_lon_waypoint - self.l_lon_gps
                self.gps_lat_error = self.l_lat_gps - self.l_lat_waypoint

                self.gps_lat_total_avarage -=  self.gps_lat_rotating_mem[ self.gps_rotating_mem_location]
                self.gps_lat_rotating_mem[ self.gps_rotating_mem_location] = self.gps_lat_error - self.gps_lat_error_previous
                self.gps_lat_total_avarage +=  self.gps_lat_rotating_mem[ self.gps_rotating_mem_location]

                self.gps_lon_total_avarage -=  self.gps_lon_rotating_mem[ self.gps_rotating_mem_location]
                self.gps_lon_rotating_mem[ self.gps_rotating_mem_location] = self.gps_lon_error - self.gps_lon_error_previous
                self.gps_lon_total_avarage +=  self.gps_lon_rotating_mem[ self.gps_rotating_mem_location]
                self.gps_rotating_mem_location += 1
                if ( self.gps_rotating_mem_location == 35):
                    self.gps_rotating_mem_location = 0

                self.gps_lat_error_previous = self.gps_lat_error
                self.gps_lon_error_previous = self.gps_lon_error
                    
                self.gps_pitch_adjust_north = (float)(self.gps_lat_error) * self.gps_p_gain + (float)(self.gps_lat_total_avarage) * self.gps_d_gain
                self.gps_roll_adjust_north = (float)(self.gps_lon_error) * self.gps_p_gain + (float)(self.gps_lon_total_avarage) * self.gps_d_gain
#                 print(self.gps_pitch_adjust_north , self.gps_roll_adjust_north)
    #             gps_roll_adjust = ((float)(gps_roll_adjust_north) * math.cos(angle_yaw * 0.017453)) + ((float)(gps_pitch_adjust_north) * math.cos((angle_yaw - 90) * 0.017453))
    #             gps_pitch_adjust = ((float)(gps_pitch_adjust_north) * math.cos(angle_yaw * 0.017453)) + ((float)(gps_roll_adjust_north) * math.cos((angle_yaw + 90) * 0.017453))

                if (self.gps_roll_adjust > 20):
                    self.gps_roll_adjust = 20
                if (self.gps_roll_adjust < -20):
                    self.gps_roll_adjust = -20
                if (self.gps_pitch_adjust > 20):
                    self.gps_pitch_adjust = 20
                if (self.gps_pitch_adjust < -20):
                    self.gps_pitch_adjust = -20

gps = GPS()
gps_add_counter = 0
end2 = 0
elapse_2 = 0
while True:
    elapse_2 = time.time() - end2
#     print(gps.read_GPS_PID())
    gps.read_GPS()
    time.sleep(1/200)
    if elapse_2 >1:
        gps.set_mode(1)
        end2 = time.time()
        print(gps.read_GPS_PID())
#         print("Hrllo")
        
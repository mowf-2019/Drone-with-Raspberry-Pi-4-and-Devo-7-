import serial
import time
import numpy as np

port="/dev/ttyS0"
ser=serial.Serial(port, baudrate=57600, timeout=0.5)

lat_gps_actual = 0
lon_gps_actual = 0
lat_gps_previous=0
lon_gps_previous=0
l_lat_gps=0
l_lon_gps=0
gps_add_counter=0
end=0
elapse_time =0

new_line_found=0
new_gps_data_counter=0
new_gps_data_available=0
waypoint_set=0

gps_lat_rotating_mem = np.zeros(35)
gps_lat_total_avarage=0
gps_lon_rotating_mem = np.zeros(35)
gps_lon_total_avarage=0

gps_rotating_mem_location=0

gps_lat_error_previous=0
gps_lon_error_previous=0

pos_status=0

flight_mode=1
waypoint_set=0

while True:
    elapse_time = time.time() - end
    if elapse_time > 1/200:
        end = time.time()
        if(ser.in_waiting):
            newdata=ser.readline()
            newdata = newdata.decode('utf-8')
            new_line_found = 1
    #     print(newdata)
        if new_line_found==1:
            new_line_found = 0
            if newdata[0:6] == "$GPRMC" and newdata[17] == "V":
                pos_status = 0
            if newdata[0:6] == "$GPRMC" and newdata[17] == "A":
                lat_gps_actual = int(newdata[21]) *  10000000
                lat_gps_actual += int(newdata[22]) * 1000000
                lat_gps_actual += int(newdata[24]) * 100000
                lat_gps_actual += int(newdata[25]) * 10000
                lat_gps_actual += int(newdata[26]) * 1000
                lat_gps_actual += int(newdata[27]) * 100
                lat_gps_actual += int(newdata[28]) * 10
                lat_gps_actual /= 6                                         
                lat_gps_actual += int(newdata[19]) * 100000000
                lat_gps_actual += int(newdata[20]) * 10000000
                lat_gps_actual /= 10
                    #############
                lon_gps_actual = int(newdata[35]) *  10000000
                lon_gps_actual += int(newdata[36]) * 1000000
                lon_gps_actual += int(newdata[38]) * 100000
                lon_gps_actual += int(newdata[39]) * 10000
                lon_gps_actual += int(newdata[40]) * 1000
                lon_gps_actual += int(newdata[41]) * 100
                lon_gps_actual += int(newdata[42]) * 10
                lon_gps_actual /= 6
                lon_gps_actual += int(newdata[32]) * 1000000000
                lon_gps_actual += int(newdata[33]) * 100000000
                lon_gps_actual += int(newdata[34]) * 10000000
                lon_gps_actual /= 100
#                 print(lon_gps_actual)
#                 print(newdata)
                if lat_gps_previous == 0 and lon_gps_previous == 0: 
                    lat_gps_previous = lat_gps_actual
                    lon_gps_previous = lon_gps_actual

                lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0
                lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0

                l_lat_gps = lat_gps_previous
                l_lon_gps = lon_gps_previous

                lat_gps_previous = lat_gps_actual
                lon_gps_previous = lon_gps_actual

                gps_add_counter = 4
                new_gps_data_counter = 9
                lat_gps_add = 0
                lon_gps_add = 0
                new_gps_data_available = 1
                pos_status=1
          
        #After 5 program loops 4 x 5ms = 20ms the gps_add_counter is 0.
        if gps_add_counter == 0 and new_gps_data_counter > 0:
            new_gps_data_available = 1
            new_gps_data_counter -=1
            gps_add_counter = 4

            lat_gps_add += lat_gps_loop_add
            if (abs(lat_gps_add) >= 1):
                l_lat_gps += (int)(lat_gps_add)
                lat_gps_add -= (int)(lat_gps_add)
              
            lon_gps_add += lon_gps_loop_add
            if (abs(lon_gps_add) >= 1):
                l_lon_gps += (int)(lon_gps_add)
                lon_gps_add -= (int)(lon_gps_add)
        
#         if (new_gps_data_available):
#             new_gps_data_available = 0
# 
#             if flight_mode >= 1 and waypoint_set == 0:
#                 waypoint_set = 1
#                 l_lat_waypoint = l_lat_gps
#                 l_lon_waypoint = l_lon_gps
# 
#             if flight_mode >= 1 and waypoint_set == 1:
#                 gps_lon_error = l_lon_waypoint - l_lon_gps
#                 gps_lat_error = l_lat_gps - l_lat_waypoint
# 
#                 gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location]
#                 gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous
#                 gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location]
# 
#                 gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location]
#                 gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous
#                 gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location]
#                 gps_rotating_mem_location += 1
#                 if ( gps_rotating_mem_location == 35):
#                     gps_rotating_mem_location = 0
# 
#                 gps_lat_error_previous = gps_lat_error
#                 gps_lon_error_previous = gps_lon_error
#                 
#                 gps_pitch_adjust_north = (float)(gps_lat_error) * gps_p_gain + (float)(gps_lat_total_avarage) * gps_d_gain
#                 gps_roll_adjust_north = (float)(gps_lon_error) * gps_p_gain + (float)(gps_lon_total_avarage) * gps_d_gain
# 
#                 gps_roll_adjust = ((float)(gps_roll_adjust_north) * cos(angle_yaw * 0.017453)) + ((float)(gps_pitch_adjust_north) * cos((angle_yaw - 90) * 0.017453))
#                 gps_pitch_adjust = ((float)(gps_pitch_adjust_north) * cos(angle_yaw * 0.017453)) + ((float)(gps_roll_adjust_north) * cos((angle_yaw + 90) * 0.017453))
# 
#                 if (gps_roll_adjust > 20):
#                     gps_roll_adjust = 20
#                 if (gps_roll_adjust < -20):
#                     gps_roll_adjust = -20
#                 if (gps_pitch_adjust > 20):
#                     gps_pitch_adjust = 20
#                 if (gps_pitch_adjust < -20):
#                     gps_pitch_adjust = -20
#         if pos_status == 1:
#             gps_roll_adjust_out = gps_roll_adjust
#             gps_pitch_adjust_out = gps_pitch_adjust
#         else:
#             gps_roll_adjust_out = 0
#             gps_pitch_adjust_out = 0
                
                
    #             print("latitude", newdata[19:29])
    #             print("latitude dir", newdata[30])
                
    #             print("longitude", newdata[32:43])
    #             print("longitude dir", newdata[44])

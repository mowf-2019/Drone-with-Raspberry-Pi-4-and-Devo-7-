import smbus
import time
import math


motion_rate = 200
adc_frequency = 1000
FULL_FIFO_BATCHES=20
sampling_rate=500
motion_rate = 200
####################################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
####################################################################################################
class I2C:

    def __init__(self, address, bus=smbus.SMBus(1)):
        self.address = address
        self.bus = bus
        self.misses = 0

    def writeByte(self, value):
        self.bus.write_byte(self.address, value)

    def write8(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def writeList(self, reg, list):
        self.bus.write_i2c_block_data(self.address, reg, list)

    def readU8(self, reg):
        result = self.bus.read_byte_data(self.address, reg)
        return result

    def readS8(self, reg):
        result = self.bus.read_byte_data(self.address, reg)
        result = result - 256 if result > 127 else result
        return result

    def readU16(self, reg):
        hibyte = self.bus.read_byte_data(self.address, reg)
        result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
        return result

    def readS16(self, reg):
        hibyte = self.bus.read_byte_data(self.address, reg)
        hibyte = hibyte - 256 if hibyte > 127 else hibyte
        result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
        return result

    def readList(self, reg, length):
        "Reads a byte array value from the I2C device. The content depends on the device.  The "
        "FIFO read return sequential values from the same register.  For all other, sequestial"
        "regester values are returned"
        result = self.bus.read_i2c_block_data(self.address, reg, length)
        return result

####################################################################################################
#
#  Gyroscope / Accelerometer class for reading position / movement.  Works with the Invensense IMUs:
#
#  - MPU-6050
#  - MPU-9150
#  - MPU-9250
#
####################################################################################################
class MPU6050:
    i2c = None

    # Registers/etc.
    __MPU6050_RA_SELF_TEST_XG = 0x00
    __MPU6050_RA_SELF_TEST_YG = 0x01
    __MPU6050_RA_SELF_TEST_ZG = 0x02
    __MPU6050_RA_SELF_TEST_XA = 0x0D
    __MPU6050_RA_SELF_TEST_YA = 0x0E
    __MPU6050_RA_SELF_TEST_ZA = 0x0F
    __MPU6050_RA_XG_OFFS_USRH = 0x13
    __MPU6050_RA_XG_OFFS_USRL = 0x14
    __MPU6050_RA_YG_OFFS_USRH = 0x15
    __MPU6050_RA_YG_OFFS_USRL = 0x16
    __MPU6050_RA_ZG_OFFS_USRH = 0x17
    __MPU6050_RA_ZG_OFFS_USRL = 0x18
    __MPU6050_RA_SMPLRT_DIV = 0x19
    __MPU6050_RA_CONFIG = 0x1A
    __MPU6050_RA_GYRO_CONFIG = 0x1B
    __MPU6050_RA_ACCEL_CONFIG = 0x1C
    __MPU9250_RA_ACCEL_CFG_2 = 0x1D
    __MPU6050_RA_FF_THR = 0x1D
    __MPU6050_RA_FF_DUR = 0x1E
    __MPU6050_RA_MOT_THR = 0x1F
    __MPU6050_RA_MOT_DUR = 0x20
    __MPU6050_RA_ZRMOT_THR = 0x21
    __MPU6050_RA_ZRMOT_DUR = 0x22
    __MPU6050_RA_FIFO_EN = 0x23
    __MPU6050_RA_I2C_MST_CTRL = 0x24
    __MPU6050_RA_I2C_SLV0_ADDR = 0x25
    __MPU6050_RA_I2C_SLV0_REG = 0x26
    __MPU6050_RA_I2C_SLV0_CTRL = 0x27
    __MPU6050_RA_I2C_SLV1_ADDR = 0x28
    __MPU6050_RA_I2C_SLV1_REG = 0x29
    __MPU6050_RA_I2C_SLV1_CTRL = 0x2A
    __MPU6050_RA_I2C_SLV2_ADDR = 0x2B
    __MPU6050_RA_I2C_SLV2_REG = 0x2C
    __MPU6050_RA_I2C_SLV2_CTRL = 0x2D
    __MPU6050_RA_I2C_SLV3_ADDR = 0x2E
    __MPU6050_RA_I2C_SLV3_REG = 0x2F
    __MPU6050_RA_I2C_SLV3_CTRL = 0x30
    __MPU6050_RA_I2C_SLV4_ADDR = 0x31
    __MPU6050_RA_I2C_SLV4_REG = 0x32
    __MPU6050_RA_I2C_SLV4_DO = 0x33
    __MPU6050_RA_I2C_SLV4_CTRL = 0x34
    __MPU6050_RA_I2C_SLV4_DI = 0x35
    __MPU6050_RA_I2C_MST_STATUS = 0x36
    __MPU6050_RA_INT_PIN_CFG = 0x37
    __MPU6050_RA_INT_ENABLE = 0x38
    __MPU6050_RA_DMP_INT_STATUS = 0x39
    __MPU6050_RA_INT_STATUS = 0x3A
    __MPU6050_RA_ACCEL_XOUT_H = 0x3B
    __MPU6050_RA_ACCEL_XOUT_L = 0x3C
    __MPU6050_RA_ACCEL_YOUT_H = 0x3D
    __MPU6050_RA_ACCEL_YOUT_L = 0x3E
    __MPU6050_RA_ACCEL_ZOUT_H = 0x3F
    __MPU6050_RA_ACCEL_ZOUT_L = 0x40
    __MPU6050_RA_TEMP_OUT_H = 0x41
    __MPU6050_RA_TEMP_OUT_L = 0x42
    __MPU6050_RA_GYRO_XOUT_H = 0x43
    __MPU6050_RA_GYRO_XOUT_L = 0x44
    __MPU6050_RA_GYRO_YOUT_H = 0x45
    __MPU6050_RA_GYRO_YOUT_L = 0x46
    __MPU6050_RA_GYRO_ZOUT_H = 0x47
    __MPU6050_RA_GYRO_ZOUT_L = 0x48
    __MPU6050_RA_EXT_SENS_DATA_00 = 0x49
    __MPU6050_RA_EXT_SENS_DATA_01 = 0x4A
    __MPU6050_RA_EXT_SENS_DATA_02 = 0x4B
    __MPU6050_RA_EXT_SENS_DATA_03 = 0x4C
    __MPU6050_RA_EXT_SENS_DATA_04 = 0x4D
    __MPU6050_RA_EXT_SENS_DATA_05 = 0x4E
    __MPU6050_RA_EXT_SENS_DATA_06 = 0x4F
    __MPU6050_RA_EXT_SENS_DATA_07 = 0x50
    __MPU6050_RA_EXT_SENS_DATA_08 = 0x51
    __MPU6050_RA_EXT_SENS_DATA_09 = 0x52
    __MPU6050_RA_EXT_SENS_DATA_10 = 0x53
    __MPU6050_RA_EXT_SENS_DATA_11 = 0x54
    __MPU6050_RA_EXT_SENS_DATA_12 = 0x55
    __MPU6050_RA_EXT_SENS_DATA_13 = 0x56
    __MPU6050_RA_EXT_SENS_DATA_14 = 0x57
    __MPU6050_RA_EXT_SENS_DATA_15 = 0x58
    __MPU6050_RA_EXT_SENS_DATA_16 = 0x59
    __MPU6050_RA_EXT_SENS_DATA_17 = 0x5A
    __MPU6050_RA_EXT_SENS_DATA_18 = 0x5B
    __MPU6050_RA_EXT_SENS_DATA_19 = 0x5C
    __MPU6050_RA_EXT_SENS_DATA_20 = 0x5D
    __MPU6050_RA_EXT_SENS_DATA_21 = 0x5E
    __MPU6050_RA_EXT_SENS_DATA_22 = 0x5F
    __MPU6050_RA_EXT_SENS_DATA_23 = 0x60
    __MPU6050_RA_MOT_DETECT_STATUS = 0x61
    __MPU6050_RA_I2C_SLV0_DO = 0x63
    __MPU6050_RA_I2C_SLV1_DO = 0x64
    __MPU6050_RA_I2C_SLV2_DO = 0x65
    __MPU6050_RA_I2C_SLV3_DO = 0x66
    __MPU6050_RA_I2C_MST_DELAY_CTRL = 0x67
    __MPU6050_RA_SIGNAL_PATH_RESET = 0x68
    __MPU6050_RA_MOT_DETECT_CTRL = 0x69
    __MPU6050_RA_USER_CTRL = 0x6A
    __MPU6050_RA_PWR_MGMT_1 = 0x6B
    __MPU6050_RA_PWR_MGMT_2 = 0x6C
    __MPU6050_RA_BANK_SEL = 0x6D
    __MPU6050_RA_MEM_START_ADDR = 0x6E
    __MPU6050_RA_MEM_R_W = 0x6F
    __MPU6050_RA_DMP_CFG_1 = 0x70
    __MPU6050_RA_DMP_CFG_2 = 0x71
    __MPU6050_RA_FIFO_COUNTH = 0x72
    __MPU6050_RA_FIFO_COUNTL = 0x73
    __MPU6050_RA_FIFO_R_W = 0x74
    __MPU6050_RA_WHO_AM_I = 0x75

    #-----------------------------------------------------------------------------------------------
    # Compass output registers when using the I2C master / slave
    #-----------------------------------------------------------------------------------------------
    __MPU9250_RA_MAG_XOUT_L = 0x4A
    __MPU9250_RA_MAG_XOUT_H = 0x4B
    __MPU9250_RA_MAG_YOUT_L = 0x4C
    __MPU9250_RA_MAG_YOUT_H = 0x4D
    __MPU9250_RA_MAG_ZOUT_L = 0x4E
    __MPU9250_RA_MAG_ZOUT_H = 0x4F

    #-----------------------------------------------------------------------------------------------
    # Compass output registers when directly accessing via IMU bypass
    #-----------------------------------------------------------------------------------------------
    __HMC5883L_ADDRESS      = 0x1E
    __HMC5883L_REG_CONFIG_A = 0x00
    __HMC5883L_REG_CONFIG_B = 0x01
    __HMC5883L_REG_MODE     = 0x02
    __HMC5883L_REG_OUT_X_M  = 0x03
    __HMC5883L_REG_OUT_X_L  = 0x04
    __HMC5883L_REG_OUT_Z_M  = 0x05
    __HMC5883L_REG_OUT_Z_L  = 0x06
    __HMC5883L_REG_OUT_Y_M  = 0x07
    __HMC5883L_REG_OUT_Y_L  = 0x08
    __HMC5883L_REG_STATUS   = 0x09
    __HMC5883L_REG_IDENT_A  = 0x0A
    __HMC5883L_REG_IDENT_B  = 0x0B
    __HMC5883L_REG_IDENT_C  = 0x0C
    
    __HMC5883L_SAMPLES_8     = 0b11
    __HMC5883L_SAMPLES_4     = 0b10
    __HMC5883L_SAMPLES_2     = 0b01
    __HMC5883L_SAMPLES_1     = 0b00
    
    __HMC5883L_DATARATE_75HZ       = 0b110
    __HMC5883L_DATARATE_30HZ       = 0b101
    __HMC5883L_DATARATE_15HZ       = 0b100
    __HMC5883L_DATARATE_7_5HZ      = 0b011
    __HMC5883L_DATARATE_3HZ        = 0b010
    __HMC5883L_DATARATE_1_5HZ      = 0b001
    __HMC5883L_DATARATE_0_75_HZ    = 0b000
    
    __HMC5883L_RANGE_8_1GA     = 0b111
    __HMC5883L_RANGE_5_6GA     = 0b110
    __HMC5883L_RANGE_4_7GA     = 0b101
    __HMC5883L_RANGE_4GA       = 0b100
    __HMC5883L_RANGE_2_5GA     = 0b011
    __HMC5883L_RANGE_1_9GA     = 0b010
    __HMC5883L_RANGE_1_3GA     = 0b001
    __HMC5883L_RANGE_0_88GA    = 0b000
    
    __HMC5883L_IDLE          = 0b10
    __HMC5883L_SINGLE        = 0b01
    __HMC5883L_CONTINOUS     = 0b00

    __RANGE_ACCEL = 8                                                            #AB: +/- 8g
    __RANGE_GYRO = 250                                                           #AB: +/- 250o/s

    __SCALE_GYRO = math.radians(2 * __RANGE_GYRO / 65536)
    __SCALE_ACCEL = 2 * __RANGE_ACCEL / 65536

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address

        self.min_az = 0.0
        self.max_az = 0.0
        self.min_gx = 0.0
        self.max_gx = 0.0
        self.min_gy = 0.0
        self.max_gy = 0.0
        self.min_gz = 0.0
        self.max_gz = 0.0

        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.az_offset = 0.0

        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0

        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets sample rate to 1kHz/(1+0) = 1kHz or 1ms (note 1kHz assumes dlpf is on - setting
        # dlpf to 0 or 7 changes 1kHz to 8kHz and therefore will require sample rate divider
        # to be changed to 7 to obtain the same 1kHz sample rate.
        #-------------------------------------------------------------------------------------------
        sample_rate_divisor = int(round(adc_frequency / sampling_rate))
        self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, sample_rate_divisor - 1)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets clock source to gyro reference w/ PLL
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Gyro DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 =  250Hz @ 8kHz sampling - DO NOT USE, THE ACCELEROMETER STILL SAMPLES AT 1kHz WHICH PRODUCES EXPECTED BUT NOT CODED FOR TIMING AND FIFO CONTENT PROBLEMS
        # 0x01 =  184Hz
        # 0x02 =   92Hz
        # 0x03 =   41Hz
        # 0x04 =   20Hz
        # 0x05 =   10Hz
        # 0x06 =    5Hz
        # 0x07 = 3600Hz @ 8kHz
        #
        # 0x0* FIFO overflow overwrites oldest FIFO contents
        # 0x4* FIFO overflow does not overwrite full FIFO contents
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x40 | glpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable gyro self tests, scale of +/- 250 degrees/s
        #
        # 0x00 =  +/- 250 degrees/s
        # 0x08 =  +/- 500 degrees/s
        # 0x10 = +/- 1000 degrees/s
        # 0x18 = +/- 2000 degrees/s
        # See SCALE_GYRO for conversion from raw data to units of radians per second
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, int(round(math.log(self.__RANGE_GYRO / 250, 2))) << 3)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Accel DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 = 460Hz
        # 0x01 = 184Hz
        # 0x02 =  92Hz
        # 0x03 =  41Hz
        # 0x04 =  20Hz
        # 0x05 =  10Hz
        # 0x06 =   5Hz
        # 0x07 = 460Hz
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU9250_RA_ACCEL_CFG_2, alpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable accel self tests, scale of +/-8g
        #
        # 0x00 =  +/- 2g
        # 0x08 =  +/- 4g
        # 0x10 =  +/- 8g
        # 0x18 = +/- 16g
        # See SCALE_ACCEL for convertion from raw data to units of meters per second squared
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, int(round(math.log(self.__RANGE_ACCEL / 2, 2))) << 3)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set INT pin to push/pull, latch 'til read, any read to clear
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x30)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Initialize the FIFO overflow interrupt 0x10 (turned off at startup).
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enabled the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x40)

        #-------------------------------------------------------------------------------------------
        # Accelerometer / gyro goes into FIFO later on - see flushFIFO()
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Read ambient temperature
        #-------------------------------------------------------------------------------------------
        temp = self.readTemperature()
        
    def readTemperature(self):
        temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
        return temp

    def enableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Clear the interrupt status register and enable the FIFO overflow interrupt 0x10
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x10)
        self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)

    def disableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Disable the FIFO overflow interrupt.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)

    def numFIFOBatches(self):
        #-------------------------------------------------------------------------------------------
        # The FIFO is 512 bytes long, and we're storing 6 signed shorts (ax, ay, az, gx, gy, gz) i.e.
        # 12 bytes per batch of sensor readings
        #-------------------------------------------------------------------------------------------
        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)
        fifo_batches = int(fifo_bytes / 12)  # This rounds down
        return fifo_batches

    def readFIFO(self, fifo_batches):
        #-------------------------------------------------------------------------------------------
        # Read n x 12 bytes of FIFO data averaging, and return the averaged values and inferred time
        # based upon the sampling rate and the number of samples.
        #-------------------------------------------------------------------------------------------
        ax = 0
        ay = 0
        az = 0
        gx = 0
        gy = 0
        gz = 0

        for ii in range(fifo_batches):
            sensor_data = []
            fifo_batch = self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, 12)
            for jj in range(0, 12, 2):
                hibyte = fifo_batch[jj]
                hibyte = hibyte - 256 if hibyte > 127 else hibyte
                lobyte = fifo_batch[jj + 1]
                sensor_data.append((hibyte << 8) + lobyte)

            ax += sensor_data[0]
            ay += sensor_data[1]
            az += sensor_data[2]
            gx += sensor_data[3]
            gy += sensor_data[4]
            gz += sensor_data[5]

            '''
            self.max_az = self.max_az if sensor_data[2] < self.max_az else sensor_data[2]
            self.min_az = self.min_az if sensor_data[2] > self.min_az else sensor_data[2]

            self.max_gx = self.max_gx if sensor_data[3] < self.max_gx else sensor_data[3]
            self.min_gx = self.min_gx if sensor_data[3] > self.min_gx else sensor_data[3]

            self.max_gy = self.max_gy if sensor_data[4] < self.max_gy else sensor_data[4]
            self.min_gy = self.min_gy if sensor_data[4] > self.min_gy else sensor_data[4]

            self.max_gz = self.max_gz if sensor_data[5] < self.max_gz else sensor_data[5]
            self.min_gz = self.min_gz if sensor_data[5] > self.min_gz else sensor_data[5]
            '''

        return ax / fifo_batches, ay / fifo_batches, az / fifo_batches, gx / fifo_batches, gy / fifo_batches, gz / fifo_batches, fifo_batches / sampling_rate

    def flushFIFO(self):
        #-------------------------------------------------------------------------------------------
        # First shut off the feed in the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Empty the FIFO by reading whatever is there
        #-------------------------------------------------------------------------------------------
        SMBUS_MAX_BUF_SIZE = 32

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(int(fifo_bytes / SMBUS_MAX_BUF_SIZE)):
            self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, SMBUS_MAX_BUF_SIZE)

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(fifo_bytes):
            self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)

        #-------------------------------------------------------------------------------------------
        # Finally start feeding the FIFO with sensor data again
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x78)

    def setGyroOffsets(self, gx, gy, gz):
        self.gx_offset = gx
        self.gy_offset = gy
        self.gz_offset = gz

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = (ax - self.ax_offset) * self.__SCALE_ACCEL
        qay = (ay - self.ay_offset) * self.__SCALE_ACCEL
        qaz = (az - self.az_offset) * self.__SCALE_ACCEL

        qrx = (gx - self.gx_offset) * self.__SCALE_GYRO
        qry = (gy - self.gy_offset) * self.__SCALE_GYRO
        qrz = (gz - self.gz_offset) * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz


    def getStats(self):
        return (self.max_az * self.__SCALE_ACCEL,
                self.min_az * self.__SCALE_ACCEL,
                self.max_gx * self.__SCALE_GYRO,
                self.min_gx * self.__SCALE_GYRO,
                self.max_gy * self.__SCALE_GYRO,
                self.min_gy * self.__SCALE_GYRO,
                self.max_gz * self.__SCALE_GYRO,
                self.min_gz * self.__SCALE_GYRO)
#-------------------------------------------------------------------------------------------
# HMC5883L to MPU6050 via AUX I2C
#-------------------------------------------------------------------------------------------
#     def initCompass(self):
#         SLV0_RW = 0      # 0: write, 1: read
#         
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_ADDR, (SLV0_RW<<7)|self.__HMC5883L_ADDRESS)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_CTRL, 0xA1)
#         self.Compass_setRange(self.__HMC5883L_RANGE_1_3GA)
#         self.Compass_setMesurementMode(self.__HMC5883L_CONTINOUS)
#         self.Compass_setDataRate(self.__HMC5883L_DATARATE_30HZ)
#         self.Compass_setSamples(self.__HMC5883L_SAMPLES_8)
#         
#     def Compass_readRaw(self):
#         
#         magX = 0
#         magY = 0
#         magZ = 0
#         
#         SLV0_RW = 1
#         
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_ADDR, (SLV0_RW<<7)|self.__HMC5883L_ADDRESS)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_CTRL, 0xA6)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_REG, self.__HMC5883L_REG_OUT_X_M)
#         
#         
#         compass_raw_data = []
#         data_batch = self.i2c.readList(self.__MPU6050_RA_EXT_SENS_DATA_00, 6)
#         for k in range(0, 6, 2):
#             hibyte1 = data_batch[k]
#             hibyte1 = hibyte - 256 if hibyte1 > 127 else hibyte1
#             lobyte1 = data_batch[k + 1]
#             compass_raw_data.append((hibyte1 << 8) + lobyte1)
# 
#         magX = compass_raw_data[0]
#         magZ = compass_raw_data[1]
#         magY = compass_raw_data[2]
#         return magX, magY, magZ
# 
#     def Compass_setRange(self,argument):
#         switcher = {
#             self.__HMC5883L_RANGE_0_88GA: 0.073,
#             self.__HMC5883L_RANGE_1_3GA:  0.92,
#             self.__HMC5883L_RANGE_1_9GA:  1.22,
#             self.__HMC5883L_RANGE_2_5GA:  1.52,
#             self.__HMC5883L_RANGE_4GA:    2.27,
#             self.__HMC5883L_RANGE_4_7GA:  2.56,
#             self.__HMC5883L_RANGE_5_6GA:  3.03,
#             self.__HMC5883L_RANGE_8_1GA:  4.35,
#         }
#         compass_range = switcher.get(argument)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_REG, self.__HMC5883L_REG_CONFIG_B)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_DO, argument << 5)
#         
#     def Compass_setMesurementMode(self,mode):
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_REG, self.__HMC5883L_REG_MODE)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_DO, mode)
#         
#     def Compass_setDataRate(self,dataRate):
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_REG, self.__HMC5883L_REG_CONFIG_A)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_DO, dataRate << 2)
#         
#     def Compass_setSamples(self,samples):
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_REG, self.__HMC5883L_REG_CONFIG_A)
#         self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_DO, samples << 5) 
#         
#     def Compass_setOffset(self,xo,yo):
#         self.xOffset = xo
#         self.yOffset = yo
#     
#     def Compass_test(self):
#         SLV0_RW=1
#         test=self.i2c.readU8(self.__MPU6050_RA_I2C_SLV0_ADDR)
#         return test

mpu6050 = MPU6050(0x68, 1, 1)
mpu6050.flushFIFO()
# mpu6050.initCompass()
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

end=0
end2=0
while True:
    elapse = time.time() - end
    elapse2 = time.time() - end2
    if elapse > 0.005:
        end = time.time()
        nfb = mpu6050.numFIFOBatches()
        ax, ay, az, rx, ry, rz, dt = mpu6050.readFIFO(nfb)
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)
    if elapse2 > 1:
        end2 = time.time()
        print(qax)
#     test1=mpu6050.Compass_test()
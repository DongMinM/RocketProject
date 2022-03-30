import serial
import smbus                    #import SMBus module of I2C
import time                      #import
import rospy
from std_msgs.msg import String
import numpy as np
import struct
import surbo


class MPU6050:
        def __init__ (self):
            self.bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
            self.Device_Address = 0x68
            self.PWR_MGMT_1   = 0x6B
            self.SMPLRT_DIV   = 0x19
            self.CONFIG       = 0x1A
            self.GYRO_CONFIG  = 0x1B
            self.INT_ENABLE   = 0x38
            self.ACCEL_XOUT_H = 0x3B
            self.ACCEL_YOUT_H = 0x3D
            self.ACCEL_ZOUT_H = 0x3F
            self.GYRO_XOUT_H  = 0x43
            self.GYRO_YOUT_H  = 0x45
            self.GYRO_ZOUT_H  = 0x47
            self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
                #Write to power management register
            self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

                #Write to Configuration register
            self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)

                #Write to Gyro configuration register
            self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)

                #Write to interrupt enable register
            self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)
            self.ser =  serial.Serial(port='/dev/ttyUSB0',
                                          baudrate=9600,
                                          parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE,
                                          bytesize=serial.EIGHTBITS,
                                          timeout=1)

        def read_raw_data(self,addr):
                #Accelero and Gyro value are 16-bit
                high = self.bus.read_byte_data(self.Device_Address, addr)
                low = self.bus.read_byte_data(self.Device_Address, addr+1)

                #concatenate higher and lower value
                value = ((high << 8) | low)

                #to get signed value from mpu6050
                if(value > 32768):
                        value = value - 65536
                return value

        def run(self):

                #Read Accelerometer raw value
                acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
                acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
                acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

                #Read Gyroscope raw value
                gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
                gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
                gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

                #Full scale range +/- 250 degree/C as per sensitivity scale factor
                Ax = acc_x/16384.0+0.07
                Ay = acc_y/16384.0-0.01
                Az = acc_z/16384.0-0.11

                Gx = gyro_x/131.0+0.18
                Gy = gyro_y/131.0+0.08
                Gz = gyro_z/131.0

                angleX = np.arctan(Ay/(Ax**2+Az**2)**0.5)*180/np.pi
                angleY = np.arctan(-Ax/(Ay**2+Az**2)**0.5)*180/np.pi+7
                # angleZ = np.arctan((Ax**2+Ay**2)**0.5/Az)*180/np.pi

                print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s",' ROLL : {:.2f} , PITCH : {:.2f}'.format(angleX,angleY))
                #print('PITCH : {:.2f}'.format(angleY))
                #data = struct.pack('5f',Gx+128,Gy+128,Gz+128,angleX+128,angleY+128)
                #self.ser.write(data)
                return [angleX,angleY]

if __name__ == "__main__":
    Mpu6050 = MPU6050()
    print("---")
    while True:
        angle = Mpu6050.run()
        for i in [0,1]:
                if angle[i] >= 20:
                       angle[i] = 20
                elif angle[i] <= -20:
                       angle[i] = -20
        surbo.setServo2Pos(90+angle[1])
        surbo.setServoPos(90+angle[0])
        time.sleep(0.2)
        print('=========================')

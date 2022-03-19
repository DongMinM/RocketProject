import serial
import numpy as np
import time
import struct

ser = serial.Serial(port='/dev/ttyUSB0',
                    baudrate = 9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=0.1)

while True:
        i = 0
        data = np.array([0.0,0.0,0.0,0.0,0.0])

        x=ser.read(20)
        if len(x) > 0 :
        #        for n in range(0,5):
        #                msg = struct.unpack('f',x[4*n:4*(n+1)])
        #                msg = list(msg)[0]-128
        #                data[i] = round(msg,2)
        #                i+=1

        #        print ("Gx=%.2f" %data[0], u'\u00b0'+ "/s", "\tGy=%.2f" %data[1], u'\u00b0'+ "/s", "\tGz=%.2f" %data[2], u'\u00b0'+ "/s",' ROLL : %.2f' %data[3] , u'\u00b0'' PITCH : %.2f' % data[4], u'\u00b0')
                 print('수신 받은 데이터 :  ',x[0])
                 print('===================')
#       time.sleep(0.1)

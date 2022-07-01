from re import A
import serial
import numpy as np
import time
import struct
import matplotlib.pyplot as plt
import asyncio

ser = serial.Serial(port='/dev/ttyUSB0',
                    baudrate = 9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=0.001)




run_time = [0]
roll = []
pitch = []
loop = asyncio.get_event_loop()
time1 = 0
time2 = 0

data = np.array([0.0,0.0,0.0,0.0,0.0])

while run_time[-1] < 10:
        x=ser.read(12)
        if len(x) > 0 :
                time1 = time.time()
                try:
                        for n in range(0,3):
                                msg = struct.unpack('f',x[4*n:4*(n+1)])
                                msg = list(msg)[0]
                                data[n] = round(msg,2)
                        
                        roll.append(data[1])
                        pitch.append(data[2])
                        # print(1)
                        
                except:
                        print('pass')
                        roll.append(roll[-1])
                        pitch.append(pitch[-1]) 
                        continue  
                time2 = time.time()  
                run_time.append(run_time[-1]+time2-time1+data[0]+0.03) 


plt.grid(True)
plt.ylim([-180,180])
plt.plot(run_time[:-1],roll[:])
plt.show()


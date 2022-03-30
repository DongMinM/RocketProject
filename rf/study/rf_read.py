import serial
import struct

ser = serial.Serial(port='/dev/ttyUSB0',
                    baudrate = 9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=0.1)

while True:

        x=ser.read(20)
        if len(x) > 0 :
                msg = struct.unpack('f',x[0:4])
                print(msg)

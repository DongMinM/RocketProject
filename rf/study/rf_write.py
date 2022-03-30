import struct
import serial

ser =  serial.Serial(port='/dev/ttyUSB0',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1)

while True:
        msg = float(input())
        print(msg)
        data = struct.pack('1f',msg)
        ser.write(data)

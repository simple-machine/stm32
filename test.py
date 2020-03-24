import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(1)
ser.write(b'smov')
ser.read(4)
ser.read(2)
ser.write(b'\x00')
ser.write(b'\x02')
print('a')
print(ser.read())
print('a')

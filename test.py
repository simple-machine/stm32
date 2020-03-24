import serial
import time

ser = serial.Serial('/dev/ttyACM1', 115200)
time.sleep(2)
print('a')
ser.write(b'smov')
ser.read(4)
ser.read(2)
ser.write(b'\x00')
print('b')
while True:
    ser.write(b'\x00')
    ser.read()
    time.sleep(.1)

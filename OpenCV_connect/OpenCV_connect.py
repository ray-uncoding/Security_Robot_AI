import numpy
import serial
from time import sleep
import sys

COM_PORT = 'COM4'

BAUD_RATES = 9600

print("Start!")
ser = serial.Serial(COM_PORT, BAUD_RATES, timeout = 0.5)
sx = 'ON'

try:
    while True:
        ser.write(sx.encode('ascii'))
        sleep(4)
        while ser.in_waiting:
            mcu_feedback = ser.readline().decode()
            print('response: ', mcu_feedback)
except KeyboardInterrupt:
    ser.close()
    print('colse')
print("end")

    